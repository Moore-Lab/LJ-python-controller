"""
LabJack T7 controller — xsphere-slow-control plugin.

MQTT interface
--------------
Commands (inbound):
  xsphere/commands/labjack_t7/read
      {"channels": ["AIN0", "FIO0", ...]}
  xsphere/commands/labjack_t7/write_dac
      {"channel": "DAC0", "value": 2.5}
  xsphere/commands/labjack_t7/write_dio
      {"channel": "FIO0", "value": 1}
  xsphere/commands/labjack_t7/configure
      {"poll_interval_s": 2.0,
       "ain_channels": [{"name": "AIN0", "range_v": 10.0}, ...],
       "dio_read_channels": ["FIO0"],
       "read_internal_temp": true}
  xsphere/commands/labjack_t7/reset
      {}

Sensors (outbound, polled and on-demand):
  xsphere/sensors/labjack_t7/ain/{ch}      {"value": float, "unit": "V"}
  xsphere/sensors/labjack_t7/dio/{ch}      {"value": int}
  xsphere/sensors/labjack_t7/temperature   {"value": float, "unit": "K"}

Status (outbound):
  xsphere/status/labjack_t7
      {"connected": bool, "serial": str|null, "error": str|null,
       "last_read_utc": str|null, "poll_interval_s": float}
"""

from __future__ import annotations

import logging
import threading
from datetime import datetime, timezone
from typing import Optional

from .config import AinChannelConfig, LabjackT7Config
from .device import LabJackT7Device

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Compatibility shim — import from xsphere if available, else define stubs
# so the module can be imported and tested without the full slow-control stack.
# ---------------------------------------------------------------------------
try:
    from slowcontrol.controllers.base import Controller
    from slowcontrol.core.mqtt import command_topic
except ImportError:
    from abc import ABC, abstractmethod

    class Controller(ABC):  # type: ignore[no-redef]
        NAME: str = "unnamed_controller"

        def __init__(self, config, mqtt):
            self._config = config
            self._mqtt = mqtt

        @abstractmethod
        def start(self) -> None: ...

        @abstractmethod
        def stop(self) -> None: ...

    def command_topic(*parts: str) -> str:  # type: ignore[misc]
        return "xsphere/commands/" + "/".join(parts)


class LabJackT7Controller(Controller):
    """xsphere-slow-control plugin for a LabJack T7 DAQ device.

    Wire into SlowControlService.__init__() alongside other controllers:

        from labjack_t7 import LabJackT7Controller, LabjackT7Config

        lj_cfg = LabjackT7Config(
            connection_type="USB",
            ain_channels=[AinChannelConfig("AIN0"), AinChannelConfig("AIN1")],
        )
        self._controllers.append(LabJackT7Controller(self._config, self._mqtt, lj_cfg))
    """

    NAME = "labjack_t7"

    def __init__(self, config, mqtt, labjack_config: Optional[LabjackT7Config] = None) -> None:
        super().__init__(config, mqtt)

        # Resolve plugin config: explicit arg > attribute on ServiceConfig > defaults
        if labjack_config is not None:
            self._lj_cfg = labjack_config
        elif hasattr(config, "labjack") and config.labjack is not None:
            self._lj_cfg = LabjackT7Config.from_dict(vars(config.labjack))
        else:
            self._lj_cfg = LabjackT7Config()

        self._device = LabJackT7Device(self._lj_cfg)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_read_utc: Optional[str] = None
        self._error: Optional[str] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        self._stop_event.clear()
        self._connect_device()

        self._mqtt.subscribe(command_topic(self.NAME, "read"), self._on_read)
        self._mqtt.subscribe(command_topic(self.NAME, "write_dac"), self._on_write_dac)
        self._mqtt.subscribe(command_topic(self.NAME, "write_dio"), self._on_write_dio)
        self._mqtt.subscribe(command_topic(self.NAME, "configure"), self._on_configure)
        self._mqtt.subscribe(command_topic(self.NAME, "reset"), self._on_reset)

        self._thread = threading.Thread(
            target=self._poll_loop, name="labjack-t7-poll", daemon=True
        )
        self._thread.start()
        log.info("[labjack_t7] started")

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=15)
        self._device.disconnect()
        log.info("[labjack_t7] stopped")

    # ------------------------------------------------------------------
    # Connection helpers
    # ------------------------------------------------------------------

    def _connect_device(self) -> None:
        try:
            self._device.connect()
            with self._lock:
                self._error = None
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
            log.error("[labjack_t7] connection failed: %s", exc)
        self._publish_status()

    # ------------------------------------------------------------------
    # Poll loop
    # ------------------------------------------------------------------

    def _poll_loop(self) -> None:
        while not self._stop_event.is_set():
            if not self._device.connected:
                log.info(
                    "[labjack_t7] not connected; retrying in %.1f s",
                    self._lj_cfg.reconnect_delay_s,
                )
                self._stop_event.wait(self._lj_cfg.reconnect_delay_s)
                if not self._stop_event.is_set():
                    self._connect_device()
                continue

            self._do_poll()
            self._stop_event.wait(self._lj_cfg.poll_interval_s)

    def _do_poll(self) -> None:
        cfg = self._lj_cfg
        try:
            if cfg.ain_channels:
                names = [ch.name for ch in cfg.ain_channels]
                values = self._device.read_ains(names)
                for name, value in zip(names, values):
                    self._mqtt.publish_sensor(
                        self.NAME, "ain", name.lower(),
                        payload={"value": value, "unit": "V"},
                    )

            for ch in cfg.dio_read_channels:
                val = self._device.read_dio(ch)
                self._mqtt.publish_sensor(
                    self.NAME, "dio", ch.lower(),
                    payload={"value": val},
                )

            if cfg.read_internal_temp:
                temp_k = self._device.read_internal_temp_k()
                self._mqtt.publish_sensor(
                    self.NAME, "temperature",
                    payload={"value": temp_k, "unit": "K"},
                )

            with self._lock:
                self._last_read_utc = datetime.now(timezone.utc).isoformat()
                self._error = None

        except Exception as exc:
            log.error("[labjack_t7] poll error: %s", exc)
            with self._lock:
                self._error = str(exc)
            self._device.disconnect()

        self._publish_status()

    # ------------------------------------------------------------------
    # MQTT command handlers
    # ------------------------------------------------------------------

    def _on_read(self, topic: str, payload: dict) -> None:
        channels = payload.get("channels", [])
        if not channels:
            log.warning("[labjack_t7] read: no channels specified")
            return
        if not self._device.connected:
            log.warning("[labjack_t7] read: device not connected")
            return
        try:
            for ch in channels:
                upper = ch.upper()
                if upper.startswith("AIN"):
                    val = self._device.read_ain(upper)
                    self._mqtt.publish_sensor(
                        self.NAME, "ain", ch.lower(),
                        payload={"value": val, "unit": "V"},
                    )
                else:
                    val = self._device.read_dio(upper)
                    self._mqtt.publish_sensor(
                        self.NAME, "dio", ch.lower(),
                        payload={"value": val},
                    )
        except Exception as exc:
            log.error("[labjack_t7] on_read error: %s", exc)
            with self._lock:
                self._error = str(exc)
            self._publish_status()

    def _on_write_dac(self, topic: str, payload: dict) -> None:
        try:
            channel = str(payload["channel"])
            value = float(payload["value"])
        except (KeyError, ValueError) as exc:
            log.warning("[labjack_t7] write_dac bad payload: %s", exc)
            return
        if not self._device.connected:
            log.warning("[labjack_t7] write_dac: device not connected")
            return
        try:
            self._device.write_dac(channel, value)
            log.info("[labjack_t7] %s = %.4f V", channel.upper(), value)
        except Exception as exc:
            log.error("[labjack_t7] write_dac error: %s", exc)
            with self._lock:
                self._error = str(exc)
            self._publish_status()

    def _on_write_dio(self, topic: str, payload: dict) -> None:
        try:
            channel = str(payload["channel"])
            value = int(payload["value"])
        except (KeyError, ValueError) as exc:
            log.warning("[labjack_t7] write_dio bad payload: %s", exc)
            return
        if not self._device.connected:
            log.warning("[labjack_t7] write_dio: device not connected")
            return
        try:
            self._device.write_dio(channel, value)
            log.info("[labjack_t7] %s = %d", channel.upper(), value)
        except Exception as exc:
            log.error("[labjack_t7] write_dio error: %s", exc)
            with self._lock:
                self._error = str(exc)
            self._publish_status()

    def _on_configure(self, topic: str, payload: dict) -> None:
        cfg = self._lj_cfg
        try:
            if "poll_interval_s" in payload:
                cfg.poll_interval_s = float(payload["poll_interval_s"])
            if "ain_channels" in payload:
                new_ains = [
                    AinChannelConfig(name=ch) if isinstance(ch, str)
                    else AinChannelConfig(**ch)
                    for ch in payload["ain_channels"]
                ]
                cfg.ain_channels = new_ains
                if self._device.connected:
                    for ch_cfg in new_ains:
                        self._device.configure_ain(ch_cfg)
            if "dio_read_channels" in payload:
                cfg.dio_read_channels = list(payload["dio_read_channels"])
            if "read_internal_temp" in payload:
                cfg.read_internal_temp = bool(payload["read_internal_temp"])
            log.info("[labjack_t7] config updated: %s", payload)
        except (KeyError, ValueError, TypeError) as exc:
            log.warning("[labjack_t7] configure bad payload: %s", exc)

    def _on_reset(self, topic: str, payload: dict) -> None:
        log.info("[labjack_t7] reset requested")
        self._device.disconnect()
        self._connect_device()

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def _publish_status(self) -> None:
        with self._lock:
            error = self._error
            last = self._last_read_utc
        self._mqtt.publish_status(
            self.NAME,
            payload={
                "connected": self._device.connected,
                "serial": self._device.serial,
                "error": error,
                "last_read_utc": last,
                "poll_interval_s": self._lj_cfg.poll_interval_s,
            },
        )
