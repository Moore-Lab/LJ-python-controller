from __future__ import annotations

import logging
from typing import List, Optional

from .config import AinChannelConfig, LabjackT7Config

log = logging.getLogger(__name__)

try:
    from labjack import ljm as _ljm
    _LJM_AVAILABLE = True
except ImportError:
    _ljm = None  # type: ignore[assignment]
    _LJM_AVAILABLE = False
    log.warning("labjack-ljm not installed; LabJack T7 will not be available")

# Valid T7 voltage ranges for AIN channels
_VALID_RANGES = {10.0, 1.0, 0.1, 0.01}

# DAC channels supported by the T7
_DAC_CHANNELS = {"DAC0", "DAC1"}

# AIN_EF feature indices (T-series datasheet § Extended Features)
_EF_INDEX_PT100 = 40   # 4-wire PT100, output in K on EF_READ_A
_EF_INDEX_TYPEK = 22   # Type-K thermocouple, output in K on EF_READ_A
# Modbus address of TEMPERATURE_DEVICE_K — used as the cold-junction source
# for the type-K EF (datasheet § Thermocouples).
_TEMPERATURE_DEVICE_K_ADDR = 60052


class LabJackT7Device:
    """Low-level LJM wrapper for a LabJack T7."""

    def __init__(self, config: LabjackT7Config) -> None:
        self._config = config
        self._handle: Optional[int] = None
        self._serial: Optional[str] = None

    @property
    def connected(self) -> bool:
        return self._handle is not None

    @property
    def serial(self) -> Optional[str]:
        return self._serial

    @property
    def handle(self) -> Optional[int]:
        return self._handle

    def connect(self) -> None:
        if not _LJM_AVAILABLE:
            raise RuntimeError("labjack-ljm package is not installed")
        cfg = self._config
        self._handle = _ljm.openS("T7", cfg.connection_type, cfg.device_identifier)
        self._serial = str(int(_ljm.eReadName(self._handle, "SERIAL_NUMBER")))
        for ch_cfg in cfg.ain_channels:
            self._configure_ain(ch_cfg)
        # AIN_EF (extended-feature) config lives in T7 RAM and is wiped by a
        # power cycle. Re-apply it on every (re)connect so a freshly-rebooted
        # T7 is self-healing — no need to run a separate setup script.
        if cfg.thermometry_channels:
            self._configure_thermometry(cfg.thermometry_channels)
        log.info("[labjack_t7] connected to T7 serial=%s", self._serial)

    def disconnect(self) -> None:
        if self._handle is not None:
            try:
                _ljm.close(self._handle)
            except Exception as exc:
                log.warning("[labjack_t7] error during close: %s", exc)
            self._handle = None
            self._serial = None
            log.info("[labjack_t7] disconnected")

    def _require_connected(self) -> None:
        if self._handle is None:
            raise RuntimeError("LabJack T7 is not connected")

    def _configure_ain(self, ch: AinChannelConfig) -> None:
        n = ch.name.upper()
        if ch.range_v not in _VALID_RANGES:
            raise ValueError(f"Invalid AIN range {ch.range_v} V for {n}; choose from {_VALID_RANGES}")
        _ljm.eWriteName(self._handle, f"{n}_RANGE", ch.range_v)
        _ljm.eWriteName(self._handle, f"{n}_RESOLUTION_INDEX", float(ch.resolution_index))
        _ljm.eWriteName(self._handle, f"{n}_SETTLING_US", ch.settling_us)

    def _configure_thermometry(self, channels) -> None:
        """Apply AIN_EF configuration for the thermometry channels.

        RTDs use EF index 40 (PT100, 4-wire, output in K, ±0.1 V range).
        Type-K TCs use EF index 22 (output in K, ±0.01 V range), with
        TEMPERATURE_DEVICE_K wired in as the cold-junction source.
        """
        names: list[str] = []
        values: list[float] = []
        for ch in channels:
            ain = int(ch.ain)
            kind = str(ch.kind).lower()
            p = f"AIN{ain}"
            if kind == "rtd":
                names += [f"{p}_NEGATIVE_CH", f"{p}_RANGE",
                          f"{p}_RESOLUTION_INDEX", f"{p}_EF_INDEX",
                          f"{p}_EF_CONFIG_A"]
                values += [float(ain + 1), 0.1, 0.0,
                           float(_EF_INDEX_PT100), 0.0]
            elif kind == "tc":
                names += [f"{p}_NEGATIVE_CH", f"{p}_RANGE",
                          f"{p}_RESOLUTION_INDEX", f"{p}_EF_INDEX",
                          f"{p}_EF_CONFIG_A", f"{p}_EF_CONFIG_B",
                          f"{p}_EF_CONFIG_D", f"{p}_EF_CONFIG_E"]
                values += [float(ain + 1), 0.01, 0.0,
                           float(_EF_INDEX_TYPEK), 0.0,
                           float(_TEMPERATURE_DEVICE_K_ADDR), 1.0, 0.0]
            else:
                log.warning("[labjack_t7] unknown thermometry kind %r for %s",
                            kind, ch.name)
        if names:
            _ljm.eWriteNames(self._handle, len(names), names, values)
            log.info("[labjack_t7] applied AIN_EF config to %d channel(s)",
                     len(channels))

    # ------------------------------------------------------------------
    # Reads
    # ------------------------------------------------------------------

    def read_ain(self, channel: str) -> float:
        self._require_connected()
        return _ljm.eReadName(self._handle, channel.upper())

    def read_ains(self, channels: List[str]) -> List[float]:
        self._require_connected()
        names = [c.upper() for c in channels]
        return list(_ljm.eReadNames(self._handle, len(names), names))

    def read_dio(self, channel: str) -> int:
        self._require_connected()
        return int(_ljm.eReadName(self._handle, channel.upper()))

    def read_internal_temp_k(self) -> float:
        self._require_connected()
        return _ljm.eReadName(self._handle, "TEMPERATURE_DEVICE_K")

    # ------------------------------------------------------------------
    # Writes
    # ------------------------------------------------------------------

    def write_dac(self, channel: str, value: float) -> None:
        self._require_connected()
        ch = channel.upper()
        if ch not in _DAC_CHANNELS:
            raise ValueError(f"Invalid DAC channel {channel!r}; choose from {_DAC_CHANNELS}")
        _ljm.eWriteName(self._handle, ch, value)

    def write_dio(self, channel: str, value: int) -> None:
        self._require_connected()
        _ljm.eWriteName(self._handle, channel.upper(), float(value))

    # ------------------------------------------------------------------
    # Raw register access
    # ------------------------------------------------------------------

    def read_register(self, name: str) -> float:
        self._require_connected()
        return _ljm.eReadName(self._handle, name)

    def write_register(self, name: str, value: float) -> None:
        self._require_connected()
        _ljm.eWriteName(self._handle, name, value)

    def read_registers(self, names: List[str]) -> List[float]:
        self._require_connected()
        return list(_ljm.eReadNames(self._handle, len(names), names))

    def write_registers(self, names: List[str], values: List[float]) -> None:
        self._require_connected()
        _ljm.eWriteNames(self._handle, len(names), names, values)

    # ------------------------------------------------------------------
    # Runtime AIN reconfiguration
    # ------------------------------------------------------------------

    def configure_ain(self, ch: AinChannelConfig) -> None:
        self._require_connected()
        self._configure_ain(ch)
