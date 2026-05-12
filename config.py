from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class AinChannelConfig:
    name: str
    range_v: float = 10.0          # ±10 V, ±1 V, ±0.1 V, or ±0.01 V
    resolution_index: int = 0      # 0 = auto; 1–8 for T7 (higher = slower, more accurate)
    settling_us: float = 0.0       # 0 = auto


@dataclass
class ThermometryChannelConfig:
    """One temperature channel read via a T7 AIN extended feature.

    kind="rtd": the AIN is configured AINx_EF_INDEX=40 (PT100); the device
        reports temperature (K), resistance (Ω) and voltage (V) on
        AINx_EF_READ_A/B/C — no software conversion needed.
    kind="tc":  the AIN is configured AINx_EF_INDEX=22 (type K) and wired as a
        differential gradiometer; we take the measured ΔEMF (AINx_EF_READ_B, V)
        and re-reference it to `reference` — either the `name` of an RTD channel
        in this same list, or "plc/rtd/<n>" to use a PLC RTD value received over
        MQTT.  Since the ΔT is small, either end of the gradiometer sits within
        a few K of the reference, so this gives a usable absolute temperature.
    """
    name: str                          # short id, e.g. "rtd1" / "tc1"
    kind: str                          # "rtd" | "tc"
    ain: int                           # positive AIN number (0,2,4,... / 6,8,...)
    channel: int                       # MQTT channel number (global numbering)
    label: str = ""                    # human label, e.g. "nozzle"
    reference: str = ""                # for kind="tc": reference RTD source


@dataclass
class LabjackT7Config:
    connection_type: str = "ANY"          # "USB", "TCP", "ETHERNET", "WIFI", or "ANY"
    device_identifier: str = "ANY"        # serial number, IP address, or "ANY"
    poll_interval_s: float = 1.0
    ain_channels: List[AinChannelConfig] = field(default_factory=list)
    thermometry_channels: List[ThermometryChannelConfig] = field(default_factory=list)
    dio_read_channels: List[str] = field(default_factory=list)  # e.g. ["FIO0", "EIO1"]
    read_internal_temp: bool = True
    reconnect_delay_s: float = 5.0

    @classmethod
    def from_dict(cls, d: dict) -> "LabjackT7Config":
        ain_list = []
        for ch in d.get("ain_channels", []) or []:
            if isinstance(ch, str):
                ain_list.append(AinChannelConfig(name=ch))
            else:
                ain_list.append(AinChannelConfig(**ch))
        therm_list = []
        for ch in d.get("thermometry_channels", []) or []:
            therm_list.append(ThermometryChannelConfig(
                name=str(ch["name"]),
                kind=str(ch["kind"]).lower(),
                ain=int(ch["ain"]),
                channel=int(ch["channel"]),
                label=str(ch.get("label", "")),
                reference=str(ch.get("reference", "")),
            ))
        return cls(
            connection_type=str(d.get("connection_type", "ANY")),
            device_identifier=str(d.get("device_identifier", "ANY")),
            poll_interval_s=float(d.get("poll_interval_s", 1.0)),
            ain_channels=ain_list,
            thermometry_channels=therm_list,
            dio_read_channels=list(d.get("dio_read_channels", []) or []),
            read_internal_temp=bool(d.get("read_internal_temp", True)),
            reconnect_delay_s=float(d.get("reconnect_delay_s", 5.0)),
        )
