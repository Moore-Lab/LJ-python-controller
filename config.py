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
class LabjackT7Config:
    connection_type: str = "ANY"          # "USB", "TCP", "WIFI", or "ANY"
    device_identifier: str = "ANY"        # serial number, IP address, or "ANY"
    poll_interval_s: float = 1.0
    ain_channels: List[AinChannelConfig] = field(default_factory=list)
    dio_read_channels: List[str] = field(default_factory=list)  # e.g. ["FIO0", "EIO1"]
    read_internal_temp: bool = True
    reconnect_delay_s: float = 5.0

    @classmethod
    def from_dict(cls, d: dict) -> "LabjackT7Config":
        ain_list = []
        for ch in d.get("ain_channels", []):
            if isinstance(ch, str):
                ain_list.append(AinChannelConfig(name=ch))
            else:
                ain_list.append(AinChannelConfig(**ch))
        return cls(
            connection_type=str(d.get("connection_type", "ANY")),
            device_identifier=str(d.get("device_identifier", "ANY")),
            poll_interval_s=float(d.get("poll_interval_s", 1.0)),
            ain_channels=ain_list,
            dio_read_channels=list(d.get("dio_read_channels", [])),
            read_internal_temp=bool(d.get("read_internal_temp", True)),
            reconnect_delay_s=float(d.get("reconnect_delay_s", 5.0)),
        )
