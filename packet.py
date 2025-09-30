import asyncio
import numpy as np
import random
from dataclasses import dataclass
from typing import Any, Literal

from global_var import SimConfig


#TODO: PacketのdetaをAnyから具体的な型に変更する．たとえば，残留電力や信頼度などを含む辞書型など

@dataclass(slots=True)
class TelemetryPayload:
    energy: float
    trust: float
    pos: tuple[float, float, float]
    cluster_id: int | None = None
    role: Literal["leader", "sub", "member"] = "member"
    pdr: float | None = None
    delay: float | None = None
    extras: dict[str, float] | None = None

class Packet:
    def __init__(self, source_id: int, dest_id: int, data: Any, timestamp: float):
        self.source_id = source_id
        self.dest_id = dest_id
        self.data = data
        self.timestamp = timestamp
        
        
    