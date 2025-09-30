from typing import Any, Literal



#TODO: PacketのdetaをAnyから具体的な型に変更する．たとえば，残留電力や信頼度などを含む辞書型など

class TelemetryPayload:
    def __init__(self, energy: float, trust: float, pos: tuple[float, float, float], cluster_id: int | None = None,
                 role: Literal["leader", "sub", "member"] = "member", pdr: float | None = None,
                 delay: float | None = None, extras: dict[str, float] | None = None):
        self.energy = energy
        self.trust = trust
        self.pos = pos
        self.cluster_id = cluster_id
        self.role = role
        self.pdr = pdr
        self.delay = delay
        self.extras = extras if extras is not None else {}

class Packet:
    def __init__(self, source_id: int, dest_id: int, data: Any, timestamp: float):
        self.source_id = source_id
        self.dest_id = dest_id
        self.data = data
        self.timestamp = timestamp
        
        
    