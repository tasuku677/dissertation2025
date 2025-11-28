from typing import List, Literal, TYPE_CHECKING


class TelemetryPayload:
    def __init__(self, energy: float, trust: float, pos: tuple[float, float, float], neighbors:List[int], direct_trust_to_neighbors:dict, cluster_id: int | None = None,
                 role: Literal["leader", "sub", "member"] = "member", timestamp: float = 0.0):
        self.energy = energy
        self.trust = trust
        self.pos = pos
        self.neighbors = neighbors
        self.direct_trust_to_neighbors = direct_trust_to_neighbors
        self.cluster_id = cluster_id
        self.role = role
        self.timestamp = timestamp

class ClusterReportPayload:
    """クラスタメンバーからリーダーへの報告用ペイロード"""
    def __init__(self, member_id: int):
        self.member_id = member_id


class Packet:
    def __init__(self, source_id: int, dest_id: int, payload: TelemetryPayload, timestamp: float):
        self.source_id = source_id
        self.dest_id = dest_id
        self.payload = payload
        self.timestamp = timestamp
        
    