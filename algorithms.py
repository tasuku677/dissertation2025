from typing import List, Tuple

from uav import UAV
from utils import *
from global_set import *


# アルゴリズム2: Direct Trust
def calculate_direct_trust(uavs: List[UAV]):
    ss_norm, pdr_norm, energy_norm,delay_norm = normalize_components(uavs)
    w_ss, w_pdr, w_energy,w_delay = 0.25, 0.25, 0.25, 0.25 #R_i1, R_i2, R_i3, R_i4
    for i,u in enumerate(uavs):
        total_weight = w_ss * ss_norm[i] + w_pdr * pdr_norm[i] + w_energy * energy_norm[i]  + w_delay * delay_norm[i]
        if total_weight <= 0.29:
            trust = 0.5 * (0.5 ** total_weight)
        elif 0.3 < total_weight < 0.69:
            trust = 0.5 * (1.5 ** total_weight)
        else:
            trust = 0.5 * (2.0 ** total_weight)
        u.direct_trust = max(0.0, min(1.0, trust))

# アルゴリズム3: Indirect Trust
def calculate_indirect_trust(uavs: List[UAV]):
    for i,u in enumerate(uavs):
        others = [v.direct_trust for j,v in enumerate(uavs) if j != i]
        recom = sum(others) / len(others) if others else u.direct_trust
        u.indirect_trust = math.sqrt(u.direct_trust * recom)

# アルゴリズム1: Final Trust
def calculate_final_trust(uavs: List[UAV]):
    for u in uavs:
        HT = l * u.direct_trust + m * u.indirect_trust
        FTi = a * u.fitness + b * HT
        u.final_trust = max(0.0, min(1.0, FTi))

# アルゴリズム4: Cluster Formation
def form_clusters(uavs: List[UAV], CLUSTER_SIZE: int) -> List[List[UAV]]:
    remaining = sorted(uavs, key=lambda u: (u.x, u.y, u.z))
    clusters = []
    processed = set()
    for u in remaining:
        if u.id in processed:
            continue
        cluster = [u]
        processed.add(u.id)
        candidates = [(distance(u, v), v) for v in remaining if v.id not in processed]
        candidates.sort(key=lambda x: x[0])
        for _, v in candidates[:CLUSTER_SIZE-1]:
            cluster.append(v)
            processed.add(v.id)
        clusters.append(cluster)
    return clusters

# アルゴリズム5: Leader Selection
def select_leader(cluster: List[UAV]) -> Tuple[UAV, UAV]:
    tlist = []
    for u in cluster:
        sumd = sum(distance(u, v) for v in cluster if v.id != u.id)
        denom = sumd if sumd > 0 else 1e-6
        topt = u.final_trust / denom
        tlist.append((topt, u))
    tlist.sort(key=lambda x: x[0], reverse=True)
    leader = tlist[0][1]
    subleader = tlist[1][1] if len(tlist) > 1 else None
    return leader, subleader