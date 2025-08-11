import math
import random
from typing import List, Tuple
# import pandas as pd
from uav import UAV
from global_set import *

random.seed(1)


def distance(u: UAV, v: UAV) -> float:
    return math.sqrt((u.x - v.x)**2 + (u.y - v.y)**2 + (u.z - v.z)**2)


# コンポーネント正規化
def normalize_components(uavs: List[UAV]):
    ss_vals = [u.ss for u in uavs]
    pdr_vals = [u.pdr for u in uavs]
    delay_vals = [u.delay for u in uavs]

    ss_trans = [min(1.0, v / 3.24) for v in ss_vals]
    pdr_trans = [min(1.0, v / 100.0) for v in pdr_vals]
    delay_trans = [max(0.0, (2.45 - (v/1000.0)) / 1.54) for v in delay_vals]

    def minmax(arr):
        mn, mx = min(arr), max(arr)
        if mx == mn:
            return [0.5]*len(arr)
        return [(x - mn) / (mx - mn) for x in arr]

    return minmax(ss_trans), minmax(pdr_trans), minmax(delay_trans)

# アルゴリズム2: Direct Trust
def calculate_direct_trust(uavs: List[UAV]):
    ss_norm, pdr_norm, delay_norm = normalize_components(uavs)
    w_ss, w_pdr, w_delay = 0.4, 0.4, 0.2
    for i,u in enumerate(uavs):
        R = w_ss * ss_norm[i] + w_pdr * pdr_norm[i] + w_delay * delay_norm[i]
        weight = R
        if weight <= 0.29:
            trust = 0.5 * (0.5 ** weight)
        elif 0.3 < weight < 0.69:
            trust = 0.5 * (1.5 ** weight)
        else:
            trust = 0.5 * (2.0 ** weight)
        u.direct_trust = max(0.0, min(1.0, trust))

# アルゴリズム3: Indirect Trust
def calculate_indirect_trust(uavs: List[UAV]):
    for i,u in enumerate(uavs):
        others = [v.direct_trust for j,v in enumerate(uavs) if j != i]
        recom = sum(others) / len(others) if others else u.direct_trust
        u.indirect_trust = math.sqrt(u.direct_trust * recom)

# Fitness 計算 (式1,2,3)
def calculate_fitness(uavs: List[UAV]):
    for u in uavs:
        davg = sum(distance(u, v) for v in uavs if v.id != u.id) / (len(uavs)-1)
        davg_norm = davg / 2000.0
        Fi = (A_COEFF * davg_norm) + (0.45 * u.energy)
        u.fitness = Fi

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

# UAV生成（悪意ノード含む）
def generate_random_uavs(n: int, MALICIOUS_RATE: float=0.1) -> List[UAV]:
    uavs = []
    for i in range(n):
        x = random.uniform(AREA[0], AREA[1])
        y = random.uniform(AREA[2], AREA[3])
        z = random.uniform(AREA[4], AREA[5])
        energy = random.uniform(0.2, 1.0)
        ss = random.uniform(20, 100)
        pdr = random.uniform(30, 100)
        delay = random.uniform(5, 100)
        u = UAV(i, x, y, z, energy, ss, pdr, delay)
        uavs.append(u)
    k = max(1, int(n * MALICIOUS_RATE))
    malicious_indices = random.sample(range(n), k)
    for idx in malicious_indices:
        uavs[idx].malicious = True
        uavs[idx].pdr = random.uniform(0, 40)
        uavs[idx].ss = random.uniform(0, 30)
        uavs[idx].delay = random.uniform(80, 200)
        uavs[idx].energy = random.uniform(0.05, 0.5)
    return uavs

# ==== 実行 ====
uavs = generate_random_uavs(NUM_OF_DRONES, MALICIOUS_RATE)
time_steps = 8
for t in range(time_steps):
    calculate_direct_trust(uavs)
    calculate_indirect_trust(uavs)
    calculate_fitness(uavs)
    calculate_final_trust(uavs)
    clusters = form_clusters(uavs, CLUSTER_SIZE)
    leaders = [select_leader(c) for c in clusters]
    # 簡易表示
    print(f"Step {t}: Clusters={len(clusters)}, AvgFinalTrust={sum(u.final_trust for u in uavs)/len(uavs):.3f}")

# 最終クラスタのリーダー表示
for cid, c in enumerate(clusters):
    leader, sub = select_leader(c)
    print(f"Cluster {cid}: Leader={leader.id} Trust={leader.final_trust:.3f}, SubLeader={sub.id if sub else None}")
