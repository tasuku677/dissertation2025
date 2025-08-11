import math
import random
from typing import List

from uav import UAV 
from global_set import *


def distance(u: UAV, v: UAV) -> float:
    return math.sqrt((u.x - v.x)**2 + (u.y - v.y)**2 + (u.z - v.z)**2)


# コンポーネント正規化
def normalize_components(uavs: List[UAV]):
    ss_vals = [u.ss for u in uavs]
    pdr_vals = [u.pdr for u in uavs]
    energy_vals = [u.energy for u in uavs]
    delay_vals = [u.delay for u in uavs]

    ss_trans = [min(1.0, v / 3.24) for v in ss_vals]
    pdr_trans = [min(1.0, v / 100.0) for v in pdr_vals]
    energy_trans = [min(1.0, v) for v in energy_vals]
    delay_trans = [max(0.0, (2.45 - (v/1000.0)) / 1.54) for v in delay_vals]

    def minmax(arr):
        mn, mx = min(arr), max(arr)
        if mx == mn:
            return [0.5]*len(arr)
        return [(x - mn) / (mx - mn) for x in arr]

    return minmax(ss_trans), minmax(pdr_trans), minmax(energy_trans), minmax(delay_trans)

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
        uavs.append(UAV(i, x, y, z, energy, ss, pdr, delay))
    k = max(1, int(n * MALICIOUS_RATE))
    malicious_indices = random.sample(range(n), k)
    for idx in malicious_indices:
        uavs[idx].malicious = True
        uavs[idx].pdr = random.uniform(0, 40)
        uavs[idx].ss = random.uniform(0, 30)
        uavs[idx].delay = random.uniform(80, 200)
        uavs[idx].energy = random.uniform(0.05, 0.5)
    return uavs
