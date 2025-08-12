import csv
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
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

# Fitness 計算 (式1,2,3)
def calculate_fitness(uavs: List[UAV]):
    for u in uavs:
        distances = [distance(u, v) for v in uavs if v.id != u.id]
        davg = sum(distances) / (len(uavs) - 1)
        davg_norm = davg / max(distances) # 正規化
        Fi = (A_COEFF * davg_norm) + (0.45 * u.energy)
        u.fitness = Fi
        
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
        uavs.append(UAV(i, x, y, z, ss, pdr, delay, energy))
    k = max(1, int(n * MALICIOUS_RATE))
    malicious_indices = random.sample(range(n), k)
    for idx in malicious_indices:
        uavs[idx].malicious = True
        uavs[idx].pdr = random.uniform(0, 40)
        uavs[idx].ss = random.uniform(0, 30)
        uavs[idx].delay = random.uniform(80, 200)
        uavs[idx].energy = random.uniform(0.05, 0.5)
    
    export_uavs_to_csv(uavs, "uavs_initial.csv")
    return uavs


def export_uavs_to_csv(uavs, filename="uavs_output.csv"):
    with open(filename, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        # ヘッダー
        writer.writerow([
            "id", "x", "y", "z", "energy", "ss", "pdr", "delay",
            "direct_trust", "indirect_trust", "fitness", "final_trust", "malicious"
        ])
        # 各UAVのデータ
        for u in uavs:
            writer.writerow([
                u.id, u.x, u.y, u.z, u.energy, u.ss, u.pdr, u.delay,
                u.direct_trust, u.indirect_trust, u.fitness, u.final_trust, u.malicious
            ])

def plot_uavs_3d(uavs):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    xs = [u.x for u in uavs]
    ys = [u.y for u in uavs]
    zs = [u.z for u in uavs]
    ax.scatter(xs, ys, zs, c=[u.energy for u in uavs],cmap='Blues', marker='o')
    # 各ノードに番号（id）を表示
    for u in uavs:
        ax.text(u.x, u.y, u.z, str(u.id), color='red', fontsize=9)
        # Y軸の向きを逆にする
    y_min = min(ys) if ys else 0
    y_max = max(ys) if ys else 1
    ax.set_ylim(y_max, y_min)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()