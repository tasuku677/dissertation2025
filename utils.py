from typing import List
import math

import numpy as np

from global_var import SimConfig
from uav import UAV 
from global_var import SimConfig

def _calculate_signal_strength(distance:float): 
    fspl =  20 * math.log10((4 * math.pi * distance * SimConfig.F) / SimConfig.C)
    ss = (20 - fspl)  # dbm 
    ss = 10 ** (ss / 10) # mWに変換
    return ss

def normalize_components(uav_i:UAV):
    if not uav_i.neighbors:
        return {}
    # 各指標の「隣接ごとの値」をまず集計
    ss_raw = {}
    pdr_raw = {}
    energy_raw = {}
    delay_raw = {}

    for uav_j in uav_i.neighbors:
        dist = np.linalg.norm(uav_i.pos - uav_j.pos)
        ss_raw[uav_j.id] = _calculate_signal_strength(dist) / 3.24
        pdr_raw[uav_j.id] = uav_j.sample_pdr(uav_j.type)
        energy_raw[uav_j.id] = min(uav_j.energy / SimConfig.INITIAL_ENERGY, 1.0)
        delay_raw[uav_j.id] = (2.45 - uav_j.sample_delay(uav_j.type)) / 1.54

    # コンポーネントごとに min-max 正規化（全て同値なら 0.5 固定）
    def minmax_dict(d: dict[int, float]) -> dict[int, float]:
        vals = list(d.values())
        mn, mx = min(vals), max(vals)
        if mx == mn:
            return {k: 0.5 for k in d.keys()}
        return {k: (v - mn) / (mx - mn) for k, v in d.items()}

    ss_norm = minmax_dict(ss_raw)
    pdr_norm = minmax_dict(pdr_raw)
    energy_norm = minmax_dict(energy_raw)
    delay_norm = minmax_dict(delay_raw)

    # 隣接IDごとにまとめた辞書を構築
    metrics_by_neighbor = {}
    for nid in ss_norm.keys():
        metrics_by_neighbor[nid] = {
            'ss': ss_norm[nid],
            'pdr': pdr_norm[nid],
            'energy': energy_norm[nid],
            'delay': delay_norm[nid],
        }
    return metrics_by_neighbor