from typing import List
import math

import numpy as np

from global_var import SimConfig
from uav import UAV 
from global_var import SimConfig

def _calculate_signal_strength(distance:float): 
    fspl =  20 * math.log10((4 * math.pi * distance * SimConfig.F) / SimConfig.C)
    ss = (20 - fspl)  # dbm 
    ss_mw = 10 ** (ss / 10) # mW
    return ss_mw

def normalize_components(uav_x:UAV):
    if not uav_x.neighbors:
        return {}
    # 各指標の「隣接ごとの値」をまず集計
    ss_raw = {} # {uav_id: value}
    pdr_raw = {}
    energy_raw = {}
    delay_raw = {}

    for uav_j_id in uav_x.neighbors:
        info_of_uav_j = uav_x.packet_payload_history.get(uav_j_id, None)
        if info_of_uav_j is None:
            continue
        dist = np.linalg.norm(uav_x.pos - info_of_uav_j.pos)
        ss_raw[uav_j_id] = min(_calculate_signal_strength(dist) * 1e7 / 3.24 , 1.0)
        energy_raw[uav_j_id] = min(info_of_uav_j.energy / SimConfig.INITIAL_ENERGY, 1.0)
        
        history = uav_x.history_out.get(uav_j_id)
        if history and history['sent'] > 0:
            pdr_raw[uav_j_id] = history['success'] / history['sent']
            # 平均遅延 (Delay)
            if history['delays']:
                delay_raw[uav_j_id] = (2.45 - np.mean(history['delays']))/1.54
            else:
                delay_raw[uav_j_id] = 0.0 # 成功パケットなし = 遅延評価0 (最低評価)
        else:
            # 通信履歴がない場合 (PDR 0, 遅延評価 0)
            pdr_raw[uav_j_id] = 0.0
            delay_raw[uav_j_id] = 0.0

    # コンポーネントごとに min-max 正規化（全て同値なら 0.5 固定）
    def minmax_dict(d: dict[int, float]) -> dict[int, float]:
        if not d:
            return {}
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

def combine_gaussians(means, variances):
    """
    Combine multiple independent Gaussian distributions N(mu_i, sigma_i^2)
    into a single equivalent Gaussian N(mu*, sigma*^2).
    
    Parameters:
        means (list or np.ndarray): 各ガウス分布の平均 μ_i
        variances (list or np.ndarray): 各ガウス分布の分散 σ_i^2

    Returns:
        (mu_star, var_star): 合成後の平均と分散
    """
    precisions = 1 / np.array(variances)  # 分散の逆数 = precision（精度）
    var_star = 1 / np.sum(precisions)
    mu_star = var_star * np.sum(np.array(means) * precisions)
    return mu_star, var_star