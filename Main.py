import random

# import pandas as pd

from utils import *
from global_set import *
from algorithms import *    


# ==== 実行 ====
# random.seed(1)
uavs = generate_random_uavs(NUM_OF_DRONES, MALICIOUS_RATE)

for t in range(TIME_STEPS):
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

plot_uavs_3d(uavs)