import numpy as np
import matplotlib.pyplot as plt
import random

from draw import LiveVisualizer
from uav import UAV

# --- TDLS-FANET シミュレーションクラス ---
class TdlsFanetSimulation:
    def __init__(self, config):
        self.config = config
        self.drones = [UAV(i, config) for i in range(config.NUM_DRONES)]
        self.clusters = {}
        # 評価指標記録用
        self.history = {'time': [], 'energy': [], 'delay': [], 'pdr': []}
        # 描画クラス
        self.visualizer = LiveVisualizer(config.AREA_SIZE, self.drones)


    # Algorithm 2: 直接信頼度の計算 (簡略版)
    def _calculate_direct_trust(self, uav_i, uav_j): #uav_i から見た uav_j の直接信頼値
        # 論文のアルゴリズムは非常に複雑なため、ここではPDR, 遅延, 信号強度を模擬
        # 信号強度は距離の逆二乗に比例すると仮定
        dist = np.linalg.norm(uav_i.pos - uav_j.pos)
        signal_strength = 1 / (dist**2) if dist > 0 else 1.0

        # PDRと遅延はドローンの種類によって変動
        if uav_j.type == 'good':
            pdr = random.uniform(0.95, 1.0)
            delay = random.uniform(0.01, 0.05) # seconds
        elif uav_j.type == 'neutral':
            pdr = random.uniform(0.7, 0.94)
            delay = random.uniform(0.05, 0.1)
        else: # bad
            pdr = random.uniform(0.1, 0.5)
            delay = random.uniform(0.1, 0.5)
            
        # パラメータを正規化 (0-1の範囲に)
        norm_ss = min(signal_strength * 1e-4, 1.0)
        norm_pdr = pdr
        norm_delay = 1 - min(delay, 1.0) # 遅延が小さいほど高評価

        # 重み付けして直接信頼度を算出
        direct_trust = 0.4 * norm_ss + 0.4 * norm_pdr + 0.2 * norm_delay
        return direct_trust

    # Algorithm 3: 間接信頼度の計算
    def _calculate_indirect_trust(self, uav_i):
        if not uav_i.neighbors:
            return 0.0
        
        recommendations = []
        for neighbor_k in uav_i.neighbors:
            # neighbor_kから見たuav_iへの直接信頼度 (本来はkがiを評価する)
            # ここでは簡単のため、iがkを評価した値を使う
            rec = self._calculate_direct_trust(uav_i, neighbor_k)
            recommendations.append(rec)
            
        avg_rec = np.mean(recommendations)
        return avg_rec

    # Fitnessスコア計算 (論文 Eq. 1)
    def _calculate_fitness_score(self, uav):
        if not uav.neighbors:
            return 0.0
        
        d_avg = np.mean([np.linalg.norm(uav.pos - n.pos) for n in uav.neighbors])
        r_e = uav.energy / self.config.INITIAL_ENERGY # 残存エネルギー率 (正規化)

        # D_avgも正規化が必要
        norm_d_avg = 1 - (d_avg / (self.config.COMM_RANGE * 1.5))

        fitness_score = self.config.A_COEFFICIENT * norm_d_avg + 0.45 * r_e
        return fitness_score

    # Algorithm 1: 最終信頼度の計算
    def _calculate_final_trust(self, uav):
        total_direct_trust = 0
        if uav.neighbors:
            total_direct_trust = np.mean([self._calculate_direct_trust(uav, n) for n in uav.neighbors])
        
        indirect_trust = self._calculate_indirect_trust(uav)
        
        # Hybrid Trust
        hybrid_trust = 0.5 * total_direct_trust + 0.5 * indirect_trust
        
        fitness_score = self._calculate_fitness_score(uav)
        
        final_trust = (self.config.B_COEFFICIENT_FITNESS * fitness_score +
                       self.config.B_COEFFICIENT_HYBRID * hybrid_trust)
        
        uav.trust_score = final_trust # ドローンの信頼度を更新
        return final_trust

    # Algorithm 4: クラスタ形成
    def form_clusters(self):
        # 信頼度が低いドローンを除外
        eligible_drones = [d for d in self.drones if d.trust_score > 0.4]
        
        processed_drones = set()
        self.clusters = {}
        cluster_id_counter = 0

        # ドローンをX座標でソート
        sorted_drones = sorted(eligible_drones, key=lambda d: d.pos[0])

        for drone in sorted_drones:
            if drone.id in processed_drones:
                continue

            cluster_id_counter += 1
            new_cluster = [drone]
            drone.cluster_id = cluster_id_counter
            processed_drones.add(drone.id)

            # 距離が近いドローンを同じクラスタに追加
            for other_drone in sorted_drones:
                if other_drone.id not in processed_drones:
                    dist = np.linalg.norm(drone.pos - other_drone.pos)
                    # クラスタのサイズも考慮 (仮に5機まで)
                    if dist < self.config.COMM_RANGE * 0.7 and len(new_cluster) < 5:
                        new_cluster.append(other_drone)
                        other_drone.cluster_id = cluster_id_counter
                        processed_drones.add(other_drone.id)
            
            self.clusters[cluster_id_counter] = new_cluster

    # Algorithm 5: リーダー選出
    def select_leaders(self):
        for cid, members in self.clusters.items():
            if len(members) < 2:
                if members: members[0].is_leader = True
                continue
                
            leader_scores = []
            for member in members:
                # 論文 Eq. 5
                sum_of_distances = sum([np.linalg.norm(member.pos - other.pos) for other in members if member.id != other.id])
                if sum_of_distances == 0:
                    score = member.trust_score
                else:
                    score = member.trust_score / sum_of_distances
                leader_scores.append((score, member))

            # スコアでソートし、リーダーとサブリーダーを決定
            leader_scores.sort(key=lambda x: x[0], reverse=True)
            
            leader = leader_scores[0][1]
            leader.is_leader = True
            
            if len(leader_scores) > 1:
                sub_leader = leader_scores[1][1]
                sub_leader.is_sub_leader = True

    def _simulate_communication(self):
        # 通信をシミュレートし、PDRと遅延を計測
        total_packets = 0
        successful_packets = 0
        total_delay = 0

        for cid, members in self.clusters.items():
            leader = next((m for m in members if m.is_leader), None)
            if not leader:
                continue

            for member in members:
                if member.is_leader:
                    continue
                
                # メンバーからリーダーへの通信
                total_packets += 1
                member.consume_energy_tx(self.config.PACKET_SIZE)
                
                dist = np.linalg.norm(member.pos - leader.pos)
                # 成功確率 (PDR) は信頼度と距離に依存すると仮定
                success_prob = leader.trust_score * (1 - dist / (self.config.COMM_RANGE * 2))
                if random.random() < success_prob:
                    successful_packets += 1
                    # 遅延は距離に比例
                    total_delay += dist / (3 * 1e8) # 光速で割る
                else:
                    total_delay += 0.5 # タイムアウト遅延

        pdr = successful_packets / total_packets if total_packets > 0 else 1.0
        avg_delay = total_delay / total_packets if total_packets > 0 else 0.0
        return pdr, avg_delay
        
    def run(self):
        for t in range(0, self.config.SIM_DURATION, self.config.TIME_STEP):
            # 1. ドローンの移動と近隣情報の更新
            for drone in self.drones:
                drone.move(self.config.TIME_STEP)
            for drone in self.drones:
                drone.update_neighbors(self.drones)

            # 2. 信頼度の計算
            for drone in self.drones:
                self._calculate_final_trust(drone)

            # 3. クラスタ形成とリーダー選出
            self.form_clusters()
            self.select_leaders()
            
            self.visualizer.update(self.drones)


            # 4. 評価指標の計測
            pdr, avg_delay = self._simulate_communication()
            total_energy = sum([d.energy for d in self.drones])
            
            leader_map = {}
            for cid, members in self.clusters.items():
                # is_leaderがTrueのメンバーを探す
                leader = next((m for m in members if m.is_leader), None)
                if leader:
                    leader_map[cid] = leader.id # {クラスタID: リーダードローンID}
            
            self.history['time'].append(t)
            self.history['energy'].append(total_energy)
            self.history['pdr'].append(pdr)
            self.history['delay'].append(avg_delay)

            print(f"Time: {t}s, Clusters: {len(self.clusters)}, PDR: {pdr:.3f}, Delay: {avg_delay*1000:.2f}ms, Leaders: {leader_map}")

    def plot_results(self):
        self.visualizer.close()
        fig, axs = plt.subplots(3, 1, figsize=(10, 15))

        # エネルギー消費
        initial_total_energy = self.config.NUM_DRONES * self.config.INITIAL_ENERGY
        consumed_energy = (initial_total_energy - np.array(self.history['energy'])) / 1e3 # kJ
        axs[0].plot(self.history['time'], consumed_energy)
        axs[0].set_title('Energy Consumption over Time')
        axs[0].set_xlabel('Time (s)')
        axs[0].set_ylabel('Energy Consumed (kJ)')

        # パケット配送率
        axs[1].plot(self.history['time'], self.history['pdr'])
        axs[1].set_title('Packet Delivery Ratio (PDR) over Time')
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylabel('PDR')
        axs[1].set_ylim(0, 1.1)

        # 遅延
        delay_ms = np.array(self.history['delay']) * 1000
        axs[2].plot(self.history['time'], delay_ms)
        axs[2].set_title('Average Delay over Time')
        axs[2].set_xlabel('Time (s)')
        axs[2].set_ylabel('Delay (ms)')

        plt.tight_layout()
        plt.show()