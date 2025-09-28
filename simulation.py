import numpy as np
import matplotlib.pyplot as plt
import random

from draw import LiveVisualizer
from uav import UAV

from utils import *

# --- TDLS-FANET シミュレーションクラス ---
class TdlsFanetSimulation:
    def __init__(self, config):
        self.config = config
        self.drones = [UAV(i, config) for i in range(config.NUM_DRONES)]
        self.clusters = {} # clusterid: [UAV, UAV, ...]
        # 評価指標記録用
        self.history = {'time': [], 'energy': [], 'delay': [], 'pdr': [], 'trust': {i:[] for i in range(config.NUM_DRONES)}}
        # 描画クラス
        self.visualizer = LiveVisualizer(config.AREA_SIZE, self.drones)


    # Algorithm 2: 直接信頼度の計算 (簡略版)
    def update_direct_trust(self, uav_x): #uav_x から見た隣接ノード uav_j の直接信頼値を求める．
        """
        引数：評価者 uav_x,
        返り値：辞書 uav_x の隣接ノード郡 uav_id: trust_value,
        """
        metrics = normalize_components(uav_x)  # {neighbor_id: {'ss','pdr','energy','delay'}}
        if not metrics:
            uav_x.direct_trust_to_neightbors = {}
            return uav_x.direct_trust_to_neightbors
        trust_map = {}
        for uav_j in uav_x.neighbors:
            m = metrics.get(uav_j.id)
            if m is None:
                continue
            total_weight = 0.25 * m['ss'] + 0.25 * m['pdr'] + 0.25 * m['energy'] + 0.25 * m['delay']
            if total_weight >= 0.7:
                direct_trust = 0.5 * (2 ** total_weight)
            elif total_weight >= 0.3:
                direct_trust = 0.5 * (1.5 ** total_weight)
            else:
                direct_trust = 0.5 * (0.5 ** total_weight)
            trust_map[uav_j.id] = direct_trust
        uav_x.direct_trust_to_neightbors = trust_map
        return uav_x.direct_trust_to_neightbors

    # Algorithm 3: 間接信頼度の計算
    def update_indirect_trust(self, uav_x): # uav_x から見たノード uav_j の間接信頼値を求める．
        """
        引数:評価者 uav_x, 評価対象 uav_j
        返り値:uav_x から見た uav_j の間接信頼値
        """
        if not uav_x.neighbors:
            return 0.0
        
        for uav_j in self.clusters.get(uav_x.cluster_id, []): #クラスタ内の全ノードに対して．被評価ノード:uav_j
            if uav_j.id == uav_x.id:
                continue
            recommendations = []
            for neighbor_k in uav_x.neighbors: # 隣接ノード k が評価対象 uav_j の信頼値をノードiにリコメンドする 
                if(uav_j.id not in neighbor_k.direct_trust_to_neightbors):
                    neighbor_k.direct_trust_to_neightbors[uav_j.id] = 0.5
                rec = neighbor_k.direct_trust_to_neightbors[uav_j.id]
                recommendations.append(rec)
            
            avg_rec = np.mean(recommendations)
            uav_x.indirect_trust_to_others[uav_j.id] = avg_rec
        return uav_x.indirect_trust_to_others

    # Fitnessスコア計算 (論文 Eq. 1)
    def _calculate_fitness_score(self, uav):
        if not uav.neighbors:
            return 0.0
        d_sum = [np.linalg.norm(uav.pos - n.pos) for n in self.drones if uav.id != n.id]
        d_avg = np.mean(d_sum) ### 全ドローンとの平均距離
        d_max = np.max(d_sum) if d_sum else 1.0
        r_e = uav.energy / self.config.INITIAL_ENERGY # 残存エネルギー率 (正規化)

        # # D_avgも正規化が必要
        norm_d_avg = 1 - min(d_avg / d_max, 1.0) # 距離が近いほど高評価

        fitness_score = self.config.A_COEFFICIENT * norm_d_avg + 0.45 * r_e
        return fitness_score
    
    # Algorithm 1: 最終信頼度の計算
    def update_final_trust(self, uav_j): 
        members = self.clusters.get(uav_j.cluster_id, [])
        fitness_score = self._calculate_fitness_score(uav_j)
        if not members:
            return 0.5 + 0.5 * fitness_score

        # 1) 各評価者 uav_x が被評価者 uav_j へのハイブリッド信頼を更新
        vals = []
        for uav_x in members:
            if uav_x.id == uav_j.id:
                continue
            uav_x.hybrid_trust_to_others[uav_j.id] = 0.5 * uav_x.direct_trust_to_neightbors.get(uav_j.id, 0.0) +  0.5 * uav_x.indirect_trust_to_others.get(uav_j.id, 0.5)
            v = uav_x.hybrid_trust_to_others.get(uav_j.id)
            if v is not None:
                vals.append(v)
        # 2) 各評価者がもつuav_jへの信頼値平均を計算
        avg_hybrid_by_target = float(np.mean(vals))
        # 3) Fitness と重み付けして最終信頼に反映
        final_trust = (
            self.config.B_COEFFICIENT_FITNESS * fitness_score
            + self.config.B_COEFFICIENT_HYBRID * avg_hybrid_by_target
        )
        uav_j.trust_score = final_trust
        return final_trust
         
        
    # Algorithm 4: クラスタ形成
    def form_clusters(self):
        # 信頼度が低いドローンを除外
        eligible_drones = [d for d in self.drones if d.trust_score > 0.29 and d.energy > 0]
        
        processed_drones = set()
        self.clusters = {}
        cluster_id_counter = 0

        sorted_drones = sorted(eligible_drones, key=lambda d: (d.pos[0],d.pos[1],d.pos[2]))

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
                    if dist < self.config.COMM_RANGE and len(new_cluster) < 10:
                        new_cluster.append(other_drone)
                        other_drone.cluster_id = cluster_id_counter
                        processed_drones.add(other_drone.id)
            
            self.clusters[cluster_id_counter] = new_cluster

    # Algorithm 5: リーダー選出
    def select_leaders(self):
        # 全ドローンのリーダーフラグをリセット
        for drone in self.drones:
            drone.is_leader = False
            drone.is_sub_leader = False
        # 各クラスタでリーダーとサブリーダーを選出
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
                    k = len(member.neighbors)
                    mu = member.trust_score
                    sampled_score = float(np.random.normal(mu, np.sqrt(1.0 / k)))
                    score = sampled_score / sum_of_distances
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
            print(f"Time {t}s: Drones moved.")
            for drone in self.drones:
                drone.update_neighbors(self.drones)
            print(f"Time {t}s: Drones updated neighbors.")
            for drone in self.drones:
                self.update_direct_trust(drone)
            print(f"Time {t}s: Drones calculated direct trust.")
            for drone in self.drones:
                self.update_indirect_trust(drone)
            print(f"Time {t}s: Drones calculated indirect trust.")
            for drone in self.drones:
                self.history['trust'][drone.id].append(drone.trust_score)# 信頼値の記録を載せる．
                self.update_final_trust(drone)
            print(f"Time {t}s: Trust scores updated.")   
            #TODO:余裕があれば 各ループ度にノードが位置情報やバッテリー残量を隣接ノードに伝える処理を追加する．
            
            # 3. クラスタ形成とリーダー選出
            if t % 20 == 0: #20秒ごとにクラスタ再形成
                self.form_clusters()
                self.select_leaders()
            
            self.visualizer.update(self.drones, sim_time=t)

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

            print(f"Time: {t}s, Leaders: {leader_map}")

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
        
    # trustの推移をプロット    
    def plot_trust(self, uav_ids=None):
        if uav_ids is None:
            uav_ids = list(range(min(5, self.config.NUM_DRONES)))  # デフォルトで先頭5機
        plt.figure(figsize=(10, 6))
        for i in uav_ids:
            plt.plot(self.history['time'], self.history['trust'][i], label=f'UAV {i}')
        plt.title('UAV Trust over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Trust')
        plt.ylim(0, 1.1)
        plt.legend()
        plt.tight_layout()
        plt.show()