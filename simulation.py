import asyncio
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple
import csv
import copy

from draw import LiveVisualizer
from uav import UAV 
from packet import TelemetryPayload
from utils import *


# --- TDLS-FANET シミュレーションクラス ---
class TdlsFanetSimulation:
    def __init__(self, config):
        self.config = config
        self.drones = [UAV(i, config) for i in range(config.NUM_DRONES)]
        self.clusters = {} # clusterid: [UAV, UAV, ...]
        # 評価指標記録用
        self.history = {'time': [], 'energy': [], 'delay': [], 'pdr': [], 'trust': {i:[] for i in range(config.NUM_DRONES)}, 'cluster_id': {i:[] for i in range(config.NUM_DRONES)}}        # 描画クラス
        self.visualizer = LiveVisualizer(config.AREA_SIZE, self.drones)

    def reset(self):
        """シミュレーションの状態をリセットする"""
        print("シミュレーションの状態をリセットしています...")
        for drone in self.drones:
            drone.reset()
        self.clusters = {}
        self.history = {'time': [], 'energy': [], 'delay': [], 'pdr': [], 'trust': {i:[] for i in range(self.config.NUM_DRONES)}}
        # LiveVisualizerはリセットせずに再利用する

    # Algorithm 2: 直接信頼度の計算 #TODO: 各ノードが保持する隣接ノード情報を活用するように変更
    def update_direct_trust(self, uav_x): #uav_x から見た隣接ノード郡 uav_j の直接信頼値を求める．
        """
        引数：評価者 uav_x,
        返り値：辞書 uav_x の隣接ノード郡 uav_id: trust_value,
        """
        metrics = normalize_components(uav_x)  # {neighbor_id: {'ss','pdr','energy','delay'}} ->  TODO:{neighbor_id: {'time', 'ss','pdr','energy','delay'}}更新時の時間を追跡．この時間が古いと分散も大きくする
        if not metrics:
            return uav_x.direct_trust_to_neighbors
        for uav_j_id in uav_x.neighbors:
            if uav_j_id not in uav_x.direct_trust_to_neighbors:
                uav_x.direct_trust_to_neighbors[uav_j_id] = (0.5, SimConfig.init_sigma) # 初期値
                continue
            m = metrics.get(uav_j_id, None)
            if m is None:
                continue
            total_weight = 0.25 * m['ss'] + 0.25 * m['pdr'] + 0.25 * m['energy'] + 0.25 * m['delay']
            if total_weight >= 0.7:
                direct_trust_mu = 0.5 * (2 ** total_weight)
            elif total_weight >= 0.3:
                direct_trust_mu = 0.5 * (1.5 ** total_weight)
            else:
                direct_trust_mu = 0.5 * (0.5 ** total_weight)
            
            n_obs = uav_x.history_out.get(uav_j_id, {}).get('sent', 0)
            if n_obs == 0:
                direct_trust_sigma_sq = SimConfig.init_sigma
            else:
                p = direct_trust_mu
                beta_var = (p * (1 - p)) / (n_obs + 2)
                direct_trust_sigma_sq = max(beta_var, 1e-6)
                
            uav_x.direct_trust_to_neighbors[uav_j_id] = (direct_trust_mu, direct_trust_sigma_sq) # 信頼値と分散をセット
        return uav_x.direct_trust_to_neighbors

    # Algorithm 3: 間接信頼度の計算
    def update_indirect_trust(self, uav_x): # uav_x から見たノード uav_j の間接信頼値を求める．
        """
        引数:評価者 uav_x, 評価対象 uav_j
        返り値:uav_x から見た uav_j の間接信頼値
        """
        if not uav_x.neighbors:
            return {}
    
        for uav_j_id in self.clusters.get(uav_x.cluster_id, []): #クラスタ内の全ノードに対して．被評価ノード:uav_j
            if uav_j_id == uav_x.id:
                continue
            recommendations = [] 
            for neighbor_k_id in uav_x.neighbors: # 隣接ノード k が評価対象 uav_j の信頼値をノードiにリコメンドする 
                payload = uav_x.packet_payload_history.get(neighbor_k_id)
                if payload and uav_j_id in payload.direct_trust_to_neighbors:
                    rec = payload.direct_trust_to_neighbors[uav_j_id]
                else:
                    rec = (self.config.INITIAL_TRUST, self.config.init_sigma)
                recommendations.append(rec)
            
            if not recommendations:
                avg_rec_mu, avg_rec_star = self.config.INITIAL_TRUST, self.config.init_sigma
            else:
                avg_rec_mu, avg_rec_star = combine_gaussians([r[0] for r in recommendations], [r[1] for r in recommendations])
            uav_x.indirect_trust_to_others[uav_j_id] = (avg_rec_mu, avg_rec_star)
        return uav_x.indirect_trust_to_others
    
    

    # Fitnessスコア計算 (論文 Eq. 1)
    #TODO : window size p を考慮する　時間pごとに再計算する．
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
    def update_final_trust(self, uav_j) -> float: 
        members = self.clusters.get(uav_j.cluster_id, [])
        fitness_score = self._calculate_fitness_score(uav_j)
        if not members:
            uav_j.trust_score = 0.5 * self.config.INITIAL_TRUST + 0.5 * fitness_score
            return uav_j.trust_score

        # 1) 各評価者 uav_x が被評価者 uav_j へのハイブリッド信頼を更新
        vals = []
        for uav_x in members:
            if uav_x.id == uav_j.id:
                continue
            
            direct_trust = uav_x.direct_trust_to_neighbors.get(uav_j.id, (self.config.INITIAL_TRUST, self.config.init_sigma))
            indirect_trust = uav_x.indirect_trust_to_others.get(uav_j.id, (self.config.INITIAL_TRUST, self.config.init_sigma))
            
            # ハイブリッド信頼を計算
            hybrid_mu, hybrid_var = combine_gaussians(
                [direct_trust[0], indirect_trust[0]],
                [direct_trust[1], indirect_trust[1]]
            )
            uav_x.hybrid_trust_to_others[uav_j.id] = (hybrid_mu, hybrid_var)
            vals.append((hybrid_mu, hybrid_var))

        # 2) 各評価者がもつuav_jへの信頼値平均を計算
        if not vals:
            avg_hybrid_by_target = self.config.INITIAL_TRUST
        else:
            avg_hybrid_by_target = combine_gaussians([v[0] for v in vals], [v[1] for v in vals])[0]
            
        # 3) Fitness と重み付けして最終信頼に反映
        final_trust = (
            self.config.B_COEFFICIENT_FITNESS * fitness_score
            + self.config.B_COEFFICIENT_HYBRID * avg_hybrid_by_target
        )
        uav_j.trust_score = final_trust
        return uav_j.trust_score
         
        
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
                    if dist < self.config.COMM_RANGE : #TODO: すでにクラスタに属しているドローンが多ければ，追加条件を厳しくすることもアリかも．
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
                    score = member.trust_score / sum_of_distances
                leader_scores.append((score, member))

            # スコアでソートし、リーダーとサブリーダーを決定
            leader_scores.sort(key=lambda x: x[0], reverse=True)
            
            leader = leader_scores[0][1]
            leader.is_leader = True
            
            if len(leader_scores) > 1:
                sub_leader = leader_scores[1][1]
                sub_leader.is_sub_leader = True

    async def _simulate_communication(self, t: int) -> Tuple[float, float]:
        """パケット送受信をシミュレートし、PDRと平均遅延を返す"""
        tasks_with_context: List[Tuple[asyncio.Task, UAV, UAV]] = []
        total_packets_attempted = 0

        # 各UAVが隣接ノードにパケットを送信するタスクを作成
        for sender in self.drones:
            # 送信するペイロードを生成
            # TODO: neighborsやdirect_trustsはディープコピーが望ましい
            payload = TelemetryPayload(
                energy=sender.energy,
                trust=sender.trust_score,
                pos=sender.pos,
                neighbors=sender.neighbors,
                direct_trust_to_neighbors=copy.deepcopy(sender.direct_trust_to_neighbors),
                cluster_id=sender.cluster_id,
                role="leader" if sender.is_leader else "sub" if sender.is_sub_leader else "member"
            )
            
            for neighbor_id in sender.neighbors:
                total_packets_attempted += 1
                neighbor = self.drones[neighbor_id]
                task = asyncio.create_task(sender.send_packet(neighbor, payload, sim_time=t))
                # 送信者(sender)と受信者(neighbor)をタスクと共に保存
                tasks_with_context.append((task, sender, neighbor_id))

        successful_packets = 0
        total_delay = 0

        # 全ての送信タスクの完了を待つ
        for task, sender, receiver_id in tasks_with_context:
            try:
                success, delay = await task
                # --- 送信履歴 (History OUT) の記録 (PDR計算用) ---
                if receiver_id not in sender.history_out:
                    sender.history_out[receiver_id] = {'sent': 0, 'success': 0, 'delays': []}
                
                history = sender.history_out[receiver_id]
                history['sent'] += 1
                
                if success:
                    history['success'] += 1
                    history['delays'].append(delay)
                    
                    # グローバルPDR/Delay集計
                    successful_packets += 1
                    total_delay += delay

            except Exception as e:
                print(f"Error in communication task from {sender.id} to {receiver_id}: {e}")
                # タスク失敗時も 'sent' としてカウント
                if receiver_id not in sender.history_out:
                    sender.history_out[receiver_id] = {'sent': 0, 'success': 0, 'delays': []}
                sender.history_out[receiver_id]['sent'] += 1

        pdr = successful_packets / total_packets_attempted if total_packets_attempted > 0 else 1.0
        avg_delay = total_delay / total_packets_attempted if total_packets_attempted > 0 else 0.0
        return pdr, avg_delay

        
    async def run_uav_task(self, drone):
        #TODO: パケット受け取る度に保持データを更新する
        """個々のUAVの1ステップ分のタスク"""
        await drone.move(self.config.TIME_STEP)
        drone.update_neighbors(self.drones)
        self.update_direct_trust(drone)
        self.update_indirect_trust(drone)
        self.update_final_trust(drone)
        self.history['trust'][drone.id].append(drone.trust_score)

        
    async def run(self):
        # 各UAVのパケットハンドラーをバックグラウンドタスクとして起動
        self.packet_handlers = [asyncio.create_task(drone.packet_handler()) for drone in self.drones]
        
        for t in range(0, self.config.SIM_DURATION, self.config.TIME_STEP):
            # 1. 全UAVのタスク（移動、近隣更新、信頼度計算）を並行実行
            uav_tasks = [self.run_uav_task(drone) for drone in self.drones]
            await asyncio.gather(*uav_tasks)
            print(f"Time {t}s: Drones moved and updated their states.")

            # 2. クラスタ形成とリーダー選出 (10秒ごと)
            if t % 10 == 0:
                self.form_clusters()
                self.select_leaders()
                print(f"Time {t}s: Clusters and leaders have been updated.")
            
            # 3. 通信をシミュレートし、結果を取得
            pdr, avg_delay = await self._simulate_communication(t)
            
            # 4. 評価指標を記録
            total_energy = sum(d.energy for d in self.drones)
            self.history['time'].append(t)
            self.history['energy'].append(total_energy)
            self.history['pdr'].append(pdr)
            self.history['delay'].append(avg_delay)
            for drone in self.drones:
                self.history['cluster_id'][drone.id].append(drone.cluster_id)
            
            # 5. 可視化とログ表示
            self.visualizer.update(self.drones, sim_time=t)
            leader_map = {cid: next((m.id for m in mems if m.is_leader), None) for cid, mems in self.clusters.items()}
            print(f"Time: {t}s, PDR: {pdr:.2f}, Avg Delay: {avg_delay*1000:.2f}ms, Leaders: {leader_map}")

        # シミュレーション終了時にパケットハンドラーを安全に停止
        for task in self.packet_handlers:
            task.cancel()
        await asyncio.gather(*self.packet_handlers, return_exceptions=True)
        
    def save_cluster_history_to_csv(self, filename="cluster_history.csv"):
        """クラスタ所属履歴をCSVファイルに保存する"""
        print(f"クラスタ所属履歴を {filename} に保存しています...")
        header = ['time'] + [f'UAV_{i}_cluster_id' for i in range(self.config.NUM_DRONES)]
        
        num_data_points = len(self.history['time'])
        rows = []
        for i in range(num_data_points):
            time_point = self.history['time'][i]
            row = [time_point]
            for drone_id in range(self.config.NUM_DRONES):
                if i < len(self.history['cluster_id'][drone_id]):
                    cluster_id = self.history['cluster_id'][drone_id][i]
                    row.append( f"{drone_id}_{cluster_id}" if cluster_id is not None else 'N/A')
                else:
                    row.append(None)
            rows.append(row)

        try:
            with open(filename, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(rows)
            print(f"{filename} の保存が完了しました。")
        except IOError as e:
            print(f"ファイルの書き込み中にエラーが発生しました: {e}")


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
        
    # trustの推移をプロット    
    def plot_trust(self, uav_ids=None):
        if uav_ids is None:
            uav_ids = list(range(min(5, self.config.NUM_DRONES)))  # デフォルトで先頭5機
        plt.figure(figsize=(10, 6))
        for i in uav_ids:
            plt.plot(self.history['time'], self.history['trust'][i], label=f'UAV {i}:{self.drones[i].type}')
        plt.title('UAV Trust over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Trust')
        plt.ylim(0, 1.1)
        plt.legend()
        plt.tight_layout()
        
    def save_trust_history_to_csv(self, filename="trust_history.csv"):
        """信頼度の履歴をCSVファイルに保存する"""
        print(f"信頼度履歴を {filename} に保存しています...")
        header = ['time'] + [f'UAV_{i}_trust' for i in range(self.config.NUM_DRONES)]
        
        # timeの長さを基準にデータ行を作成
        num_data_points = len(self.history['time'])
        rows = []
        for i in range(num_data_points):
            time_point = self.history['time'][i]
            row = [time_point]
            for drone_id in range(self.config.NUM_DRONES):
                # 各ドローンの信頼度リストに十分なデータがあるか確認
                if i < len(self.history['trust'][drone_id]):
                    row.append(self.history['trust'][drone_id][i])
                else:
                    row.append(None)  # データがない場合は空欄
            rows.append(row)

        try:
            with open(filename, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(rows)
            print(f"{filename} の保存が完了しました。")
        except IOError as e:
            print(f"ファイルの書き込み中にエラーが発生しました: {e}")
            
    def plot_aggregated_trust(self):
        """全ノードの信頼値の平均、最大、最小値をプロットする"""
        # グラフのサイズを横に広げてテキスト用のスペースを確保
        fig = plt.figure(figsize=(12, 6))
        
        # history['trust']から信頼値データを2D配列に変換
        # 行がドローンID、列が時間ステップに対応
        trust_data = []
        for i in range(self.config.NUM_DRONES):
            # データの長さが time と一致する場合のみ追加
            if len(self.history['trust'][i]) == len(self.history['time']):
                trust_data.append(self.history['trust'][i])
        
        if not trust_data:
            print("プロットする信頼度データがありません。")
            plt.close(fig) # プロットするものがない場合はフィギュアを閉じる
            return
            
        trust_array = np.array(trust_data)
        
        # 時間軸に沿って平均、最大、最小を計算 (axis=0)
        avg_trust = np.mean(trust_array, axis=0)
        max_trust = np.max(trust_array, axis=0)
        min_trust = np.min(trust_array, axis=0)
        
        # プロット
        plt.plot(self.history['time'], avg_trust, label='Average Trust', color='blue', linewidth=2)
        plt.plot(self.history['time'], max_trust, label='Maximum Trust', color='green', linestyle='--')
        plt.plot(self.history['time'], min_trust, label='Minimum Trust', color='red', linestyle='--')
        
        # グラフの体裁
        plt.title('Aggregated Trust of All Nodes over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Trust Value')
        plt.ylim(0, 1.1)
        plt.legend(loc='lower left')
        plt.grid(True, which='both', linestyle='--', linewidth=0.5)
        
        # global_varの値をテキストとして整形
        config = self.config
        param_text = "Simulation Parameters\n-------------------------\n"
        # SimConfigクラスのすべての属性をループで取得
        for key, value in config.__class__.__dict__.items():
            param_text += f"{key}: {value}\n"
        
        # テキストをグラフの右側に配置
        # figtextはFigure座標(0-1)で位置を指定
        plt.figtext(0.75, 0.85, param_text, ha="left", va="top",
                    bbox=dict(boxstyle="round,pad=0.5", fc="wheat", alpha=0.5))

        # レイアウトを調整してテキストが重ならないようにする
        plt.tight_layout(rect=[0, 0, 0.75, 1])