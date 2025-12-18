import asyncio
import numpy as np
import random
from typing import Any, Dict


from global_var import SimConfig
from packet import Packet, TelemetryPayload, ClusterReportPayload

# --- UAV (ドローン) クラス ---
class UAV:
    def __init__(self, drone_id, config):
        self.id = drone_id
        self.config = config
        
        # 初期状態を保存
        self.initial_pos = np.array(config.AREA_SIZE) / 2.0
        self.initial_velocity = self._get_random_velocity(config.VELOCITY_RANGE)
        self.initial_energy = config.INITIAL_ENERGY
        
        # 現在の状態を初期化
        self.reset()

    def reset(self):
        """UAVを初期状態にリセットする"""
        self.pos = self.initial_pos.copy()
        self.velocity = self.initial_velocity.copy()
        self.destination = np.random.rand(3) * self.config.AREA_SIZE
        self.trust_score = self.config.INITIAL_TRUST
        self.trust_var = self.config.INIT_SIGMA
        
        # IDを3で割った余りに基づいてタイプを決定的に割り当てる
        remainder = self.id % 3
        if remainder == 0:
            self.type = 'good'
        elif remainder == 1:
            self.type = 'neutral'
        else: # remainder == 2
            self.type = 'bad'
        self.initial_type = self.type 
        self.current_behavior_type = self.type # 現在の振る舞いを管理
        
        if self.type == 'good':
            self.transmittion_rate = random.uniform(8, 11)  # Good nodes: 20-25 Mbps
            self.energy = self.initial_energy
        elif self.type == 'neutral':
            self.transmittion_rate = random.uniform(6, 8)   # Neutral nodes: 16-20 Mbps
            # self.energy = self.initial_energy
            self.energy = self.initial_energy * random.uniform(0.75, 0.9)  # Neutral nodes may start with less energy
        else:  # 'bad'
            self.transmittion_rate = random.uniform(8, 11)   # Bad nodes: 54-6 Mbps
            self.energy = self.initial_energy
            # self.energy = self.initial_energy * random.uniform(0.45, 0.5)  # Bad nodes may start with less energy
        
        self.neighbors = []
        self.direct_trust_to_neighbors = {}
        self.indirect_trust_to_others = {}
        self.hybrid_trust_to_others = {}
        self.cluster_id = None
        self.is_leader = False
        self.is_sub_leader = False
        self.has_been_leader = False # リーダー経験フラグ
        
        
        self.inbox = asyncio.Queue()
        
        self.history_out: Dict[int, Dict[str, Any]] = {} #送信履歴 {相手ID: {'sent': int, 'success': int, 'delays': list}}
        self.history_in: Dict[int, Dict[str, Any]] = {} #受信履歴 {相手ID: {'received': int, 'reception_times': list}}
        self.packet_payload_history: Dict[int, TelemetryPayload] = {} # 受信したペイロード履歴 {送信者ID: Payload}
        self.report_packets_received = 0 # リーダーとして受信したレポート数
        self.report_packets_sent = 0 # メンバーとして送信したレポート数
        self.reports_addressed_to_me = 0 # リーダーとして自身に送られるはずだったレポート総数

    async def send_packet(self, destination_uav: 'UAV', payload: TelemetryPayload, sim_time: float) -> tuple[bool, float]:
        #まずパケットを送るかどうかをタイプ別にランダムに決める
        current_type = getattr(self, 'behavior_type', self.type)
        # Badノードは 50% の確率で送信をサボる（不調、または意図的な沈黙）
        if current_type == 'bad':
            if random.random() < 0.5:
                # 送信失敗（サボり）
                # 遅延0でFalseを返す（相手には届かない）
                return False, 0.0
        if current_type == 'neutral':
            if random.random() < 0.1:
                return False, 0.0
        packet = Packet(self.id, destination_uav.id, payload, sim_time)
        dist = np.linalg.norm(self.pos - destination_uav.pos) 
        self.consume_energy_tx(SimConfig.PACKET_SIZE, dist)
        transmission_delay = SimConfig.PACKET_SIZE / (self.transmittion_rate * 1e6) * 1e3  # milliseconds
        await asyncio.sleep(transmission_delay)
        
        # 成功確率は相手(受信側)のタイプに基づく
        success = destination_uav.receive_packet(packet)
        if success:
            await destination_uav.inbox.put(packet)
            # print(f"✅ Packet sent: {self.id} -> {destination_uav.id}")
            return True, transmission_delay # 成功フラグと遅延を返す
        else:
            # print(f"❌ Packet loss: {self.id} -> {destination_uav.id}")
            return False, transmission_delay # 失敗フラグと遅延を返す


    async def packet_handler(self):
        """受信ボックスを監視し、受信したパケットを処理する"""
        while True:
            try:
                # タイムアウトを設けて、シミュレーション終了時にタスクを停止できるようにする
                packet: Packet = await self.inbox.get()
                
                source_id = packet.source_id
                # 受信履歴を初期化
                if source_id not in self.history_in:
                    self.history_in[source_id] = {'received': 0, 'reception_times': []}
                self.history_in[source_id]['received'] += 1
                self.history_in[source_id]['reception_times'].append(packet.timestamp)
                
                if isinstance(packet.payload, TelemetryPayload):
                    self.packet_payload_history[source_id] = packet.payload
                    # print(f"📦 Telemetry received by {self.id} from {packet.source_id}")
                elif isinstance(packet.payload, ClusterReportPayload):
                    # リーダーがメンバーからのレポートを受信した際の処理
                    self.report_packets_received += 1
                    # print(f"📈 Report received by Leader {self.id} from member {packet.source_id}(Total reports: {self.report_packets_received})")
                    
                #TODO:ここで受信したデータに応じた処理を行う (例: 信頼度更新のトリガーなど)
                self.inbox.task_done()
                
            except asyncio.CancelledError:
                # タスクがキャンセルされたらループを抜ける
                break
            except Exception as e:
                # その他の予期せぬエラー
                print(f"Error in packet_handler for UAV {self.id}: {e}")
                break
    
    def update_behavior(self, current_time):
        """On-Off攻撃のシミュレーション: Badノードが周期的に善人として振る舞う"""
        if self.initial_type == 'bad':
            # 例: 50秒周期で 善/悪 を切り替える
            period = 50
            if (current_time % period) < (period / 2):
                self.current_behavior_type = 'good' # 信頼稼ぎモード
            else:
                self.current_behavior_type = 'bad'  # 攻撃モード
        else:
            self.current_behavior_type = self.initial_type
            
    def receive_packet(self, packet: Packet) -> bool:
        """
        UAVのタイプに基づき、パケット受信(中継)の成否を返す
        """
        #TODO:エネルギー消費も考慮する
        if self.current_behavior_type == 'good':
            return random.random() < 0.95  # 正常ノードは95%成功
        elif self.current_behavior_type == 'neutral':
            return random.random() < 0.7 # 70%の確率で成功(True)
        elif self.current_behavior_type == 'bad':
            return random.random() < 0.2 # 20%の確率で破棄(False)
        
        # 受信成功確率を判定
        if self.type == 'good':
            success = random.random() < 0.95  # 正常ノードは95%成功
        elif self.type == 'neutral':
            success = random.random() < 0.7 # 70%の確率で成功(True)
        else:  # 'bad'
            success = random.random() < 0.2 # 20%の確率で破棄(False)

        # 成功した受信に対してエネルギーを消費
        if success:
            try:
                self.consume_energy_rx(SimConfig.PACKET_SIZE)
            except Exception:
                pass
        return success   
    
    def _get_random_velocity(self, v_range):
        speed = random.uniform(v_range[0], v_range[1])
        direction = np.random.rand(3) - 0.5
        return direction / np.linalg.norm(direction) * speed

    #TODO: 他のウェイポイントを実装
    
    async def move(self, time_step):
        # ランダムウェイポイントモデル
        if np.linalg.norm(self.pos - self.destination) < 20:
            self.destination = np.random.rand(3) * SimConfig.AREA_SIZE
            self.velocity = self._get_random_velocity(SimConfig.VELOCITY_RANGE)

        # 境界チェック
        next_pos = self.pos + self.velocity * time_step
        for i in range(3):
            if not (0 < next_pos[i] < SimConfig.AREA_SIZE[i]):
                self.velocity[i] *= -1 # 壁で反射
        
        self.pos += self.velocity * time_step
        # 移動によるエネルギー消費（仮）
        self.energy -= 1 * np.linalg.norm(self.velocity) 
        
        await asyncio.sleep(time_step)

    def update_neighbors(self, all_drones):
        self.neighbors = []
        for other in all_drones:
            if self.id != other.id:
                dist = np.linalg.norm(self.pos - other.pos)
                if dist < SimConfig.COMM_RANGE:
                    self.neighbors.append(other.id)

    def consume_energy_tx(self, packet_size_bits, distance):
        """
            送信エネルギーを消費する。
            energy_consumed = l * E_elec + l * E_amp * d^2
            packet_size_bits: l (bits)
            distance: d (meters)
        """

        E_elec = getattr(self.config, 'E_ELEC', getattr(SimConfig, 'E_ELEC'))
        E_amp = getattr(self.config, 'E_AMP', getattr(SimConfig, 'E_AMP'))

        # 距離が None や負の場合は 0 とみなす
        d = distance if distance is not None else 0.0
        energy_consumed = packet_size_bits * (E_elec + E_amp * (d ** 2))
        self.energy -= energy_consumed
     
    def consume_energy_rx(self, packet_size_bits):
        """
        受信エネルギーを消費する（パケット成功受信時に呼び出す）。
        デフォルトでは global_var の ENERGY_RX を使用し、未定義時は E_ELEC を代用。
        """
        energy_consumed = packet_size_bits * getattr(self.config, 'ENERGY_RX', getattr(SimConfig, 'E_ELEC'))
        self.energy -= energy_consumed
        
    def _sample_delay(self, t:str)-> float:
        if t == 'good':
            return random.uniform(0.01, 0.05) # seconds
        elif t == 'neutral':
            return random.uniform(0.05, 0.1)
        else:  # 'bad'
            return random.uniform(0.5, 1.0)
