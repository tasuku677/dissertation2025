import asyncio
import numpy as np
import random
from typing import Any, Dict


from global_var import SimConfig
from packet import Packet

# --- UAV (ドローン) クラス ---
class UAV:
    def __init__(self, drone_id, config):
        self.id = drone_id
        self.config = config
        
        # 初期状態を保存
        self.initial_pos = np.random.rand(3) * config.AREA_SIZE
        self.initial_velocity = self._get_random_velocity(config.VELOCITY_RANGE)
        self.initial_energy = config.INITIAL_ENERGY
        
        # 現在の状態を初期化
        self.reset()

    def reset(self):
        """UAVを初期状態にリセットする"""
        self.pos = self.initial_pos.copy()
        self.velocity = self.initial_velocity.copy()
        self.destination = np.random.rand(3) * self.config.AREA_SIZE
        self.energy = self.initial_energy
        self.trust_score = self.config.INITIAL_TRUST
        
        self.neighbors:list[UAV] = []
        self.direct_trust_to_neightbors = {}
        self.indirect_trust_to_others = {}
        self.hybrid_trust_to_others = {}
        self.cluster_id = None
        self.is_leader = False
        self.is_sub_leader = False
        self.type = 'good' # or random.choice(['good', 'neutral', 'bad'])
        self.pdr = self.sample_pdr(self.type)
        self.delay = self.sample_delay(self.type)
        
        self.inbox = asyncio.Queue()
        self.sent_packets: Dict[int, Packet] = {}


    #TODO: パケット送信ごとにエネルギー消費を考慮
    async def send_packet(self, destination_uav: 'UAV', data: Any, sim_time: float) -> tuple[bool, float]:
        packet = Packet(self.id, destination_uav.id, data, sim_time)
        self.consume_energy_tx(SimConfig.PACKET_SIZE)
        self.sent_packets[destination_uav.id] = packet
        dist = np.linalg.norm(self.pos - destination_uav.pos)
        transmission_delay = dist / SimConfig.C + self.sample_delay(self.type) #TODO: データサイズの影響を考慮
        await asyncio.sleep(transmission_delay)
        success_prob = destination_uav.trust_score * (1 - dist / (SimConfig.COMM_RANGE * 1.5)) #TODO:これも後で考える
        if random.random() < success_prob:
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
                packet: Packet = await asyncio.wait_for(self.inbox.get(), timeout=1.0)
                
                print(f"📦 Packet received by {self.id} from {packet.source_id}, data: '{packet.data}'")
                # TODO:ここで受信したデータに応じた処理を行う (例: 信頼度更新のトリガーなど)
                self.inbox.task_done()
                
            except asyncio.TimeoutError:
                # タイムアウトした場合はループを継続
                pass
            except asyncio.CancelledError:
                # タスクがキャンセルされたらループを抜ける
                break


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
        self.energy -= 0.1 * np.linalg.norm(self.velocity) 
        
        await asyncio.sleep(time_step)

    def update_neighbors(self, all_drones):
        self.neighbors = []
        for other in all_drones:
            if self.id != other.id:
                dist = np.linalg.norm(self.pos - other.pos)
                if dist < SimConfig.COMM_RANGE:
                    self.neighbors.append(other)

    def consume_energy_tx(self, packet_size):
        self.energy -= SimConfig.ENERGY_TX * packet_size
     
    def sample_pdr(self, t: str) -> float:
        if t == 'good':
            return random.uniform(0.95, 1.0)
        elif t == 'neutral':
            return random.uniform(0.7, 0.94)
        else:  # 'bad'
            return random.uniform(0.1, 0.5)
        
    def sample_delay(self, t:str)-> float:
        if t == 'good':
            return random.uniform(0.01, 0.05) # seconds
        elif t == 'neutral':
            return random.uniform(0.05, 0.1)
        else:  # 'bad'
            return random.uniform(0.1, 0.5)