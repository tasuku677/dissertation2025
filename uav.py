import numpy as np
import matplotlib.pyplot as plt
import random


from global_var import SimConfig

# --- UAV (ドローン) クラス ---
class UAV:
    def __init__(self, drone_id, config):
        self.id = drone_id
        # self.pos = np.random.rand(3) * config.AREA_SIZE
        self.pos = np.array([400.0, 300.0, 200.0])
        self.velocity = self._get_random_velocity(config.VELOCITY_RANGE)
        self.destination = np.random.rand(3) * config.AREA_SIZE
        self.energy = config.INITIAL_ENERGY
        self.trust_score = config.INITIAL_TRUST
        
        self.neighbors = []
        self.direct_trust_to_neightbors = {}  # 各隣接ドローンに対する信頼度 uav_id: trust_value
        self.indirect_trust_to_others = {} # 同一クラスタ内の他のドローンに対する間接信頼度 uav_id: trust_value
        self.hybrid_trust_to_others = {} # 同一クラスタ内の他のドローンに対する最終信頼度 uav_id: trust_value
        self.cluster_id = self.id // 10
        self.is_leader = False
        self.is_sub_leader = False
        self.type = random.choice(['good', 'neutral', 'bad']) # ドローンの種類
        self.pdr = self.sample_pdr(self.type)
        self.delay = self.sample_delay(self.type)  # 通信遅延のサンプル


    def _get_random_velocity(self, v_range):
        speed = random.uniform(v_range[0], v_range[1])
        direction = np.random.rand(3) - 0.5
        return direction / np.linalg.norm(direction) * speed

    def move(self, time_step):
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