import random
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# --- シミュレーション設定 ---
NUM_UAVS = 15          # UAVの数
SIM_AREA = (500, 500, 300) # シミュレーションエリア (x, y, z) (m)
COMM_RANGE = 150       # 通信範囲 (m)
MAX_SPEED = 10         # 最大速度 (m/s)
SIM_TIME = 100         # シミュレーション時間 (s)
DELTA_T = 1            # 時間ステップ (s)

# --- UAVクラスの定義 (3D対応) ---
class UAV:
    def __init__(self, id, pos, speed):
        self.id = id
        self.pos = np.array(pos, dtype=float)
        self.speed = speed
        self.destination = self.get_new_destination()

    def get_new_destination(self):
        # 3次元の目的地をランダムに設定
        return np.array([random.uniform(0, SIM_AREA[0]),
                         random.uniform(0, SIM_AREA[1]),
                         random.uniform(0, SIM_AREA[2])])

    def move(self):
        # 3次元ベクトル計算 
        direction = self.destination - self.pos
        distance = np.linalg.norm(direction)

        if distance < 1.0:
            self.destination = self.get_new_destination()
            direction = self.destination - self.pos
            distance = np.linalg.norm(direction)

        # 移動距離を計算
        move_dist = min(self.speed * DELTA_T, distance)
        self.pos += (direction / distance) * move_dist

# --- シミュレーションの準備 ---
uavs = [UAV(i,
            pos=[random.uniform(0, SIM_AREA[0]),
                 random.uniform(0, SIM_AREA[1]),
                 random.uniform(0, SIM_AREA[2])], # Z座標を追加
            speed=random.uniform(MAX_SPEED / 2, MAX_SPEED))
        for i in range(NUM_UAVS)]

# --- 3Dプロットのセットアップ ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')


# --- アニメーション更新関数 (3D描画) ---
def update(frame):
    ax.clear()
    ax.set_xlim(0, SIM_AREA[0])
    ax.set_ylim(0, SIM_AREA[1])
    ax.set_zlim(0, SIM_AREA[2])
    ax.set_title(f"3D UAV Network Simulation (Time: {frame*DELTA_T}s)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    # UAVを移動
    for uav in uavs:
        uav.move()

    # UAVの位置をプロット (ノード)
    positions = np.array([uav.pos for uav in uavs])
    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], c='skyblue', s=50, label='UAV')

    # UAVのIDラベルを表示
    for i, pos in enumerate(positions):
        ax.text(pos[0], pos[1], pos[2], f' {i}', fontsize=9)


    # 通信リンクをプロット (エッジ)
    for i in range(NUM_UAVS):
        for j in range(i + 1, NUM_UAVS):
            # 3次元での距離計算 (NumPyなのでコードの変更は不要)
            dist = np.linalg.norm(uavs[i].pos - uavs[j].pos)
            if dist <= COMM_RANGE:
                pos_i = uavs[i].pos
                pos_j = uavs[j].pos
                # 2点間に線を引く
                ax.plot([pos_i[0], pos_j[0]],
                        [pos_i[1], pos_j[1]],
                        [pos_i[2], pos_j[2]], color='gray', linestyle='--', linewidth=0.8)

# --- アニメーションの実行 ---
# 初回描画のために一度updateを呼び出す
update(0)
ax.legend()
ani = FuncAnimation(fig, update, frames=int(SIM_TIME/DELTA_T), interval=100, repeat=False)
plt.show()