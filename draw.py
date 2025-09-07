import numpy as np
import matplotlib.pyplot as plt
# --- 可視化クラス ---   
class LiveVisualizer:
    def __init__(self, area_size, drones):
        plt.ion()  # インタラクティブモードをオン
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_box_aspect([area_size[0], area_size[1], area_size[2]]) # アスペクト比をシミュレーション空間に合わせる

        self.ax.set_title('FANET Simulation')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_xlim(0, area_size[0])
        self.ax.set_ylim(0, area_size[1])
        self.ax.set_zlim(0, area_size[2])

        # 初期描画
        positions, colors = self._get_plot_data(drones)
        self.scatter = self.ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], c=colors)
        plt.show()

    def _get_plot_data(self, drones):
        positions = np.array([d.pos for d in drones])
        colors = []
        for d in drones:
            if d.is_leader:
                colors.append('red')
            elif d.is_sub_leader:
                colors.append('yellow')
            elif d.type == 'bad':
                colors.append('black')
            else:
                colors.append('blue')
        return positions, colors

    def update(self, drones):
        positions, colors = self._get_plot_data(drones)
        
        # 3D散布図のデータを更新
        self.scatter._offsets3d = (positions[:, 0], positions[:, 1], positions[:, 2])
        self.scatter.set_facecolors(colors)

        # 描画を更新
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.01) # 短いポーズを入れてGUIを更新

    def close(self):
        plt.ioff() # インタラクティブモードをオフ
        plt.close(self.fig)

