import numpy as np
import matplotlib.pyplot as plt
# --- 可視化クラス ---   
class LiveVisualizer:
    def __init__(self, area_size, drones):
        plt.ion()  # インタラクティブモードをオン
        self.text_artists = []
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
        # 右上に時間を表示するテキスト (2D 座標)
        self.time_text = self.ax.text2D(
            0.98,
            0.98,
            "",
            transform=self.ax.transAxes,
            ha='right',
            va='top',
            fontsize=10,
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none')
        )
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

    def update(self, drones, sim_time=None):
        positions, colors = self._get_plot_data(drones)
        
        # 3D散布図のデータを更新
        self.scatter._offsets3d = (positions[:, 0], positions[:, 1], positions[:, 2])
        self.scatter.set_facecolors(colors)

        # 以前のテキスト削除（必要なら）
        for t in self.text_artists:
            t.remove()
        self.text_artists.clear()
        
        # クラスタ番号 + リーダーマーク表示
        for d in drones:
            label = f"No.{d.id} C{d.cluster_id}" if d.cluster_id is not None else "-"
            if d.is_leader:
                label += " L"
            elif getattr(d, 'is_sub_leader', False):
                label += " S"
            txt = self.ax.text(d.pos[0], d.pos[1], d.pos[2],
                               label,
                               fontsize=5, color='black')
            self.text_artists.append(txt)
        
        # 時刻の表示を更新
        if sim_time is not None:
            try:
                self.time_text.set_text(f"t = {float(sim_time):.2f}s")
            except Exception:
                self.time_text.set_text(f"t = {sim_time}")
        else:
            self.time_text.set_text("")

        # 描画を更新
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
            
        plt.pause(0.01) # 短いポーズを入れてGUIを更新

    def close(self):
        plt.ioff() # インタラクティブモードをオフ
        plt.close(self.fig)

