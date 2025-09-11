
from global_var import SimConfig
from simulation import TdlsFanetSimulation


# --- メイン処理 ---
if __name__ == '__main__':
    config = SimConfig()
    simulation = TdlsFanetSimulation(config)
    try:
        simulation.run()
    except Exception as e:
        print(f"シミュレーション中にエラーが発生しました: {e}")
    finally:
        # シミュレーションが正常終了またはエラーで停止した場合でも、
        # 最終的な結果グラフを表示する
        print("シミュレーション終了。結果グラフを表示します。")
        simulation.plot_results()
        # simulation.plot_trust([ drone.id for drone in simulation.drones])
        simulation.plot_trust([0, 1, 2, 3, 4, 5])  # ドローンID 0-5の信頼度履歴を表示