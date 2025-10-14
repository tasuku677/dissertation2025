import asyncio
from global_var import SimConfig
from simulation import TdlsFanetSimulation


# --- メイン処理 ---
async def main():
    config = SimConfig()
    simulation = TdlsFanetSimulation(config)
    try:
        await simulation.run()
    except Exception as e:
        print(f"シミュレーション中にエラーが発生しました: {e}")
    finally:
        # シミュレーションが正常終了またはエラーで停止した場合でも、
        # 最終的な結果グラフを表示する
        print("シミュレーション終了。結果グラフを表示します。")
        simulation.save_trust_history_to_csv("trust_history.csv")
        simulation.plot_results()
        
        # simulation.plot_trust([ drone.id for drone in simulation.drones])
        simulation.plot_trust()  # ドローンID 0-5の信頼度履歴を表示

        
if __name__ == '__main__':
    asyncio.run(main())