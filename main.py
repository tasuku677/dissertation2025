import asyncio
import matplotlib.pyplot as plt
import numpy as np
from global_var import SimConfig
from simulation import TdlsFanetSimulation
import os
from datetime import datetime


# --- メイン処理 ---
async def main():
    output_dir = "results"
    os.makedirs(output_dir, exist_ok=True)
    
    # ファイル名に使用するタイムスタンプを生成
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    config = SimConfig()
    simulation = TdlsFanetSimulation(config)
    try:
        await simulation.run()
    except Exception as e:
        print(f"シミュレーション中にエラーが発生しました: {e}")
    finally:
     # シミュレーションが正常終了またはエラーで停止した場合でも、
        # 最終的な結果グラフを表示する
        print("シミュレーション終了。結果をファイルに保存します。")
        simulation.save_trust_history_to_csv(os.path.join(output_dir, f"trust_history_{timestamp}.csv"))
        simulation.save_cluster_history_to_csv(os.path.join(output_dir, f"cluster_history_{timestamp}.csv"))

        # 各プロットを個別のFigureとして作成し、ファイルに保存
        simulation.plot_results()
        plt.savefig(os.path.join(output_dir, f"results_{timestamp}.png"))
        plt.close() # Figureを閉じる
        
        simulation.plot_trust([i for i in range(0, 10)])  # ドローンID 0-5の信頼度履歴を表示
        plt.savefig(os.path.join(output_dir, f"trust_history_{timestamp}.png"))
        plt.close()

        simulation.plot_aggregated_trust() # 全ノードの信頼値の統計をプロット
        plt.savefig(os.path.join(output_dir, f"aggregated_trust_{timestamp}.png"))
        plt.close()

        print(f"結果のプロットとCSVを '{output_dir}' フォルダに保存しました。")
        
if __name__ == '__main__':
    asyncio.run(main())