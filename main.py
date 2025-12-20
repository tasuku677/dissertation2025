import asyncio
import matplotlib.pyplot as plt
import numpy as np
from global_var import SimConfig
from simulation import TdlsFanetSimulation
from simulationkundu import KunduTdlsFanetSimulation
import os
from datetime import datetime


# --- メイン処理 ---
async def main():
    # 最初にユーザー入力で実行種別を選択 (1: kundu, 2: nara)
    choice = input("実行するシミュレーションを選んでください (1: kundu, 2: suggested) > ").strip()
    attack_choice = input("攻撃シナリオを選んでください (1: on-off, 2: normal) > ").strip()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    config = SimConfig()
    if choice == "1":
        simulation = KunduTdlsFanetSimulation(config)
        folder = "kundu" 
    else:
        simulation = TdlsFanetSimulation(config)
        folder = "suggested"
    # 攻撃シナリオの設定
    if attack_choice == "1":
        folder += "_onoff"
    else:
        folder += "_normal"
    output_dir = os.path.join("records", folder, timestamp)
    os.makedirs(output_dir, exist_ok=True)
    
    # SimConfig のクラス変数をテキストファイルに保存
    config_items = {
        k: v for k, v in SimConfig.__dict__.items()
        if not k.startswith('__') and not callable(v)
    }
    cfg_path = os.path.join(output_dir, "simconfig.txt")
    with open(cfg_path, "w", encoding="utf-8") as f:
        for k in sorted(config_items):
            f.write(f"{k} = {repr(config_items[k])}\n")


    try:
        await simulation.run(attack_choice=attack_choice)
    except Exception as e:
        print(f"シミュレーション中にエラーが発生しました: {e}")
    finally:
     # シミュレーションが正常終了またはエラーで停止した場合でも、
        # 最終的な結果グラフを表示する
        print("シミュレーション終了。結果をファイルに保存します。")
        simulation.save_trust_history_to_csv(os.path.join(output_dir, f"trust_history_{timestamp}.csv"))
        simulation.save_cluster_history_to_csv(os.path.join(output_dir, f"cluster_history_{timestamp}.csv"))
        simulation.save_report_history_to_csv(os.path.join(output_dir, f"report_history_{timestamp}.csv"))
        
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
        
        simulation.plot_trust_by_type() # タイプ別の平均信頼値をプロット
        plt.savefig(os.path.join(output_dir, f"trust_by_type_{timestamp}.png"))
        plt.close()
        
        simulation.plot_security_metrics() # セキュリティ指標のプロット
        plt.savefig(os.path.join(output_dir, f"security_metrics_{timestamp}.png"))
        plt.close()

        print(f"結果のプロットとCSVを '{output_dir}' フォルダに保存しました。")
        
if __name__ == '__main__':
    asyncio.run(main())