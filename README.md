## 使い方

main.pyを実行するとシミュレーションを行えます．

## ファイル構成

- **main.py**: シミュレーションの実行エントリーポイントです。シミュレーション方式（Kundu / Suggested）と攻撃シナリオ（On-Off / Normal）を選択して実行し、結果を保存します。
- **global_var.py**: シミュレーションの設定値や定数を管理する `SimConfig` クラスが含まれています。
- **simulation.py**: 提案手法（Suggested）またはベースとなるTDLS FANETシミュレーションのロジック（`TdlsFanetSimulation`）を実装しています。
- **simulationkundu.py**: 比較対象となるKunduモデルのシミュレーションロジック（`KunduTdlsFanetSimulation`）を実装しています。
- **records/**: シミュレーション実行時に生成される結果ファイル（CSV, PNG, 設定ログ）が保存されるディレクトリです。
