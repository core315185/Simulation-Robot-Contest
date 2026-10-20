# Simulation-Robot-Contest

ロボット掃除機＋見守りカメラのシミュレーションロボコン（大学課題提出用）

Unity 6 (6000.3 LTS) / Universal 3D / C# / Rigidbody物理演算のみを用いて、
S字クランクコースを自動走行するロボットをシミュレートする。

## 動作環境

- Unity 6000.3 LTS（Universal 3D テンプレート）
- Visual Studio Code
- Git / GitHub
- Windows

## セットアップ手順

1. 本リポジトリをUnity Hubで開く（Unity 6000.3 LTS指定）。
2. 新規シーンを作成し、以下の順でメニューを実行する。
   1. **Tools > Create Course** — コース（Floor / Outer Walls / Inner Walls / Start / Goal / Waypoints）を生成
   2. **Tools > Create Robot** — ロボット（Base / Camera Pole / Camera / Wheel Left / Wheel Right / 各スクリプト）を生成
3. 空のGameObjectを作成し、`SceneInitializer` をアタッチする。
   - Play時に Fixed Timestep(0.01s) の設定と、コンポーネント間の自動配線が行われる。
4. `Robot` を `Course/Waypoints/WP_0` の位置（スタート地点）に配置する。
5. 必要であれば `UIManager` 用のCanvas・Textを作成し、`UIManager` に参照を設定する（任意）。
6. Playすると、`Robot` が `StartLine` トリガーに触れた時点で計測が開始し、
   `WaypointFollower` に従って自動走行、`GoalLine` を完全に通過した時点で計測終了する。

### コース形状の調整（重要）

`CourseGenerator.cs` 内の `BuildDefaultCenterline()` は、配布図面を目視で見積もった
概算のS字クランク座標です。**必須条件の寸法（全体幅2.4m×全体高さ3.0m、通路幅0.6m）は
厳密に守られていますが、コーナーの正確な位置は図面と完全一致しない可能性があります。**
生成後、`Course/Waypoints` 以下の `WP_0`〜`WP_4` をSceneビューでドラッグして
図面と一致するよう調整し、再度 **Tools > Create Course** を実行して壁を再生成してください。

## ファイル構成

```
Assets/
  Editor/
    CourseGenerator.cs   Tools > Create Course（コース一括生成）
    RobotGenerator.cs    Tools > Create Robot（ロボット一括生成）
  Scripts/
    RobotController.cs      ロボット統括制御（PID・加速度クランプ・失格判定・タイマー）
    DifferentialDrive.cs    差動二輪の運動学（純粋なC#クラス）
    PIDController.cs        汎用PID制御器
    WaypointFollower.cs     ウェイポイント追従（目標速度・角速度の生成）
    CameraController.cs     カメラ先端の水平加速度・ジャークの実測
    DataLogger.cs           走行データのCSV出力
    GraphExporter.cs        CSVから速度/加速度/ジャークグラフをPNG出力
    UIManager.cs            画面表示（タイマー・速度・加速度・状態）
    SceneInitializer.cs     Fixed Timestep設定・依存関係の自動配線
    CourseLineMarker.cs     スタート/ゴールラインの通過方向マーカー
```

## 課題必須条件との対応

| 必須条件 | 対応箇所 |
|---|---|
| Rigidbodyのみで移動（Transform直接操作禁止） | `RobotController.ApplyDriveControl` は `AddForce`/`AddTorque` のみ使用 |
| Collision Detection = Continuous Dynamic | `RobotController.InitializePhysicalProperties` |
| Fixed Timestep = 0.01（一定） | `SceneInitializer.ApplyFixedTimestep` |
| ロボット直径0.30m / カメラ高さ1.00m | `RobotGenerator` の定数 |
| 質量10kg・重心高さ0.50m | `RobotGenerator` / `RobotController` の定数 |
| 左右2輪差動駆動 | `DifferentialDrive` |
| 前進・後退・停止・信地旋回・超信地旋回 | `RobotController` の Manual Motion Primitives（および自動走行時のWaypointFollowerの出力としても実現） |
| カメラ先端水平合成加速度 ≤ 1.00 m/s² | `RobotController.ApplyDriveControl` の力クランプ、および `CameraController` による独立した実測検証 |
| 壁接触・壁通過・転倒 = 失格 | `RobotController` の `OnCollisionEnter` / `CheckOutOfBounds` / `CheckTipOver` |
| 走破時間・最大/平均加速度・最大速度・最大角速度・最大ジャーク | `DataLogger` が記録するCSVから `GraphExporter`/手元集計で算出 |
| CSV出力・加速度グラフ | `DataLogger`（CSV）、`GraphExporter`（PNGグラフ） |

## 提出物の作成

1. Playしてロボットを走破させる（複数回試行し、最良のログを選ぶ）。
2. `Application.persistentDataPath/SimulationLogs/` に生成される
   `run_log_*.csv` と、同フォルダに自動出力される `run_log_*_speed.png` /
   `_accel.png` / `_jerk.png` を確認する。
3. 画面録画で `.mp4` を作成する。
4. CSV・PNG・走破時間などの数値（コンソールログにも出力される）をまとめて
   PDFドキュメントを作成する（Word等、別途作業。データが揃った時点で支援可能）。
5. アピールポイントをまとめたPDFを作成する（本READMEの「設計上の要点」を参考にできる）。

## 設計上の要点（アピールポイント）

- **物理演算のみで移動**: `Transform` は一切書き換えず、`Rigidbody.AddForce/AddTorque` のみ使用。
- **加速度制約の機構的保証**: 左右車輪PIDの出力（力）を合成し、
  `F_max = m × 1.00 [N]` にクランプすることで、水平合成加速度が理論上常に
  1.00 m/s² 以下になるよう設計（①運動方程式によるモデル化）。
- **独立検証**: `CameraController` が `Rigidbody.GetPointVelocity` によって
  カメラ先端の実速度・実加速度を直接計測し、制御側の想定（重心＝カメラ先端の
  水平加速度）が実際に成立しているかをログ上で検証できる。
- **状態フィードバック**: 左右車輪それぞれに独立したPID制御器を持つ、
  真の差動二輪フィードバック制御（②PID制御、⑤状態フィードバック）。
- **軌道生成の土台**: `WaypointFollower` は曲率に応じて先読み減速する
  速度プロファイルを持つ（③最短時間制御・④軌道生成への拡張余地）。
- **データ可視化**: 外部ライブラリ不使用でUnity内完結のCSV+PNGグラフ出力
  （⑥データ可視化）。

## 既知の簡略化（正直な申告）

- 差動二輪の駆動力は、車輪接地点への直接作用（`AddForceAtPosition`）ではなく、
  重心に対する一般化力（並進力・旋回トルク）として適用している。
  接地点作用モデルは重心高さとの位置関係からピッチ方向の余計なトルクを生み、
  加速度制約の機構的保証を不安定にするため、あえて簡略化している。
  より高精度な `WheelCollider` ベースのモデルへの拡張は良い発展課題になる。
- `CourseGenerator` の壁座標は図面の目視概算であり、`Course/Waypoints` の
  調整が別途必要。
- PIDゲイン・巡航速度は初期値であり、実際の走行を見ながらのチューニングが必要
  （評価項目「制御器の設計・プログラム」の核心部分）。

## GitHubへのPush

```bash
git init
git add .
git commit -m "Initial commit: Simulation-Robot-Contest"
git branch -M main
git remote add origin <あなたのGitHubリポジトリURL>
git push -u origin main
```

`.gitignore` により `Library/` 等のUnity生成物と動画ファイルは追跡対象外になっている。
