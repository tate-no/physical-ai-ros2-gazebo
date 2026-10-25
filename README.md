# TurtleBot3 Burger：色を順番に探して接触する自律走行

ROS 2 Humble / Gazebo Classic上で、カメラのHSV色検出と360度LiDARを使い、**赤→青→黄→赤…** の順に箱へ接近・接触する追加パッケージです。

既存の `burger_color_*.py`、`src/obstacle_detector`、カメラモデル、バックアップは変更していません。追加パッケージは [`src/color_target_nav`](src/color_target_nav) です。

- **最新版：`recovery_v9_open_space_escape`**
- [Windows・既存Docker向けの起動と録画手順](START_HERE_JA.md)
- [検証結果と未確認事項](VERIFICATION.md)
- [色順・繰り返し・時間制限の設定](src/color_target_nav/config/mission.yaml)

## 何ができるか

1. 現在の目標色をカメラの新しい3フレームで確認する。
2. LiDARで車体が通過する範囲を確認し、向きを合わせて接近する。
3. 指定した色の箱とのGazebo接触メッセージを確認した場合だけ、次の色へ進む。
4. 接触した向きから20cm以上後退し、接触なし・回転用空間・前方距離を異なる3スキャンで確認してから次の色を探す。
5. 進路が塞がれていれば、空いた経路へ退避し、回転・通過後に同じ目標色を探し直す。
6. 通常の回避を3回繰り返したら、LiDARから開放方向を選び、約50cm離脱してから同じ色を再確認する。

開放方向は16方向の候補を評価し、元の正面を避け、左右18cmの帯に80cm未満の測距点がない方向から選びます。旋回と移動中も空間を確認し、通れなければ停止します。これは局所制御で、SLAMや地図に基づく大域的な経路計画ではありません。

## 確認できた範囲

| 対象 | 結果 |
|---|---|
| v7 / 利用者環境のGazeboログ | 赤→青→黄を2周し、次の赤を含む7回の接触を記録。その後は赤への再接触で停止。 |
| v8 / 利用者環境のGazeboログ | 赤・青に接触後、黄へ向かう途中で回避・再試行上限に到達。 |
| v9 / ローカルテスト | 制御、HSV認識、測距、XML、合成環境での周回・離脱を検証。詳細は検証記録。 |
| v9 / Gazebo | 完走回数・開放空間への離脱成功は未確認。旧版の2周実績と区別する。 |

合成テストは円形物体・合成画像・運動学モデルによるもので、Gazeboの摩擦・物理接触・描画・DDS接続を再現していません。録画ファイルの保存成功も、連続動作の成功とは別に扱います。

## 既存Dockerへの追加（概要）

このリポジトリにはDockerfileやComposeはありません。既存イメージ `airobotbook/ros2-desktop-ai-robot-book-humble:latest` を使った環境で作業しています。既存コンテナーを削除・再作成せず、追加ワークスペースへコピーします。初めての方は[手順書](START_HERE_JA.md)を使ってください。

Windows PowerShellで、取得したこのリポジトリのフォルダーを開いて実行します。コンテナー名は既存環境に合わせて変更します。

```powershell
$containerName = 'competent_banzai'
docker start $containerName
docker exec -u root $containerName mkdir -p /root/color_nav_ws/src
docker cp .\src\color_target_nav "${containerName}:/root/color_nav_ws/src/"
docker exec -it -u root $containerName bash
```

Docker内でビルドします。既存ワークスペースの場所が異なる場合は2行目を変更します。

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
cd /root/color_nav_ws
colcon build --packages-select color_target_nav --symlink-install
source install/setup.bash
export DISPLAY=:0
export XAUTHORITY=/home/ubuntu/.Xauthority
export TURTLEBOT3_MODEL=burger
ros2 launch color_target_nav demo.launch.py
```

別のDocker内端末で同じ3つのsetup.bashを読み込んだ後、まず既存の速度指令発行元がないことを確認し、ミッションを1組だけ起動します。

```bash
ros2 topic info /burger_test/cmd_vel
ros2 launch color_target_nav mission.launch.py
```

起動前は `Publisher count: 0`、起動後は `Publisher count: 1` を確認します。以前のミッション、既存の色制御スクリプト、teleopなどを同時に実行しないでください。

## コード構成

| ファイル | 役割 |
|---|---|
| `color_target_nav/perception.py` | HSV検出、画像内の色領域・方位、連続認識 |
| `color_target_nav/control.py` | 色順、探索、接近、接触、退避、再試行の状態遷移 |
| `color_target_nav/avoidance.py` | 通過範囲確認、通常回避、開放空間への離脱 |
| `color_target_nav/node.py` | ROS入出力、センサー受信鮮度、状態JSON |
| `color_target_nav/watchdog.py` | 要求指令の途絶時にゼロ速度を出す独立プロセス |
| `config/mission.yaml` | 色順、繰り返し、トピック、時間制限 |
| `models/burger_contact.sdf` | カメラ・LiDAR・接触バンパー付き派生モデル |
| `worlds/color_targets.world` | 赤・青・黄の箱と灰色障害物 |
| `test/` | ROS不要の単体・合成環境テスト |

表のパスは `src/color_target_nav/` からの相対パスです。

## 設定・停止条件

既定は `targets: [RED, BLUE, YELLOW]`、`repeat: true`。変更は `config/mission.yaml` を編集してビルド・ミッション再起動します。

- 画像・LiDAR・接触・オドメトリの受信が0.7秒以上途絶した場合、不正・不完全なLiDAR測定の場合は停止。
- 目標ごとの総時間300秒、通常回避の時間90秒。待機後の自動再試行は最大2回。開放空間への離脱は各目標最大2回。
- 想定外の物体へのバンパー接触はFAILED。自動的に無視して走り続けません。
- watchdogは要求速度が0.3秒途絶すると停止指令を出します。全プロセス強制終了・ホスト切断までの停止は保証しません。異常時はGazeboを一時停止してください。

対象名は `red_box` / `blue_box` / `yellow_box`、ロボット名は `burger_camera_main`。既存モデルには追加バンパーがないため、派生モデルを使います。既存DockerのDAEメッシュ配置を前提とし、メッシュ自体は同梱しません。モデル生成は `scripts/build_demo_assets.py`。

LiDARより低い物体、遮蔽、細い障害物、画像と測距の非同期、目標色の手前に重なる別物体には限界があります。実ロボットの安全制御としては未検証です。

## 状態確認

環境を読み込んだ別のDocker内端末で実行します。

```bash
ros2 topic echo /color_nav/status --once --full-length
```

`state`, `target`, `completed_targets`, `laps`, `reason`, `failure_reason`, `recovery_triggers`, `relocation_count` などを確認できます。`check_motion.py` と `watch_motion.py` は読み取り専用の補助診断です。`watch_motion.py` はミッションの稼働を待ち、45秒の記録を保存します。

## ローカルテスト

Python 3.10以降、NumPy、OpenCVが必要です。ROSノード自体を起動せず、制御・合成画像・XMLを検証します。

```bash
PYTHONPATH=src/color_target_nav python3 -m unittest discover -s src/color_target_nav/test -v
python3 -m compileall -q src/color_target_nav
git diff --check
```

Windows PowerShellの場合：

```powershell
$env:PYTHONPATH = 'src/color_target_nav'
python -m unittest discover -s src/color_target_nav/test -v
```

既存プログラムの実行方法や必要環境はそれぞれのファイルを参照してください。新規パッケージとの同時走行は避けてください。
