# Windows・既存Dockerでの起動と録画

最新版は **recovery_v9_open_space_escape**。赤→青→黄を繰り返し、通常回避3回後に開放方向へ離脱して同じ色を再探索します。v9のGazebo完走は未確認です。[検証記録](VERIFICATION.md)を参照してください。

既存のcompetent_banzaiコンテナーを使い、削除・再作成は行いません。別名なら置き換えてください。

## 1. 入力先を確認

| 目印 | 入力するもの |
|---|---|
| Windows PowerShell：PS C:\...> | docker、explorer |
| Docker内：root@...:/# | source、ros2、ffmpeg |

画面の目印とコード枠のバッククォート3個は入力しません。枠内をコピーし、1行ごとにEnter。長いコマンドも途中でEnterを押さないでください。画面幅の自動折り返しは問題ありません。

画面AはGazebo、Bは自律走行、Cは診断、Dは録画に使います。

## 2. Dockerを起動

WindowsのスタートメニューからDocker Desktopを起動し、エンジンの起動を待ちます。Windows PowerShellを開きます。

```powershell
docker version
docker ps -a --format "table {{.Names}}\t{{.Image}}\t{{.Status}}"
docker start competent_banzai
docker ps --format "table {{.Names}}\t{{.Status}}"
```

ClientとServerの両方、以前使ったコンテナー名、Upの表示を確認します。

## 3. 初回・更新時：コピーとビルド

リポジトリを取得・展開したフォルダーをエクスプローラーで開きます。README.mdとsrcが見える場所です。アドレス欄にpowershellと入力してEnterを押します。更新時は先に画面Bの旧missionをCtrl+Cで止めます。

Windows PowerShellで実行します。Test-PathがTrueになってからコピーします。

```powershell
Test-Path .\src\color_target_nav\package.xml
docker exec -u root competent_banzai mkdir -p /root/color_nav_ws/src
docker cp .\src\color_target_nav competent_banzai:/root/color_nav_ws/src/
docker exec -it -u root competent_banzai bash
```

Successfully copiedの後、root@に変わったら実行します。

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
cd /root/color_nav_ws
colcon build --packages-select color_target_nav --symlink-install
source /root/color_nav_ws/install/setup.bash
ros2 pkg executables color_target_nav
```

1 package finished、navigator、watchdogが確認できれば完了です。既存ワークスペースの場所が違う場合は2行目を変更します。

既存イメージでは必要なツールが導入済みでした。依存不足はpackage.xmlに基づき対応します。過去のイメージではROSのapt署名期限切れがありました。署名確認を無効化せず、公式パッケージ設定やイメージを確認してください。

## 4. 画面A：Gazebo

すでにGazeboとセンサーが動いていれば重ねて起動せず手順5へ進みます。新たに起動する場合、Docker内で実行します。

```bash
export DISPLAY=:0
export XAUTHORITY=/home/ubuntu/.Xauthority
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
source /root/color_nav_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH="$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH}"
ros2 launch color_target_nav demo.launch.py
```

Windowsのブラウザーで http://localhost:6080/vnc.html を開きます。Ubuntu内のGazeboにロボット、赤・青・黄の箱、灰色障害物が出ることを確認し、画面Aは開いたままにします。

開かない場合はWindows PowerShellのdocker port competent_banzaiで既存ポートを確認します。コンテナーの再作成はしません。

## 5. 画面B：自律走行を1組だけ起動

新しいWindows PowerShellで実行します。

```powershell
docker exec -it -u root competent_banzai bash
```

root@で以下を実行します。

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
source /root/color_nav_ws/install/setup.bash
ros2 topic info /burger_test/cmd_vel
```

**Publisher count: 0**を確認してから次を1回だけ実行します。1以上なら旧missionやteleopを終了します。FAILEDの旧missionも停止指令を送り続けるため残しません。

```bash
ros2 launch color_target_nav mission.launch.py
```

recovery_v9_open_space_escapeとrepeat=Trueを確認します。画面Bは開いたままにし、別画面で同じミッションを起動しません。現在位置から赤を探し直し、再起動だけでは初期位置へ戻りません。

## 6. 画面C：状態確認

新しいPowerShellからDockerへ入り、手順5と同じ3つのsetup.bashを読み込んで実行します。

```bash
ros2 topic echo /color_nav/status --once --full-length
ros2 topic info /burger_test/cmd_vel
```

正常起動時は送信元1。2以上なら重複起動です。target（現在色）、completed_targets（接触累計）、laps（周回数）、reason / failure_reason（動作・停止理由）、camera_confirmed、relocation_count（離脱回数）を確認します。

必要に応じてセンサーを確認します。連続表示の終了はCtrl+Cです。

```bash
ros2 topic echo /burger_camera/color_camera/image_raw --once --field width
ros2 topic hz /scan
ros2 topic echo /color_nav/contacts --once
```

未接触時のstates: []は正常です。状態トピックがなく送信元0なら画面Bの起動ログを確認します。

## 7. 画面D：毎回の録画

Gazeboで全体が見える視点に整えます。Ubuntuデスクトップだけが映り、WindowsのPowerShellは映りません。氏名・所属・メール等が画面にないことを確認します。

新しいWindows PowerShellからDockerへ入ります。

```powershell
docker exec -it -u root competent_banzai bash
```

root@で実行します。最後のコマンドも途中でEnterを押さず1行で実行します。

```bash
export DISPLAY=:0
export XAUTHORITY=/home/ubuntu/.Xauthority
mkdir -p /root/color_nav_ws/recordings
ffmpeg -n -f x11grab -framerate 15 -i :0.0 -an -c:v libx264 -preset ultrafast -crf 23 -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" -pix_fmt yuv420p -movflags +faststart "/root/color_nav_ws/recordings/color_nav_$(date +%Y%m%d_%H%M%S).mp4"
```

frame=が増えれば録画中です。日時入りなので毎回同じコマンドが使えます。日時はコンテナーのタイムゾーンに従い、Windows時刻と異なる場合があります。

最初は10秒の試し撮りを行います。**終了は画面Dで半角q**。root@に戻るまで保存を待ちます。保存完了前のコピーは再生できない場合があります。

```bash
ls -lht /root/color_nav_ws/recordings/
```

本番は録画開始後に画面Bから1組だけミッションを起動します。赤→青→黄、後退、回避、再探索と失敗場面も原本に残します。

## 8. Windowsへ動画を取り出す

保存完了後、**Windows PowerShell（PSで始まる画面）**で実行します。これは録画ではなくコピーです。

```powershell
docker cp competent_banzai:/root/color_nav_ws/recordings "$env:USERPROFILE\Videos"
explorer "$env:USERPROFILE\Videos\recordings"
```

「ビデオ」→「recordings」に日時入りMP4があります。ファイル名・サイズ・録画開始時の出力を照合して開きます。コンテナー時刻が違う場合があるため、表示時刻だけで判断しません。

dockerやexplorerがcommand not foundならDocker内に入力しています。Windows PowerShellで実行し直します。docker cpとコピー元・先は分けず、1行全部を実行してください。

## 9. 終了・次回

1. 画面Dでqを押して録画保存を待つ。
2. 画面BのmissionをCtrl+Cで終了。
3. 画面Cで速度送信元0を確認。
4. 必要なら画面AのdemoもCtrl+Cで終了。
5. Docker自体を止める場合だけ、Windows PowerShellでdocker stop competent_banzaiを実行。

次回は手順2→4→5。ソースに変更がなければコピー・ビルドは不要です。

## 困ったとき

### ミッションの重複

同名navigator / watchdogが複数、速度送信元2以上なら各mission端末でCtrl+Cを押します。端末が見つからない場合は現在のプロセスを確認します。

```bash
ps -eo pid,ppid,args | grep -E '[m]ission.launch.py|[c]olor_target_nav/(navigator|watchdog)'
```

現在のmission起動元PIDを特定して割り込み終了し、送信元0を確認します。過去ログのPIDを再利用したり、Gazebo全体を無差別に終了したりしません。

### Gazeboの画面だけ閉じた・車体が見えない

```bash
pgrep -a -x gzserver
pgrep -a -x gzclient
```

gzserverが残っていれば物理計算は継続している場合があります。別のDocker内端末で手順4の表示環境、ROS、モデル検索先を設定し、gzclientを起動します。無応答のgzclientが残る場合は現在のPIDを特定し、そのGUIだけを終了してから再起動します。

既存Dockerのメッシュはturtlebot3_common/meshes/burger_base.dae等です。別の上流版のSTLパスへ推測で変更しません。

### FAILEDで停止

状態全文と画面Bのログを残し、target_timeout、unexpected_contact、上限到達等を確認します。安全判定を無効化せず、旧missionを終了してから必要な変更・再起動を行います。駆動異常時はGazeboを一時停止してください。
