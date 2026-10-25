# 自然言語指示による設備点検ロボット

ROS 2 Jazzy / Gazebo Sim / OpenAI LLM・VLMによる設備巡回点検デモです。

## 概要

自然言語「設備Bの後に設備Aを点検してください」をLLMが `{"waypoints":["B","A"]}` に変換します。差動二輪ロボットがB→Aの順に移動し、設備方向に旋回して画像を保存します。VLMは画像内の表示灯を識別し、B＝赤（模擬異常）、A＝緑（模擬正常）を判定します。結果はJSONLに記録され、HTMLダッシュボードへ集計されます。

**重要**: これはシミュレーション上の色による模擬状態判定です。実機故障診断や汎化性能を検証したものではありません。

## 動作環境

- Ubuntu 24.04、ROS 2 Jazzy、Gazebo Sim（実際に使用したバージョンは環境に合わせて追記）
- Python 3.12系（ROS 2）と、OpenAI SDKを導入した仮想環境
- `ros_gz_bridge`, `ros_gz_sim`, `xacro`, `cv_bridge`, OpenCV
- OpenAI APIキー（各AIノードの起動シェルで `OPENAI_API_KEY` を環境変数として設定）

## 構成

| モジュール | 主なトピック | 役割 |
| --- | --- | --- |
| `llm_planner` | `/inspection/instruction` → `/inspection/plan` | 自然言語から巡回順序を生成 |
| `waypoint_controller` | `/inspection/plan`, `/odom`, `/inspection/capture_done` → `/cmd_vel`, `/inspection/capture_request` | 移動・旋回・撮影完了待ち |
| `inspection_camera` | `/camera/image_raw`, `/inspection/capture_request` → `/inspection/capture_done` | PNG撮影保存 |
| `inspection_vlm` | `/inspection/capture_done` → `/inspection/result` | 表示灯認識と結果記録 |
| `inspection_report.py` | `inspection_results.jsonl` → HTML/CSV/JSON | 実験結果を集計 |

### デモ座標

- 設備Aモデル `(2.35, 0.0)` ／撮影停止位置 `(1.0, 0.0)`
- 設備Bモデル `(2.35, 1.5)` ／撮影停止位置 `(1.0, 1.5)`
- カメラPitch 約 -0.262 rad（調整済み構成）

## セットアップ

本リポジトリの `src/` にROS 2パッケージが配置されている想定です。`setup.py` / `package.xml` には実際の依存関係を定義してください。

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-cv-bridge python3-opencv
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

AIノードのPython環境は既存環境と整合させます。システムPythonを安易に上書きしないでください。

```bash
# APIキーは画面表示・Gitコミットしない
read -rsp 'OpenAI API Key: ' OPENAI_API_KEY; echo
export OPENAI_API_KEY
```

## デモ実行

それぞれ別ターミナルでROS環境を `source` してください。

```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
# ワールドが立ち上がった後に別ターミナルで（同梱している場合）
./spawn_equipment.sh
ros2 run my_py_pkg inspection_camera
ros2 run my_py_pkg waypoint_controller
```

OpenAI SDKが導入済みの仮想環境からAIノードを起動します。

```bash
.venv/bin/python src/my_py_pkg/my_py_pkg/llm_planner.py
.venv/bin/python src/my_py_pkg/my_py_pkg/inspection_vlm.py
```

```bash
python3 inspection_report.py mark
ros2 topic pub --once /inspection/instruction std_msgs/msg/String \
  "{data: '設備Bの後に設備Aを点検してください'}"
# VLM結果がA/B両方そろった後
python3 inspection_report.py build
xdg-open data/reports/inspection_dashboard.html
```

**備考**：実際のソースコード、設備SDF、起動スクリプトはローカルの `~/ros2_ws` からコピーして格納する必要があります。このREADME単体ではシステムは動きません。

## 結果

確認済みの代表例：B→A巡回完了、撮影完了通知2/2、VLM判定B＝ABNORMAL（赤）／A＝NORMAL（緑）。実験2件中2件一致は、この模擬条件内の結果であり一般的な認識精度ではありません。

## 安全と公開範囲

`.env`、APIキー、`.venv/`、`build/`、`install/`、`log/`、実験時の個人パスを含むログは公開しません。公開前に `git diff --cached` でコミット対象を確認してください。

## ライセンス

第三者コードやGazeboモデルを含む場合は、それぞれのライセンスを確認して記載してください。未確認の状態で一律のライセンスを付与しないでください。
