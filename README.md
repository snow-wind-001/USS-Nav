# USS-Nav Reproduction (ROS2)

该仓库提供论文 **USS-Nav: Unified Spatio-Semantic Scene Graph for Lightweight UAV Zero-Shot Object Navigation** 的可运行复现骨架。

## 目录

- `uss_nav_ws/src/uss_nav_interfaces`: 自定义消息定义
- `uss_nav_ws/src/uss_nav_stack`: 核心算法与节点
- `uss_nav_ws/src/uss_nav_bringup`: launch / params / rviz
- `docs/repro_issues_and_solutions.md`: 复现缺口与解决策略

## 环境建议（重要）

- ROS2 Humble 的 Python ABI 是 **3.10**，自定义 `rosidl` 消息运行时也需要 Python 3.10。
- 当前机器上：
  - `verl` = Python 3.11.7（会触发消息类型支持加载失败）
  - `TLCForMer-main` = Python 3.10.19（可稳定运行）
- 建议优先使用 `TLCForMer-main` 进行复现。

## 环境与构建

```bash
source ~/.bashrc
conda activate TLCForMer-main
python -m pip install -U "empy==3.3.4" catkin_pkg backports.tarfile lark-parser
python -m pip install -U matplotlib
source /opt/ros/humble/setup.bash
cd /home/a4201/owncode/USS-Nav/uss_nav_ws
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=$(which python) -DPYTHON_EXECUTABLE=$(which python)
source install/setup.bash
```

## 运行

```bash
source ~/.bashrc
conda activate TLCForMer-main
source /opt/ros/humble/setup.bash
cd /home/a4201/owncode/USS-Nav/uss_nav_ws
source install/setup.bash
ros2 launch uss_nav_bringup explore_a.launch.py
```

打开 RViz 预配置（自动显示四类 marker）：

```bash
ros2 launch uss_nav_bringup explore_a.launch.py use_rviz:=true
```

## 本地 VLM 服务（Qwen2-VL-2B + vLLM）

先启动服务（默认优先 `youtu-agent`，失败会按你的规则自动切到 `llmtest`）：

```bash
source ~/.bashrc
cd /home/a4201/owncode/USS-Nav/uss_nav_ws
bash scripts/start_vllm_qwen2vl.sh
```

停止服务：

```bash
bash scripts/stop_vllm_qwen2vl.sh
```

若你希望手动启动（固定在 `youtu-agent`）：

```bash
source ~/.bashrc
conda activate youtu-agent
vllm serve /home/a4201/checkpoints/Qwen2-VL-2B-Instruct \
  --host 127.0.0.1 --port 8001 \
  --served-model-name Qwen2-VL-2B-Instruct \
  --max-model-len 4096 \
  --generation-config vllm
```

## MARSIM 接入（A 纯探索几何验证）

MARSIM 侧至少准备两个 topic：

- `pose`：`PoseStamped` 或 `Odometry`
- `points`：`PointCloud2`

使用模板参数直接切换到外部仿真输入：

```bash
ros2 launch uss_nav_bringup explore_a.launch.py \
  params_file:=/home/a4201/owncode/USS-Nav/uss_nav_ws/src/uss_nav_bringup/config/params_explore_a_marsim.yaml \
  use_rviz:=true
```

联调前可先做 topic 合同检查（MARSIM 是否给到可用输入）：

```bash
source ~/.bashrc
conda activate TLCForMer-main
source /opt/ros/humble/setup.bash
cd /home/a4201/owncode/USS-Nav/uss_nav_ws
source install/setup.bash
bash scripts/check_marsim_topics.sh --check-image
```

说明：

- `params_explore_a_marsim.yaml` 已将 `simulate_motion=false`、`use_synthetic_fallback=false`。
- 默认 `points_timeout_sec=2.0`，若你的 LiDAR 发布频率更低，可再适当调大。
- 你只需要按自己的 MARSIM 实际话题修改：
  - `rolling_grid_node.ros__parameters.pose_topic`
  - `rolling_grid_node.ros__parameters.pose_topic_type`
  - `rolling_grid_node.ros__parameters.points_topic`

## 纯探索模式（A）

- `explore_a.launch.py` 仅启动：
  - `rolling_grid_node`
  - `gcm_node`
  - `frontier_node`
  - `planner_node`
  - `exploration_monitor_node`
- 不依赖 SCG / Region / Object / LLM，适合先打通仿真探索闭环。

## A 模式新增能力

- RViz Marker：
  - `/viz/gcm`（visited coarse cells）
  - `/viz/frontiers`（frontier 点）
  - `/viz/waypoint`（当前目标点）
  - `/viz/explore_status`（覆盖率/前沿数/done 状态文本）
  - `/viz/vlm_status`（视觉搜索 found/confidence 状态）
- 探索终止：
  - 终止信号：`/nav/explore_done`（`std_msgs/Bool`）
  - 终止条件：覆盖率阈值 + 前沿阈值 / 无前沿持续时间 / 指标平台期 / `target_found`
- 基础指标：
  - `/nav/explore_metrics`（JSON 字符串）
  - 指标包含：`coverage_ratio`, `frontier_count`, `coverage_delta_window`, `frontier_delta_window`, `done`, `done_reason`

## 快速“见图搜索”接入

启动探索 + VLM 搜索（需先确保 vLLM 服务已启动）：

```bash
source ~/.bashrc
conda activate TLCForMer-main
source /opt/ros/humble/setup.bash
cd /home/a4201/owncode/USS-Nav/uss_nav_ws
source install/setup.bash
ros2 launch uss_nav_bringup explore_a.launch.py \
  use_rviz:=true \
  use_vlm:=true \
  params_file:=/home/a4201/owncode/USS-Nav/uss_nav_ws/src/uss_nav_bringup/config/params_explore_a_marsim.yaml \
  vlm_search_node.ros__parameters.enabled:=true
```

动态下发目标词（例如“灭火器”）：

```bash
ros2 topic pub --once /nav/target_query std_msgs/msg/String "{data: fire extinguisher}"
```

查看视觉搜索输出：

```bash
ros2 topic echo /vlm/search_result
ros2 topic echo /nav/target_found
```

说明：

- 默认读取压缩图像话题：`/camera/image/compressed`。
- 若仿真只提供 `sensor_msgs/Image`，可把 `image_is_compressed=false` 并改 `image_topic`。
- 当 `target_found=true` 且 monitor 配置 `stop_on_target_found=true` 时，`/nav/explore_done` 会置为 true。

## rosbag2 自动录制与评估

```bash
source ~/.bashrc
conda activate TLCForMer-main
source /opt/ros/humble/setup.bash
cd /home/a4201/owncode/USS-Nav/uss_nav_ws
source install/setup.bash
bash scripts/record_and_evaluate_explore.sh --duration 120
```

如果是 MARSIM 联调，推荐直接：

```bash
bash scripts/record_and_evaluate_explore.sh --duration 120 --marsim
```

如果你希望直接得到“建图能力 + 算法能力”的 PASS/FAIL 报告（推荐）：

```bash
bash scripts/run_marsim_validation.sh --duration 120 --check-image
```

输出报告：

- `eval/marsim_validation_report.json`
- `eval/marsim_validation_report.md`

- 自动行为：
  - 启动 `explore_a` 纯探索
  - 录制关键 topic 到 rosbag2
  - 监听 `/nav/explore_done`（或超时）自动停止
  - 自动导出：
    - `explore_metrics_curve.csv`
    - `explore_curves.png`（覆盖率曲线+前沿数曲线）
    - `explore_summary.json`
    - `explore_terrain_map.png`（重建地形图：占据顶视图 + 高度图）
    - `explore_terrain_summary.json`（地形体素统计）
    - `explore_terrain_evolution.gif`（时间序列地形演化动画）
    - `explore_terrain_evolution.mp4`（时间序列地形演化动画，若 ffmpeg 可用）

如果你已经手动启动了 MARSIM + explore_a（不希望脚本重复 launch），可用：

```bash
bash scripts/record_and_evaluate_explore.sh --duration 120 --skip-launch
```

如果你只想手动做评估并控制动画参数（例如只导出 GIF）：

```bash
python scripts/evaluate_explore_metrics.py \
  --bag-dir <bag目录> \
  --output-dir <输出目录> \
  --terrain-anim-format gif \
  --terrain-anim-fps 8 \
  --terrain-anim-interval-sec 0.8 \
  --terrain-anim-max-frames 300
```

## 默认话题

- `/map/rolling_grid`
- `/nav/gcm_json`
- `/nav/frontiers`
- `/nav/explore_metrics`
- `/nav/explore_done`
- `/scg/graph`
- `/region/labels`
- `/objects/graph`
- `/nav/target_region`
- `/nav/waypoint`
- `/viz/gcm`
- `/viz/frontiers`
- `/viz/waypoint`
- `/viz/explore_status`
- `/vlm/search_result`
- `/nav/target_found`
- `/nav/target_query`
- `/viz/vlm_status`

## Qwen2-VL-2B 方案合理性评估

- 优点：本地部署快、显存开销低（2B），可通过 vLLM 标准接口与 ROS2 解耦，适合快速闭环验证。
- 复现一致性：满足“论文中 LLM/VLM 决策模块”的工程替代思路，但不是同规模模型，语义鲁棒性会弱于大模型版本。
- 性能预期：对目标“粗粒度可见性判断”较快；对遮挡、远距离小目标、细粒度类别区分可能不稳定。
- 建议：把 `Qwen2-VL-2B` 作为快速验证基线，后续可在同接口下升级到更强模型并做 A/B 对比。
