# 双车 360° 体素探索：强基线与异步 V2

本分支仅保留两套探索算法的独立入口：全候选强基线，以及包含 R2 空闲任务补充修复的异步 V2。旧 rolling/RL 观测算法、定向体素演示、独立 fast 版、异步 V1 和修复前的实验结果已清理。原项目的 Nav2、SLAM 和竞拍探索功能仍保留。

## 共同定义

- 体素边长 0.05 m，状态为 UNKNOWN / FREE / OCCUPIED。以扫描证据更新；没有测到不等于空闲。
- 教室使用 `worlds/classroom.sdf.xacro`。观测任务范围为 `[-8, 8] × [-6, 6] × [0.05, 2.85] m`。
- 雷达参数在 `config/lidar360_os0.yaml`：360°、64×1024、10 Hz、垂直 ±45°、0.3–5 m 有效范围、安装高 0.22 m。采用 OS0 类采样密度的模拟模型，不声称逐束复刻某台实物雷达。
- 任务只有位置，不为全周雷达枚举朝向。候选位置间距 0.25 m；地面路径检查整段净空，默认安全净空 0.25 m，双车速度各 0.22 m/s。
- 根据可通行路径距离从近到远分配未知体素所有权，减少多个候选重复认领同一体素。路径距离不是简单欧氏距离。强基线在全候选域内认领；V2 为降低延迟，在当前已评估候选池内认领，池外更近视点可能要到后续轮次才能发现。
- 前沿优先；26 邻域未知连通簇小于 0.005 m³（5 cm 下 40 个体素）时不生成任务。被忽略的体素仍是 UNKNOWN，仍遮挡射线，不伪装成已观测或空闲。
- 每轮最多选择 8 个观测任务，联合分配和排序，最小化双车的最大任务完成时间。路线子问题可精确求解，但候选筛选、前沿优先和有限任务批次意味着整体探索策略不是全局最优证明。

## 两套实现

| | 强基线 | 异步 V2 |
| --- | --- | --- |
| 入口 | `scripts/demo_voxel360_exploration.py` | `scripts/demo_voxel360_async_v2.py` |
| 配置 | `config/voxel360_planner_params.yaml` | `config/voxel360_async_v2_params.yaml` |
| 候选计算 | 全候选射线可见性评估 | 前沿聚类、增量更新、缓存，每次通常评估最多 32 个位置，2 个射线工作线程 |
| 任一车到达 | 双车停稳，接收新扫描，重新规划 | 触发新规划，期间继续旧路线 |
| 规划接入 | 求解后双车同时执行 | 预测未来位置、短期路线承诺、按实际位置重新连接并校验 |
| 仿真计算时钟 | 求解墙钟耗时单独记录，不进入视频 | 实测求解耗时计入模拟时钟和视频 |

异步 V2 的旧路线承诺期上限为 1.5 s，并在剩余路线即将用完时提前补充。初始扫描后尚无路线可执行，因此首次规划仍可能等待。任一车空闲时每 5 s 继续搜索补充任务，不要求另一辆车也空闲。

R2 修复包括 `frontier_fill_detail_tasks: true`：先选择前沿任务，再用剩余任务名额保留已通过可见性与收益检查的三维细节任务，使黑板等区域不会仅因不属于主要前沿而被丢掉。若当前没有有效、安全的任务，小车仍允许空闲。

`voxel_fast_policy.py`、`voxel_incremental_frontier.py`、`voxel_async_execution.py` 是最新版使用的共享实现，并非保留的第三套算法。`voxel_ros_transport.py` 只负责 ROS 订阅、TF、消息和 action 客户端；历史节点名 `rolling_observation_planner` 保留用于 YAML 兼容，不再包含旧 rolling 或 RL 选择器。`voxel_route_solver.py` 保存两套算法共同使用的子集动态规划。

## 停止条件

默认没有固定仿真时长。只有保留的可观测未知任务清空、不可观测分类审计完成且稳定期达到 8 s 才报告完成。找不到候选的一轮、尚未完成的审计、规划失败、任务失败或扫描失败均不能直接把未知清零。

不可达或在配置传感器几何下无法观测的体素作为排除项记录；小孤立簇作为忽略项记录。它们与真实扫描得到的 FREE / OCCUPIED 分开统计。未知总数未必降至零，所以不能用“未知减少比例”替代探索完成率。

`--duration` 和 `--max-events` 只用于实验截断。到达上限记录 `complete: false`；本次保留的 160/1500 秒结果均为截断数据，均未完成全场探索。

## 运行与测试

在仓库根目录执行。Python 依赖为 NumPy、PyYAML、OpenCV、Pillow；场景解析使用 ROS 的 xacro。可选 `g++` 提供 C++17 原生体素内核，缺失时回退到 NumPy，速度会明显变慢。视频编码使用系统 FFmpeg / OpenCV。

```bash
source /opt/ros/lyrical/setup.bash
python3 scripts/demo_voxel360_exploration.py
python3 scripts/demo_voxel360_async_v2.py

# 有上限的测试，另存结果
python3 scripts/demo_voxel360_async_v2.py --duration 160 --simulation-rate 0 \
  --record-period 1 --output-dir maps/async_v2_new_run

# 六类场景各三组出生点；并发任务使用不同物理核心
python3 scripts/benchmark_indoor_suite.py --duration 160 --jobs 3 \
  --output maps/indoor_async_v2_new_run

python3 -m unittest discover -s scripts -p 'test_*.py'
```

`--simulation-rate 0` 取消实时播放节流；异步规划的实测计算时间仍计入模拟时钟。因机器负载和测量耗时影响接入位置，同配置的异步轨迹不保证逐帧相同。

Gazebo / Nav2：

```bash
EXPLORATION_MODE=voxel360 WORLD=classroom USE_RVIZ=True \
  ./scripts/tb3_multi_explore_slam.sh

EXPLORATION_MODE=voxel360 WORLD=classroom USE_RVIZ=True \
  VOXEL_PARAMS="$PWD/config/voxel360_async_v2_params.yaml" \
  ./scripts/tb3_multi_explore_slam.sh
```

ROS 共用 `voxel_exploration_planner.py`。主要输出为 `/observation/markers`、`/observation/voxel_stats`、`/observation/completed`、每辆车的 `pose_world`、`route_world` 和 `observed_points_world`。`config/voxel_observation.rviz` 用于查看实时数据；`scripts/record_voxel_visualization.py` 用于记录真实 ROS 话题。该启动路径使用真实计算时间，强基线在 ROS 中会等待规划，离线基线的“瞬时计算”假设不适用于它。

## 保存的可视化与数据

| 算法与场景 | 视频或索引 | 数据目录 |
| --- | --- | --- |
| 强基线教室 160 s | [MP4](../maps/classroom_voxel360_frontier_160s/exploration.mp4) | `maps/classroom_voxel360_frontier_160s` |
| 强基线教室 1500 s | [MP4](../maps/classroom_voxel360_frontier_1500s_rerun/exploration.mp4) | `maps/classroom_voxel360_frontier_1500s_rerun` |
| 最新 V2 教室 160 s | [MP4](../maps/classroom_voxel360_async_v2_r2fix_160s/exploration.mp4) | `maps/classroom_voxel360_async_v2_r2fix_160s` |
| 最新 V2：18 组测试 | [本地 HTML 视频索引](../maps/indoor_async_v2_suite_160s/index.html) | `maps/indoor_async_v2_suite_160s` |

视频保持 MP4 格式；同时提交初始/最终 PNG、事件与规划统计、CSV/JSON 汇总。画面显示该时刻已观测的 2D 地图、3D 点云、小车位置和规划路线。场景布局预览是已知真值，视频中的点云和观测图按扫描时序显示。GitHub 仓库中的 HTML 索引需下载后在浏览器打开，不会自动成为 GitHub Pages 网站。

每个点只保存一次坐标和首次观测时间；逐帧文件保存地图、姿态、路线和引用信息，不复制完整累计点云。为减少 Git 小文件数量，四个保留结果目录分别提供 `replay_data.zip` 和 `replay_data_manifest.json`，包括帧输入、点时间线、最终体素（若原记录提供）、日志以及原始源码快照。源码快照是核验历史实验所用的只读归档，可能包含当时并存的实验代码；当前可运行实现以 `scripts/` 和 `config/` 为准。快照及原始数据哈希不因本次文件拆分而重写。

展开的回放文件和重复 GIF/实时预览留在本地但被 Git 忽略；仓库中只提交一次回放 ZIP，避免重复体积。每个 ZIP 以所在结果目录为解压根目录：

```bash
python3 -m zipfile -e maps/classroom_voxel360_async_v2_r2fix_160s/replay_data.zip \
  maps/classroom_voxel360_async_v2_r2fix_160s
python3 -m zipfile -e maps/indoor_async_v2_suite_160s/replay_data.zip \
  maps/indoor_async_v2_suite_160s

# 从已保存观测重渲染，不重新规划
python3 scripts/demo_voxel360_async_v2.py --render-only \
  --output-dir maps/classroom_voxel360_async_v2_r2fix_160s
python3 scripts/verify_voxel360_async.py maps/classroom_voxel360_async_v2_r2fix_160s

# 将新实验的回放输入打包，并逐文件核对 SHA-256 与 ZIP CRC
python3 scripts/package_voxel_replay.py maps/async_v2_new_run
```

## 18 组室内回归的范围与结论

教室、办公室、会议室、图书室、实验室、多房间布局各有前侧分开、后侧相邻、对角分开三组出生点，场景和配置见 `worlds/indoor_suite/` 与 `config/indoor_suite/`。全部 18 组运行至 160 s，通过原始归档、运动路径、点时间线和视频时长核验；详见 [summary.csv](../maps/indoor_async_v2_suite_160s/summary.csv)。

每组平均规划耗时 0.445–0.562 s，P95 为 0.541–0.847 s。全部运行在首次规划后没有因求解而等待，采样最小双车距离为 1.3 m。密集书架和多房间后侧出生的覆盖较慢，说明布局和出生点仍会影响探索效率。

这些记录使用已知二维通行先验的离线几何仿真，没有模拟完整 SLAM 误差、真实轮式动力学或通信延迟，尚未验证真车。当前联合优化任务分配与路线顺序，未实现双车时空轨迹避碰；采样距离只是一项回归检查。各场景仅一次运行，且基线与 V2 时间语义不同，因此上述耗时和覆盖记录不能直接作为严格的端到端速度胜出结论。
