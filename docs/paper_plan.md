# Lite-HMRE 论文级升级执行计划与交接状态（Paper Branch）

> **分支**：`paper/hier-25d3d-explore`（原演示完整版归档在 `feat/mine-inspection-console`）  
> **目标投稿**：IEEE RA-L（联动 ICRA / IROS）或 ICRA / IROS 主会（双栏 8 页，纯几何规划 + 在线配准）  
> **硬件与环境**：RTX 5090 服务器 + 1 台配备 3D 激光雷达与 Linux 系统的实车底盘；统一采用 **ROS 2 Lyrical（Ubuntu 26.04）** + Docker 容器化。

---

## 一、已对齐的全部核心决策（无需重新确认）

1. **主攻目标与节奏**：发论文优先，无硬性截稿限制；聚焦 **低算力轻量化 2.5D-3D 分层多机协同探索（Lite-HMRE）**，不引入视觉语义检测分支。
2. **模块裁剪与仓库组织**：
   - 已移除半成品 RL 模块（`src/rl_nav2_controller` 及相关脚本/配置）与 `models/` 视觉权重；
   - 原 Web 控制台已移至 `extras/console/`，仅作可选补充视频录制工具，**严禁参与论文定量评测与资源统计**；
   - 保持 `colcon` 标准 ROS 2 功能包架构，采用 **混合语言架构**：上层调度/决策/测试用模块化 Python，高频热路径（SE(2) 配准、点云合并、测地距离场、LoS/FoV 可见性射线）用 C++ / `pybind11`。
3. **定位与去真值里程计策略**：
   - 去掉 `config/gz_waffle_3d_lidar.sdf.xacro` 中默认的 Gazebo 真值里程计（`OdometryPublisher`），恢复带真实噪声的轮式里程计 + IMU EKF（`robot_localization`），保留 `use_gt_odom` 开关仅用于与外部 SOTA 对比时隔离定位变量（同时将真实位姿单独发布至 `/robot_i/gt_pose` 仅供离线评测 ATE/RPE）；
   - 落地 **C++ 在线 SE(2) 动态配准与长廊退化门控（TODO #13）** 消除多车 3D 重影。
4. **三大核心算法支柱**：
   - **Pillar 1**：退化门控在线 SE(2) 多机配准（FFT 相位相关粗配 + 2D 点到线 ICP 精配 + Hessian 特征值退化投影 Solution Remapping + 李代数指数平滑）+ C++ 高频点云合并节点（`src/se2_map_merge/`）；
   - **Pillar 2**：C++ 多源栅格测地距离替代欧氏距离驱动 Local Open-TSP 与 Global ROI 竞拍 + 各车 `global_costmap` 共享 `/map`（TODO #12）+ 队友足迹/预测轨迹注入 `local_costmap` + 拓扑窄道互斥预约与礼让点（TODO #9）；
   - **Pillar 3**：2.5D `VoxelColumnIndex` 结构信息增益 + 2D Bresenham 视线（LoS）与 3D 雷达垂直视场角 $[-30^\circ, +55^\circ]$ 约束的次模残差补盲视点优化 + 三判据可重开收敛 + 通信带宽量化分析。
5. **实验与对比策略**：
   - **多机规模**：参数化启动支持 $N = 1, 2, 3, 4$ 台车可扩展性实验；
   - **场景套件（6 个）**：现有 `house`、`hospital`、`mine`、`warehouse` + 导入 2 个 CMU M-TARE 官方标准网格世界（`cmu_indoor`、`cmu_tunnel`）；
   - **对比基线**：同 Gazebo 环境 ROS 2 基线（`m-explore-ros2` legacy、2D-Auction、`frontier_exploration_ros2` + 7 组内部消融 `E0`–`Full`）+ 1 个核心外部 3D SOTA（**M-TARE 官方 Docker**，统一 `--cpus=4 --memory=8g` 边缘算力限额与相同地图/速度/雷达约束）；
   - **实车验证**：使用现有 1 台 3D 雷达实车跑真实杂物室内/走廊单机闭环验证（`pointcloud_to_laserscan` 喂 `slam_toolbox`，对比 2D-only vs. Ours 各 3 轮，采集 3D 补盲特写与板载 CPU/RAM 实时曲线），多机协同实验全部在仿真完成。

---

## 二、当前进度总览（交接检查点）

- [x] **M1：工程地基、模块重构、Docker 统一环境与离线评测标尺**（已完成，4 commits）
  - `61faf7d`：建立 `paper/hier-25d3d-explore`，删除 RL/`models/`，迁移 `extras/console/`，新增 `docker/Dockerfile.ros2`（Lyrical）、`docker/Dockerfile.mtare`、`docker/docker-compose.yml`、`requirements-dev.txt`，修复 `docs/figures/dual_robot_mapping_fig.py`。
  - `f05b86d`：将原 3,463 行 `scripts/multi_robot_coordinator.py` 与 `scripts/exploration_3d.py` 拆分为标准 ROS 2 包 `src/hier_explore_core/`（`geometry.py`、`occupancy_view.py`、`voxel_index.py`、`residual_nbv.py`、`frontier.py`、`scheduler.py`、`convergence.py`、`robot_tracker.py`、`supervisor.py`、`visualizer.py`、`coordinator_node.py`），并保留原 `scripts/` 兼容垫片。
  - `ad39411`：新增标准评测包 `src/hier_explore_bench/`（`pcd_io.py`、`sample_gt_mesh.py`、`eval_metrics.py`、`bench_recorder.py`、`run_benchmark.py`、`plot_paper_figs.py`）。离线验证 `mine`/`house`/`hospital` 真值采样已通过（`mine`: 127 primitives, $A_{\text{GT}}=280.09\text{ m}^2$; `house`: $88.16\text{ m}^2$; `hospital`: $172.16\text{ m}^2$）。
  - `94f9dd8`：同步 `README.md` 与当前仓库配置（16×360 雷达、仅局部代价图点云、Docker/Benchmark 用法）。
  - **已通过测试**：`51 passed`（`src/hier_explore_core/test`、`src/hier_explore_bench/test`、`scripts/test_exploration_3d.py`、`scripts/test_cloud_stamp.py`）。
- [ ] **M2：定位去作弊 + C++ `se2_map_merge`（在线 SE(2) 动态配准与退化门控）+ C++ 点云合并节点** ← **下一个立即执行的里程碑**
- [ ] **M3：C++ `pybind11` 几何核（测地距离 + LoS/FoV 残差视点优化 + 窄道骨架）+ 共享 `/map` 全局代价图 + 车际避碰与互斥预约**
- [ ] **M4：参数化 $N=1\sim 4$ 启动包 `src/hier_explore_bringup/` + 导入 CMU `cmu_indoor` / `cmu_tunnel` 世界**
- [ ] **M5：5090 服务器全量多随机种子 Benchmark + 消融 + M-TARE Docker 对照 + 论文矢量图与 LaTeX 表格生成**
- [ ] **M6：单台 3D 雷达实车部署（`real_robot.launch.py`）+ 真机 2D-only vs. Ours 对比实验 + 补充视频**

---

## 三、待执行里程碑详细规格（M2 – M6）

### M2（WP2）：定位去作弊 + C++ `src/se2_map_merge/`
1. **恢复真实噪声轮式里程计 + IMU EKF**：
   - 修改 `config/gz_waffle_3d_lidar.sdf.xacro`，增加 `<xacro:arg name="use_gt_odom" default="false"/>`：
     - `use_gt_odom=false`（默认）：启用 `gz::sim::systems::DiffDrive` 的轮式里程计输出到 `$(arg namespace)/wheel_odom`（不直接发 `odom->base_footprint` TF），保留 `OdometryPublisher` 但仅发布真值到 `$(arg namespace)/gt_pose`（关闭其 TF 广播，仅供 `bench_recorder.py` 算 ATE/RPE）；
     - 新增 `config/ekf_odom.yaml`，每台车启动 `robot_localization` (`ekf_node`) 融合 `wheel_odom` + `imu`，发布平滑 `/robot_i/odom` 与 `odom -> base_footprint` TF；
     - `use_gt_odom=true`：仅作为与外部 SOTA 对比时的理想定位对照模式。
2. **新建 C++ ROS 2 包 `src/se2_map_merge/`**：
   - `include/se2_map_merge/phase_corr.hpp` + `src/phase_corr.cpp`：重叠区占据边缘提取 + Hanning 窗 OpenCV FFT 相位相关平移粗配准（响应置信度门控 $\rho_{\text{pc}} > \tau_{\text{pc}}$）。
   - `include/se2_map_merge/icp_2d.hpp` + `src/icp_2d.cpp`：基于 2D KD-Tree（nanoflann 或自写轻量 KD-Tree）+ Huber 鲁棒核的点到线（Point-to-Line）ICP 精配准；对 $3\times 3$ 近似 Hessian $H = \sum J_k^\top J_k$ 做特征值分解，检测矿井/长直巷道退化方向（$\lambda_{\min}(H_{tt}) < \tau_{\text{deg}}$）并执行 **Solution Remapping**（仅更新非退化横向与航向分量，冻结退化纵向分量）；加入 IoU 单调增益校验、步进限幅（每周期 $|\Delta t|\le 0.05\text{ m}, |\Delta\theta|\le 1^\circ$）与李代数指数平滑（$\alpha=0.25$）。
   - `src/se2_map_merge_node.cpp`：订阅 `/robot_1/map`..`/robot_N/map`，以 `robot1` 为基准在线估计并广播 `world -> robot_i/map` TF，合并发布 `/map`（`transient_local + reliable`），并可选为每车发布局部对齐的 `/robot_i/shared_map` 与诊断话题 `/map_merge/alignment_diagnostics`。
   - `src/cloud_merger_node.cpp`：用 C++ 重写 `scripts/merge_pointclouds.py`（支持任意 $N$ 车、零拷贝解析 `PointCloud2`、自反框过滤、TF 时钟抖动与位姿跳变拒绝、int64 打包哈希体素降采样 0.02 m，发布 `/points_merged`）。
   - **M2 验收门禁**：
     - C++ gtest 单元测试覆盖：合成旋转 $5^\circ$ + 平移 $1.2\text{ m}$ 的 2D 栅格恢复精度 $< 2\text{ cm}, < 0.3^\circ$；长直平行墙走廊退化场景下纵向漂移为 $0$ 且横向/旋转正确收敛；
     - 5090 服务器 4 核容器内运行 `mine` 与 `warehouse`（`use_gt_odom=false`），对比原 `map_merge`，合并图 `occupied_iou_tol1` 与 3D `chamfer_l1_m` 显著提升，C++ `cloud_merger_node` CPU 占用较原 Python 版降低 $\ge 50\%$。

### M3（WP3）：C++ `pybind11` 几何核 + 共享 `/map` + 窄道协同避碰
1. **在 `src/hier_explore_core/cpp/` 新增 `pybind11` 模块 `_geom_kernel`**：
   - `geodesic_grid.cpp`：给定安全自由栅格与源点列表，有界多源 Dijkstra 计算精确测地距离矩阵 $D_{\text{geo}}(a,b)$ 与路径点列（500×600 栅格目标 $< 6\text{ ms}$）；注入 `scheduler.py` 替换欧氏距离。
   - `visibility_fov.cpp`：针对每个 3D 残差聚类，在 $[0.8, 3.5]\text{ m}$ 环带采样 $K=24$ 个安全视点，执行 2D Bresenham 墙体遮挡射线检测 + 垂直视场角 $[\phi_{\min}, \phi_{\max}]$（默认 $[-30^\circ, +55^\circ]$，可参数化适配实车雷达）+ 有效观测距离 $R_{\text{eff}} \le 5.0\text{ m}$ 校验，贪心次模最大化覆盖残差柱集合。
   - `corridor_graph.cpp`：基于 `obstacle_dt` 提取通道半宽 $w(p) < 2 r_{\text{robot}} + 0.35\text{ m}$ 的窄道瓶颈段 $E_{\text{narrow}}$ 及其两端宽敞礼让点（Yield Points）。
2. **共享全局代价图与双层车际避碰**：
   - `config/nav2_multirobot_slam_params.yaml`：`global_costmap.static_layer` 改为订阅全局合并图（`/map` 或 `/robot_i/shared_map`），彻底解决 TODO #12；
   - `src/hier_explore_core/hier_explore_core/deconfliction.py`：5 Hz 广播 `/robot_i/teammates_cloud`（其余车辆足迹圆 + 未来 2 s 规划轨迹采样点）注入各车 `local_costmap.voxel_layer`；
   - 在 `scheduler.py` 接入拓扑窄道互斥预约（Corridor Mutex Reservation）：低优先级车进入被占瓶颈段时自动暂停于礼让点或切换另一分支前沿，消除矿井 3.2 m 窄巷与医院门洞对顶死锁。
   - **M3 验收门禁**：C++ 核 + Python 单测全通过；4 核容器下 `mine` 与 `hospital` 连续 5 次零对向死锁、零车际碰撞；规划周期 P95 $< 50\text{ ms}$；3D 表面完整度较 2D-only 显著提升。

### M4（WP4）：参数化 $N=1\sim 4$ 启动与 CMU 公开场景（`src/hier_explore_bringup/`）
- 编写 `multi_sim.launch.py`（支持 `num_robots=1..4`, `world`, `spawn_seed`, `odom_mode`, `ablation_profile`, `planner_backend`）替换硬编码 2 车 Bash；
- 从 CMU Autonomous Exploration Development Environment 导入 `cmu_indoor` 与 `cmu_tunnel` 网格封装为 Gazebo SDF，并生成 6 场景全部 `maps/gt/<world>/` 真值档案与 4 车无重叠出生点池。

### M5（WP5）：5090 服务器全量实验与论文图表输出
- 跑完主对比表（Table I / Fig. 3：5 方法 × 6 场景 × 5 种子，含 M-TARE Docker 在 `--cpus=4 --memory=8g` 下同图对比）、7 组消融（Table II）、噪声里程计配准专项对比（Fig. 2）、$N=1\sim 4$ 扩展性加速比与带宽（Fig. 4）、16/32 线与体素分辨率敏感性附表，使用 `plot_paper_figs.py` 自动产出矢量图与 LaTeX 表格。

### M6（WP6）：单台 3D 雷达实车验证
- 编写 `src/hier_explore_bringup/launch/real_robot.launch.py`（`pointcloud_to_laserscan` + `slam_toolbox` + `cloud_merger_node` + `octomap_server` + Nav2 + `hier_explore_core` 单机模式，参数化实车雷达垂直 FoV）；
- 使用同一 `docker/Dockerfile.ros2` 部署到实车，在真实杂物室内/走廊运行 2D-only vs. Ours 各 3 轮，产出 Fig. 5 与论文补充视频。

---

## 四、5090 服务器快速验证命令（接手后第一步）

```bash
# 1. 拉取分支
git fetch origin
git checkout paper/hier-25d3d-explore

# 2. 宿主机或 Docker 内编译并跑离线全量单元测试
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select \
  explore_lite_msgs explore_lite multirobot_map_merge hier_explore_core hier_explore_bench
source install/setup.bash

PYTHONPATH=src/hier_explore_core:src/hier_explore_bench:scripts pytest -v \
  src/hier_explore_core/test src/hier_explore_bench/test scripts/test_exploration_3d.py scripts/test_cloud_stamp.py

# 3. 预生成 mine / house / hospital 真值 3D 点云与 2D 可探索地图
for entry in "mine:-19.0 0.0" "house:-3.0 -2.0" "hospital:-6.0 0.0"; do
  w="${entry%%:*}"; s="${entry#*:}"
  PYTHONPATH=src/hier_explore_bench python3 -m hier_explore_bench.sample_gt_mesh \
    --world "worlds/${w}.sdf.xacro" --seed ${s} --out-dir "maps/gt/${w}"
done
```
