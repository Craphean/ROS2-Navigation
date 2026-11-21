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
- [x] **M2（代码、单测与 5090 4 核实跑验收已全部通过）：定位去作弊 + C++ `se2_map_merge` + C++ 点云合并节点**
  - **WP2.1（去真值里程计 + IMU EKF）**：`config/gz_waffle_3d_lidar.sdf.xacro` 新增 `use_gt_odom` 参数（默认 `false`），恢复 `DiffDrive` 噪声轮式里程计（`wheel_odom` @ 30 Hz，不发 TF），`OdometryPublisher` 仅发评估用 `/<ns>/gt_pose`（10 Hz，不进 EKF/SLAM）；新增 `config/tb3_bridge_ekf.yaml` 与 `config/ekf_odom.yaml`（`robot_localization::ekf_node` 融合 `wheel_odom` `vx/vy` + 50 Hz `imu` `vyaw/ax`），并在 `scripts/tb3_multi_explore_slam.sh` 与 `scripts/kill_nav2_sim.sh` 中完成全链路接入（保留 `USE_GT_ODOM=true` 消融开关）。
  - **WP2.2（C++ `src/se2_map_merge/` 在线 SE(2) 动态配准 + 退化门控）**：纯 C++17 核心库 `se2_map_merge_core`（零 OpenCV/Eigen 外部头文件依赖，支持独立 CMake 与 ROS 2 `colcon` 双编译）实现 Hanning 窗 2D FFT 相位相关粗配 + 2D KD-Tree PCA 法向 + Huber 核点到线 ICP + 2×2/3×3 Jacobi 特征分解 **Solution Remapping** 长廊退化门控 + IoU 单调校验 + 步进限幅李代数平滑；`se2_map_merge_node` 支持任意 $N$ 车渐进 DAG 配准、动态 20 Hz `world -> <ns>/map` TF（带 `tf_future_offset`）、`/<ns>/tf` 与 `/<ns>/tf_static` 完整前缀中继、`/map_merge/alignment_diagnostics` JSON 诊断（已接入 `bench_recorder.py`）。
  - **WP2.3（C++ `cloud_merger_node`）**：零拷贝 `PointCloud2` 解析/打包、自反框过滤、TF 时钟偏斜/跳变拒绝、`uint64` 打包体素降采样；在 `tb3_multi_explore_slam.sh` 中设为默认（缺包自动回退 Python 版）。
  - **已通过离线单测与 5090 4 核 ROS 2 Lyrical 实跑验收**：
    - `colcon build --symlink-install` 全 6 包零错编译通过；`colcon test --packages-select se2_map_merge` 4/4 全通过（零警告 `-O3 -Wall -Wextra -Wpedantic`）：1.2 m + 5° 滑移恢复误差 **0.016 cm / 0.0012°**；长直矿井走廊退化测试纵向漂移 **0.00 cm**；Python `pytest` **57 passed**。
    - 5090 4 核隔离容器 `USE_GT_ODOM=false` 240 s A/B 实测（`legacy` vs `se2_map_merge + cloud_merger_node`）：
      - **`mine`**：`occupied_iou_tol1` `0.635 -> 0.869`（+36.8%），`occupied_hd95_m` `0.901 -> 0.552 m`，`ate_rmse_mean_m` `0.207 -> 0.045 m`（-78.4%），同高度带 `[0.08, 2.00] m` 3D `chamfer_l1_m` `0.1334 -> 0.1277 m`（`accuracy_mean_m` `0.0513 -> 0.0407 m`，`precision_10cm` `0.943 -> 0.988`），`cloud_merger` CPU 均值 `18.97% -> 2.97%`（**-84.4%**，峰值 `184.1% -> 12.9%`），SE(2) 接受 `117` 次。
      - **`warehouse`**：`ate_rmse_mean_m` `0.350 -> 0.049 m`（-86.0%），2D 覆盖率 `44.8% -> 46.4%`、`n_auc` `0.233 -> 0.272`、`overlap_ratio` `0.113 -> 0.0054`、3D `fscore_15cm` `0.381 -> 0.388`，`cloud_merger` CPU 均值 `20.53% -> 3.05%`（**-85.1%**，峰值 `135.3% -> 6.9%`），SE(2) 接受 `113` 次并实测触发长廊退化门控（`degenerate_trans=true`, `eig_min=10.45`, IoU `0.926 -> 0.932`）。
    - 顺带修复 5090 无头实机适配：`kill_nav2_sim.sh` 祖先链保护、`tb3_multi_explore_slam.sh` 的 `GZ_IP=127.0.0.1` + `--headless-rendering` + `~/.gz/models` 离线资源索引、`fuel_prefetch.py` 自动建 `model://` 符号链接、`bench_recorder.py` 的 `use_sim_time` 覆盖与分节点 CPU 采样、`sample_gt_mesh.py` 的 Fuel `<include>` + `.stl`/`.dae` 支持。
- [x] **M3（代码、单测与 5090 4 核实跑验收已全部通过）：C++ `pybind11` 几何核（测地距离 + LoS/FoV 残差视点优化 + 窄道骨架）+ 共享 `/map` 全局代价图 + 车际避碰与互斥预约**
  - **WP3.1（C++17 `pybind11` 几何核 `_geom_kernel`）**：在 `src/hier_explore_core/cpp/` 实现 `geodesic_grid.cpp`（有界 32 桶 Dial 队列多源 Dijkstra 测地距离矩阵与 A* 精确路径，500×600 栅格 2 源×16 目标实测 **`4.23 ms`** `< 6 ms`）、`visibility_fov.cpp`（$[0.8, 3.5]\text{ m}$ 3 环 $K=24$ 安全视点采样 + 2D Bresenham 视线遮挡 + 垂直视场角 $[-30^\circ, +55^\circ]$ + $R_{\text{eff}} \le 5.0\text{ m}$ 贪心次模最大覆盖）、`corridor_graph.cpp`（双侧致命墙体中轴窄道瓶颈段 $E_{\text{narrow}}$ 提取 + 0 拐角假阳性 + 两端宽敞礼让点定位，实测 `2.96 ms`）及 Python 封装 `geom_kernel.py`。
  - **WP3.2（共享全局代价图 TODO #12 + 双层车际避碰 TODO #9）**：`config/se2_map_merge_params.yaml` 开启 `publish_per_robot_shared_map: true`（在 `se2_map_merge_node.cpp` 释放锁后重投影发布 `/<ns>/shared_map`），`config/nav2_multirobot_slam_params.yaml` 中 `global_costmap.static_layer.map_topic` 切换为 `shared_map`（`tb3_multi_explore_slam.sh` 在 Nav2 激活前启动 `se2_map_merge_node`，legacy 模式自动回退 `map`）；新增 `deconfliction.py`（Layer-1 5 Hz `/<ns>/teammates_cloud` 足迹圆 + 2 s 轨迹预测 + 2.85 m 射线清除环注入 `local_costmap.voxel_layer`；Layer-2 `CorridorMutexManager` 拓扑窄道优先级互斥预约、分支改道与礼让点驻留）。
  - **已通过离线单测与 5090 4 核 ROS 2 Lyrical 实跑验收**：
    - `colcon test --packages-select se2_map_merge hier_explore_core` 100% 通过（`test_se2_map_merge` 4/4，`test_geom_kernel` 3/3）；Python `pytest` **80 passed in 2.38s**。
    - 5090 4 核隔离容器 `mine`（连续 5 种子 `0..4`）与 `hospital`（连续 5 种子 `0..4`）实测：
      - **零死锁 / 零碰撞**：`mine` 连续 5 轮与 `hospital` 连续 5 轮均实现 **0 对向死锁（`stall_events = 0`）、0 车际碰撞（`inter_robot_collisions = 0`）**；
      - **规划周期 P95 $< 50\text{ ms}$**：`mine` 5 轮中位规划周期 `cycle_ms_p50` 均值 **`4.21 ms`**（`3.71–4.83 ms`），95 分位 `cycle_ms_p95` 均值 **`44.88 ms < 50 ms`**（240 s 实跑 `39.04 ms`）；`hospital` 5 轮 `cycle_ms_p50` 均值 **`4.61 ms`**（`3.61–5.68 ms`），`cycle_ms_p95` 均值 **`47.67 ms < 50 ms`**；
      - **3D 表面完整度与覆盖提升（`full` vs `e1_2d` 2D-only）**：`mine` 2D 覆盖率均值 `0.646 -> 0.951`（+47.2%）、`n_auc` `0.392 -> 0.590`（+50.5%）、3D `completeness_10cm` `0.350 -> 0.430`（峰值 `0.485`，+22.9%）、`completeness_15cm` `0.461 -> 0.566`（峰值 `0.647`，+22.8%）、`fscore_15cm` `0.629 -> 0.716`（峰值 `0.785`）、`chamfer_l1_m` `1.371 -> 0.283 m`（最优 `0.079 m`，**-79.3%**）；`hospital` 5 轮覆盖率均值 **`0.997`**（4/5 轮 `1.000`），240 s `n_auc` `0.721 -> 0.760`、`chamfer_l1_m` `0.066 -> 0.062 m`、`occupied_iou_tol1` `0.428 -> 0.648`（+51.4%）、ATE `0.914 -> 0.306 m`（-66.5%）。
- [x] **M4（代码、单测与 5090 4 核实跑验收已全部通过）：参数化 $N=1\sim 4$ 启动包 `src/hier_explore_bringup/` + 导入 CMU `cmu_indoor` / `cmu_tunnel` 世界**
  - **WP4.1（参数化 ROS 2 启动包 `src/hier_explore_bringup/`）**：新增标准 `ament_cmake` + `ament_cmake_python` 包，实现 `launch/multi_sim.launch.py`（支持 `num_robots=1..4`、`world`、`spawn_seed`、`odom_mode=ekf|gt`、`ablation_profile=legacy|e1_2d|e2_3dgain|e3_hier|e4_geodesic|e5_deconflict|full|no_se2|no_degen_gate`、`planner_backend=auto|coordinator|legacy|off`、`enable_cameras=false` 节能默认开关、按车错峰启动）；扩充 `config/nav2_multirobot_slam_params.yaml` 支持 `robot1..robot4` `slam_toolbox` 参数块，并在 `run_benchmark.py` 中接入 `--num-robots`、`--odom-mode`、`--launcher launch|bash`。
  - **WP4.2（CMU AEDE 官方网格导入与全 6 场景真值档案）**：从 CMU `HongbiaoZ/autonomous_exploration_development_environment` 下载官方 `indoor.dae`（211,832 三角面）与 `tunnel.dae`（13,880 三角面，保留原偏移 `(-22.0, 92.5, 0)`），新增 [scripts/fetch_cmu_worlds.py](file:///home/liangqingcai/ros/ROS2-Navigation/scripts/fetch_cmu_worlds.py)、[worlds/cmu_indoor.sdf.xacro](file:///home/liangqingcai/ros/ROS2-Navigation/worlds/cmu_indoor.sdf.xacro)、[worlds/cmu_tunnel.sdf.xacro](file:///home/liangqingcai/ros/ROS2-Navigation/worlds/cmu_tunnel.sdf.xacro) 与 [scripts/generate_all_gt_maps.py](file:///home/liangqingcai/ros/ROS2-Navigation/scripts/generate_all_gt_maps.py)；增强 `sample_gt_mesh.py` 的 COLLADA `<visual_scene>` 节点变换解析与 `model://` 搜索路径，上调 `octomap_params.yaml` 的 `sensor_model.max_range` 至 `500.0 m`，生成全部 6 场景 `maps/gt/<world>/` 真值档案（零边界泄漏：`house` $76.13\text{ m}^2$、`hospital` $150.44\text{ m}^2$、`mine` $280.09\text{ m}^2$、`warehouse` $1370.35\text{ m}^2$、`cmu_indoor` $2667.21\text{ m}^2$、`cmu_tunnel` $13559.88\text{ m}^2$）与经 `gt_2d.npz` 净空校验的 4 车默认编队及 9–10 基站随机出生池；顺带将 `/<ns>/teammates_cloud` 改为在 `base_footprint` 坐标系发布，消除大场景远距局部代价图射线原点越界警告。
  - **已通过离线单测与 5090 4 核 ROS 2 Lyrical 实跑验收**：
    - `colcon build --symlink-install` 全 7 包零错通过；`colcon test` 全 4 包 **68/68 通过（0 失败）**；Python `pytest` **115 passed in 2.54s**（含全部 6 场景 `gt_2d.npz` 4 车无重叠净空校验与 $N=1\sim 4$ 确定性采样测试）。
    - 5090 4 核隔离容器 `ros2 launch hier_explore_bringup multi_sim.launch.py` 6 组 120 s 实跑全通：
      - **`mine` ($N=1$, `full`, `ekf`)**：探索 `121.3 m²`（`43.3%`），行程 `21.6 m`，ATE `0.016 m`，`CycP50/P95 = 1.84 / 32.88 ms`，`0` 碰撞；
      - **`mine` ($N=3$, `full`, `ekf`)**：3 车全活跃（`23.3 / 18.6 / 17.8 m`），探索 `143.2 m²`（`51.1%`，`Comp15 = 0.364`），渐进 DAG SE(2) 接受 `107` 次，`0` 碰撞；
      - **`mine` ($N=4$, `full`, `ekf`)**：4 车在 4 核容器内全部平稳驱动（`18.7 / 19.8 / 22.0 / 19.0 m`，总行程 `79.5 m`），探索 `146.2 m²`（`Comp15 = 0.380`, `IoU2D = 0.626`），渐进 DAG SE(2) 接受 `167` 次，`0` 碰撞；
      - **`cmu_indoor` ($N=2$, `full`, `ekf`)**：双车正常探索 CMU 官方建筑（`24.5 / 37.0 m`），120 s 建图 `292.0 m²`，ATE `0.198 m`，`CycP50/P95 = 3.83 / 31.93 ms`，SE(2) 接受 `55` 次，`0` 碰撞；
      - **`cmu_tunnel` ($N=2$, `full`, `ekf`)**：双车驶入 CMU 官方地下隧道（`16.6 / 24.6 m`），120 s 建图 `161.4 m²`，ATE `0.291 m`，`CycP50/P95 = 2.84 / 42.11 ms`，SE(2) 接受 `51` 次，`0` 碰撞；
      - **`house` ($N=2$, `legacy`, `gt`)**：双车覆盖率 `100.0%`，ATE `0.000 m`，`IoU2D = 0.771`，`0` 碰撞。
- [x] **M5：5090 服务器全量多随机种子 Benchmark + 消融 + M-TARE Docker 对照 + 论文矢量图与 LaTeX 表格生成**（已完成，`faefd2e`，239/239 轮 4 核实跑全通，产出 `docs/paper_artifacts/` 与初稿 `docs/paper_draft.md`）
- [ ] **M6：单台 3D 雷达实车部署（`real_robot.launch.py`）+ 真机 2D-only vs. Ours 对比实验 + 补充视频** ← **下一个立即执行的里程碑**

---

## 三、待执行里程碑详细规格（M2 – M6）

### M2（WP2）：定位去作弊 + C++ `src/se2_map_merge/`
1. **恢复真实噪声轮式里程计 + IMU EKF**：
   - 修改 `config/gz_waffle_3d_lidar.sdf.xacro`，增加 `<xacro:arg name="use_gt_odom" default="false"/>`：
     - `use_gt_odom=false`（默认）：启用 `gz::sim::systems::DiffDrive` 的轮式里程计输出到 `$(arg namespace)/wheel_odom`（不直接发 `odom->base_footprint` TF），保留 `OdometryPublisher` 但仅发布真值到 `$(arg namespace)/gt_pose`（关闭其 TF 广播，仅供 `bench_recorder.py` 算 ATE/RPE）；
     - 新增 `config/ekf_odom.yaml`，每台车启动 `robot_localization` (`ekf_node`) 融合 `wheel_odom` + `imu`，发布平滑 `/robot_i/odom` 与 `odom -> base_footprint` TF；
     - `use_gt_odom=true`：仅作为与外部 SOTA 对比时的理想定位对照模式。
2. **新建 C++ ROS 2 包 `src/se2_map_merge/`**：
   - `include/se2_map_merge/phase_corr.hpp` + `src/phase_corr.cpp`：重叠区占据边缘提取 + Hanning 窗 2D Radix-2 FFT 相位相关平移粗配准（响应置信度门控 $\rho_{\text{pc}} > \tau_{\text{pc}}$）。
   - `include/se2_map_merge/icp_2d.hpp` + `src/icp_2d.cpp`：基于 2D KD-Tree + Huber 鲁棒核的点到线（Point-to-Line）ICP 精配准；对 $3\times 3$ 近似 Hessian $H = \sum J_k^\top J_k$ 做特征值分解，检测矿井/长直巷道退化方向（$\lambda_{\min}(H_{tt}) < \tau_{\text{deg}}$）并执行 **Solution Remapping**（仅更新非退化横向与航向分量，冻结退化纵向分量）；加入 IoU 单调增益校验、步进限幅（每周期 $|\Delta t|\le 0.05\text{ m}, |\Delta\theta|\le 1^\circ$）与李代数指数平滑（$\alpha=0.25$）。
   - `src/se2_map_merge_node.cpp`：订阅 `/robot_1/map`..`/robot_N/map`，以 `robot1` 为基准渐进 DAG 在线估计并广播 `world -> robot_i/map` TF，中继 `/<ns>/tf` 与 `/<ns>/tf_static`，合并发布 `/map`（`transient_local + reliable`），并可选为每车发布局部对齐的 `/robot_i/shared_map` 与诊断话题 `/map_merge/alignment_diagnostics`。
   - `src/cloud_merger_node.cpp`：用 C++ 重写 `scripts/merge_pointclouds.py`（支持任意 $N$ 车、零拷贝解析 `PointCloud2`、自反框过滤、TF 时钟抖动与位姿跳变拒绝、uint64 打包排序体素降采样 0.02 m，发布 `/points_merged`）。
   - **M2 验收门禁**：
     - [x] C++ 单元测试覆盖：合成旋转 $5^\circ$ + 平移 $1.2\text{ m}$ 的 2D 栅格恢复精度 $< 2\text{ cm}, < 0.3^\circ$（实测 `0.016 cm / 0.0012°`）；长直平行墙走廊退化场景下纵向漂移为 $0$ 且横向/旋转正确收敛（实测 `0.00 cm / 0.009 cm / 0.003°`）；
     - [x] 5090 服务器 4 核容器内运行 `mine` 与 `warehouse`（`use_gt_odom=false`），对比原 `map_merge`，合并图 `occupied_iou_tol1`（`mine`: `0.635 -> 0.869`）、ATE（`mine`: `0.207 -> 0.045 m`, `warehouse`: `0.350 -> 0.049 m`）与同高度带 3D `chamfer_l1_m`（`mine`: `0.1334 -> 0.1277 m`, `precision_10cm`: `0.943 -> 0.988`）显著提升，C++ `cloud_merger_node` CPU 占用较原 Python 版降低 **84.4%（`mine`）/ 85.1%（`warehouse`）**（远超 $\ge 50\%$）。

### M3（WP3）：C++ `pybind11` 几何核 + 共享 `/map` + 窄道协同避碰
1. **在 `src/hier_explore_core/cpp/` 新增 `pybind11` 模块 `_geom_kernel`**：
   - `geodesic_grid.cpp`：给定安全自由栅格与源点列表，有界多源 Dijkstra 计算精确测地距离矩阵 $D_{\text{geo}}(a,b)$ 与路径点列（500×600 栅格目标 $< 6\text{ ms}$）；注入 `scheduler.py` 替换欧氏距离。
   - `visibility_fov.cpp`：针对每个 3D 残差聚类，在 $[0.8, 3.5]\text{ m}$ 环带采样 $K=24$ 个安全视点，执行 2D Bresenham 墙体遮挡射线检测 + 垂直视场角 $[\phi_{\min}, \phi_{\max}]$（默认 $[-30^\circ, +55^\circ]$，可参数化适配实车雷达）+ 有效观测距离 $R_{\text{eff}} \le 5.0\text{ m}$ 校验，贪心次模最大化覆盖残差柱集合。
   - `corridor_graph.cpp`：基于 `obstacle_dt` 提取通道半宽 $w(p) < 2 r_{\text{robot}} + 0.35\text{ m}$ 的窄道瓶颈段 $E_{\text{narrow}}$ 及其两端宽敞礼让点（Yield Points）。
2. **共享全局代价图与双层车际避碰**：
   - `config/nav2_multirobot_slam_params.yaml`：`global_costmap.static_layer` 改为订阅全局合并图（`/map` 或 `/robot_i/shared_map`），彻底解决 TODO #12；
   - `src/hier_explore_core/hier_explore_core/deconfliction.py`：5 Hz 广播 `/robot_i/teammates_cloud`（其余车辆足迹圆 + 未来 2 s 规划轨迹采样点）注入各车 `local_costmap.voxel_layer`；
   - 在 `scheduler.py` 接入拓扑窄道互斥预约（Corridor Mutex Reservation）：低优先级车进入被占瓶颈段时自动暂停于礼让点或切换另一分支前沿，消除矿井 3.2 m 窄巷与医院门洞对顶死锁。
   - **M3 验收门禁**：
     - [x] C++ 核 + Python 单测全通过（`test_geom_kernel` 3/3 + `test_se2_map_merge` 4/4 + `pytest` 80/80 全通过）；
     - [x] 4 核容器下 `mine`（连续 5 种子 `0..4`）与 `hospital`（连续 5 种子 `0..4`）零对向死锁（`stall_events = 0`）、零车际碰撞（`inter_robot_collisions = 0`）；规划周期 P95 $< 50\text{ ms}$（`mine` 5 轮均值 `44.88 ms` / P50 `4.21 ms`，`hospital` 5 轮均值 `47.67 ms` / P50 `4.61 ms`）；3D 表面完整度较 2D-only 显著提升（`mine` `completeness_15cm` `0.461 -> 0.566` / 峰值 `0.647`，`chamfer_l1_m` `1.371 -> 0.283 m`）。

### M4（WP4）：参数化 $N=1\sim 4$ 启动与 CMU 公开场景（`src/hier_explore_bringup/`）
- [x] 编写 `multi_sim.launch.py`（支持 `num_robots=1..4`, `world`, `spawn_seed`, `odom_mode`, `ablation_profile`, `planner_backend`）替换硬编码 2 车 Bash，并在 5090 4 核容器实测通过 $N=1,2,3,4$ 与 `legacy+gt` 模式；
- [x] 从 CMU Autonomous Exploration Development Environment 导入 `cmu_indoor` 与 `cmu_tunnel` 网格封装为 Gazebo SDF，并生成 6 场景全部 `maps/gt/<world>/` 真值档案与经净空校验的 4 车无重叠出生点池（`cmu_indoor` / `cmu_tunnel` 实跑均通过，0 碰撞、`CycP95 < 50 ms`）。

### M5（WP5）：5090 服务器全量实验与论文图表输出
- [x] 完成 5090 4 核边缘算力、严格同速 `0.5 m/s` 约束下全部 **239 轮隔离基准实验（v2）**（229 轮 ROS 2 + 10 轮官方 CMU M-TARE `0.5 m/s` 同速对比，**239/239 零失败**）：
  - **Table I, Fig. 3 & Fig. 3b（主多场景对比 + 边缘帕累托分析，130 轮）**：6 场景 × 5 种子，统一消除启动预热偏差并与官方 M-TARE 拉平至同速 `0.5 m/s`；在退化矿井 `mine` 上 Lite-HMRE 全面领先全部 ROS 2 基线（面积 **`275.6 ± 14.2 m²`**、3D Comp@15cm **`65.5 ± 6.9%`**、Chamfer-$L_1$ **`0.207 m`** [-67.6% vs. Legacy]、Occ. IoU **`0.811`** [$p=0.044$]、P95 **`49.8 ms`**）；相较同速官方 M-TARE，峰值内存降低 **49.2%–52.3%**（`2.61–3.02 GB` 稳居 4 GB 边缘单板机可行域 vs. M-TARE `5.48–5.94 GB`）、规划 P95 快 **4.0×–6.3×**（`139–195 ms` vs. `777–876 ms`）、`cmu_tunnel` 覆盖效率 CE 反超 **+8.8%**（`4.97` vs. `4.57 m²/m`），全 6 场景保持 **0.0 车际碰撞**；
  - **Table II & Fig. 5a–b（7 步递进消融 `E0`–`E6`，70 轮含复用）**：`mine` 3D Comp@15cm 由 `53.5%` 单调攀升至 **`65.5%`**（+12.0 pp）、重叠率降 **73.7%**；`hospital` 覆盖效率 CE 由 `1.91` 升至 **`2.97 m²/m`**（+55.5%）、路径重叠率由 `21.7%` 压降至 **`0.9%`（-95.9%）**；
  - **Table III & Fig. 2（噪声里程计 SE(2) 配准与退化门控专项，50 轮含复用）**：引入重叠边缘质量门槛（`min_overlap_points=150, min_inlier_ratio=0.55, max_rmse_m=0.12, iou_tolerance=0.002`）后，Lite-HMRE 在 `mine`（`0.046 m`，较无退化门控降 **41.0%**）与 `warehouse`（`0.052 m`，较 `No SE(2)` 降 **23.5%**、较 Legacy 降 **27.8%**）双双取得噪声里程计最低 ATE，C++ `cloud_merger_node` CPU 降低 **84.4%–85.6%**；
  - **Table IV & Fig. 4（多机扩展性 $N=1\sim 4$，40 轮含复用）**：`cmu_indoor` $N=1\to 4$ 面积加速比 **2.61×**、3D 完整度加速比 **2.92×**，4 机峰值内存（`5.52 GB`）与 P95（`276 ms`）仍低于双机 M-TARE（`5.80 GB`, `876 ms`），全 40 轮 **0 碰撞**；
  - **Table V & Fig. 5c（16/32 线与体素分辨率敏感性，12 轮含复用）**：默认 `16 线 / 0.10 m` 取得最优边缘平衡（`280.6 m²`, `66.7%` 3D Comp, `46.2 ms` P95），`0.05 m` 进一步将 Chamfer-$L_1$ 压至 `0.124 m`，`32 线` 峰值 3D 完整度达 `77.3%` 且内存几乎零增长；
- [x] 全自动产出 5 组 IEEE 矢量图与 5 张 Booktabs LaTeX 表格于 `docs/paper_artifacts/`（含新增 `fig3b_edge_pareto.{pdf,png}`），并完成中英文论文完整初稿（`docs/paper_draft.md` & `docs/paper_draft_zh.md`）。

### M6（WP6）：单台 3D 雷达实车验证
- 编写 `src/hier_explore_bringup/launch/real_robot.launch.py`（`pointcloud_to_laserscan` + `slam_toolbox` + `cloud_merger_node` + `octomap_server` + Nav2 + `hier_explore_core` 单机模式，参数化实车雷达垂直 FoV）；
- 使用同一 `docker/Dockerfile.ros2` 部署到实车，在真实杂物室内/走廊运行 2D-only vs. Ours 各 3 轮，产出 Fig. 5 与论文补充视频。

---

## 四、5090 服务器快速验证命令（M2 实机编译与 Gazebo 验收）

```bash
# 1. 拉取最新 M2 代码
git fetch origin
git checkout paper/hier-25d3d-explore
git pull --ff-only origin paper/hier-25d3d-explore

# 2. 宿主机或 Docker 内编译全部 ROS 2 包（含新增 C++ se2_map_merge）并跑 C++ + Python 全部单测
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select \
  explore_lite_msgs explore_lite multirobot_map_merge se2_map_merge hier_explore_core hier_explore_bench
source install/setup.bash

colcon test --packages-select se2_map_merge --event-handlers console_direct+
PYTHONPATH=src/hier_explore_core:src/hier_explore_bench:scripts pytest -v \
  src/hier_explore_core/test src/hier_explore_bench/test tests/test_m2_configs.py \
  scripts/test_exploration_3d.py scripts/test_cloud_stamp.py

# 3. 预生成 mine / house / hospital 真值 3D 点云与 2D 可探索地图
for entry in "mine:-19.0 0.0" "house:-3.0 -2.0" "hospital:-6.0 0.0"; do
  w="${entry%%:*}"; s="${entry#*:}"
  PYTHONPATH=src/hier_explore_bench python3 -m hier_explore_bench.sample_gt_mesh \
    --world "worlds/${w}.sdf.xacro" --seed ${s} --out-dir "maps/gt/${w}"
done

# 4. M2 Gazebo 噪声里程计 + EKF + C++ SE(2) 配准实跑验收（默认 USE_GT_ODOM=false）
#    先单次验收矿井与仓库（观察 /tmp/tb3_multi_robot*_ekf.log, /tmp/tb3_multi_map_merge.log,
#    ros2 topic echo /map_merge/alignment_diagnostics）
HEADLESS=True USE_RVIZ=False DASHBOARD=False WORLD=mine ./scripts/tb3_multi_explore_slam.sh

# 5. A/B 对照：旧 map_merge + Python 点云合并 vs 新 C++ se2_map_merge + C++ cloud_merger_node
#    （在 4 核 8 GB Docker 中分别测 1 轮 mine & warehouse，对比 occupied_iou_tol1, chamfer_l1_m, CPU%）
USE_GT_ODOM=false USE_SE2_MERGE=false USE_CPP_CLOUD_MERGER=false \
  python3 -m hier_explore_bench.run_benchmark \
  --worlds mine warehouse --methods full --seeds 0 --max-sim-sec 240 --out-dir benchmark_results/m2_legacy
USE_GT_ODOM=false USE_SE2_MERGE=true USE_CPP_CLOUD_MERGER=true \
  python3 -m hier_explore_bench.run_benchmark \
  --worlds mine warehouse --methods full --seeds 0 --max-sim-sec 240 --out-dir benchmark_results/m2_se2
```
