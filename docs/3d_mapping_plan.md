# TurtleBot3 从 2D LiDAR 到 3D 点云建图（仅 Gazebo 仿真）

## Context

当前项目是 ROS 2 **Lyrical** + **GZ Sim 10.5** + Nav2 1.5.2 + slam_toolbox 的双 TurtleBot3 仿真，全部感知/建图/探索链路都是 2D：

- 机器人默认 SDF：`/opt/ros/lyrical/share/nav2_minimal_tb3_sim/urdf/gz_waffle.sdf.xacro`
  - 单线 `gpu_lidar`，360 线/圈、5 Hz，发布 `/robotX/scan`
  - 已有 IMU 和深度相机模型，但 IMU 未融合、深度相机未桥接
- `slam_toolbox` 用 `/robotX/scan` 建 2D `OccupancyGrid`
- `multirobot_map_merge` 用已知初始位姿把 `/robot1/map`、`/robot2/map` 合成 `world` 下的 `/map`
- `explore_lite` / `multi_robot_coordinator.py` 在 2D 栅格上提 frontier、竞拍、发 Nav2 goal
- Nav2 的 local `voxel_layer` / global `obstacle_layer` 只吃 `LaserScan`
- 场景是 `worlds/house.sdf.xacro`、`worlds/hospital.sdf.xacro`：单层平地，无楼梯/坡道/天花板，有桌、床、柜、椅等 3D 家具

用户目标（已确认）：

1. **仅 Gazebo 仿真**，不涉及实车驱动、供电、标定
2. 3D 地图用于**展示 + 存档点云**，并把桌/床等三维结构**投影进 Nav2 的 2D 代价地图**
3. 精度要求不高，**不用于后续导航/重定位**
4. 场景只使用现有家具，不新增瓦砾/废墟
5. **双机器人保留，合并成一张 3D 图**
6. 3D 点云位姿**沿用现有 slam_toolbox**，不换 3D SLAM

**实施范围：只改双机链路。** 单车脚本和单车参数（`tb3_explore_slam.sh`、`tb3_slam_gazebo.sh`、`tb3_nav_gazebo.sh`、`nav2_params_vmware.yaml`、`nav2_params_rl.yaml`）保持原样，不纳入本次改动。

**运行/验收环境：无独立显卡，全程无头模式。** 验收时设置 `HEADLESS=True`、不启动 RViz、不启动 Gazebo GUI；所有检查通过 ROS 2 CLI、TF、topic 数值和存档文件离线完成。为避免 VMware 软件渲染把宿主机拖死，3D LiDAR 默认降到 16 线、3 Hz（有独显时再改回 24 线、5 Hz）。本计划的完整版本保存在 `/home/qing/.claude/plans/turtlebot3-2d-lidar-lds-01-02-breezy-hickey.md`；批准后第一步会复制到仓库内 `docs/3d_mapping_plan.md`，作为长期文档。

## 先回答三个核心问题

### 1. 建图算法需要更新吗？

**不需要替换 slam_toolbox，也不需要引入 FAST-LIO/GLIM/RTAB-Map。** 现有 2D SLAM 继续负责：

- `map -> odom -> base_footprint` 位姿
- 2D 栅格地图
- frontier 探索与 Nav2 规划

新增一条**并行的 3D 建图层**：3D LiDAR → 点云桥接 → 双机点云合并 → OctoMap。3D 点云直接使用 slam_toolbox 已经给出的位姿做累积。因为地面绝对平坦、只考虑已知初始位姿，2D 位姿的 z/roll/pitch 由轮式里程计和地面约束保证，足以支撑低精度 3D 点云。

### 2. 三维建图的前端/探索怎么办？

**继续用 2D frontier，不做 3D frontier/NBV。** 理由：

- TurtleBot3 是地面机器人，可通行空间本质上是 2.5D；3D 未知体素大多在机器人不可达的高度
- 现有 `explore_lite` / 竞拍协调器在 2D 上成熟稳定，改 3D 前端会同时动探索、调度、Nav2 goal 三个层面
- 3D frontier/NBV 在 ROS 2 Lyrical 上没有成熟可直接用的包；`fkie-nbv-planner` 只有 ROS 1，`ADVRHumanoids/robot_exploration` 可作未来参考但依赖较重
- 3D LiDAR 是 360° 的，机器人探索过程中自然会扫到家具；必要时在终点加一次原地旋转即可补盲

### 3. 二维图建好，三维图就算建好了吗？

**不等于。** 2D frontier 清空只说明“地面上没有未知边界”，3D 图仍可能有：

- 家具背面/桌下的遮挡空洞
- 垂直 FOV 外的桌面、床面
- 两车 2D 地图拼合误差带来的重影

因此需要单独的 3D 完成判据（见“验收标准”），而不是把 `explore_lite` 结束当作 3D 建图完成。

## 目标架构

```text
GZ Sim (每台 TB3)
  ├─ 2D LDS  ── /robotX/scan ────────► slam_toolbox ──► /robotX/map
  │                                                    │
  └─ 3D LiDAR ─ /robotX/points ──┬──► Nav2 local voxel_layer
                                 │    Nav2 global obstacle_layer   (3D→2D 投影)
                                 │
                                 └──► merge_pointclouds.py
                                        │  TF: robotX/base_scan -> world
                                        ▼
                                  /points_merged (world)
                                        │
                                        ▼
                                  octomap_server
                                   ├─ /octomap_point_cloud_centers  (展示/存档)
                                   ├─ /octomap_binary / /octomap_full
                                   └─ /projected_map                (2D 预览/存档)
```

- **2D 栈完全不动**：SLAM、frontier、竞拍、Nav2 全局规划仍用 `/robotX/scan` 和 `/map`
- **3D 只做加法**：多一颗 3D 雷达、一个点云桥接、一个双机点云合并节点、一个 `octomap_server`
- **3D→2D 代价图**由 Nav2 自带的 `VoxelLayer`/`ObstacleLayer` 直接吃 `PointCloud2` 完成，不额外写投影节点

## 关键设计决策

| 决策 | 选择 | 原因 |
|---|---|---|
| 实施范围 | **只改双机链路** | 单车脚本/参数保持原样，避免影响已有单机流程 |
| 2D 传感器 | **保留现有 LDS**，另加 3D LiDAR | 对现有 SLAM/frontier 零风险；仿真里多一个传感器没有成本 |
| 3D 雷达模型 | GZ 内置 `gpu_lidar` + vertical scan，默认 16 线、360 水平、3 Hz | 无独显 + 软件渲染下控制负载；有独显时再升到 24 线 / 5 Hz |
| 验收方式 | **纯无头 CLI + 存档文件**，不启动 RViz | 避免 VMware 软件渲染把电脑拖死 |
| 3D 建图 | `octomap_server`（ROS2 分支，apt 可直接装） | 成熟、轻量、直接给 `projected_map` 和 `.bt` 存档 |
| 点云合并 | 自写 `merge_pointclouds.py` | `octomap_server` 只有单路 `cloud_in`；需要把两车点云变换到 `world` 后拼接 |
| 3D→2D 代价图 | Nav2 `VoxelLayer`/`ObstacleLayer` 的 `PointCloud2` source | 原生支持，无需额外投影节点；局部和全局代价图都能看到桌/床 |
| 地面处理 | `point_cloud_min_z` / `occupancy_min_z ≈ 0.08 m` | 不滤地面的话，`projected_map` 会把整个地板标成占据 |
| 探索策略 | 保持 2D frontier | 地面机器人 + 平地，3D NBV 收益低、风险高 |
| 3D 完成判据 | 2D frontier 完成 + 3D 体素增长收敛 + 无头指标检查 | 2D 完成 ≠ 3D 完成 |

## 实施步骤

### Phase 0：依赖与基线

1. 安装 3D 建图依赖（Lyrical 源里已有）：

   ```bash
   sudo apt install -y ros-lyrical-octomap-server ros-lyrical-octomap-msgs
   # 可选，点云滤波时用：
   sudo apt install -y ros-lyrical-pcl-ros
   ```

2. 跑一次现有双机 2D 基线并留档（无头，不开 RViz）：

   ```bash
   HEADLESS=True USE_RVIZ=False WORLD=house ./scripts/tb3_multi_explore_slam.sh
   ```

   记录探索完成时间、`/map` 覆盖率、CPU/内存占用，作为 3D 改造后的回归基准。另开一个终端监控资源：

   ```bash
   free -h
   top -b -d 2 | grep -E "gz sim|ruby|python3|octomap|nav2" | head
   ```

   如果基线本身就接近内存上限，先减少 Gazebo Fuel 模型或缩短探索时间，再进入 Phase 1。

3. 确认 GZ 能发布 3D 点云（最小验证，仍为无头）：

   ```bash
   gz topic -l | grep -i lidar
   ros2 topic list | grep points
   ```

### Phase 1：给 TB3 加 3D LiDAR（仿真）

**新增** `config/gz_waffle_3d_lidar.sdf.xacro`：

- 复制 `/opt/ros/lyrical/share/nav2_minimal_tb3_sim/urdf/gz_waffle.sdf.xacro`
- 在 `<link name="base_scan">` 内、现有 2D `hls_lfcd_lds` 传感器之后，增加第二个 `gpu_lidar`：

```xml
<sensor name="tb3_3d_lidar" type="gpu_lidar">
  <always_on>true</always_on>
  <visualize>false</visualize>
  <!-- SDF 的 fixed joint 不会把 base_scan link 移到关节位姿：GZ 中该 link
       仍在模型原点（用 /world/default/pose/info 实测）。robot_state_publisher
       按 URDF scan_joint 把 base_scan 放在 (-0.064, 0, 0.122)，所以这里必须
       显式写同一偏移，让 GZ 传感器原点与 ROS TF 的 base_scan 重合。 -->
  <pose>-0.064 0 0.122 0 0 0</pose>
  <update_rate>3</update_rate>       <!-- 无独显默认 3 Hz；有独显可升到 5 -->
  <topic>$(arg namespace)/lidar3d</topic>
  <frame_id>base_scan</frame_id>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1.0</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>         <!-- 无独显默认 16 线；有独显可升到 24 -->
        <resolution>1.0</resolution>
        <min_angle>-0.5236</min_angle>   <!-- -30°，看低矮物/地面 -->
        <max_angle>0.9599</max_angle>    <!-- +55°，看桌面/床面 -->
      </vertical>
    </scan>
    <range>
      <min>0.10</min>
      <max>12.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </ray>
</sensor>
```

说明：

- GZ 的 `gpu_lidar` 会把点云发布在 `<topic>/points`，即 `/robotX/lidar3d/points`
- 如果 `<ray>` 的 `<vertical>` 不生效，改用 SDF 1.9+ 的 `<lidar><scan><vertical>…` 写法
- 默认 16 线 × 360 点 × 3 Hz ≈ 17k 点/s/车，双车约 35k 点/s；如果仍然卡顿，先降到 12 线或 240 水平点，再加回
- 传感器挂在现有 `base_scan` link 上，**不需要改 URDF/TF**；2D 和 3D 共用 `base_scan` 坐标系
- **传感器 pose 必须显式写 `-0.064 0 0.122`**。本 SDF 的 fixed joint 只是运动学连接，GZ 中 `base_scan` link 实际停在模型原点；如果 pose 写 0，GZ 传感器原点会比 ROS TF 的 `base_scan` 低 12.2 cm、后 6.4 cm，点云变换到 `world` 后会整体偏移。已用已知几何验证：2 m 处 1 m 立方体的前表面应出现在局部 x≈1.564 m，地面应在 z≈−0.13 m，实测均吻合（误差 < 1 cm）
- 无头模式下 `visualize` 保持 `false`，不要打开 Gazebo GUI 或 RViz

**新增** `config/tb3_points_bridge.yaml`：

```yaml
- ros_topic_name: "points"
  gz_topic_name: "lidar3d/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS
```

**修改** `config/tb3_bridge_no_clock.yaml`：追加同样的 `points` 条目（robot2 使用这份配置）。

**修改** `scripts/tb3_multi_explore_slam.sh`：

- `SDF_XACRO` 从安装路径改为 `${ROOT_DIR}/config/gz_waffle_3d_lidar.sdf.xacro`
- robot1 的 `tb3_simulation_launch.py` 参数里增加 `robot_sdf:="${SDF_XACRO}"`
- robot1 额外启动一个 points bridge（robot2 的 bridge 已经在 `tb3_bridge_no_clock.yaml` 里）：

```bash
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -r __ns:=/robot1 \
  -p config_file:="${ROOT_DIR}/config/tb3_points_bridge.yaml" \
  -p use_sim_time:=true \
  -p expand_gz_topic_names:=true &
```

**验证（无头）**：

```bash
ros2 topic hz /robot1/points
ros2 topic hz /robot2/points
ros2 topic echo /robot1/points --field header.frame_id --once
ros2 topic echo /robot1/points --field width --once   # 每帧点数应接近 360 × 16
ros2 run tf2_ros tf2_echo world robot1/base_scan
```

以上命令都不需要 RViz/Gazebo GUI；如果 `ros2 topic hz` 明显低于 3 Hz，先降线数/采样率，不要继续加功能。

### Phase 2：3D 点云进 Nav2 的 2D 代价地图

**修改** `config/nav2_multirobot_slam_params.yaml`（top-level 的 local/global costmap 会被 Nav2 自动重写到 `/robot1`、`/robot2` 命名空间）：

local `voxel_layer`：

```yaml
voxel_layer:
  observation_sources: scan points
  z_resolution: 0.05
  z_voxels: 40                 # 0.05 × 40 = 2.0 m，覆盖 1.0 m 桌面
  scan: { ... 保持原样 ... }
  points:
    topic: points
    data_type: "PointCloud2"
    min_obstacle_height: 0.05
    max_obstacle_height: 2.0
    clearing: true
    marking: true
    raytrace_max_range: 6.0
    raytrace_min_range: 0.2
    obstacle_max_range: 5.0
    obstacle_min_range: 0.2
```

global `obstacle_layer` 同样增加 `points` source。

说明：

- **必须改 `z_voxels`**：当前 local `VoxelLayer` 是 `z_resolution: 0.05`、`z_voxels: 16`，只有 0.8 m 高，会把 1.0 m 的桌面裁掉；改成 40 后覆盖到 2.0 m。全局 `ObstacleLayer` 是 2D 投影，不受此限制
- `topic: points` 是相对名，Nav2 会解析成 `/robotX/points`
- `min_obstacle_height: 0.05` 滤掉地面和噪声；`max_obstacle_height: 2.0` 覆盖墙和家具
- 桌子/床的**完整投影**会让其下方不可通行。如果以后想让小车钻桌底，把 `min_obstacle_height` 改成 `0.05`、`max_obstacle_height` 改成机器人高度（约 0.25 m）即可；3D 点云里仍然保留桌面
- `mark_threshold` 先保持现有值；如果点云噪声导致代价图毛刺，再改成 1 或加 `pcl_ros` VoxelGrid 滤波
- 本次每台车只吃自己的 `/robotX/points`；跨车家具信息共享仍受现有 global costmap 静态层来源限制（README TODO #12），不在本次范围。3D 地图本身是两车合并的

**验证（无头，不开 RViz）**：

```bash
ros2 topic hz /robot1/global_costmap/costmap
ros2 topic hz /robot1/local_costmap/costmap
```

暂停探索，手动把 robot1 派到桌子附近，再用检查脚本读取代价图：

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: world}, pose: {position: {x: <table_x>, y: <table_y>, z: 0.0}, orientation: {w: 1.0}}}}"

# 无头检查脚本（costmap 模式在 Phase 2 实现，其余模式在 Phase 4 补齐）
python3 scripts/check_3d_map.py costmap \
  --robot robot1 --world-point <table_x>,<table_y> \
  --expect-lethal
```

检查脚本从 `/robot1/global_costmap/costmap` 读取栅格，并用 TF 把 `world` 下的桌面坐标换算到 `robot1/map`，确认该格为 lethal。其他确认项：

- 桌面/床面投影区域出现 lethal cell；没有 3D 点时同一区域不是 lethal（可切换 `points` source 做 A/B）
- Nav2 全局/局部规划绕开桌子/床，不穿越家具投影
- 2D frontier 探索仍能正常结束（恢复 `/explore/resume` 后继续）

### Phase 3：双机合并 3D 点云 + OctoMap + 存档

**新增** `scripts/merge_pointclouds.py`（独立 rclpy 节点，和现有 `multi_robot_coordinator.py` 风格一致）：

职责：

1. 订阅 `/robot1/points`、`/robot2/points`（`SensorDataQoS`）
2. 把每帧点云的 `header.frame_id` 从 `base_scan` 规范成 `robotX/base_scan`
   - `map_merge` 会把各车 TF 前缀成 `robotX/...`，全局 TF 树里是 `robot1/base_scan`
3. 用 `tf2_ros` 查 `world -> robotX/base_scan`，把点云变换到 `world`
4. 按 `voxel_size`（默认 0.05 m）做体素降采样
5. 以 3 Hz（与 3D LiDAR 同频）发布 `sensor_msgs/PointCloud2` 到 `/points_merged`，frame_id = `world`
6. 可选：维护一个全局体素集合，低频发布累积点云 `/map_cloud`，并提供 `/save_map_cloud` 服务写 PCD（如果不做，直接用 `octomap_point_cloud_centers` 存档）

实现要点：

- 用 `sensor_msgs_py.point_cloud2.read_points_numpy` 取 `x/y/z`
- TF 查询失败时按 0.1 s 超时重试，不要阻塞订阅回调
- 点云 z 范围先裁到 `[0.05, 2.0]`，距离裁到 12 m
- 两车点云时间戳不同步，不需要 `message_filters`；各自保留最新一帧，定时拼接即可

**新增** `config/octomap_params.yaml`：

```yaml
octomap_server:
  ros__parameters:
    frame_id: world
    base_frame_id: base_footprint
    resolution: 0.05
    point_cloud_min_z: 0.08
    point_cloud_max_z: 2.0
    occupancy_min_z: 0.08
    occupancy_max_z: 2.0
    filter_ground_plane: false
    sensor_model.max_range: 12.0
    publish_free_space: false
    latch: false          # 避免每次更新都序列化整棵树；检查器用 volatile 订阅
    incremental_2D_projection: false
```

**修改** `scripts/tb3_multi_explore_slam.sh`：在 `map_merge` 启动后增加：

```bash
python3 "${ROOT_DIR}/scripts/merge_pointclouds.py" --ros-args \
  -p use_sim_time:=true \
  -p robots:="['robot1', 'robot2']" \
  -p world_frame:=world \
  -p voxel_size:=0.05 &

ros2 run octomap_server octomap_server_node --ros-args \
  --params-file "${ROOT_DIR}/config/octomap_params.yaml" \
  -p use_sim_time:=true \
  -r cloud_in:=/points_merged &
```

**新增** `scripts/save_cloud_pcd.py`：

- 订阅 `/octomap_point_cloud_centers`（volatile reliable QoS，匹配 `latch: false`）
- 用 `sensor_msgs_py.point_cloud2.read_points_numpy` 取 `x/y/z`
- 手写 ASCII PCD（`FIELDS x y z`，`DATA ascii`），不依赖 Open3D/PCL
- 用法：

```bash
python3 scripts/save_cloud_pcd.py \
  --topic /octomap_point_cloud_centers \
  --output maps/tb3_3d_map.pcd
```

**存档命令**（探索结束后、`octomap_server` 仍存活时执行）：

```bash
# 3D OctoMap 二进制树
ros2 run octomap_server octomap_saver_node --ros-args \
  -p octomap_path:="$PWD/maps/tb3_3d_map.bt" -p full:=true

# 3D 占据体素中心点云
python3 scripts/save_cloud_pcd.py \
  --topic /octomap_point_cloud_centers \
  --output maps/tb3_3d_map.pcd

# 2D 投影图（world 坐标系）
ros2 run nav2_map_server map_saver_cli -f maps/tb3_3d_projected \
  --ros-args -r map:=/projected_map \
  -p map_subscribe_transient_local:=false
```

**展示（不作为验收步骤）**：

- 本次验收全程无头，**不修改、不启动 RViz**，也不要求 Gazebo GUI
- 需要看 3D 效果时，在探索结束后把 `maps/tb3_3d_map.pcd` / `.bt` 复制到有图形界面的机器，用 CloudCompare、`octomap` 工具或 Foxglove 离线打开
- 如果后续确实要在本机看，再单独加 RViz 配置；它不属于本次实现和验收范围

### Phase 4：3D 完成判据与无头验收

**补齐** `scripts/check_3d_map.py`：无 GUI 的验收检查器，rclpy + `sensor_msgs_py` + `tf2_ros` + numpy，所有结果打印 JSON，便于写日志和对比。Phase 2 先实现 `costmap` 模式，本阶段补齐其余 3 个模式：

- `cloud`：订阅 `/points_merged` 指定时长，输出点数、xyz 包围盒、z 直方图（0.05–0.3 / 0.3–0.6 / 0.6–1.2 / 1.2–2.0 m）、0.05 m 体素数
- `growth`：按固定间隔采样 `/octomap_point_cloud_centers`，输出最近 30 s 的体素增长百分比
- `costmap`：订阅 `/robotX/global_costmap/costmap`，用 TF 把 `world` 下的指定点换算到 `robotX/map`，再按 `info.origin` 的平移+旋转换算成栅格索引，检查该格是否 lethal；用于验证桌面投影
- `pcd`：离线读取 ASCII PCD，输出点数、包围盒、z 直方图；不依赖 CloudCompare/RViz

**3D 完成判据**（建议默认值，可按实际调）：

1. 2D 侧：协调器报告无剩余可达 frontier（现有逻辑）
2. 3D 侧：`check_3d_map.py growth` 显示占据体素数在连续 30 s 仿真时间内增长 < 1%；小地图额外允许 ≤ 30 个体素的绝对波动（OctoMap 树剪枝/噪声会让小型饱和地图的计数上下浮动几个体素）
3. 自动覆盖侧：`check_3d_map.py cloud` 的点数/包围盒覆盖 house/hospital 的已探索区域；**离线抽查**（把 PCD 复制到有图形界面的机器）确认每张桌子的桌面、每张床的床面、每面墙都有点，单个空洞最大边长 ≤ 0.15 m
4. 高度侧：z 直方图在 0.3–1.2 m 有家具峰，0.0–0.08 m 地面点已被滤掉
5. 可选精度指标：以 Gazebo world 的 mesh/box 表面采样为参考，90% 参考点在 0.10 m 内能找到地图点；本次不做在线视觉检查

**无头验收流程**：

```bash
# 1. 启动（无 Gazebo GUI、无 RViz）
HEADLESS=True USE_RVIZ=False WORLD=house ./scripts/tb3_multi_explore_slam.sh

# 2. 探索完成后，另开终端检查点云与体素增长
python3 scripts/check_3d_map.py cloud \
  --topic /points_merged --duration 10 --json
python3 scripts/check_3d_map.py growth \
  --topic /octomap_point_cloud_centers --window 30 --json

# 3. 检查桌面/床面是否投影进代价图（坐标从 world SDF 中取）
python3 scripts/check_3d_map.py costmap \
  --robot robot1 --world-point <table_x>,<table_y> --expect-lethal --json
python3 scripts/check_3d_map.py costmap \
  --robot robot2 --world-point <bed_x>,<bed_y> --expect-lethal --json

# 4. 存档并离线校验 PCD
python3 scripts/save_cloud_pcd.py \
  --topic /octomap_point_cloud_centers --output maps/tb3_3d_map.pcd
python3 scripts/check_3d_map.py pcd --input maps/tb3_3d_map.pcd --json
```

**回归验收**：

- `WORLD=house` / `WORLD=hospital` 双机：`/points_merged` 在 `world` 下无重影（或重影 ≤ 2 cm），`check_3d_map.py cloud` 的包围盒同时覆盖两个房间
- 2D 探索完成时间相对基线增加 ≤ 20%
- VMware 下 CPU/内存不长期打满；用 `free -h`、`top` 监控，若接近上限，先降 3D 线数/采样率
- 生成的 `.bt` / `.pcd` / `projected_map.yaml+pgm` 能用 `check_3d_map.py pcd`、`octomap` 命令行工具离线打开；**不要求在本机打开 RViz/CloudCompare**

### Phase 5：文档与脚本收尾

- **把本计划完整复制到仓库** `docs/3d_mapping_plan.md`（批准后第一步执行；若 `docs/` 不存在则创建），作为长期文档；同时保留 `.claude/plans/` 下的工作副本
- 更新 `README.md`：
  - 新增“双机 3D 点云建图（无头）”章节，写清 `HEADLESS=True USE_RVIZ=False`、运行命令、存档命令、参数含义
  - 把 TODO #2 标记为“仿真 3D 点云部分完成”，说明“不用于后续导航”
  - 明确写下 2D 完成 ≠ 3D 完成，以及 3D 完成判据
- 在 `scripts/tb3_multi_explore_slam.sh` 的 usage 注释里增加 `SAVE_3D=true` 或 `--save-3d` 的用法（可选）
- 增加清理逻辑：把 merger、octomap_server 的 PID 加入现有 `cleanup()`
- 日志：`/tmp/tb3_multi_merge_clouds.log`、`/tmp/tb3_multi_octomap.log`、`/tmp/tb3_multi_check_3d.json`
- 默认不启动 RViz；确认脚本里 `USE_RVIZ` 默认为 `False`，并打印当前资源限制

## 关键文件清单

**新增**

- `config/gz_waffle_3d_lidar.sdf.xacro`
- `config/tb3_points_bridge.yaml`
- `config/octomap_params.yaml`
- `scripts/merge_pointclouds.py`
- `scripts/save_cloud_pcd.py`
- `scripts/check_3d_map.py`
- `docs/3d_mapping_plan.md`（本计划的仓库内副本）

**修改**

- `config/tb3_bridge_no_clock.yaml`（robot2 加 points bridge）
- `config/nav2_multirobot_slam_params.yaml`（双机 costmap 加 PointCloud2 source）
- `scripts/tb3_multi_explore_slam.sh`（robot1 SDF、points bridge、merger、octomap、清理、无头默认值）
- `README.md`

**不改**

- **所有单车脚本和参数**：`scripts/tb3_explore_slam.sh`、`scripts/tb3_slam_gazebo.sh`、`scripts/tb3_nav_gazebo.sh`、`config/nav2_params_vmware.yaml`、`config/nav2_params_rl.yaml`
- `slam_toolbox` 参数与 2D SLAM 逻辑
- `explore_lite` / `multi_robot_coordinator.py` 的 frontier 逻辑
- `multirobot_map_merge` 的 2D 地图合并逻辑
- 现有 2D 地图存档/加载流程

## 风险与备选方案

| 风险 | 应对 |
|---|---|
| GZ 10.5 的 `<ray><vertical>` 不生效 | 改用 SDF 1.9+ 的 `<lidar><scan><vertical>`；或在 `gpu_lidar` 中启用 `ogre` 渲染引擎 |
| 无独显 + VMware 软件渲染导致卡顿/死机 | 默认 16 线、360 水平、3 Hz；必要时降到 12 线或 240 水平点；全程无头，不启动 RViz/Gazebo GUI；先跑 `free -h`/`top` 观察余量 |
| 3D 点云 + 双车 + OctoMap 内存占用过高 | 体素 0.05 m；merger 只保留每车最新帧；`octomap_server` 不做 free-space 发布；必要时把 3D 图分辨率降到 0.08–0.10 m |
| `ros_gz_bridge` 点云 frame_id 带 model 前缀 | 在 merger 里统一规范成 `robotX/base_scan`；代价图侧用 robot 局部 TF，不受影响 |
| `octomap_server` 的 `projected_map` 把地板标成占据 | 必须设 `point_cloud_min_z` 和 `occupancy_min_z`；先用小世界验证 |
| 3D 点云让代价图出现毛刺/抖动 | 提高 `min_obstacle_height`、减小 `max_range`、加 `pcl_ros` VoxelGrid，或把 `mark_threshold` 调到 1 |
| 双机 2D SLAM 漂移导致 3D 重影 | 3D 图继承 2D 拼合精度；低精度要求下可接受。后续可按 README TODO #13 做动态 SE(2) 配准，或换 3D LIO |
| 桌子投影导致无法钻桌底 | 这是当前需求的预期行为；如果以后要钻桌底，把 costmap 的高度带改成机器人高度带 |
| 以后要 3D frontier / NBV | 再看 `ADVRHumanoids/robot_exploration`（ROS2 分支）或移植 `fkie-nbv-planner`（ROS1）；当前不做 |

## 可直接复用/参考的开源项目

- **`OctoMap/octomap_mapping`（ros2 分支）**：3D 占据地图、`projected_map`、`octomap_point_cloud_centers`、`octomap_saver_node`；Lyrical 已有 apt 包，首选
- **`ros_gz_bridge` 内置转换**：`gz.msgs.PointCloudPacked` ↔ `sensor_msgs/msg/PointCloud2`，无需自己写
- **Nav2 `nav2_costmap_2d::VoxelLayer` / `ObstacleLayer`**：原生支持 `PointCloud2` observation source，3D→2D 投影由它完成
- **`sensor_msgs_py.point_cloud2.read_points_numpy`**：已安装，用于 merger 和 PCD 存档
- **现有 `m-explore-ros2`**：2D frontier、竞价、地图合并全部复用，不改
- **`Taeyoung96/Multi-turtlebot3-Gazebo-ROS2`**：TurtleBot3 + VLP-16 的 GZ 仿真参考（本项目直接用 GZ 内置 vertical lidar，不引入该仓库）
- **`CMU-cabot/livox_laser_simulation_ros2`**：只有切回 Gazebo Classic 才需要；当前 GZ Sim 方案不用
- **`ADVRHumanoids/robot_exploration`**：未来若真要做 3D OctoMap frontier，可作为起点；本次不做

**不推荐当前引入**：RTAB-Map（更重，且本需求不需要 3D 闭环/重定位）、nvblox（需要 GPU，VMware 环境不合适）、FAST-LIO/GLIM（已知初始位姿 + 平地 + 2D 栈已够；换 3D LIO 会同时改 TF、地图和探索链路）。
