# ros2_navigation

在 ROS 2 Lyrical 上跑 Nav2 仿真：TurtleBot3 / TurtleBot4 的 Gazebo 建图、已知地图导航、自主前沿探索（Frontier Exploration），以及用 TD3 策略替换 Nav2 局部控制器（全局规划仍用 Navfn）。

脚本会自动 `source /opt/ros/lyrical/setup.bash`（以及本工作空间的 `install/setup.bash`）。在虚拟机里检测到 VMware 时，默认关掉 Gazebo 三维窗口（`HEADLESS=True`）、关闭 Nav2 组合节点（`USE_COMPOSITION=False`），并启用 Mesa 软件 OpenGL（`LIBGL_ALWAYS_SOFTWARE=1`），保证激光雷达在无独显环境下能正常进行射线检测。

## 环境要求与依赖安装 (Prerequisites)

- **操作系统**：Ubuntu 26.04 LTS (Resolute)
- **ROS 版本**：ROS 2 Lyrical（默认安装路径 `/opt/ros/lyrical`）
- **仿真器**：Gazebo (`gz-sim`，随 Nav2 最小仿真包自动安装)

在运行或编译本仓库前，请先安装所需的 ROS 2 功能包、CycloneDDS 中间件及系统工具：

```bash
sudo apt update && sudo apt install -y \
  ros-lyrical-navigation2 \
  ros-lyrical-nav2-bringup \
  ros-lyrical-slam-toolbox \
  ros-lyrical-nav2-minimal-tb3-sim \
  ros-lyrical-nav2-minimal-tb4-sim \
  ros-lyrical-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions \
  python3-rosdep \
  pciutils
```

## 快速开始 (Getting Started)

### 1. 克隆仓库

```bash
git clone https://github.com/Craphean/ROS2-Navigation.git ~/ros2_navigation
cd ~/ros2_navigation
chmod +x scripts/*.sh
```

### 2. 编译工作空间

本工作空间包含需要编译的源码包：`rl_nav2_controller`（TD3 局部控制器插件）、`explore_lite_msgs` 与 `explore_lite`（自主探索节点）以及 `multirobot_map_merge`（多机器人地图合并节点）。

```bash
cd ~/ros2_navigation
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select rl_nav2_controller explore_lite_msgs explore_lite multirobot_map_merge
source install/setup.bash
```

### 3. 一键启动仿真

仓库已在 `maps/` 目录下内置了建好的 TurtleBot3 `sandbox` 默认地图（`maps/tb3_map.yaml`），编译完成后可直接运行任意脚本：

```bash
# 直接运行 TurtleBot3 已知地图导航（默认加载 maps/tb3_map.yaml，MPPI 控制器）
./scripts/tb3_nav_gazebo.sh

# 或使用 TD3 强化学习局部控制器导航
./scripts/tb3_nav_rl_gazebo.sh

# 换一套仿真前先停掉残留进程并清理 DDS 共享内存：
./scripts/kill_nav2_sim.sh
```

## 脚本一览

| 脚本 | 作用 |
| --- | --- |
| `scripts/tb3_slam_gazebo.sh` | TurtleBot3 + Gazebo + SLAM 手动建图 |
| `scripts/tb3_nav_gazebo.sh` | TurtleBot3 已知地图导航（MPPI 局部控制器，默认使用 `maps/tb3_map.yaml`） |
| `scripts/tb3_nav_rl_gazebo.sh` | 同上，局部控制器换成 TD3（`rl_nav2_controller::RlController`） |
| `scripts/tb3_explore_slam.sh` | TurtleBot3 自主前沿探索并建图 |
| `scripts/tb3_multi_explore_slam.sh` | 双车 TurtleBot3 实时 SLAM 自主探索 + `multirobot_map_merge` 地图合并（默认 `HEADLESS=True`） |
| `scripts/tb4_slam_gazebo.sh` | TurtleBot4 + Gazebo + SLAM（`warehouse` / `depot`） |
| `scripts/tb4_nav_gazebo.sh` | TurtleBot4 使用 Nav2 自带地图导航 |
| `scripts/tb4_explore_slam.sh` | TurtleBot4 自主探索并建图 |
| `scripts/kill_nav2_sim.sh` | 结束 Gazebo、Nav2、RViz，并清理 DDS 共享内存 |

世界可用环境变量切换。TurtleBot3 默认 `sandbox`，也可 `WORLD=empty`。TurtleBot4 默认 `warehouse`，也可 `WORLD=depot`。VMware 上若要打开 Gazebo 窗口：`HEADLESS=False ./scripts/...`。

## TurtleBot3：建图与已知地图导航

### 直接使用内置地图导航

如果不传参数，导航脚本默认加载仓库自带的 `maps/tb3_map.yaml`：

```bash
./scripts/kill_nav2_sim.sh
./scripts/tb3_nav_gazebo.sh          # MPPI 局部控制器
# 或
./scripts/tb3_nav_rl_gazebo.sh       # TD3 局部控制器
```

### 从头建图再导航

也可以自己跑一遍 SLAM 并保存新地图（需要另开一个终端保存地图）：

```bash
./scripts/tb3_slam_gazebo.sh
# 另一个终端
source /opt/ros/lyrical/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/tb3_map
```

随后将自定义地图路径作为参数传入：

```bash
./scripts/kill_nav2_sim.sh
./scripts/tb3_nav_gazebo.sh ~/tb3_map.yaml
# 或使用 TD3 局部控制器：
./scripts/tb3_nav_rl_gazebo.sh ~/tb3_map.yaml
```

RViz 里用 **Nav2 Goal** 点目标。导航和定位都显示 active 后再发目标。

这张地图的坐标系是建图时的里程计，原点在默认出生点 Gazebo `(-2, -0.5)`，不是世界原点。导航脚本把车出生在世界坐标 `(0, 0.5)`，对应的地图位姿是 `(2, 1)`，并自动发给 AMCL。激光和墙壁重合时不要再点 **2D Pose Estimate**；只有明显对不齐时才手动改。里程计来自差速插件，按轮子转角积分。

## RL 局部控制器

`rl_nav2_controller` 是 Nav2 的 `FollowPath` 插件，只替换局部控制。全局路径仍由 Navfn 给出。当前这版策略在仿真里的跟踪和避障还不理想，车可能贴到障碍，或走得不稳。

策略是 TD3 actor（25 → 800 → 600 → 2，输出经 Tanh）。状态为 20 个激光距离桶、到路径前视点的距离和方向，以及上一步的线速度、角速度。线速度由第一个动作映射到 `[0, vx_max]`。模型权重在 `src/rl_nav2_controller/models/actor.onnx`。本机没有 ONNX Runtime，插件实际加载的是由该文件导出的 `td3_actor.bin`。参数在 `config/nav2_params_rl.yaml`。

碰撞监控仍在生命周期里，但接近减速被关掉，避免它把速度一直乘成 0。车是否会贴到障碍，取决于策略本身。

## TurtleBot4

首次启动 `warehouse` / `depot` 会从 Fuel 下载模型，需要网络。这两张图是 Nav2 自带的、与 Gazebo 世界坐标对齐的地图，初始位姿就是出生点，不要套用 TurtleBot3 那张 SLAM 图的偏移。

```bash
./scripts/tb4_slam_gazebo.sh
WORLD=depot ./scripts/tb4_nav_gazebo.sh
./scripts/tb4_explore_slam.sh
```

`tb4_nav_gazebo.sh` 也可以把一张自己保存的 yaml 作为第一个参数传入。

## 自主探索与多车地图合并

单车探索建图：

```bash
./scripts/tb3_explore_slam.sh
./scripts/tb4_explore_slam.sh
```

双车 TurtleBot3 实时 SLAM 探索 + `multirobot_map_merge` 地图合并（默认 `HEADLESS=True`，合并后的全局地图发布在 `/map`，坐标系为 `world`）：

```bash
# 默认模式：共享全局合并地图 + 竞拍协商机制 (multi_robot_coordinator)
./scripts/tb3_multi_explore_slam.sh

# 切换回旧版各自独立的 explore_lite 盲探索模式
EXPLORATION_MODE=legacy ./scripts/tb3_multi_explore_slam.sh

# 切换到 TurtleBot4 的 warehouse 或 depot 大型地图（自动匹配走廊双车初始位姿）
WORLD=warehouse ./scripts/tb3_multi_explore_slam.sh
WORLD=depot ./scripts/tb3_multi_explore_slam.sh

# 同时打开 RViz 实时查看合并后的全局地图 /map 与前沿竞标标记 /exploration/markers
USE_RVIZ=True WORLD=warehouse ./scripts/tb3_multi_explore_slam.sh

# 保存合并后的全局地图
ros2 run nav2_map_server map_saver_cli -f ~/tb3_merged_map --ros-args -r map:=/map
```

暂停和继续：

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
```

探索与地图合并包来自 [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2)，放在 `src/m-explore-ros2`。

## 目录

- `scripts/`：启动和清理脚本
- `maps/`：内置的 TurtleBot3 默认仿真地图（`tb3_map.yaml` / `tb3_map.pgm`）
- `config/nav2_params_rl.yaml`：TD3 导航参数
- `config/nav2_params_vmware.yaml`：TurtleBot4 / MPPI 在虚拟机上的参数
- `config/nav2_multirobot_slam_params.yaml`：双车多命名空间 Nav2 + `slam_toolbox` 参数
- `config/cyclonedds.xml`：CycloneDDS 配置。TB3 RL 导航和 TB4 导航默认用它，避免 Fast DDS 共享内存锁
- `src/rl_nav2_controller/`：TD3 局部控制器
- `src/m-explore-ros2/`：前沿探索 (`explore_lite` / `explore_lite_msgs`) 与多车地图合并 (`multirobot_map_merge`)


## TODO

1. [x] **共享合并地图 + 竞拍协商机制（已完成）**：实现 `scripts/multi_robot_coordinator.py`，统一订阅全局合并大地图 `/map`，基于边际效益（前沿增益减去距离/航向代价）进行多车竞拍协商调度，并通过 TF / 初始位姿矩阵逆变换下发 Nav2 导航目标，消除重复探索与航向冲突。详细配置见 `config/coordinator_params.yaml`。

2. TurtleBot3 搭载的是单线激光雷达（2D LiDAR）（在实物上是 LDS-01/02，在 Gazebo 仿真里是单层激光扫描插件）。但是项目的最终目标是构建出有高度、立体的 3D 地图，需要升级传感器和建图算法。只需要考虑已知初始位姿的情况。

3. 现在使用的 TurtleBot3 Waffle 车头正前方安装了一颗 Intel RealSense 深度摄像头，但在 Nav2 官方的精简仿真包（nav2_minimal_tb3_sim）中，为了极大降低对 CPU/GPU 的算力消耗（特别是虚拟机环境），默认只启动了激光雷达和里程计的桥接，摄像头的图像渲染插件默认被精简关闭了。
目标：在拥有独立显卡直通的高性能主机上开启摄像头的视频流展示

4. 地图世界方面，目前能使用的地图包括带有圆柱体障碍物的 sandbox 和 大型物流仓储 warehouse ，工业厂房与中转站世界 depot，无限开阔平地世界 empty。当前项目使用的仿真器是新版 Gazebo（gz-sim），它天生支持直接加载 Gazebo Fuel 官方云端模型库中的所有开源世界模型。如家庭生活环境、医院病房走廊。前提是地面必须平整。

5. benchmark 基准测试，设计对比试验

6. 中控台：
    展示内容：
    实时建图进度（实时全局地图，rviz视图）
    集成 Gazebo 3D 数字孪生（3D Digital Twin）：无需开启笨重的 Gazebo GUI，加载环境 Mesh 和机器人 URDF 模型。
    画中画或双窗口并列显示 robot1 和 robot2 车载摄像头实时画面。
    小车当前生命周期状态监控：IDLE（待命）、PLANNING（规划中）、NAVIGATING（巡航中）、RECOVERY（正在脱困自旋）。

    控制内容：
    一键启停，召回返航
    保存地图
    人工应急接管车辆
    点选导航目标，直接调用nav2

    追求美观和专业。使用foxglove

7. [x] **rviz界面加上goal, path等display用于调试**：在 `map_merge.rviz` 中分组集成了 Robot1/Robot2 的全局规划路径 (`/plan`)、局部跟踪路径 (`/transformed_global_plan`)、当前目标点 (`/current_goal`)、激光雷达点云 (`/scan`)、车体多边形 (`published_footprint`)、代价地图以及前沿探索竞拍标记 (`/exploration/markers`)。

8. [x] **出生点随机or固定（可选，默认固定默认出生点）**：
    - **设计策略**：针对大型仓储地图 `warehouse`，在空旷走廊与通道区域标定出 10 个合法的**巡检基站候选池**（各点离墙壁与货架净空均 $\ge 1.05\text{m}\sim 2.94\text{m}$，两两间距 $\ge 1.50\text{m}$）。
    - **候选点位明细**：
      - `[0] STATION_SOUTH_DOCK_1`：`x=2.00, y=-19.65, yaw=1.57`（南区装卸站1，默认起点R1）
      - `[1] STATION_SOUTH_DOCK_2`：`x=0.50, y=-19.65, yaw=1.57`（南区装卸站2，默认起点R2）
      - `[2] STATION_SOUTHWEST_AISLE`：`x=-11.50, y=-19.65, yaw=1.57`（西南主通道路口巡检站）
      - `[3] STATION_SOUTHEAST_AISLE`：`x=8.75, y=-19.65, yaw=1.57`（东南主通道路口巡检站）
      - `[4] STATION_SOUTHWEST_RACK`：`x=-6.25, y=-10.75, yaw=1.57`（西南仓储货架中继站）
      - `[5] STATION_SOUTHEAST_RACK`：`x=8.75, y=-10.75, yaw=1.57`（东南立体货架巡检站）
      - `[6] STATION_CENTRAL_CROSS_WEST`：`x=-6.25, y=6.00, yaw=0.00`（中区横向主干道西枢纽站）
      - `[7] STATION_CENTRAL_CROSS_MID`：`x=1.60, y=6.00, yaw=0.00`（中区十字交叉核心枢纽站）
      - `[8] STATION_CENTRAL_CROSS_EAST`：`x=8.75, y=6.00, yaw=0.00`（中区横向主干道东枢纽站）
      - `[9] STATION_NORTHWEST_BAY`：`x=-9.00, y=13.50, yaw=1.57`（西北高位货架巡检基站）
    - **随机抽取机制**：当开启 `RANDOM_SPAWN=true` 或传入 `--random-spawn` 时，启动脚本采用**无放回均匀随机抽样**从上述 10 个候选基站中抽取 2 个互不重复的点分别赋予 Robot 1 和 Robot 2，并自动同步给 Gazebo 仿真模型生成、`multirobot_map_merge`（已知初始位姿变换）以及 `multi_robot_coordinator`（分布式竞拍坐标系换算）。若设为 `false`（默认），则 100% 保持原有固定出生点位 `(2.0, -19.65)` 与 `(0.5, -19.65)`。
    - **运行指令**：
      ```bash
      # 1. 默认固定出生点启动（原默认位姿）
      WORLD=warehouse ./scripts/tb3_multi_explore_slam.sh

      # 2. 随机基站出生点启动（环境变量方式）
      WORLD=warehouse RANDOM_SPAWN=true ./scripts/tb3_multi_explore_slam.sh

      # 3. 随机基站出生点启动（命令行参数快捷方式，自动缺省切至 warehouse）
      ./scripts/tb3_multi_explore_slam.sh --random-spawn
      ```

9. 避免两车碰撞机制（动态避障）

10. 针对在大型仓库环境（WORLD=warehouse）实测中发现的新问题（结合日志）：

    Robot2 到达边界后卡住不动，唯独 Robot1 独自探索；
    地图仍然出现严重双层重影与角度偏移（货架轮廓错位约 5°）；
    探索后期远处明明有大片未探索栅格，双车却全部停滞不再前进。