# ros2_navigation

在 ROS 2 Lyrical 上跑 Nav2 仿真：TurtleBot3 / TurtleBot4 的 Gazebo 建图、已知地图导航，以及用 TD3 策略替换 Nav2 局部控制器。全局规划仍用 Navfn。

脚本会自己 `source /opt/ros/lyrical/setup.bash`。在虚拟机里检测到 VMware 时，默认关掉 Gazebo 三维窗口、关闭 Nav2 composition，并用软件 OpenGL，保证激光能做射线检测。

## 环境

- ROS 2 Lyrical，以及其中的 Nav2、`nav2_minimal_tb3_sim`、`nav2_minimal_tb4_sim`
- 本工作空间需要自行编译的包：`rl_nav2_controller`、`explore_lite`、`explore_lite_msgs`

```bash
cd ~/ros2_navigation
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select rl_nav2_controller explore_lite_msgs explore_lite
source install/setup.bash
```

换一套仿真前先停掉残留进程：

```bash
./scripts/kill_nav2_sim.sh
```

## 脚本

| 脚本 | 作用 |
| --- | --- |
| `scripts/tb3_slam_gazebo.sh` | TurtleBot3 + Gazebo + SLAM |
| `scripts/tb3_nav_gazebo.sh` | TurtleBot3 已知地图导航（MPPI 局部控制器） |
| `scripts/tb3_nav_rl_gazebo.sh` | 同上，局部控制器换成 TD3 |
| `scripts/tb3_explore_slam.sh` | TurtleBot3 自主前沿探索并建图 |
| `scripts/tb4_slam_gazebo.sh` | TurtleBot4 + Gazebo + SLAM（warehouse / depot） |
| `scripts/tb4_nav_gazebo.sh` | TurtleBot4 使用 Nav2 自带地图导航 |
| `scripts/tb4_explore_slam.sh` | TurtleBot4 自主探索并建图 |
| `scripts/kill_nav2_sim.sh` | 结束 Gazebo、Nav2、RViz，并清理 DDS 共享内存 |

世界可用环境变量切换。TurtleBot3 默认 `sandbox`，也可 `WORLD=empty`。TurtleBot4 默认 `warehouse`，也可 `WORLD=depot`。VMware 上若要打开 Gazebo 窗口：`HEADLESS=False ./scripts/...`。

## TurtleBot3：建图再导航

先建图。在 RViz 里用键盘或 Nav2 把车开一遍沙盒，然后保存地图（需要另开一个已经 source 过的终端）：

```bash
./scripts/tb3_slam_gazebo.sh
# 另一个终端
source /opt/ros/lyrical/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/tb3_map
```

用 MPPI 在这张图上导航：

```bash
./scripts/kill_nav2_sim.sh
./scripts/tb3_nav_gazebo.sh ~/tb3_map.yaml
```

用 TD3 局部控制器：

```bash
./scripts/kill_nav2_sim.sh
./scripts/tb3_nav_rl_gazebo.sh ~/tb3_map.yaml
```

RViz 里用 **Nav2 Goal** 点目标。导航和定位都显示 active 后再发目标。

这张地图的坐标系是建图时的里程计，原点在默认出生点 Gazebo `(-2, -0.5)`，不是世界原点。导航脚本把车出生在世界坐标 `(0, 0.5)`，对应的地图位姿是 `(2, 1)`，并自动发给 AMCL。激光和墙壁重合时不要再点 **2D Pose Estimate**；只有明显对不齐时才手动改。里程计来自差速插件，按轮子转角积分。

## RL 局部控制器

`rl_nav2_controller` 是 Nav2 的 `FollowPath` 插件，只替换局部控制。全局路径仍由 Navfn 给出。当前这版策略在仿真里的跟踪和避障还不理想，车可能贴到障碍，或走得不稳。

策略是 TD3 actor（25 → 800 → 600 → 2，输出经 Tanh）。状态为 20 个激光距离桶、到路径前视点的距离和方向，以及上一步的线速度、角速度。线速度由第一个动作映射到 `[0, vx_max]`。模型权重在 `src/rl_nav2_controller/models/actor.onnx`。本机没有 ONNX Runtime，插件实际加载的是由该文件导出的 `td3_actor.bin`。参数在 `config/nav2_params_rl.yaml`。

碰撞监控仍在生命周期里，但接近减速被关掉，避免它把速度一直乘成 0。车是否会贴到障碍，取决于策略本身。

## TurtleBot4

首次启动 warehouse / depot 会从 Fuel 下载模型，需要网络。这两张图是 Nav2 自带的、与 Gazebo 世界坐标对齐的地图，初始位姿就是出生点，不要套用 TurtleBot3 那张 SLAM 图的偏移。

```bash
./scripts/tb4_slam_gazebo.sh
WORLD=depot ./scripts/tb4_nav_gazebo.sh
./scripts/tb4_explore_slam.sh
```

`tb4_nav_gazebo.sh` 也可以把一张自己保存的 yaml 作为第一个参数传入。

## 自主探索

```bash
./scripts/tb3_explore_slam.sh
./scripts/tb4_explore_slam.sh
```

暂停和继续：

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
```

探索包来自 [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2)，放在 `src/m-explore-ros2`。

## 目录

- `scripts/`：启动和清理
- `config/nav2_params_rl.yaml`：TD3 导航参数
- `config/nav2_params_vmware.yaml`：TurtleBot4 / MPPI 在虚拟机上的参数
- `config/cyclonedds.xml`：CycloneDDS 配置。TB3 RL 导航和 TB4 导航默认用它，避免 Fast DDS 共享内存锁
- `src/rl_nav2_controller/`：TD3 局部控制器
- `src/m-explore-ros2/`：前沿探索
