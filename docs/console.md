# 矿井智能巡检机器人控制台

浏览器控制台，用于在双 TurtleBot3 仿真上演示矿井巡检：实时建图、双车位姿与激光、双路车载视频、虚拟摇杆遥控、单点导航、多航点巡检任务、一键启停与公网访问。

## 1. 组成

```
评委浏览器 ──HTTPS──> cpolar ──> 127.0.0.1:8080 FastAPI
                                      ├── console/web/        静态单页（无构建步骤）
                                      ├── /api/*  /ws         状态、控制、任务
                                      ├── /api/video/*.mjpeg 双路视频
                                      └── rclpy 节点（独立线程）
                                             │ CycloneDDS
                                             ▼
                     Gazebo + /robot1 + /robot2 + slam_toolbox
                     + map_merge + octomap + Nav2
```

**单端口原则**：页面、API、WebSocket、MJPEG 全部同源 `8080`，cpolar 只需要一条隧道。Foxglove 的 `8765` 不对外。

## 2. 快速开始

### 2.1 控制台一键启动（推荐）

```bash
cd ~/ros2_navigation
./console/run.sh
```

打开 `http://127.0.0.1:8080`，用操作口令登录（默认 `mine2026`），点底部「启动仿真」。控制台会以 `WORLD=mine EXPLORATION_MODE=off HEADLESS=True` 拉起仿真，约 60–90 秒后地图与视频出现。

### 2.2 终端启动仿真 + 控制台

两个终端：

```bash
# 终端 1：矿井、无自主探索，等待控制台派发任务
SKIP_DDS_SHM_CLEAN=1 WORLD=mine EXPLORATION_MODE=off HEADLESS=True \
  DASHBOARD=False ./scripts/tb3_multi_explore_slam.sh

# 终端 2
./console/run.sh
```

> `SKIP_DDS_SHM_CLEAN=1` 很关键：仿真脚本退出时会调用 `kill_nav2_sim.sh`，默认会删除 `/dev/shm/cyclonedds*`，把正在运行的控制台自己的 DDS 连接一起弄坏。

### 2.3 只启动控制台（仿真已在跑）

```bash
CONSOLE_MANAGE_SIM=0 ./console/run.sh
```

### 2.4 环境变量

| 变量 | 默认 | 说明 |
| --- | --- | --- |
| `CONSOLE_HOST` | `127.0.0.1` | 监听地址。公网暴露时保持 127.0.0.1，由 cpolar 转发 |
| `CONSOLE_PORT` | `8080` | 端口 |
| `CONSOLE_TOKEN` | `mine2026` | 操作口令；只读访问无需口令 |
| `CONSOLE_WORLD` | `mine` | 「启动仿真」使用的世界 |
| `CONSOLE_MANAGE_SIM` | `1` | 是否允许控制台启停仿真 |
| `CONSOLE_VIDEO_FPS` | `5` | 视频源帧率上限 |
| `CONSOLE_MAX_STREAMS` | `6` | 同时 MJPEG 连接数 |
| `KILL_SIM_ON_EXIT` | `0` | 控制台退出时是否停仿真 |

## 3. 界面与操作

- **顶栏**：链路、仿真时钟、地图尺寸、勘探/巡检模式、急停、登录。
- **左栏**：两车状态卡（位姿、速度、导航状态、剩余距离）；巡检任务选择、进度、航点状态。
- **中栏**：2D 建图（OccupancyGrid + 激光点 + 车辆 + 航点 + 目标连线），支持拖拽平移、滚轮缩放、点击拾取。
- **右栏**：双路 320×240 视频；事件流（任务、航点、失败、急停）。
- **底栏**：虚拟摇杆、WASD/方向键、接管/释放、召回、保存地图、启停仿真、急停。

### 巡检任务

`console/missions/*.json` 是任务定义，例如 `mine_patrol.json`：

```json
{"id":"mine_patrol","name":"主巷例行巡检","robot":"robot1","loop":false,
 "on_fail":"retry_then_skip",
 "waypoints":[{"label":"井口","x":-19.2,"y":0.0,"yaw":0.0,"dwell":4}]}
```

执行语义：

- 逐点 `NavigateToPose`，成功后停留 `dwell` 秒，再下一个点。
- `retry_then_skip`：每点重试 1 次（3 秒退避），仍失败则跳过；连续 2 点失败中止整个任务。
- 「暂停」取消当前目标并记住 cursor；「继续」从当前点重发；「跳过」取消并前进；「急停」停两车并置 `ESTOP`。
- 每次状态迁移写入 `console/run/missions/<id>.json`；后端重启后任务标为 `INTERRUPTED`，可继续。

### 遥控

- 点「接管」取得租约，后端关闭该车 `collision_monitor`，以 20 Hz 向 `/robotX/cmd_vel` 发 `TwistStamped`。
- 松开摇杆/按键 300 ms 内速度归零；10 秒无操作自动释放租约并恢复碰撞监控。
- 限幅 `vx ∈ [-0.15, 0.35] m/s`、`wz ∈ [-0.8, 0.8] rad/s`；前向 ±25° 内 0.22 m 有障碍时禁止前进。
- 同一时刻只允许一个操作员；浏览器断开时自动释放。

## 4. 关键实现说明

### 4.1 坐标系

前端一律使用 `world`。后端发目标前用 TF `world → robotX/map` 做 SE(2) 变换：

```
local = T(map←world) · world
```

TF 不可用时退化为「出生位姿 + 里程计」解析变换。**不能**把 world 坐标直接塞进 Nav2 goal，否则车会跑飞。

### 4.2 地图传输

`/map` 是 `transient_local + reliable`；控制台以同样 QoS 订阅。栅格在后端转 PNG（约 2–60 KB），前端 Canvas 贴图并按 `origin/resolution` 换算世界坐标，不传 JSON 数组。

### 4.3 视频

Gazebo 相机传感器（`intel_realsense_r200_depth`，`type="camera"`，320×240@5Hz）发布在 `/robotX/camera` 与 `/robotX/camera_info`，经 `ros_gz_bridge` 转到 `/robotX/camera/image`。后端用 numpy 解析 `sensor_msgs/Image`，每路只编码一次 JPEG，所有浏览器共享同一份字节，MJPEG 走独立 HTTP 连接。

### 4.4 进程管理

- 子进程 `start_new_session=True`，停止用 `killpg`，避免 `setsid` 子进程成为孤儿。
- 控制台**不调用** `kill_nav2_sim.sh`（会删 `/dev/shm`）；子进程继承 `SKIP_DDS_SHM_CLEAN=1`。
- 不用 `pkill -f` 模糊匹配，避免误杀控制台本身。

## 5. HTTP / WebSocket

| 方法 | 路径 | 鉴权 | 说明 |
| --- | --- | --- | --- |
| GET | `/api/health` | 否 | 地图、仿真、探索、WS 连接数 |
| GET | `/api/state` | 否 | 全量快照 |
| GET | `/api/map.png`、`/api/map/meta` | 否 | 地图图与几何元数据 |
| GET | `/api/video/{ns}.mjpeg`、`snapshot.jpg` | 否 | 视频（限流 6 路） |
| POST | `/api/login`、`/api/logout` | — | 口令换 cookie |
| POST | `/api/robot/{ns}/goal`、`cancel`、`recall` | 是 | 单点导航 |
| POST | `/api/teleop/lease`、`release`、`cmd` | 是 | 遥控 |
| POST | `/api/estop`、`/api/estop/clear` | 是 | 急停 |
| GET/POST | `/api/missions`、`/{id}/start|pause|resume|skip|stop|waypoints` | 写操作需登录 | 任务 |
| POST | `/api/map/save` | 是 | 保存合并地图到 `maps/inspection_<ts>.yaml` |
| POST | `/api/sim/start|stop`、`/api/exploration/start|stop` | 是 | 进程管理 |
| POST | `/api/detections` | 是 | YOLO 结果注入（预留） |

WebSocket `/ws?token=<口令>`：连接后先收到 `snapshot`、`role`，随后 10 Hz `snapshot`。控制消息：`{"type":"teleop","ns":"robot1","vx":0.2,"wz":0.0}`、`{"type":"estop"}`、`{"type":"clear_estop"}`、`{"type":"ping"}`。只读角色发送控制消息会收到 `{"type":"error"}`。

## 6. 公网访问（cpolar）

```bash
# 1) 安装（需要 sudo 与 cpolar 账号）
curl -L https://www.cpolar.com/static/downloads/install-release-cpolar.sh | sudo bash

# 2) 绑定 authtoken（在 cpolar 后台复制）
cpolar authtoken <YOUR_TOKEN>

# 3) 映射控制台的唯一端口
cpolar http 8080
```

终端会打印形如 `https://xxxx.r6.cpolar.top` 的地址，手机 4G 直接打开即可。

注意：

- 免费版域名每次重启会变；答辩当天先启动隧道再印二维码。
- 只映射 `8080`；不要映射 `8765`（Foxglove 无鉴权）。
- 未登录只能看地图/视频/状态；发目标、遥控、任务、启停都需要口令。
- 带宽有限时，视频加 `?fps=2` 或 `?fps=1`；状态 WS 与视频是两条连接，互不拖累。

## 7. 故障排查

| 现象 | 检查 |
| --- | --- |
| 页面能开但无地图 | 仿真是否起来；`ros2 topic hz /map`；控制台日志是否有 QoS 报错 |
| 地图有、车不动 | 是否在勘探模式（协调器抢占）；`/api/health` 的 exploration 状态；Nav2 lifecycle 是否 active |
| 视频无信号 | `gz topic -l \| grep camera`；`ros2 topic hz /robot1/camera/image`；相机是否被 `HEADLESS` 关掉 |
| 遥控不动 | `ros2 service list \| grep collision_monitor/toggle`；接管是否成功；`ros2 topic echo /robot1/cmd_vel` 是否 20 Hz |
| 目标下发后车跑飞 | 检查 `world → robot1/map` TF 是否存在；`sim_meta.json` 的出生位姿是否与实际一致 |
| 控制台起不来 | 用 `/usr/bin/python3` 建 venv（PATH 上的 3.13 无法 import rclpy）；`console/run.sh` 已处理 |
| 启动仿真后控制台 DDS 断 | 确认 `SKIP_DDS_SHM_CLEAN=1` 或由控制台启动仿真 |

## 8. 目录

```
console/
  backend/{app,api,ws,ros_node,ros_runtime,state,map_render,video,mission,teleop,procman,auth,config}.py
  web/{index.html,css/app.css,js/*.js}
  missions/*.json          任务定义
  run/                     pid、sim_meta.json、任务运行态（git 忽略）
  run.sh
worlds/mine.sdf.xacro      矿井巷道世界（纯几何体，离线可用）
config/tb3_points_bridge.yaml / tb3_bridge_no_clock.yaml   相机与点云桥
```
