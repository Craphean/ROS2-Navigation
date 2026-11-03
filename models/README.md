# 视觉模型包（YOLO11n）

给矿井巡检控制台用的两个 ONNX 模型，配合 `console/backend/api.py` 里预留的
`POST /api/detections` 接口使用（前端 2.5 s 过期，WebSocket 同步下发）。

| 文件 | 作用 | 大小 | 输出 |
| --- | --- | --- | --- |
| `detect.onnx` | 目标检测：person(0) / fire(1) / smoke(2) | 10.2 MB | `output0 [1, 7, 8400]` |
| `pose.onnx` | 人体姿态：COCO 17 关键点 | 11.3 MB | `output0 [1, 56, 8400]` |
| `classes.yaml` | 类别名、关键点名、骨架、阈值、颜色 | — | — |
| `infer.py` | 参考推理脚本（onnxruntime，letterbox + NMS + 推送控制台） | — | — |

## 输入规格（两个模型一致）

- 输入名 `images`：`float32 [1, 3, 640, 640]`，RGB，`0~1`
- 等比例缩放 + 灰边（letterbox，填充 114）到 640×640
- OPset 12，未内置 NMS，onnxruntime / OpenCV DNN 都能直接跑

## 输出解码

**detect.onnx** `[1, 7, 8400]`：转置后每行 `cx, cy, w, h, person, fire, smoke`
（坐标是 letterbox 像素）。类别分数取 argmax，阈值 0.35、NMS IoU 0.45，
再按 `scale`/`pad` 反算回原图并归一化到 0~1。

**pose.onnx** `[1, 56, 8400]`：转置后每行
`cx, cy, w, h, person_conf, 17×(x, y, conf)`。关键点顺序和 `skeleton` 见
`classes.yaml`。

## 接入控制台

控制台只认归一化框（左上角原点，相对原始相机帧）：

```bash
# 单帧：拉一次车载快照 → 检测 → 推给控制台
python3 infer.py --ns robot1 --url http://127.0.0.1:18080 --token mine2026 --push

# 连续跑，2 秒一帧（前端 2.5 秒过期，间隔必须小于它）
python3 infer.py --ns robot1 --url http://127.0.0.1:18080 --token mine2026 --push --loop 2

# 本地图片测试，顺便存一张画好框的图
python3 infer.py --image frame.jpg --draw out.jpg
```

推上去的内容：

```json
{"ns": "robot1", "ts": 1762243000.1, "frame_id": "onnx",
 "boxes": [{"cls": "fire", "conf": 0.82, "x": 0.41, "y": 0.55, "w": 0.12, "h": 0.18}]}
```

实测（本机 10 核 CPU，onnxruntime 1.20，Python 3.12）：

| 模型 | CPU 单帧 | 备注 |
| --- | --- | --- |
| detect.onnx | ~183 ms | 5~6 fps，够 1~2 s 一帧的巡检节奏 |
| pose.onnx | ~193 ms | 同上 |
| 两个一起 | ~380 ms | 建议隔帧跑，或只跑 detect |

想更快可以把 `infer.py` 里 provider 换成 `CUDAExecutionProvider`（4090 上 ~5 ms），
或用 `onnxruntime.quantization` 做 INT8（约 2~3 倍提速，精度略降）。

### person 框的两种来源

`detect.onnx` 的 person 和 fire/smoke 共用一套特征，人形多、距离远时召回一般；
`pose.onnx` 的 person 框是 COCO 全量姿态数据训出来的，更稳。加 `--merge-person`：

```bash
python3 infer.py --ns robot1 --url http://127.0.0.1:18080 --token mine2026 --push --merge-person
```

会把 person 框换成 pose 模型的、fire/smoke 仍用 detect 模型，同一组人实测召回从
3/11 提升到 11/11。

## 如何测试

mine 世界本身没有火焰/人物模型，车载相机拍不到目标，所以要先喂测试素材。
`samples/` 里已经放了 4 张验证图和一段 8 秒 `test.mp4`（由这些图拼的），
可以直接用。

**1. 单张图，本地看画框结果（最快）**

```bash
cd /home/ubuntu/ros2_navigation/models
python3 infer.py --image samples/fire_smoke.jpg --draw /tmp/out_fire.jpg --no-pose
python3 infer.py --image samples/firefighters.jpg --draw /tmp/out_person.jpg --merge-person
```

`--draw` 会把检测框和 17 点骨架画出来；控制台前端目前只画框，不画骨架，
所以姿态效果要用这种方式看。实测：fire_smoke 出 2 个 fire + 1 个 smoke，
firefighters 出 2 个人、置信度 0.88/0.78。

**2. 看控制台前端叠加层（端到端）**

```bash
python3 infer.py --video samples/test.mp4 --repeat \
  --url http://127.0.0.1:18080 --token mine2026 --push --loop 2 --merge-person
```

浏览器打开 `http://127.0.0.1:18080`，看 robot1 的视频面板：每 2 秒推一帧检测结果，
叠加层会画出 fire/smoke/person 框（TTL 2.5 s，间隔不能超过它）。
注意前端叠加的是**实时相机画面**（黑乎乎的巷道），框的位置来自测试视频；
这足以验证「推理 → POST → WebSocket → 前端」整条链路，但不是真实拍摄画面。
实测返回 `{"ok":true,"count":2}`，`GET /api/detections` 也能读到。

**3. 自己的视频批量过一遍**

```bash
python3 infer.py --video /path/to/clip.mp4 --stride 5 --draw /tmp/annotated --no-pose
python3 infer.py --video /path/to/clip.mp4 --repeat --url http://127.0.0.1:18080 \
  --token mine2026 --push --loop 2 --no-pose
```

**4. 想让仿真里真的出现火和人**

需要往 `worlds/mine.sdf.xacro` 里加火焰视觉（发光材质/粒子）和人物 actor，
再重启仿真，让车载相机真正拍到目标 —— 这一步我没动你的世界文件，
需要的话告诉我，我可以加一个可开关的测试道具，不影响正常巡检。

## 训练与指标

- 检测：YOLO11n（COCO 预训练迁移），`imgsz=640`、batch 128、AdamW + cosine，
  共约 92 epoch，单卡 4090 约 1 小时。
- 数据：8169 张训练图 / 934 张验证图 —— 明火+烟雾 6940 张（VOC 标注转 YOLO，
  fire 13.5 k、smoke 12.5 k 框）+ COCO person 子集（person 10.9 k 框）。
- 姿态：官方 `yolo11n-pose.pt` 直接导出，未再训练（COCO 17 关键点）。

验证集（934 张，onnxruntime 导出的 ONNX 复测）：

| 类别 | AP50 | AP50-95 |
| --- | --- | --- |
| all | 0.301 | 0.132 |
| person | 0.439 | 0.194 |
| fire | 0.309 | 0.152 |
| smoke | 0.155 | 0.051 |

## 已知限制

- **当前 mine 世界里既没有火/烟模型也没有人物模型**，仿真画面里检测结果为空是正常的；
  验证得用图片/视频源，或往 world 里加火焰/人物模型后再看叠加层。
- 火/烟数据集的标注偏稀疏（很多真实烟雾、远处火焰没标），mAP 被低估，但
  smoke 召回确实偏低，阈值建议 0.3~0.4。
- 训练集多为地面明火、森林火、厂房火灾；矿井巷道低照度、粉尘、红外画面属于域外场景。
  要上真机建议采 200~500 张现场图补标后微调（`lr0=0.003`，20~30 epoch 即可）。
- detect.onnx 的 person 只出框；要骨架用 pose.onnx。
