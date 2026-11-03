#!/usr/bin/env python3
"""Reference runner for the mine-console vision models.

Runs detect.onnx (+ optional pose.onnx) on a camera frame and pushes the
result to the console:  POST /api/detections  {"ns","boxes":[{cls,conf,x,y,w,h}]}
Boxes are normalised 0..1 with the origin at the top-left of the raw frame.

  python infer.py --ns robot1 --image frame.jpg --draw out.jpg
  python infer.py --ns robot1 --url http://127.0.0.1:18080 --loop 2.0 --push
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

HERE = Path(__file__).resolve().parent

DET_NAMES = ["person", "fire", "smoke"]
KPT_NAMES = [
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
]
# COCO 17-keypoint skeleton, index pairs
SKELETON = [(15, 13), (13, 11), (16, 14), (14, 12), (11, 12), (5, 11), (6, 12),
            (5, 6), (5, 7), (6, 8), (7, 9), (8, 10), (1, 2), (0, 1), (0, 2),
            (1, 3), (2, 4), (3, 5), (4, 6)]


def letterbox(img, size=640, color=114):
    """Resize keeping aspect ratio, pad to (size, size). Returns img, scale, (pad_x, pad_y)."""
    h, w = img.shape[:2]
    r = min(size / h, size / w)
    nw, nh = int(round(w * r)), int(round(h * r))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    out = np.full((size, size, 3), color, dtype=np.uint8)
    top, left = (size - nh) // 2, (size - nw) // 2
    out[top:top + nh, left:left + nw] = resized
    return out, r, (left, top)


def to_input(bgr, size=640):
    img, r, pad = letterbox(bgr, size)
    x = img[:, :, ::-1].transpose(2, 0, 1)[None]          # BGR->RGB, HWC->CHW
    return np.ascontiguousarray(x, dtype=np.float32) / 255.0, r, pad


def nms(boxes, scores, iou_thr=0.45):
    idx = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), 0.0, iou_thr)
    return np.array(idx).reshape(-1) if len(idx) else np.zeros(0, dtype=int)


class Detector:
    def __init__(self, onnx_path, imgsz=640, providers=None):
        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.sess = ort.InferenceSession(str(onnx_path), so,
                                         providers=providers or ["CPUExecutionProvider"])
        self.imgsz = imgsz
        self.inp = self.sess.get_inputs()[0].name

    def __call__(self, bgr, conf_thr=0.35, iou_thr=0.45):
        h, w = bgr.shape[:2]
        x, r, (px, py) = to_input(bgr, self.imgsz)
        out = self.sess.run(None, {self.inp: x})[0]        # [1, 4+nc, 8400]
        pred = out[0].T                                    # [8400, 4+nc]
        nc = pred.shape[1] - 4
        cls_scores = pred[:, 4:4 + nc]
        cls_id = cls_scores.argmax(1)
        score = cls_scores[np.arange(len(pred)), cls_id]
        keep = score >= conf_thr
        if not keep.any():
            return []
        pred, cls_id, score = pred[keep], cls_id[keep], score[keep]
        cx, cy, bw, bh = pred[:, 0], pred[:, 1], pred[:, 2], pred[:, 3]
        xyxy = np.stack([cx - bw / 2, cy - bh / 2, bw, bh], 1)   # x,y,w,h in letterbox space
        dets = []
        for c in np.unique(cls_id):
            m = cls_id == c
            for i in nms(xyxy[m], score[m], iou_thr):
                bx, by, bwd, bht = xyxy[m][i]
                # back to original frame, then normalise
                x0 = (bx - px) / r / w
                y0 = (by - py) / r / h
                x1 = (bx - px + bwd) / r / w
                y1 = (by - py + bht) / r / h
                x0, y0 = max(0.0, x0), max(0.0, y0)
                x1, y1 = min(1.0, x1), min(1.0, y1)
                dets.append({"cls": DET_NAMES[int(c)] if int(c) < len(DET_NAMES) else str(int(c)),
                             "conf": float(score[m][i]),
                             "x": float(x0), "y": float(y0),
                             "w": float(x1 - x0), "h": float(y1 - y0)})
        return sorted(dets, key=lambda d: -d["conf"])


class PoseEstimator:
    def __init__(self, onnx_path, imgsz=640, providers=None):
        self.sess = ort.InferenceSession(str(onnx_path),
                                         providers=providers or ["CPUExecutionProvider"])
        self.imgsz = imgsz
        self.inp = self.sess.get_inputs()[0].name

    def __call__(self, bgr, conf_thr=0.35, iou_thr=0.45):
        h, w = bgr.shape[:2]
        x, r, (px, py) = to_input(bgr, self.imgsz)
        out = self.sess.run(None, {self.inp: x})[0]        # [1, 56, 8400]
        pred = out[0].T
        score = pred[:, 4]
        keep = score >= conf_thr
        if not keep.any():
            return []
        pred = pred[keep]
        boxes = pred[:, :4]
        xywh = np.stack([boxes[:, 0] - boxes[:, 2] / 2, boxes[:, 1] - boxes[:, 3] / 2,
                         boxes[:, 2], boxes[:, 3]], 1)
        people = []
        for i in nms(xywh, pred[:, 4], iou_thr):
            kpts = pred[i, 5:].reshape(-1, 3).copy()
            kpts[:, 0] = (kpts[:, 0] - px) / r / w
            kpts[:, 1] = (kpts[:, 1] - py) / r / h
            bx, by, bwd, bht = xywh[i]
            people.append({
                "conf": float(pred[i, 4]),
                "x": float((bx - px) / r / w), "y": float((by - py) / r / h),
                "w": float(bwd / r / w), "h": float(bht / r / h),
                "keypoints": [{"name": KPT_NAMES[k], "x": float(p[0]), "y": float(p[1]),
                               "conf": float(p[2])} for k, p in enumerate(kpts)],
            })
        return people


def draw(bgr, dets, people=()):
    img = bgr.copy()
    h, w = img.shape[:2]
    for d in dets:
        x0, y0 = int(d["x"] * w), int(d["y"] * h)
        x1, y1 = int((d["x"] + d["w"]) * w), int((d["y"] + d["h"]) * h)
        cv2.rectangle(img, (x0, y0), (x1, y1), (60, 60, 255), 2)
        cv2.putText(img, f'{d["cls"]} {d["conf"]:.2f}', (x0, max(12, y0 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 255), 1, cv2.LINE_AA)
    for p in people:
        pts = [(int(k["x"] * w), int(k["y"] * h), k["conf"]) for k in p["keypoints"]]
        for a, b in SKELETON:
            if pts[a][2] > 0.3 and pts[b][2] > 0.3:
                cv2.line(img, pts[a][:2], pts[b][:2], (80, 220, 80), 2)
        for px, py, c in pts:
            if c > 0.3:
                cv2.circle(img, (px, py), 3, (80, 220, 80), -1)
    return img


def push(base, ns, dets, token=None):
    import json
    import urllib.request
    body = json.dumps({"ns": ns, "ts": time.time(), "frame_id": "onnx",
                       "boxes": [{k: d[k] for k in ("cls", "conf", "x", "y", "w", "h")}
                                 for d in dets]}).encode()
    req = urllib.request.Request(base.rstrip("/") + "/api/detections", data=body,
                                 headers={"Content-Type": "application/json",
                                          **({"X-Console-Token": token} if token else {})})
    with urllib.request.urlopen(req, timeout=3) as resp:
        return resp.read().decode()


def grab(url):
    with urllib.request.urlopen(url, timeout=5) as resp:
        buf = np.frombuffer(resp.read(), np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--det", default=str(HERE / "detect.onnx"))
    ap.add_argument("--pose", default=str(HERE / "pose.onnx"))
    ap.add_argument("--no-pose", action="store_true")
    ap.add_argument("--image", help="单张图片")
    ap.add_argument("--video", help="视频文件/摄像头（如 test.mp4 或 0）")
    ap.add_argument("--repeat", action="store_true", help="视频播完从头循环")
    ap.add_argument("--stride", type=int, default=1, help="视频每隔 N 帧取 1 帧")
    ap.add_argument("--ns", default="robot1")
    ap.add_argument("--url", default="")
    ap.add_argument("--token", default="")
    ap.add_argument("--loop", type=float, default=0.0,
                    help="帧间隔秒数；单张图片时为 0 表示只跑一次")
    ap.add_argument("--push", action="store_true")
    ap.add_argument("--merge-person", action="store_true",
                    help="用 pose.onnx 的人框替换检测模型的 person 框（人形更稳）")
    ap.add_argument("--draw", help="输出画框图；视频时作为输出目录")
    ap.add_argument("--max-frames", type=int, default=0, help="最多处理多少帧，0=不限")
    ap.add_argument("--conf", type=float, default=0.35)
    args = ap.parse_args()

    det = Detector(args.det, providers=ort.get_available_providers()[:1])
    pose = None if args.no_pose else PoseEstimator(args.pose,
                                                  providers=ort.get_available_providers()[:1])
    cap = None
    if args.video:
        src = int(args.video) if args.video.isdigit() else args.video
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            print("打不开视频:", args.video)
            return
    if args.draw and args.video:
        Path(args.draw).mkdir(parents=True, exist_ok=True)

    n = 0
    while True:
        if cap is not None:
            ok, frame = cap.read()
            if not ok:
                if args.repeat:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                break
            for _ in range(args.stride - 1):
                cap.grab()
        elif args.image:
            frame = cv2.imread(args.image)
        else:
            frame = grab(f"{args.url.rstrip('/')}/api/video/{args.ns}/snapshot.jpg")
        if frame is None:
            print("no frame")
            break
        t0 = time.time()
        dets = det(frame, conf_thr=args.conf)
        people = pose(frame, conf_thr=args.conf) if pose else []
        if args.merge_person and people:
            dets = [d for d in dets if d["cls"] != "person"] + [
                {"cls": "person", "conf": p["conf"], "x": p["x"], "y": p["y"],
                 "w": p["w"], "h": p["h"]} for p in people
            ]
            dets.sort(key=lambda d: -d["conf"])
        dt = (time.time() - t0) * 1000
        n += 1
        print(f"#{n} {dt:6.1f} ms  boxes={len(dets)} people={len(people)}  "
              + ", ".join(f'{d["cls"]}:{d["conf"]:.2f}' for d in dets[:5]))
        if args.draw:
            out = str(Path(args.draw) / f"frame_{n:06d}.jpg") if cap is not None else args.draw
            cv2.imwrite(out, draw(frame, dets, people))
        if args.push:
            print(push(args.url, args.ns, dets, args.token))
        if args.max_frames and n >= args.max_frames:
            break
        if cap is None and args.loop <= 0:
            break
        time.sleep(max(0.0, args.loop - dt / 1000))

    if cap is not None:
        cap.release()


if __name__ == "__main__":
    main()
