#!/usr/bin/env python3
"""Run official CMU M-TARE (ROS 1 Noetic + ROS 2 Galactic bridge) 4-core benchmarks on cmu_indoor & cmu_tunnel.

Outputs standard benchmark directories compatible with ``hier_explore_bench.eval_metrics``
and ``scripts/generate_paper_artifacts.py`` under ``benchmark_results/m5_paper/mtare/<world>/seed_<k>/``.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "hier_explore_bringup"))
sys.path.insert(0, str(REPO_ROOT / "src" / "hier_explore_bench"))

from hier_explore_bringup.worlds import sample_spawn_poses

MTARE_RUNNER = "/NAS/62/liangqingcai/mtare_oci_cache/run_mtare_bwrap.sh"
LYRICAL_RUNNER = "/NAS/62/liangqingcai/ros2_lyrical/run_lyrical.sh"

WORLD_TO_MTARE_SCENARIO = {
    "cmu_indoor": ("indoor", "03021000"),
    "cmu_tunnel": ("tunnel", "00021000"),
}


def _build_inner_script(
    scenario: str,
    test_id: str,
    spawns: List[Tuple[float, float, float]],
    gt_2d_cont: str,
    out_cont: str,
    max_sim_sec: int,
) -> str:
    s0 = spawns[0]
    s1 = spawns[1]
    return f"""set -e
ROOT=/home/docker-user/mtare
mkdir -p "{out_cont}"

# Background process CPU/RAM monitor (1 Hz inside this 4-core isolated container)
python3 - << 'PYMON' >"{out_cont}/proc_mon.log" 2>&1 &
import json, os, time, psutil
MARKERS = (
    "gzserver", "tare_planner_node", "localPlanner", "pathFollower",
    "terrainAnalysis", "sensorScanGeneration", "vehicleSimulator",
    "parameter_bridge", "roscore", "rosmaster"
)
self_pid = os.getpid()
procs = {{}}
cpu_s, ram_s = [], []
t0 = time.monotonic()
while time.monotonic() - t0 < {max_sim_sec * 3 + 120}:
    if os.path.exists("/tmp/mtare_done"):
        break
    alive = {{}}
    for p in psutil.process_iter(["pid"]):
        if p.pid == self_pid:
            continue
        if p.pid in procs:
            alive[p.pid] = procs[p.pid]
            continue
        try:
            cmd = " ".join(p.cmdline()) or p.name()
        except psutil.Error:
            continue
        if "mtare_recorder_node" in cmd or "PYMON" in cmd:
            continue
        if any(m in cmd for m in MARKERS):
            try:
                p.cpu_percent(interval=None)
                alive[p.pid] = p
            except psutil.Error:
                pass
    procs = alive
    tc, tr = 0.0, 0.0
    for p in list(procs.values()):
        try:
            tc += float(p.cpu_percent(interval=None))
            tr += float(p.memory_info().rss) / (1024.0 * 1024.0)
        except psutil.Error:
            pass
    if tc > 0.1 or tr > 10.0:
        cpu_s.append(tc)
        ram_s.append(tr)
    time.sleep(1.0)
with open("{out_cont}/proc_resources.json", "w") as f:
    json.dump({{"cpu_samples": cpu_s, "ram_samples": ram_s}}, f)
PYMON
MON_PID=$!

launch_robot() {{
  local RID=$1
  local ROS_PORT=$((11311 + RID))
  local GZ_PORT=$((11345 + RID))
  local X=$2
  local Y=$3
  local YAW=$4
  (
    export ROS_MASTER_URI=http://127.0.0.1:${{ROS_PORT}}
    export ROS_IP=127.0.0.1
    export GAZEBO_MASTER_URI=http://127.0.0.1:${{GZ_PORT}}
    export ROS_LOG_DIR=/tmp/roslog_r${{RID}}
    mkdir -p "$ROS_LOG_DIR"
    source /opt/ros/noetic/setup.bash
    roscore -p ${{ROS_PORT}} >/tmp/roscore_${{RID}}.log 2>&1 &
    sleep 2
    source "$ROOT/tare_system/devel/setup.bash"
    roslaunch --wait tare_planner param.launch >/tmp/param_${{RID}}.log 2>&1 &
    sleep 1
    (
      source /opt/ros/galactic/setup.bash
      source "$ROOT/tare_system/devel/setup.bash"
      source "$ROOT/colcon_ws/install/local_setup.bash"
      source "$ROOT/ros1_bridge/install/local_setup.bash"
      ros2 run ros1_bridge parameter_bridge >/tmp/bridge_${{RID}}.log 2>&1
    ) &
    (
      source /opt/ros/noetic/setup.bash
      source "$ROOT/autonomous_exploration_development_environment/devel/setup.bash"
      roslaunch --wait vehicle_simulator system.launch world_name:={scenario} robot_id:=${{RID}} rviz:=false gazebo_gui:=false vis_tools:=false vehicleX:=${{X}} vehicleY:=${{Y}} vehicleYaw:=${{YAW}} >/tmp/sim_${{RID}}.log 2>&1
    ) &
    sleep 6
    (
      source /opt/ros/noetic/setup.bash
      source "$ROOT/tare_system/devel/setup.bash"
      roslaunch --wait tare_planner explore_{scenario}.launch robot_id:=${{RID}} robot_num:=2 test_id:={test_id} rviz:=false >/tmp/tare_${{RID}}.log 2>&1
    ) &
    wait
  ) &
}}

launch_robot 0 {s0[0]:.3f} {s0[1]:.3f} {s0[2]:.3f}
launch_robot 1 {s1[0]:.3f} {s1[1]:.3f} {s1[2]:.3f}
sleep 8

source /opt/ros/noetic/setup.bash
ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 python3 /ws/scripts/mtare_recorder_node.py \
  --robot-id 0 --out-dir "{out_cont}" --gt-2d "{gt_2d_cont}" --max-sim-sec {max_sim_sec} >"{out_cont}/rec0.log" 2>&1 &
REC0=$!
ROS_MASTER_URI=http://127.0.0.1:11312 ROS_IP=127.0.0.1 python3 /ws/scripts/mtare_recorder_node.py \
  --robot-id 1 --out-dir "{out_cont}" --gt-2d "{gt_2d_cont}" --max-sim-sec {max_sim_sec} >"{out_cont}/rec1.log" 2>&1 &
REC1=$!

wait $REC0 || true
wait $REC1 || true
touch /tmp/mtare_done
wait $MON_PID || true
kill $(jobs -p) 2>/dev/null || true
"""


def _merge_mtare_run(run_dir: Path, max_sim_sec: int) -> bool:
    import numpy as np
    from hier_explore_bench.pcd_io import voxel_downsample, write_pcd

    m0_path = run_dir / "robot_0_meta.json"
    m1_path = run_dir / "robot_1_meta.json"
    g0_path = run_dir / "robot_0_maps.npz"
    g1_path = run_dir / "robot_1_maps.npz"
    if not (m0_path.is_file() and m1_path.is_file() and g0_path.is_file() and g1_path.is_file()):
        return False

    m0 = json.loads(m0_path.read_text(encoding="utf-8"))
    m1 = json.loads(m1_path.read_text(encoding="utf-8"))
    g0 = np.load(g0_path)
    g1 = np.load(g1_path)

    res = float(g0["resolution"])
    origin_x = float(g0["origin_x"])
    origin_y = float(g0["origin_y"])

    free_u = (g0["free_map"] > 0) | (g1["free_map"] > 0)
    occ_u = (g0["occ_map"] > 0) | (g1["occ_map"] > 0)
    grid = np.full(free_u.shape, -1, dtype=np.int8)
    grid[free_u] = 0
    grid[occ_u] = 100
    np.savez_compressed(
        run_dir / "map_final.npz",
        grid=grid,
        origin_x=origin_x,
        origin_y=origin_y,
        resolution=res,
    )

    # Exact multi-robot union explored area time series from per-cell first_seen timestamps
    first_seen_u = np.minimum(g0["first_seen"], g1["first_seen"])
    valid_free = free_u & (~occ_u)
    fs_vals = first_seen_u[valid_free]
    dur = max(float(m0.get("duration_sim_s", 0.0)), float(m1.get("duration_sim_s", 0.0)), 1.0)
    max_t = int(min(max_sim_sec, max(1, round(dur))))

    proc_res_path = run_dir / "proc_resources.json"
    proc_res = json.loads(proc_res_path.read_text(encoding="utf-8")) if proc_res_path.is_file() else {}
    cpu_samples = np.asarray(proc_res.get("cpu_samples", []), dtype=np.float64)
    ram_samples = np.asarray(proc_res.get("ram_samples", []), dtype=np.float64)
    rt_all = np.asarray(list(m0.get("runtime_ms", [])) + list(m1.get("runtime_ms", [])), dtype=np.float64)
    total_comms_bytes = int(m0.get("comms_bytes", 0)) + int(m1.get("comms_bytes", 0))
    avg_kbps = float(total_comms_bytes / 1024.0 / dur)

    with (run_dir / "timeseries.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "sim_time_s",
                "explored_area_m2",
                "occupied_voxels_3d",
                "cycle_time_ms",
                "cpu_percent",
                "ram_mb",
                "comms_kbps",
                "converged",
            ]
        )
        cyc_med = float(np.median(rt_all)) if len(rt_all) else 0.0
        cpu_m = float(np.mean(cpu_samples)) if len(cpu_samples) else 0.0
        ram_m = float(np.mean(ram_samples)) if len(ram_samples) else 0.0
        for sec in range(0, max_t + 1):
            area_t = float(np.sum(fs_vals <= float(sec))) * (res ** 2)
            w.writerow([sec, round(area_t, 3), 0, round(cyc_med, 2), round(cpu_m, 2), round(ram_m, 2), round(avg_kbps, 2), 0])

    # Union 3D cloud
    pts_list = []
    for arr in (g0["points"], g1["points"]):
        if arr.size:
            pts_list.append(arr.reshape(-1, 3))
    if pts_list:
        pts_u = voxel_downsample(np.vstack(pts_list).astype(np.float32), 0.05)
        write_pcd(run_dir / "cloud_final.pcd", pts_u, binary=True)

    # Trajectories + inter-robot distance/collisions
    t0 = m0.get("traj", [])
    t1 = m1.get("traj", [])
    (run_dir / "trajectories.json").write_text(
        json.dumps(
            {
                "estimated": {"robot1": t0, "robot2": t1},
                "ground_truth": {"robot1": t0, "robot2": t1},  # M-TARE uses exact Gazebo state_estimation
            }
        ),
        encoding="utf-8",
    )

    min_d = float("inf")
    collisions = 0
    in_coll = False
    n_pair = min(len(t0), len(t1))
    for i in range(n_pair):
        d = math.hypot(t0[i][0] - t1[i][0], t0[i][1] - t1[i][1])
        min_d = min(min_d, d)
        if d < 0.34:
            if not in_coll:
                collisions += 1
                in_coll = True
        else:
            in_coll = False

    resources = {
        "converged": False,
        "duration_sim_s": round(dur, 2),
        "cpu_mean_pct": float(np.mean(cpu_samples)) if len(cpu_samples) else 0.0,
        "cpu_peak_pct": float(np.max(cpu_samples)) if len(cpu_samples) else 0.0,
        "ram_peak_mb": float(np.max(ram_samples)) if len(ram_samples) else 0.0,
        "cycle_ms_p50": float(np.percentile(rt_all, 50)) if len(rt_all) else float("nan"),
        "cycle_ms_p95": float(np.percentile(rt_all, 95)) if len(rt_all) else float("nan"),
        "cycle_ms_max": float(np.max(rt_all)) if len(rt_all) else float("nan"),
        "comms_avg_kbps": avg_kbps,
        "min_inter_robot_dist_m": min_d if math.isfinite(min_d) else float("nan"),
        "inter_robot_collisions": int(collisions),
    }
    (run_dir / "resources.json").write_text(json.dumps(resources, indent=2), encoding="utf-8")
    return True


def run_one(slot: int, world: str, seed: int, max_sim_sec: int, out_root: Path, base_cpu: int) -> Tuple[str, bool, str]:
    scenario, test_id = WORLD_TO_MTARE_SCENARIO[world]
    run_dir = out_root / world / f"seed_{seed}"
    mp = run_dir / "metrics.json"
    if mp.is_file():
        try:
            d = json.loads(mp.read_text(encoding="utf-8"))
            if float(d.get("final_area_m2", 0.0)) > 10.0:
                return f"{world}/seed_{seed}", True, f"SKIP area={d['final_area_m2']:.1f}m2"
        except Exception:
            pass

    run_dir.mkdir(parents=True, exist_ok=True)
    spawns = [(p.x, p.y, p.yaw) for p in sample_spawn_poses(world, num_robots=2, spawn_seed=seed)]
    cpus = f"{base_cpu + slot * 4}-{base_cpu + slot * 4 + 3}"
    tag = f"mtare_{slot:02d}"
    rel_out = run_dir.relative_to(REPO_ROOT)
    out_cont = f"/ws/{rel_out}"
    gt_2d_cont = f"/ws/maps/gt/{world}/gt_2d.npz"

    for p in (Path(f"/dev/shm/liangqingcai_mtare_tmp_{tag}"), Path(f"/dev/shm/liangqingcai_mtare_shm_{tag}")):
        shutil.rmtree(p, ignore_errors=True)

    script = _build_inner_script(scenario, test_id, spawns, gt_2d_cont, out_cont, max_sim_sec)
    env = os.environ.copy()
    env.update({"EDGE_CPUS": cpus, "MTARE_RUN_TAG": tag})

    t0 = time.monotonic()
    with (run_dir / "mtare_runner.log").open("w", encoding="utf-8") as lf:
        subprocess.run(
            [MTARE_RUNNER, "bash", "-c", script],
            cwd=str(REPO_ROOT),
            env=env,
            stdout=lf,
            stderr=subprocess.STDOUT,
            timeout=max_sim_sec * 4 + 240,
            check=False,
        )

    for p in (Path(f"/dev/shm/liangqingcai_mtare_tmp_{tag}"), Path(f"/dev/shm/liangqingcai_mtare_shm_{tag}")):
        shutil.rmtree(p, ignore_errors=True)

    # Merge & evaluate using the exact same hier_explore_bench.eval_metrics inside Lyrical container
    eval_tag = f"mtare_eval_{slot:02d}"
    eval_env = os.environ.copy()
    eval_env.update({"EDGE_4CORE": "1", "EDGE_CPUS": cpus, "LYRICAL_RUN_TAG": eval_tag})
    eval_cmd = (
        "source /opt/ros/lyrical/setup.bash && source install/setup.bash && "
        f"python3 /ws/scripts/run_mtare_benchmark.py --merge-only '{out_cont}' --max-sim-sec {max_sim_sec} && "
        f"python3 -m hier_explore_bench.eval_metrics --run-dir '{out_cont}' "
        f"--gt-dir '/ws/maps/gt/{world}' --max-time {max_sim_sec}"
    )
    with (run_dir / "eval.log").open("w", encoding="utf-8") as ef:
        subprocess.run(
            [LYRICAL_RUNNER, "bash", "-c", eval_cmd],
            cwd=str(REPO_ROOT),
            env=eval_env,
            stdout=ef,
            stderr=subprocess.STDOUT,
            timeout=180,
            check=False,
        )
    for p in (Path(f"/dev/shm/liangqingcai_lyrical_tmp_{eval_tag}"), Path(f"/dev/shm/liangqingcai_lyrical_shm_{eval_tag}")):
        shutil.rmtree(p, ignore_errors=True)

    if mp.is_file():
        d = json.loads(mp.read_text(encoding="utf-8"))
        return (
            f"{world}/seed_{seed}",
            float(d.get("final_area_m2", 0.0)) > 10.0,
            f"area={d.get('final_area_m2',0):.1f}m2 comp15={d.get('completeness_15cm',0)*100:.1f}% "
            f"cd={d.get('chamfer_l1_m',0):.3f}m cpu={d.get('cpu_mean_pct',0):.1f}% p95={d.get('cycle_ms_p95',0):.1f}ms ({time.monotonic()-t0:.0f}s)",
        )
    return f"{world}/seed_{seed}", False, f"EVAL_FAIL ({time.monotonic()-t0:.0f}s)"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--worlds", nargs="+", default=["cmu_indoor", "cmu_tunnel"])
    ap.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2, 3, 4])
    ap.add_argument("--max-sim-sec", type=int, default=180)
    ap.add_argument("--base-cpu", type=int, default=80)
    ap.add_argument("--out-dir", default=str(REPO_ROOT / "benchmark_results" / "m5_paper" / "mtare"))
    ap.add_argument("--merge-only", default="", help="Internal: merge robot_0/robot_1 artifacts in one run dir.")
    args = ap.parse_args()

    if args.merge_only:
        ok = _merge_mtare_run(Path(args.merge_only), args.max_sim_sec)
        sys.exit(0 if ok else 1)

    out_root = Path(args.out_dir)
    jobs = [(w, s) for w in args.worlds for s in args.seeds]
    print(f"[mtare_bench] Launching {len(jobs)} 4-core M-TARE runs starting at CPU {args.base_cpu}...", flush=True)

    ok_cnt = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(jobs)) as ex:
        futs = [
            ex.submit(run_one, idx, w, s, args.max_sim_sec, out_root, args.base_cpu)
            for idx, (w, s) in enumerate(jobs)
        ]
        for f in concurrent.futures.as_completed(futs):
            name, ok, msg = f.result()
            if ok:
                ok_cnt += 1
            print(f"[mtare_bench] {'OK' if ok else 'FAIL'} {name}: {msg}", flush=True)
    print(f"[mtare_bench] FINISHED! {ok_cnt}/{len(jobs)} succeeded.", flush=True)


if __name__ == "__main__":
    main()
