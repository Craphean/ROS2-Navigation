#!/usr/bin/env python3
"""Run A1 Algorithm on isolated indoor cases and publish a local report."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import csv
import hashlib
import html
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import time
import traceback

import numpy as np
from PIL import Image

from classroom_voxel_scene import ROOT, scene
from voxel_replay_metrics import metrics
from voxel_motion import advance
from indoor_scenario_suite import SUITE, LABELS


def atomic_json(path, data):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(data, indent=2, ensure_ascii=False)+'\n')
    temp.replace(path)


def cpu_groups(jobs):
    """Give each replay different physical cores, excluding sibling threads."""
    cores = {}
    for cpu in sorted(os.sched_getaffinity(0)):
        topology = Path(f'/sys/devices/system/cpu/cpu{cpu}/topology')
        key = tuple((topology/f).read_text().strip() for f in ('physical_package_id', 'core_id'))
        cores.setdefault(key, cpu)
    physical = list(cores.values())
    if len(physical) < jobs*3:
        raise ValueError('Need at least three independent physical cores per concurrent replay')
    groups = np.array_split(physical, jobs)
    return [[int(c) for c in group] for group in groups]


def freeze(output):
    files = sorted(set(list((ROOT/'scripts').glob('*.py')) + list((ROOT/'scripts').glob('*.cpp'))
                       + list((ROOT/'config').glob('voxel*.yaml')) + [ROOT/'config/lidar360_os0.yaml']
                       + list(SUITE.glob('*.yaml')) + [SUITE/'suite.json']
                       + list((ROOT/'worlds/indoor_suite').glob('*.sdf'))))
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    with tarfile.open(output/'source_snapshot.tar.gz', 'w:gz') as archive:
        for p in files:
            archive.add(p, arcname=str(p.relative_to(ROOT)))
    return hashes


def inspect_motion(folder, case):
    """Static path clearance and sampled inter-robot separation, using actual paths."""
    _, _, ground, _, _, _, _ = scene(ROOT/case['config'], ROOT/case['world'])
    frames, curve = [], []
    for path in sorted((folder/'frames').glob('frame_*.npz')):
        with np.load(path) as f:
            stamp = float(f['elapsed'])
            routes = [f[f'route{r}'].reshape(-1, 2).copy() for r in range(2)]
            poses = f['poses'][:, :2].copy()
            for points in routes + [poses]:
                for point in points:
                    assert ground.valid(ground.cell(point), ground.safe), (case['id'], stamp, point)
            metadata = json.loads(str(f['metadata']))
            curve.append(dict(time=stamp, unknown=metadata['stats'].get('unknown')))
            frames.append((stamp, poses, [route.tolist() for route in routes]))
    closest, close_at, close_count = float('inf'), 0., 0
    for (a, poses, routes), (b, _, _) in zip(frames, frames[1:]):
        for elapsed in np.arange(0., b-a+1e-9, .1):
            positions = [advance(route, .22*elapsed) if len(route) > 1 else poses[r]
                         for r, route in enumerate(routes)]
            separation = float(np.linalg.norm(np.asarray(positions[0])-positions[1]))
            if separation < closest:
                closest, close_at = separation, a+elapsed
            close_count += separation < .5
    return dict(static_path_clearance_passed=True, sampled_min_robot_separation_m=closest,
                closest_approach_time=close_at, sampled_separation_below_0_5m=close_count,
                separation_sample_period=.1, coverage_curve=curve)


def run_case(case, cpus, args, hashes):
    folder = args.output/case['id']
    folder.mkdir(exist_ok=True)
    shutil.copy2(ROOT/case['preview'], folder/'layout.png')
    archive = folder/'source_snapshot.tar.gz'
    if not archive.exists():
        archive.symlink_to('../source_snapshot.tar.gz')
    command = ['taskset', '-c', ','.join(map(str, cpus)), sys.executable,
               str(ROOT/'scripts/demo_voxel360_a1.py'), '--world', str(ROOT/case['world']),
               '--config', str(ROOT/case['config']), '--duration', str(args.duration),
               '--simulation-rate', '0', '--record-period', '1', '--output-dir', str(folder)]
    manifest = dict(started_at_utc=datetime.now(timezone.utc).isoformat(), source_sha256=hashes,
                    config=case['config'], world=case['world'], case=case, command=command,
                    cpu_affinity=cpus, concurrent_replays=args.jobs, simulated_seconds_limit=args.duration,
                    replan_policy='async_v2', planning_time_model='measured_latency',
                    record_period=1., playback_speed=1.,
                    timing_note='Separate physical cores per case; shared machine/memory and no repeated trials.')
    atomic_json(folder/'run_manifest.json', manifest)
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    start = time.monotonic()
    print(f'START {case["id"]} cpus={cpus}', flush=True)
    result = dict(id=case['id'], layout=case['layout'], spawn=case['spawn'], starts=case['starts'])
    try:
        with (folder/'run.log').open('w') as log:
            subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
        with (folder/'validation.log').open('w') as log:
            subprocess.run([sys.executable, str(ROOT/'scripts/verify_voxel360_async.py'), str(folder)],
                           cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
        result.update(metrics(folder))
        result.update(inspect_motion(folder, case))
        report = json.loads((folder/'events.json').read_text())
        result.update(status='verified', unknown_reduction_percent=100*(1-result['final_unknown']/result['initial_unknown']),
                      observed_volume_percent=100*(1-result['final_unknown']/report['voxel_count']),
                      idle_searches=len(report['idle_search_events']),
                      post_startup_idle_seconds=[max(0., x-result['startup_seconds']) for x in result['total_idle_seconds']],
                      saved_points=report['saved_points'], point_storage_bytes=report['point_storage_bytes'],
                      classification_fresh=report['classification_fresh'])
    except Exception as error:
        result.update(status='failed', error=str(error))
        (folder/'failure.log').write_text(traceback.format_exc())
        print(f'FAIL {case["id"]}: {error}', flush=True)
    result['wall_seconds'] = time.monotonic()-start
    atomic_json(folder/'result.json', result)
    print(f'DONE {case["id"]} {result["status"]} wall={result["wall_seconds"]:.1f}s', flush=True)
    return result


def write_report(output, cases, results, args):
    by_id = {r['id']: r for r in results}
    rows = [by_id[c['id']] for c in cases if c['id'] in by_id]
    atomic_json(output/'summary.json', dict(planned=len(cases), finished=len(rows), duration_cap=args.duration,
                verified=sum(r['status'] == 'verified' for r in rows), cases=rows,
                caveats=['Offline finite-beam replay with known 2D ground prior; not a ROS/Gazebo/hardware trial.',
                         'Unknown reduction is relative to each case after its initial scan, not observable-space completion.',
                         'One run per scene/start; measured latency changes trajectories. Timing is descriptive.',
                         'Inter-robot trajectory collision avoidance is not implemented; sampled separation is reported.']))
    fields = ['id', 'status', 'seconds', 'initial_unknown', 'final_unknown', 'unknown_reduction_percent',
              'observed_volume_percent', 'mean_planning_seconds', 'p95_planning_seconds',
              'planning_requests', 'distance_m', 'post_startup_idle_seconds', 'idle_searches',
              'sampled_min_robot_separation_m', 'complete', 'stop_reason', 'wall_seconds']
    buffer = io.StringIO()
    writer = csv.DictWriter(buffer, fieldnames=fields, extrasaction='ignore')
    writer.writeheader()
    writer.writerows(rows)
    (output/'summary.csv').write_text(buffer.getvalue())
    cards = []
    for c in cases:
        r = by_id.get(c['id'])
        content = '<p>等待运行 / running or queued</p>'
        if r and r['status'] == 'verified':
            content = (f'<p>未知减少 <b>{r["unknown_reduction_percent"]:.2f}%</b> · 剩余 {r["final_unknown"]:,}</p>'
                       f'<p>规划均值 / P95：{r["mean_planning_seconds"]:.3f} / {r["p95_planning_seconds"]:.3f} s</p>'
                       f'<p>启动后空闲 R1 / R2：{r["post_startup_idle_seconds"][0]:.2f} / '
                       f'{r["post_startup_idle_seconds"][1]:.2f} s</p>'
                       f'<p>采样最小车距：{r["sampled_min_robot_separation_m"]:.2f} m · {r["stop_reason"]}</p>'
                       f'<p><a href="{c["id"]}/exploration.mp4">视频</a> · '
                       f'<a href="{c["id"]}/final.png">最终观测</a> · '
                       f'<a href="{c["id"]}/result.json">数据</a> · '
                       f'<a href="{c["id"]}/validation.json">核验</a></p>')
        elif r:
            content = '<p class="error">'+html.escape(r.get('error', 'failed'))+'</p>'
        cards.append(f'<article><h2>{html.escape(LABELS[c["layout"]])}</h2><p>{c["spawn"]}</p>'
                     f'<img loading="lazy" src="{c["id"]}/layout.png" alt="Layout and starts">{content}</article>')
    (output/'index.html').write_text('''<!doctype html><html lang="zh"><meta charset="utf-8">
<title>室内双车探索测试</title><style>
body{font-family:system-ui,sans-serif;background:#101720;color:#e4ebf3;max-width:1500px;margin:36px auto;padding:0 22px}
h1{font-size:30px}h2{font-size:18px}p{line-height:1.6}a{color:#65bfff}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(350px,1fr));gap:20px}
article{background:#1b2634;padding:20px;border-radius:12px}img{width:100%;border-radius:6px}.error{color:#ff9b9b}
</style>'''+f'<h1>室内双车探索 · A1 Algorithm</h1><p>6 类场景 × 3 组出生点；已完成 {len(rows)} / {len(cases)}。'
                f'每组最多 {args.duration:g} 秒，5 cm 体素、5 m / 64×1024 / 10 Hz 雷达。</p>'
                '<p>未知减少比例以首次扫描后的未知为分母，不等于可观测空间完成率。布局图是真值预览，视频只显示实测信息。'
                '这是带已知二维通行先验的离线回放；计算实测耗时计入仿真，尚未验证真车。每组仅一次运行，耗时不作为严格速度对照。'
                '当前算法未实现双车轨迹避碰，采样最小车距用于发现近距离交汇。</p>'
                '<p><a href="summary.csv">CSV 汇总</a> · <a href="summary.json">JSON 汇总</a> · '
                '<a href="overview.png">全部布局与出生点</a></p><main>'+''.join(cards)+'</main></html>')


def overview(output, cases):
    canvas = Image.new('RGB', (1200, 6*370), '#101720')
    for i, c in enumerate(cases):
        picture = Image.open(ROOT/c['preview']).resize((400, 345), Image.Resampling.LANCZOS)
        canvas.paste(picture, ((i%3)*400, (i//3)*370))
    canvas.save(output/'overview.png')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'maps/indoor_a1_suite_160s')
    parser.add_argument('--duration', type=float, default=160.)
    parser.add_argument('--jobs', type=int, default=3)
    parser.add_argument('--cases', nargs='*')
    args = parser.parse_args()
    args.output = args.output.resolve()
    if args.duration <= 0 or args.jobs < 1:
        parser.error('Use a positive regression cap and job count')
    cases = json.loads((SUITE/'suite.json').read_text())['cases']
    if args.cases:
        ids = set(args.cases)
        if ids-set(c['id'] for c in cases):
            parser.error('Unknown case id')
        cases = [c for c in cases if c['id'] in ids]
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output/'summary.json').exists():
        parser.error('Output already contains a suite; choose a new directory to preserve results')
    groups = cpu_groups(args.jobs)
    hashes = freeze(args.output)
    shutil.copy2(SUITE/'suite.json', args.output/'suite.json')
    for c in cases:
        (args.output/c['id']).mkdir(exist_ok=True)
        shutil.copy2(ROOT/c['preview'], args.output/c['id']/'layout.png')
    overview(args.output, cases)
    results = []
    write_report(args.output, cases, results, args)
    # A lane retains the same CPU set for every case; no replay shares a physical core.
    # Publish individual completions immediately, even while other lanes continue.
    from queue import Queue, Empty
    completed = Queue()
    def publishing_lane(index):
        for c in cases[index::args.jobs]:
            completed.put(run_case(c, groups[index], args, hashes))
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = [pool.submit(publishing_lane, i) for i in range(args.jobs)]
        while len(results) < len(cases):
            try:
                result = completed.get(timeout=1.)
            except Empty:
                if all(f.done() for f in futures):
                    for f in futures:
                        f.result()
                    raise RuntimeError('Worker stopped before publishing every case')
                continue
            results.append(result)
            write_report(args.output, cases, results, args)
        for f in futures:
            f.result()
    print(f'COMPLETE {len(results)} cases; report {args.output/"index.html"}', flush=True)
    return int(any(r['status'] != 'verified' for r in results))


if __name__ == '__main__':
    raise SystemExit(main())
