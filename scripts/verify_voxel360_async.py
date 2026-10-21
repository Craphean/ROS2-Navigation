#!/usr/bin/env python3
"""Audit a completed asynchronous replay's timing, motion and saved artifacts."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile

import cv2
import numpy as np

from observation_archive import POINT_DTYPE


def verify(folder):
    report = json.loads((folder/'events.json').read_text())
    assert report['replan_policy'] == 'async_v2'
    v2 = report['replan_policy'] == 'async_v2'
    assert report['planning_time_model'] == 'measured_latency'
    assert report['discarded_replans'] == 0
    duration = report['simulated_seconds']
    requests = report['planning_requests']
    accepted = [r for r in requests if r['status'] == 'accepted']
    speeds = (.22, .22)  # The recorded classroom configuration; check archived values below.
    manifest = json.loads((folder/'run_manifest.json').read_text())
    with tarfile.open(folder/'source_snapshot.tar.gz') as archive:
        for name, digest in manifest['source_sha256'].items():
            assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == digest, name
        import yaml
        config_name = manifest.get('config', 'config/voxel360_async_v2_params.yaml')
        config = yaml.safe_load(archive.extractfile(config_name).read())
        params = config['rolling_observation_planner']['ros__parameters']
        speeds = tuple(params['robot_speeds'])
        if 'world' in manifest:
            assert hashlib.sha256(archive.extractfile(manifest['world']).read()).hexdigest() == report['world_sha256']
    for index, request in enumerate(requests):
        assert request['status'] in ('accepted', 'computing')
        np.testing.assert_allclose(request['ready_time']-request['request_time'], request['planning_seconds'], atol=1e-8)
        assert max(request['scan_times']) <= request['request_time']+1e-8
        if index:
            assert request['request_time'] >= requests[index-1].get('handoff_time', requests[index-1]['ready_time'])-1e-8
        expected_handoff = max(request['ready_time'], request.get('commit_until', 0.))
        if v2:
            assert request['commit_until'] <= request['request_time']+params.get('async_commit_seconds', 1.5)+1e-8
            np.testing.assert_allclose(request['handoff_time'], expected_handoff, atol=1e-8)
            if index:
                assert request['prediction_seconds'] >= request['commit_until']-request['request_time']-1e-8
                predicted_distance = np.linalg.norm(np.asarray(request['predicted_positions'])
                                                   -np.asarray(request['robot_positions']), axis=1)
                assert np.all(predicted_distance <= np.asarray(speeds)*request['prediction_seconds']+1e-8)
        if request['status'] != 'accepted':
            assert expected_handoff > duration
            continue
        np.testing.assert_allclose(request['accepted_time'], expected_handoff, atol=1e-8)
        displacement = np.linalg.norm(np.asarray(request['handoff']['live_positions'])
                                      - np.asarray(request['robot_positions']), axis=1)
        travelled = np.asarray(request['distance_during_planning'])
        until_handoff = np.asarray(request.get('distance_until_handoff', travelled))
        assert np.all(displacement <= until_handoff+1e-8)
        assert np.all(until_handoff <= np.asarray(speeds)*(expected_handoff-request['request_time'])+1e-8)
        assert np.all(travelled <= np.asarray(speeds)*request['planning_seconds']+1e-8)
        assert request['accepted_event'] >= request['event']
    assert accepted[0]['request_time'] == 0
    assert accepted[0]['accepted_time'] > 0  # Startup planning is not free.
    assert any(sum(r['distance_during_planning']) > 0 for r in accepted[1:])
    for event in report['events']:
        assert event['trigger'] == 'arrival'
        for r in event['arrived_robots']:
            np.testing.assert_allclose(event['robot_positions'][r-1], event['task_positions'][r-1], atol=1e-8)
    previous, previous_time, planning_frames, count = None, None, 0, 0
    old_routes = {}
    for path in sorted((folder/'frames').glob('frame_*.npz')):
        with np.load(path, allow_pickle=False) as frame:
            count += 1
            assert not {'points', 'current'}.intersection(frame.files)
            stamp, positions = float(frame['elapsed']), frame['poses'][:, :2]
            assert float(frame['map_stamp']) <= stamp+1e-8
            assert max(frame['scan_stamps']) <= stamp+1e-8
            metadata = json.loads(str(frame['metadata']))
            assert metadata['route_stamp'] <= stamp+1e-8
            np.testing.assert_allclose(metadata['pose_stamps'], stamp)
            execution = metadata['stats']['execution']
            if execution['planning']:
                planning_frames += 1
                assert execution['request_time'] <= stamp <= execution.get('handoff_time', execution['ready_time'])+1e-8
                key = execution['request_time']
                if key not in old_routes:
                    np.testing.assert_allclose(stamp, key, atol=1e-8)
                    old_routes[key] = (positions.copy(), [frame[f'route{r}'].reshape(-1, 2).copy() for r in range(2)])
                origins, routes = old_routes[key]
                for r, path_points in enumerate(routes):
                    if len(path_points) < 2:
                        np.testing.assert_allclose(positions[r], origins[r], atol=1e-8)
                    else:
                        a, vector = path_points[:-1], np.diff(path_points, axis=0)
                        norm = np.sum(vector*vector, axis=1)
                        t = np.clip(np.sum((positions[r]-a)*vector, axis=1)/np.maximum(norm, 1e-30), 0, 1)
                        assert np.min(np.linalg.norm(positions[r]-a-t[:, None]*vector, axis=1)) < 1e-7, path
            if previous is not None:
                assert stamp >= previous_time
                assert np.all(np.linalg.norm(positions-previous, axis=1)
                              <= np.asarray(speeds)*(stamp-previous_time)+1e-8), path
            previous, previous_time = positions.copy(), stamp
    assert planning_frames > 0
    with np.load(folder/'final_voxels.npz', allow_pickle=False) as final:
        assert np.count_nonzero(final['state'] == -1) == report['final_unknown']
    points = np.fromfile(folder/'observed_points.bin', dtype=POINT_DTYPE)
    assert len(points) == report['saved_points']
    assert points.nbytes == report['point_storage_bytes']
    assert np.isfinite(points['xyz']).all()
    assert np.all(np.diff(points['first_seen']) >= 0)
    assert points['first_seen'].max(initial=0) <= duration
    video = cv2.VideoCapture(str(folder/'exploration.mp4'))
    assert video.isOpened()
    frames, fps = int(video.get(cv2.CAP_PROP_FRAME_COUNT)), video.get(cv2.CAP_PROP_FPS)
    assert fps == 10 and frames == round(duration/report['playback_speed']*fps)
    video.set(cv2.CAP_PROP_POS_FRAMES, frames-1)
    assert video.read()[0]
    video.release()
    values = np.array([r['planning_seconds'] for r in requests])
    output = dict(
        verified=True, simulated_seconds=duration, video_seconds=frames/fps,
        accepted_plans=len(accepted), requested_plans=len(requests), arrivals=len(report['events']),
        planning_mean_seconds=float(values.mean()), planning_p95_seconds=float(np.percentile(values, 95)),
        planning_max_seconds=float(values.max()), planning_total_seconds=float(values.sum()),
        motion_during_planning_m=report['motion_during_planning_m'],
        idle_during_planning_seconds=report['idle_during_planning_seconds'],
        handoffs_after_additional_arrivals=sum(r['accepted_event'] > r['event'] for r in accepted),
        removed_obsolete_or_unreachable_tasks=sum(r['handoff']['filtered_tasks'] for r in accepted),
        final_unknown=report['final_unknown'], robot_distance_m=report['robot_distance_m'],
        complete=report['complete'], stop_reason=report['stop_reason'],
        saved_frames=count, frames_during_planning=planning_frames, point_bytes=points.nbytes,
        checks=['archived source hashes', 'serialized/coalesced requests', 'measured computation latency',
                'motion along the previous route during planning', 'actual arrivals', 'no pose jump at handoff',
                'fresh frame timestamps', 'incremental point records', 'final voxel count', 'MP4 last frame'],
    )
    output.update(replan_policy=report['replan_policy'],
                  idle_total_seconds=report.get('idle_total_seconds',
                      [duration-d/s for d, s in zip(report['robot_distance_m'], speeds)]),
                  early_refills=len(report.get('early_refill_events', [])),
                  goal_switches=report.get('goal_switches'),
                  ready_plan_commit_wait_seconds=sum(r['accepted_time']-r['ready_time'] for r in accepted))
    (folder/'validation.json').write_text(json.dumps(output, indent=2)+'\n')
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', nargs='?', type=Path,
                        default=Path(__file__).resolve().parents[1]/'maps/classroom_voxel360_async_v2_r2fix_160s')
    verify(parser.parse_args().folder)
