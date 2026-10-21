"""Completion must follow fresh map evidence, never a duration or empty queue."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from continuous_voxel_replay import run_continuous
from voxel_replay_io import parse_args
from observation_visualization import MapImage
from voxel_exploration_core import (
    FREE, CompletionMonitor, Coverage, GroundGrid, RoutePlan, SensorModel,
    VoxelMap, coverage_exhausted,
)


class CompletionTests(unittest.TestCase):
    def test_default_has_no_time_or_event_limit(self):
        args = parse_args([])
        self.assertEqual(args.duration, 0.)
        self.assertIsNone(args.max_events)

    def test_empty_queue_or_unclassified_unknown_is_not_completion(self):
        voxels = VoxelMap([0, 3, 0, 1, 0, 1], 1.)
        coverage = Coverage([], set(), set(), {0, 1}, 0, 0)
        self.assertFalse(coverage_exhausted(voxels, coverage))
        coverage.excluded.add(2)
        self.assertTrue(coverage_exhausted(voxels, coverage))
        coverage.excluded.add(3)  # A mismatched classification is also rejected.
        self.assertFalse(coverage_exhausted(voxels, coverage))

    def test_deferred_or_actionable_unknown_blocks_completion(self):
        voxels = VoxelMap([0, 1, 0, 1, 0, 1], 1.)
        for actionable, deferred in (({0}, set()), (set(), {0})):
            coverage = Coverage([], actionable, deferred, set(), 0, 0)
            self.assertFalse(coverage_exhausted(voxels, coverage))

    def test_stable_window_restarts_after_new_evidence(self):
        monitor = CompletionMonitor(8.)
        inputs = dict(exhausted=True, version=1, fresh=True, idle=True)
        self.assertFalse(monitor.update(10., **inputs))
        self.assertFalse(monitor.update(17., **inputs))
        self.assertTrue(monitor.update(18., **inputs))
        inputs["version"] = 2
        self.assertFalse(monitor.update(20., **inputs))
        self.assertFalse(monitor.update(27., **inputs))
        self.assertTrue(monitor.update(28., **inputs))

    def test_stale_pending_moving_or_unresolved_state_resets_window(self):
        for change in (dict(fresh=False), dict(planning=True), dict(idle=False), dict(exhausted=False)):
            monitor = CompletionMonitor(8.)
            inputs = dict(exhausted=True, version=1, fresh=True, idle=True)
            monitor.update(0., **inputs)
            self.assertFalse(monitor.update(1000., **(inputs | change)))
            self.assertIsNone(monitor.since)
            self.assertFalse(monitor.update(1001., **inputs))

    def run_small_replay(self, directory, duration=0., deferred=False, max_events=None):
        root = Path(directory)
        world = root/"world.sdf"
        world.write_text("<sdf/>")
        truth = VoxelMap([0, 2, 0, 2, 0, 1], .5)
        truth.state[:] = FREE
        known = truth.snapshot(empty=deferred)
        ground = GroundGrid(np.zeros((4, 4), dtype=np.int8), .5, clearance=0.)
        starts = [(.25, .25), (1.75, 1.75)]
        sensor = SensorModel(height=.25, max_range=5.)
        config = dict(sensor_max_range=5., min_clearance=0., robot_speeds=[.22, .22],
                      robot1_init_pose=[*starts[0], 0.], robot2_init_pose=[*starts[1], 0.],
                      observation_time=1., completion_stable_seconds=8., idle_replan_period=5.,
                      candidate_position_stride=.5, exact_candidate_limit=10, failed_view_cooldown=30.)
        args = SimpleNamespace(output_dir=root, config=None, world=world, resolution=None,
                               sensor_range=None, duration=duration, max_events=max_events,
                               scan_hz=5., record_period=1., playback_speed=1., simulation_rate=0.)
        grid = MapImage(np.zeros((4, 4), dtype=np.int8), .5, (0., 0., 0.), 0.)
        coverage = Coverage([], set(), set(map(int, known.unknown)), set(), 0, 0)
        plan = RoutePlan([[], []], (0., 0.), 0., "empty", [[starts[0]], [starts[1]]])
        with patch("continuous_voxel_replay.scene", return_value=(truth, known, ground, starts, sensor, [], config)), \
                patch("continuous_voxel_replay.observe", return_value=0), \
                patch("continuous_voxel_replay.PlanarObservations") as planar, \
                patch("continuous_voxel_replay.VideoOutput") as video, \
                patch("continuous_voxel_replay.compute_plan", return_value=(coverage, plan, 0.)), \
                patch("continuous_voxel_replay.ObservationRenderer.render", return_value=np.zeros((16, 16, 3), dtype=np.uint8)), \
                contextlib.redirect_stdout(io.StringIO()):
            planar.return_value.max_range = 5.
            planar.return_value.image.return_value = grid
            video.return_value.frames_written = 0
            # Populate empty current scans while retaining unknown evidence.
            def scan(_truth, _known, _xy, _sensor, _vectors, cloud, robot, stamp, _yaw):
                cloud.add(robot, [], stamp)
                return 0
            with patch("continuous_voxel_replay.observe", side_effect=scan):
                run_continuous(args)
        return json.loads((root/"events.json").read_text())

    def test_replay_without_duration_stops_on_completion_after_stability(self):
        with tempfile.TemporaryDirectory() as folder:
            report = self.run_small_replay(folder)
            self.assertTrue(report["complete"])
            self.assertEqual(report["stop_reason"], "complete")
            self.assertEqual(report["simulated_seconds"], 8.)
            self.assertEqual(report["requested_duration"], 0.)

    def test_explicit_cap_and_no_progress_never_report_success(self):
        with tempfile.TemporaryDirectory() as folder:
            report = self.run_small_replay(folder, duration=10., deferred=True)
            self.assertFalse(report["complete"])
            self.assertEqual(report["stop_reason"], "duration_limit")
            self.assertGreater(report["deferred"], 0)
            self.assertGreater(len(report["replans"]), 1)  # Idle retries continue.

    def test_explicit_event_cap_is_an_interruption(self):
        with tempfile.TemporaryDirectory() as folder:
            report = self.run_small_replay(folder, deferred=True, max_events=0)
            self.assertFalse(report["complete"])
            self.assertEqual(report["stop_reason"], "event_limit")


if __name__ == "__main__":
    unittest.main()
