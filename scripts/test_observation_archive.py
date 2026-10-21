"""Sparse point history size, acquisition timing and frame round trips."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

from voxel_replay_io import save_frame
from observation_archive import POINT_DTYPE, PointTimeline, PointTimelineWriter
from observation_visualization import MapImage, ObservedPoints


class PointArchiveTests(unittest.TestCase):
    def test_rescans_never_duplicate_or_overwrite_first_measured_point(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"observed_points.bin"
            writer = PointTimelineWriter(path)
            cloud = ObservedPoints(.05)
            cloud.add(0, [[.011, .012, .013]], 1.25)
            writer.append(cloud)
            for stamp in range(2, 102):
                cloud.add(1, [[.021, .022, .023]], float(stamp))
                writer.append(cloud)
            writer.close()
            self.assertEqual(path.stat().st_size, POINT_DTYPE.itemsize)
            timeline = PointTimeline(path)
            np.testing.assert_allclose(timeline.records["xyz"], [[.011, .012, .013]])
            np.testing.assert_allclose(timeline.records["first_seen"], [1.25])

    def test_reconstruct_only_points_already_observed_at_requested_time(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"observed_points.bin"
            writer = PointTimelineWriter(path)
            cloud = ObservedPoints(.05)
            cloud.add(0, [[1., 1., 1.]], .2)
            writer.append(cloud)
            cloud.add(1, [[2., 2., 2.]], 2.4)
            writer.append(cloud)
            writer.close()
            timeline = PointTimeline(path)
            self.assertEqual(len(timeline.at(.1, -1., [0., 0.]).points), 0)
            np.testing.assert_equal(timeline.at(1., -1., [.2, 0.]).points, [[1., 1., 1.]])
            view = timeline.at(3., 1., [3., 3.])
            self.assertEqual(len(view.points), 2)
            np.testing.assert_equal(view.current, [[2., 2., 2.]])

    def test_frame_has_no_point_arrays_and_reconstructs_same_render_input(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            cloud = ObservedPoints()
            cloud.add(0, [[1., 1., .2]], .4)
            writer = PointTimelineWriter(root/"observed_points.bin")
            writer.append(cloud)
            writer.close()
            renderer = Mock(bounds=np.array([0., 2., 0., 2., 0., 1.]))
            renderer.render.return_value = np.zeros((16, 16, 3), dtype=np.uint8)
            grid = MapImage(np.zeros((2, 2), dtype=np.int8), 1., (0., 0., 0.), 1.)
            plan = SimpleNamespace(paths=[[], []], makespan=0.)
            coverage = SimpleNamespace(actionable=set(), deferred=set(), excluded=set())
            save_frame(root, renderer, grid, cloud, [(0., 0., 0.), (1., 1., 0.)],
                       plan, [[], []], coverage, 0, 1., "running", 0., 0, 0)
            with np.load(root/"frames/frame_0000.npz") as frame:
                self.assertNotIn("points", frame.files)
                self.assertNotIn("current", frame.files)
                view = PointTimeline(root/"observed_points.bin").at(
                    float(frame["elapsed"]), float(frame["new_points_since"]), frame["scan_stamps"])
                original = renderer.render.call_args.args[1]
                np.testing.assert_equal(view.points, original.points)
                np.testing.assert_equal(view.current, original.current)
                self.assertEqual(json.loads(str(frame["metadata"]))["highlight_label"], "new points")

    def test_empty_archive_is_readable(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"observed_points.bin"
            PointTimelineWriter(path).close()
            self.assertEqual(len(PointTimeline(path).at(10., 0., [10., 10.]).points), 0)

    def test_atomic_file_replacement_does_not_lose_subsequent_incremental_points(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"points.bin"
            writer = PointTimelineWriter(path)
            cloud = ObservedPoints(.05)
            cloud.add(0, [[1., 1., 1.]], 1.)
            writer.append(cloud)
            writer.flush()
            replacement = path.with_suffix(".copy")
            replacement.write_bytes(path.read_bytes())
            replacement.replace(path)
            cloud.add(1, [[2., 2., 2.]], 2.)
            writer.append(cloud)
            writer.close()
            self.assertEqual(path.stat().st_size, 2*POINT_DTYPE.itemsize)
            np.testing.assert_equal(PointTimeline(path).records["first_seen"], [1., 2.])

    def test_changed_archive_length_fails_before_reporting_success(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"points.bin"
            writer = PointTimelineWriter(path)
            cloud = ObservedPoints(.05)
            cloud.add(0, [[1., 1., 1.]], 1.)
            writer.append(cloud)
            writer.flush()
            path.write_bytes(b"")
            with self.assertRaisesRegex(ValueError, "length"):
                writer.close()

if __name__ == "__main__":
    unittest.main()
