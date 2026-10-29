"""Pose-continuity gate used before a 3D scan is merged into the map."""

import unittest

from cloud_stamp import pose_is_discontinuous


class PoseContinuityTest(unittest.TestCase):
    def test_normal_step_is_kept(self):
        # 0.22 m/s for one 5 Hz period.
        self.assertFalse(pose_is_discontinuous(0.2, 0.05))

    def test_teleport_in_one_period_is_dropped(self):
        # The stall-then-jump case: stamp barely moved, pose moved a metre.
        self.assertTrue(pose_is_discontinuous(0.05, 1.0))

    def test_long_gap_at_legal_speed_is_kept(self):
        # 4 s of driving at 0.4 m/s is real motion, not a ghost.
        self.assertFalse(pose_is_discontinuous(4.0, 1.6))

    def test_backward_stamp_with_a_big_shift_is_dropped(self):
        self.assertTrue(pose_is_discontinuous(-0.5, 0.8))

    def test_backward_stamp_with_noise_is_kept(self):
        self.assertFalse(pose_is_discontinuous(-0.01, 0.05))


if __name__ == "__main__":
    unittest.main()
