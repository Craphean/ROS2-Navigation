"""Scene diversity, robot spawn safety and unchanged planner configuration."""
import json
import unittest

import numpy as np
import yaml

from classroom_voxel_scene import ROOT, scene
from indoor_scenario_suite import BASE_CONFIG, SUITE


class IndoorSuiteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cases = json.loads((SUITE/'suite.json').read_text())['cases']

    def test_full_cross_product_and_distinct_geometry(self):
        self.assertEqual(len(self.cases), 18)
        self.assertEqual(len({c['id'] for c in self.cases}), 18)
        self.assertEqual(len({c['ground_sha256'] for c in self.cases}), 6)
        for layout in {c['layout'] for c in self.cases}:
            self.assertEqual({c['spawn'] for c in self.cases if c['layout'] == layout},
                             {'front_split', 'rear_together', 'diagonal'})

    def test_no_case_specific_planner_tuning(self):
        base = yaml.safe_load(BASE_CONFIG.read_text())['rolling_observation_planner']['ros__parameters']
        keys = ('robot1_init_pose', 'robot2_init_pose')
        for c in self.cases:
            config = yaml.safe_load((ROOT/c['config']).read_text())['rolling_observation_planner']['ros__parameters']
            self.assertEqual({k: v for k, v in base.items() if k not in keys},
                             {k: v for k, v in config.items() if k not in keys})
            for key, pose in zip(keys, c['starts']):
                self.assertEqual(config[key], pose)

    def test_safe_connected_spawns_and_room_access(self):
        for c in self.cases:
            _, _, ground, starts, _, _, _ = scene(ROOT/c['config'], ROOT/c['world'])
            distance = ground.distances(starts[0])
            self.assertTrue(all(ground.valid(ground.cell(s), ground.safe) for s in starts))
            self.assertTrue(np.isfinite(distance[ground.cell(starts[1])]))
            self.assertGreaterEqual(np.linalg.norm(np.subtract(*starts)), 1.)
            self.assertEqual(int(np.isfinite(distance).sum()), int(ground.safe.sum()))
            if c['layout'] == 'multiroom':
                for xy in ((-4., 3.), (4., 3.), (-4., -3.), (4., -3.), (0., 0.)):
                    self.assertTrue(np.isfinite(distance[ground.cell(xy)]))


if __name__ == '__main__':
    unittest.main()
