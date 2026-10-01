import unittest

from lio_web.trajectory import TrajectoryHistory


class TrajectoryHistoryTest(unittest.TestCase):
    def test_long_route_keeps_endpoints_corners_and_recent_detail(self):
        history = TrajectoryHistory(32)
        route = [[i * 0.1, 0.0, 0.0] for i in range(101)]
        route += [[10.0, i * 0.1, 0.0] for i in range(1, 101)]
        route += [[10.0 - i * 0.1, 10.0, 0.0] for i in range(1, 101)]
        for pose in route:
            history.append(pose)
            self.assertLessEqual(len(history.snapshot()), 32)
        kept = history.snapshot()
        self.assertEqual(kept[0], route[0])
        self.assertEqual(kept[-8:], route[-8:])
        self.assertIn([10.0, 0.0, 0.0], kept)
        self.assertIn([10.0, 10.0, 0.0], kept)

    def test_short_steps_are_skipped_and_reset_starts_new_route(self):
        history = TrajectoryHistory(8)
        history.append([0.0, 0.0, 0.0])
        history.append([0.01, 0.0, 0.0])
        self.assertEqual(history.snapshot(), [[0.0, 0.0, 0.0]])
        history.clear()
        history.append([2.0, 3.0, 4.0])
        self.assertEqual(history.snapshot(), [[2.0, 3.0, 4.0]])

    def test_rejects_too_small_budget(self):
        with self.assertRaises(ValueError):
            TrajectoryHistory(3)


if __name__ == '__main__':
    unittest.main()
