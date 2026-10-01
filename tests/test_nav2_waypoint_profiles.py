#!/usr/bin/python3

import math
import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from nav2_waypoint_profiles import (  # noqa: E402
    alignment_targets,
    approach_yaw_candidates,
    path_centerline_collision_reason,
    relative_waypoints,
)
from nav2_waypoint_sampling import CostmapView, Pose2D  # noqa: E402


def empty_costmap(size: int = 20, resolution: float = 0.1) -> CostmapView:
    return CostmapView(
        frame_id="map",
        resolution=resolution,
        size_x=size,
        size_y=size,
        origin_x=-1.0,
        origin_y=-1.0,
        origin_yaw=0.0,
        data=[0] * (size * size),
        collision_threshold=253,
        unknown_is_collision=True,
    )


class WaypointProfileHelperTests(unittest.TestCase):
    def test_relative_waypoints_preserve_path_yaw_during_translation(self) -> None:
        values = relative_waypoints(
            [Pose2D(1.0, 2.0, 0.4), Pose2D(1.5, 2.0, 0.6)],
            Pose2D(1.0, 1.0, 0.1),
        )

        self.assertEqual(len(values), 6)
        self.assertAlmostEqual(values[2], 0.3)
        self.assertAlmostEqual(values[5], 0.5)

    def test_approach_candidates_start_with_bearing_and_robot_yaw(self) -> None:
        candidates = approach_yaw_candidates(
            robot_point=(0.0, 0.0),
            robot_yaw=math.pi / 2.0,
            goal_point=(1.0, 0.0),
        )

        self.assertAlmostEqual(candidates[0], 0.0)
        self.assertAlmostEqual(candidates[1], math.pi / 2.0)
        self.assertEqual(len(candidates), 5)
        self.assertIn(math.pi / 4.0, candidates)
        self.assertIn(-math.pi / 4.0, candidates)
        self.assertIn(-math.pi / 2.0, candidates)

    def test_approach_candidates_dedupe_same_bearing_and_robot_yaw(self) -> None:
        candidates = approach_yaw_candidates(
            robot_point=(0.0, 0.0),
            robot_yaw=0.0,
            goal_point=(1.0, 0.0),
        )

        self.assertEqual(len(candidates), 5)
        self.assertAlmostEqual(candidates[0], 0.0)
        self.assertEqual(len({round(value, 6) for value in candidates}), 5)

    def test_alignment_targets_repeat_xy_and_cap_yaw_by_remain_time(self) -> None:
        targets = alignment_targets(
            goal_in_base=(0.05, -0.02),
            current_yaw=0.0,
            final_yaw=1.0,
            remain_times=[0.5, 1.0, 2.0],
            max_yaw_rate=0.4,
        )

        self.assertEqual([(target.x, target.y) for target in targets], [(0.05, -0.02)] * 3)
        self.assertEqual([round(target.relative_yaw, 2) for target in targets], [0.2, 0.4, 0.8])

    def test_alignment_targets_reach_final_yaw_when_cap_allows(self) -> None:
        targets = alignment_targets(
            goal_in_base=(0.0, 0.0),
            current_yaw=0.0,
            final_yaw=-0.3,
            remain_times=[0.5, 1.0],
            max_yaw_rate=1.0,
        )

        self.assertEqual([round(target.relative_yaw, 2) for target in targets], [-0.3, -0.3])

    def test_centerline_collision_checks_same_position_cell(self) -> None:
        costmap = empty_costmap()
        data = list(costmap.data)
        mx = math.floor((0.0 - costmap.origin_x) / costmap.resolution)
        my = math.floor((0.0 - costmap.origin_y) / costmap.resolution)
        data[my * costmap.size_x + mx] = 253
        costmap = CostmapView(**{**costmap.__dict__, "data": data})

        reason = path_centerline_collision_reason(
            [Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.0, math.pi)],
            costmap,
        )

        self.assertIn("pose 0", reason)

    def test_centerline_collision_checks_segments(self) -> None:
        costmap = empty_costmap()
        data = list(costmap.data)
        mx = math.floor((0.3 - costmap.origin_x) / costmap.resolution)
        my = math.floor((0.0 - costmap.origin_y) / costmap.resolution)
        data[my * costmap.size_x + mx] = 253
        costmap = CostmapView(**{**costmap.__dict__, "data": data})

        reason = path_centerline_collision_reason(
            [Pose2D(0.0, 0.0, 0.0), Pose2D(0.6, 0.0, 0.0)],
            costmap,
        )

        self.assertIn("segment 0", reason)


if __name__ == "__main__":
    unittest.main()
