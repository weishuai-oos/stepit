#!/usr/bin/env python3
"""Contract tests for the planner-agnostic timed waypoint core."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


REPO_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_DIR / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from nav2_waypoint_profiles import relative_waypoints  # noqa: E402
from nav2_waypoint_sampling import (  # noqa: E402
    CanonicalPath,
    Polyline,
    Pose2D,
    wrap_to_pi,
)
from nav2_waypoint_trajectory import (  # noqa: E402
    PersistentTrajectory,
    RollingDeadlineClock,
    TrajectoryLimits,
)


def limits(**overrides) -> TrajectoryLimits:
    values = {
        "waypoint_interval": 0.5,
        "num_waypoints": 5,
        "cruise_speed": 0.4,
        "max_acceleration": 1.0,
        "max_deceleration": 1.0,
        "terminal_deceleration": 1.0,
        "max_lateral_acceleration": 0.8,
        "max_yaw_rate": 0.8,
        "integration_step": 0.01,
        "curvature_window": 0.10,
        "replan_commit_time": 0.5,
        "replan_join_distance": 0.30,
        "replan_max_waypoint_shift": 1.0,
        "replan_max_heading_shift": 0.60,
    }
    values.update(overrides)
    if "terminal_deceleration" not in overrides:
        values["terminal_deceleration"] = values["max_deceleration"]
    return TrajectoryLimits(**values)


class PersistentTrajectoryTests(unittest.TestCase):
    def test_new_limits_are_positive_and_terminal_deceleration_is_bounded(self) -> None:
        with self.assertRaises(ValueError):
            limits(terminal_deceleration=0.0)
        with self.assertRaises(ValueError):
            limits(terminal_deceleration=1.1, max_deceleration=1.0)
        with self.assertRaises(ValueError):
            limits(replan_max_waypoint_shift=float("nan"))
        with self.assertRaises(ValueError):
            limits(replan_max_heading_shift=-0.1)

    def test_initial_horizon_uses_absolute_deadlines(self) -> None:
        trajectory = PersistentTrajectory(limits())
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=10.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )

        snapshot = trajectory.snapshot(10.0)
        self.assertEqual(
            [round(point.deadline, 6) for point in snapshot.points],
            [10.5, 11.0, 11.5, 12.0, 12.5],
        )
        self.assertEqual(snapshot.remain_time, (0.5, 1.0, 1.5, 2.0, 2.5))

    def test_remain_time_counts_down_without_reanchoring(self) -> None:
        trajectory = PersistentTrajectory(limits())
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )

        snapshot = trajectory.snapshot(0.2)
        self.assertEqual(snapshot.remain_time, (0.3, 0.8, 1.3, 1.8, 2.3))
        self.assertEqual(
            [round(point.pose.x, 6) for point in snapshot.points],
            [0.2, 0.4, 0.6, 0.8, 1.0],
        )

    def test_expiration_shifts_queue_and_only_appends_tail(self) -> None:
        trajectory = PersistentTrajectory(limits())
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )
        before = trajectory.snapshot(0.0).points

        after = trajectory.snapshot(0.51).points

        self.assertEqual(after[:4], before[1:])
        self.assertEqual(after[-1].deadline, 3.0)
        self.assertAlmostEqual(after[-1].pose.x, 1.2, places=5)
        self.assertTrue(all(value > 0.0 for value in trajectory.snapshot(0.51).remain_time))

    def test_world_targets_persist_when_only_robot_pose_changes(self) -> None:
        trajectory = PersistentTrajectory(limits())
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )
        world_targets = tuple(point.pose for point in trajectory.snapshot(0.2).points)

        at_origin = relative_waypoints(world_targets, Pose2D(0.0, 0.0, 0.0))
        after_motion = relative_waypoints(world_targets, Pose2D(0.1, 0.0, 0.0))

        self.assertEqual(
            tuple(point.pose for point in trajectory.snapshot(0.2).points),
            world_targets,
        )
        for first, second in zip(at_origin[0::3], after_motion[0::3]):
            self.assertAlmostEqual(first - second, 0.1, places=6)

    def test_timer_delay_advances_multiple_expired_points(self) -> None:
        trajectory = PersistentTrajectory(limits())
        trajectory.activate(
            Polyline([(0.0, 0.0), (10.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )

        snapshot = trajectory.snapshot(2.2)

        self.assertEqual(len(snapshot.points), 5)
        self.assertTrue(all(value > 0.0 for value in snapshot.remain_time))
        self.assertAlmostEqual(snapshot.points[0].deadline, 2.5)
        self.assertAlmostEqual(snapshot.points[-1].deadline, 4.5)

    def test_start_acceleration_respects_limit(self) -> None:
        trajectory = PersistentTrajectory(
            limits(cruise_speed=1.0, max_acceleration=0.4)
        )
        trajectory.activate(
            Polyline([(0.0, 0.0), (10.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.0,
            start_yaw=0.0,
        )

        points = trajectory.snapshot(0.0).points
        for index, point in enumerate(points, start=1):
            self.assertLessEqual(point.speed, 0.4 * 0.5 * index + 1e-6)
        self.assertLess(points[0].pose.x, 0.1)

    def test_activate_keeps_tracking_reference_at_or_below_cruise(self) -> None:
        trajectory = PersistentTrajectory(limits(cruise_speed=0.4, max_deceleration=0.1))
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0, start_progress=0.0, start_speed=0.8, start_yaw=0.0,
        )
        self.assertLessEqual(trajectory.snapshot(0.0).points[0].speed, 0.4)

    def test_terminal_approach_reseeds_suffix_from_actual_speed(self) -> None:
        trajectory = PersistentTrajectory(
            limits(
                replan_commit_time=0.5,
                terminal_deceleration=0.5,
                cruise_speed=1.0,
                max_acceleration=0.2,
            )
        )
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0, start_progress=0.0, start_speed=0.2, start_yaw=0.0,
        )
        before = trajectory.snapshot(0.1).points
        self.assertTrue(
            trajectory.terminal_approach_feasible(
                now=0.1,
                actual_speed=0.9,
                response_time=0.2,
            )
        )
        trajectory.begin_terminal_approach(
            now=0.1,
            actual_speed=0.9,
            response_time=0.2,
        )
        after = trajectory.snapshot(0.1).points
        self.assertEqual(after[0], before[0])
        self.assertEqual([p.deadline for p in after], [p.deadline for p in before])
        self.assertEqual(trajectory.mode, trajectory.TERMINAL_APPROACH)
        self.assertGreater(trajectory._tail_speed, before[-1].speed)

    def test_terminal_approach_rejects_measured_overspeed_outside_horizon(self) -> None:
        trajectory = PersistentTrajectory(
            limits(
                cruise_speed=0.8,
                terminal_deceleration=0.5,
                replan_commit_time=0.5,
            )
        )
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.8,
            start_yaw=0.0,
        )

        self.assertFalse(
            trajectory.terminal_approach_feasible(
                now=0.0,
                actual_speed=1.6,
                response_time=0.2,
            )
        )

    def test_terminal_approach_rejects_anchor_before_predicted_overshoot(self) -> None:
        trajectory = PersistentTrajectory(
            limits(
                cruise_speed=0.8,
                terminal_deceleration=0.5,
                replan_commit_time=0.5,
            )
        )
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.8,
            start_yaw=0.0,
        )
        anchor = trajectory.snapshot(0.0).points[0]

        self.assertFalse(
            trajectory.terminal_approach_feasible(
                now=0.0,
                actual_speed=0.8,
                response_time=0.2,
                actual_progress=anchor.progress - 0.05,
            )
        )
        before = trajectory.snapshot(0.0).points
        self.assertFalse(
            trajectory.begin_terminal_approach(
                now=0.0,
                actual_speed=0.8,
                response_time=0.2,
                actual_progress=anchor.progress - 0.05,
            )
        )
        self.assertEqual(trajectory.snapshot(0.0).points, before)

    def test_terminal_approach_rejects_stop_beyond_remaining_path(self) -> None:
        trajectory = PersistentTrajectory(
            limits(
                cruise_speed=0.8,
                terminal_deceleration=0.5,
                replan_commit_time=0.5,
            )
        )
        trajectory.activate(
            Polyline([(0.0, 0.0), (0.5, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.8,
            start_yaw=0.0,
        )

        self.assertFalse(
            trajectory.terminal_approach_feasible(
                now=0.0,
                actual_speed=0.8,
                response_time=0.2,
            )
        )

    def test_terminal_approach_never_accelerates_or_crosses_path_endpoint(
        self,
    ) -> None:
        trajectory = PersistentTrajectory(
            limits(
                cruise_speed=0.8,
                max_acceleration=1.2,
                terminal_deceleration=0.5,
                replan_commit_time=0.5,
                integration_step=0.02,
            )
        )
        trajectory.activate(
            Polyline([(0.0, 0.0), (1.0, 0.0)]),
            now=5.0,
            start_progress=0.0,
            start_speed=0.8,
            start_yaw=0.0,
            start_point=(0.0, 0.0),
        )
        trajectory.begin_terminal_approach(
            now=5.0,
            actual_speed=0.8,
            response_time=0.2,
        )

        points = trajectory.snapshot(5.0).points
        self.assertTrue(
            all(
                current.speed <= previous.speed + 1e-9
                for previous, current in zip(points, points[1:])
            )
        )
        self.assertLessEqual(max(point.pose.x for point in points), 1.0 + 1e-9)
        self.assertAlmostEqual(points[-1].speed, 0.0)

    def test_measured_speed_seeds_bounded_emergency_braking_suffix(self) -> None:
        trajectory = PersistentTrajectory(
            limits(
                cruise_speed=0.8,
                max_deceleration=1.0,
                replan_commit_time=0.5,
            )
        )
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.8,
            start_yaw=0.0,
        )
        committed = trajectory.snapshot(0.0).points[0]

        trajectory.begin_braking(now=0.0, actual_speed=1.6)
        points = trajectory.snapshot(0.0).points

        self.assertEqual(points[0], committed)
        self.assertGreater(
            points[1].pose.x - points[0].pose.x,
            committed.speed * 0.5,
        )
        for previous, current in zip(points[1:], points[2:]):
            self.assertLessEqual(previous.speed - current.speed, 0.5 + 1e-6)
            self.assertGreaterEqual(previous.speed + 1e-6, current.speed)

    def test_translation_heading_follows_reverse_xy_tangent(self) -> None:
        trajectory = PersistentTrajectory(limits(cruise_speed=0.4))
        trajectory.activate(
            Polyline([(0.0, 0.0), (-5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.0,
            start_yaw=0.0,
            start_point=(0.1, 0.0),
        )

        first = trajectory.snapshot(0.0).points[0]

        self.assertLess(first.pose.x, 0.0)
        self.assertAlmostEqual(first.pose.yaw, math.pi, places=6)

    def test_simple_heading_submillimeter_targets_keep_previous_heading(self) -> None:
        start_yaw = -0.4
        trajectory = PersistentTrajectory(
            limits(cruise_speed=0.001, max_acceleration=1.0)
        )
        trajectory.activate(
            Polyline([(0.0, 0.0), (0.0, 1.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.001,
            start_yaw=start_yaw,
            start_point=(0.0, 0.0),
        )

        points = trajectory.snapshot(0.0).points

        self.assertLessEqual(math.hypot(points[0].pose.x, points[0].pose.y), 1e-3)
        for previous, current in zip(points, points[1:]):
            self.assertLessEqual(
                math.hypot(
                    current.pose.x - previous.pose.x,
                    current.pose.y - previous.pose.y,
                ),
                1e-3,
            )
        self.assertTrue(
            all(
                math.isclose(point.pose.yaw, start_yaw, abs_tol=1e-9)
                for point in points
            )
        )

    def test_rolling_submillimeter_tail_keeps_shifted_heading(self) -> None:
        start_yaw = 0.35
        trajectory = PersistentTrajectory(limits(cruise_speed=0.001))
        trajectory.activate(
            Polyline([(0.0, 0.0), (0.0, 1.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.001,
            start_yaw=start_yaw,
            start_point=(0.0, 0.0),
        )
        before = trajectory.snapshot(0.0).points

        after = trajectory.snapshot(0.51).points

        self.assertEqual(after[:4], before[1:])
        tail_step = math.hypot(
            after[-1].pose.x - after[-2].pose.x,
            after[-1].pose.y - after[-2].pose.y,
        )
        self.assertLessEqual(tail_step, 1e-3)
        self.assertAlmostEqual(after[-1].pose.yaw, after[-2].pose.yaw, places=9)

    def test_polyline_corner_uses_training_simple_heading(self) -> None:
        trajectory = PersistentTrajectory(
            limits(
                waypoint_interval=0.5,
                cruise_speed=0.3,
                max_yaw_rate=0.8,
                curvature_window=0.15,
            )
        )
        trajectory.activate(
            Polyline([(0.0, 0.0), (0.1, 0.0), (0.1, 0.5)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.3,
            start_yaw=0.0,
        )

        points = trajectory.snapshot(0.0).points

        self.assertAlmostEqual(points[0].pose.yaw, 0.0, places=6)
        expected = math.atan2(
            points[2].pose.y - points[1].pose.y,
            points[2].pose.x - points[1].pose.x,
        )
        self.assertAlmostEqual(points[1].pose.yaw, expected, places=6)
        self.assertAlmostEqual(points[2].pose.yaw, math.pi / 2.0, places=6)

    def test_translation_speed_is_limited_by_yaw_rate_and_curvature(self) -> None:
        radius = 0.5
        path = Polyline(
            [
                (radius * math.cos(angle), radius * math.sin(angle))
                for angle in [0.0, math.pi / 12, math.pi / 6, math.pi / 4, math.pi / 3]
            ]
        )
        trajectory = PersistentTrajectory(
            limits(
                cruise_speed=1.0,
                max_lateral_acceleration=10.0,
                max_yaw_rate=0.4,
                curvature_window=0.15,
            )
        )
        trajectory.activate(
            path,
            now=0.0,
            # Start inside the curved portion so the local curvature window
            # is active from the first integration slice.
            start_progress=0.1,
            start_speed=0.0,
            start_yaw=0.0,
        )

        speeds = [point.speed for point in trajectory.snapshot(0.0).points]

        # The yaw-rate cap is active (the unconstrained cruise target is 1.0)
        # while the first slice may still include bounded acceleration.
        self.assertLess(max(speeds), 0.45)

    def test_short_path_brakes_before_goal_and_finishes_at_zero_speed(self) -> None:
        trajectory = PersistentTrajectory(
            limits(cruise_speed=1.0, max_acceleration=1.0, max_deceleration=0.5)
        )
        trajectory.activate(
            Polyline([(0.0, 0.0), (0.6, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.8,
            start_yaw=0.0,
        )

        points = trajectory.snapshot(0.0).points
        self.assertLess(points[0].speed, 0.8)
        self.assertLessEqual(
            max(point.pose.x for point in points),
            0.6 + 0.5 * 0.8**2 / 0.5 + 1e-9,
        )
        self.assertGreaterEqual(points[-1].pose.x, 0.6)
        self.assertLessEqual(
            points[-1].pose.x,
            0.6 + 0.5 * 0.8**2 / 0.5 + 1e-9,
        )
        self.assertAlmostEqual(points[-1].speed, 0.0, places=5)

    def test_endpoint_crossing_keeps_speed_bounded_before_braking(self) -> None:
        trajectory = PersistentTrajectory(
            limits(
                waypoint_interval=0.1,
                cruise_speed=1.0,
                max_deceleration=0.5,
            )
        )
        trajectory.activate(
            Polyline([(0.0, 0.0), (0.2, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.8,
            start_yaw=0.0,
        )

        points = trajectory.snapshot(0.0).points

        # The path is shorter than the stopping distance.  Reaching its
        # endpoint must not create an instantaneous speed-to-zero command.
        self.assertGreater(points[2].pose.x, 0.2)
        self.assertGreater(points[2].speed, 0.0)
        self.assertLessEqual(
            points[2].pose.x - points[1].pose.x,
            points[1].speed * 0.1 + 0.5 * 1.0 * 0.1**2 + 1e-6,
        )
        self.assertLessEqual(
            points[2].speed - points[3].speed,
            0.5 * 0.1 + 1e-6,
        )
        self.assertAlmostEqual(points[-1].speed, 0.55, places=6)

    def test_endpoint_braking_is_bounded_for_multiple_initial_speeds(self) -> None:
        for initial_speed in (0.2, 0.6, 1.0):
            trajectory = PersistentTrajectory(
                limits(
                    waypoint_interval=0.2,
                    cruise_speed=1.0,
                    max_acceleration=0.7,
                    max_deceleration=0.6,
                )
            )
            trajectory.activate(
                Polyline([(0.0, 0.0), (0.25, 0.0)]),
                now=0.0,
                start_progress=0.0,
                start_speed=initial_speed,
                start_yaw=0.0,
            )
            points = trajectory.snapshot(0.0).points
            for previous, current in zip(points, points[1:]):
                self.assertLessEqual(
                    current.pose.x - previous.pose.x,
                    previous.speed * 0.2
                    + 0.5 * 0.7 * 0.2**2
                    + 1e-6,
                )
                self.assertLessEqual(
                    previous.speed - current.speed,
                    0.6 * 0.2 + 1e-6,
                )
                self.assertLessEqual(
                    current.speed - previous.speed,
                    0.7 * 0.2 + 1e-6,
                )

    def test_pure_rotation_endpoint_never_gets_a_translational_tail(self) -> None:
        trajectory = PersistentTrajectory(limits(waypoint_interval=0.1))
        trajectory.activate(
            CanonicalPath(
                [Pose2D(1.0, 2.0, 0.0), Pose2D(1.0, 2.0, math.pi / 2.0)]
            ),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )

        points = trajectory.snapshot(0.0).points

        self.assertTrue(
            all(
                math.isclose(point.pose.x, 1.0)
                and math.isclose(point.pose.y, 2.0)
                for point in points
            )
        )

    def test_pure_rotation_yaw_steps_are_rate_bounded(self) -> None:
        trajectory = PersistentTrajectory(
            limits(waypoint_interval=0.1, max_yaw_rate=0.5)
        )
        trajectory.activate(
            CanonicalPath(
                [Pose2D(1.0, 2.0, 0.0), Pose2D(1.0, 2.0, math.pi)]
            ),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )

        points = trajectory.snapshot(0.0).points

        self.assertLess(points[1].pose.yaw, 0.0)
        self.assertLess(points[-1].pose.yaw, points[1].pose.yaw)
        for previous, current in zip(points, points[1:]):
            self.assertLessEqual(current.pose.yaw, previous.pose.yaw)
            self.assertLessEqual(
                abs(current.pose.yaw - previous.pose.yaw),
                0.5 * 0.1 + 1e-6,
            )

    def test_pure_rotation_keeps_rolling_toward_goal_after_horizon_expiry(self) -> None:
        trajectory = PersistentTrajectory(
            limits(waypoint_interval=0.1, max_yaw_rate=0.5)
        )
        trajectory.activate(
            CanonicalPath(
                [Pose2D(1.0, 2.0, 0.0), Pose2D(1.0, 2.0, math.pi)]
            ),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )

        early = trajectory.snapshot(2.6).points
        late = trajectory.snapshot(7.0).points

        self.assertGreater(early[-1].pose.yaw, -math.pi + 0.1)
        self.assertLess(late[-1].pose.yaw, early[-1].pose.yaw)
        self.assertAlmostEqual(late[-1].pose.yaw, -math.pi, places=6)


    def test_curvature_reduces_speed_without_switching_distance_modes(self) -> None:
        radius = 0.4
        points = [
            (radius * math.cos(angle), radius * math.sin(angle))
            for angle in [0.0, math.pi / 12, math.pi / 6, math.pi / 4, math.pi / 3, math.pi / 2]
        ]
        trajectory = PersistentTrajectory(
            limits(cruise_speed=1.0, max_lateral_acceleration=0.16)
        )
        trajectory.activate(
            Polyline(points),
            now=0.0,
            start_progress=0.0,
            start_speed=0.0,
            start_yaw=math.pi / 2,
        )

        speeds = [point.speed for point in trajectory.snapshot(0.0).points]
        self.assertLess(max(speeds), 1.0)
        self.assertTrue(all(first.progress <= second.progress for first, second in zip(
            trajectory.snapshot(0.0).points,
            trajectory.snapshot(0.0).points[1:],
        )))

    def test_identical_replan_preserves_committed_prefix_and_deadlines(self) -> None:
        trajectory = PersistentTrajectory(limits(replan_commit_time=0.75))
        original = Polyline([(0.0, 0.0), (5.0, 0.0)])
        trajectory.activate(
            original,
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )
        before = trajectory.snapshot(0.1).points

        adopted = trajectory.adopt_replan(
            Polyline([(0.0, 0.0), (2.5, 0.0), (5.0, 0.0)]),
            now=0.1,
        )
        after = trajectory.snapshot(0.1).points

        self.assertTrue(adopted)
        self.assertEqual(after[0], before[0])
        self.assertEqual([point.deadline for point in after], [point.deadline for point in before])
        self.assertAlmostEqual(after[-1].pose.x, before[-1].pose.x, places=5)

    def test_replans_across_rolls_preserve_committed_pose_heading_and_deadline(self) -> None:
        trajectory = PersistentTrajectory(limits(replan_commit_time=0.75))
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
            start_point=(0.0, 0.0),
        )

        replans = (
            (0.1, Polyline([(0.0, 0.0), (2.5, 0.0), (5.0, 0.0)])),
            (0.6, Polyline([(0.0, 0.0), (3.0, 0.0), (5.0, 0.0)])),
        )
        for now, path in replans:
            before = trajectory.snapshot(now).points
            committed = sum(
                point.deadline - now <= 0.75 + 1e-9 for point in before
            )

            self.assertTrue(trajectory.adopt_replan(path, now=now))
            after = trajectory.snapshot(now).points

            self.assertEqual(after[:committed], before[:committed])
            self.assertEqual(
                [point.deadline for point in after],
                [point.deadline for point in before],
            )

    def test_incompatible_replan_is_rejected_without_mutating_horizon(self) -> None:
        trajectory = PersistentTrajectory(limits())
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )
        before = trajectory.snapshot(0.1).points

        adopted = trajectory.adopt_replan(
            Polyline([(0.0, 2.0), (5.0, 2.0)]),
            now=0.1,
        )

        self.assertFalse(adopted)
        self.assertEqual(trajectory.snapshot(0.1).points, before)

    def test_replan_rejects_lateral_suffix_beyond_anchor_dynamics(self) -> None:
        trajectory = PersistentTrajectory(
            limits(replan_join_distance=0.60, replan_commit_time=0.5)
        )
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )
        before = trajectory.snapshot(0.1).points

        # This is inside join_distance, but the first new suffix target is
        # more than anchor_speed*dt + 1/2*a*dt^2 away from the committed one.
        adopted = trajectory.adopt_replan(
            Polyline([(0.0, 0.5), (5.0, 0.5)]),
            now=0.1,
        )

        self.assertFalse(adopted)
        self.assertEqual(trajectory.snapshot(0.1).points, before)

    def test_replan_accepts_dynamically_reachable_lateral_suffix(self) -> None:
        trajectory = PersistentTrajectory(
            limits(replan_join_distance=0.30, replan_commit_time=0.5, max_yaw_rate=1.0)
        )
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )

        adopted = trajectory.adopt_replan(
            Polyline([(0.0, 0.1), (5.0, 0.1)]),
            now=0.1,
        )

        self.assertTrue(adopted)

    def test_replan_rejects_heading_jump_when_xy_suffix_is_reachable(self) -> None:
        trajectory = PersistentTrajectory(
            limits(
                replan_join_distance=0.30,
                max_acceleration=4.0,
                max_deceleration=4.0,
            )
        )
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
            start_point=(0.0, 0.0),
        )

        adopted = trajectory.adopt_replan(
            Polyline([(0.0, 0.0), (0.0, 5.0)]),
            now=0.1,
        )

        self.assertFalse(adopted)

    def test_legacy_replan_keeps_original_first_suffix_xy_contract(self) -> None:
        trajectory = PersistentTrajectory(
            limits(
                replan_join_distance=0.30,
                max_acceleration=4.0,
                max_deceleration=4.0,
            )
        )
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
            start_point=(0.0, 0.0),
        )

        adopted = trajectory.adopt_replan(
            Polyline([(0.0, 0.0), (0.0, 5.0)]),
            now=0.1,
            strict_continuity=False,
        )

        self.assertTrue(adopted)

    def test_pace_deadlines_shifts_only_time_axis_and_has_budget(self) -> None:
        trajectory = PersistentTrajectory(limits())
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0, start_progress=0.0, start_speed=0.4, start_yaw=0.0,
        )
        before = trajectory.snapshot(0.0).points
        shift, exhausted = trajectory.pace_deadlines(
            now=0.0, robot_point=(-1.0, 0.0), max_urgency=0.5,
            max_step=0.1, max_consecutive_delay=0.15,
        )
        after = trajectory.snapshot(0.0).points
        self.assertAlmostEqual(shift, 0.1)
        self.assertFalse(exhausted)
        self.assertEqual([p.pose for p in after], [p.pose for p in before])
        self.assertEqual([p.speed for p in after], [p.speed for p in before])
        self.assertEqual(
            [round(after[i + 1].deadline - after[i].deadline, 6) for i in range(4)],
            [0.5] * 4,
        )
        shift, exhausted = trajectory.pace_deadlines(
            now=0.0, robot_point=(-1.0, 0.0), max_urgency=0.5,
            max_step=0.1, max_consecutive_delay=0.15,
        )
        self.assertAlmostEqual(shift, 0.05)
        self.assertTrue(exhausted)

    def test_pace_budget_resets_after_healthy_tracking(self) -> None:
        trajectory = PersistentTrajectory(limits())
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0, start_progress=0.0, start_speed=0.4, start_yaw=0.0,
        )
        trajectory.pace_deadlines(
            now=0.0, robot_point=(-1.0, 0.0), max_urgency=0.5,
            max_step=0.1, max_consecutive_delay=0.1,
        )
        shift, exhausted = trajectory.pace_deadlines(
            now=0.0, robot_point=(0.2, 0.0), max_urgency=0.5,
            max_step=0.1, max_consecutive_delay=0.1,
        )
        self.assertEqual((shift, exhausted), (0.0, False))
        shift, _ = trajectory.pace_deadlines(
            now=0.0, robot_point=(-1.0, 0.0), max_urgency=0.5,
            max_step=0.1, max_consecutive_delay=0.1,
        )
        self.assertAlmostEqual(shift, 0.1)

    def test_pace_deadlines_recovers_one_late_timer_tick(self) -> None:
        trajectory = PersistentTrajectory(limits())
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )
        first = trajectory._points[0]
        now = first.deadline + 0.01

        shift, exhausted = trajectory.pace_deadlines(
            now=now,
            robot_point=first.pose.point,
            max_urgency=0.5,
            max_step=0.02,
            max_consecutive_delay=0.1,
        )

        self.assertGreater(shift, 0.01)
        self.assertFalse(exhausted)
        self.assertGreater(trajectory.snapshot(now).remain_time[0], 0.0)

    def test_pacing_does_not_chase_a_waypoint_already_passed_on_path(self) -> None:
        """A robot ahead of the first target must not consume pacing budget."""
        trajectory = PersistentTrajectory(limits())
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )

        # At t=.40 the robot is already at x=.30 while the first target is
        # x=.20 and is still due at t=.50.  Euclidean distance to that target
        # looks late to the old implementation, but path progress proves it
        # has already passed the target.
        for now in (0.40, 0.42, 0.44, 0.46, 0.48):
            shift, exhausted = trajectory.pace_deadlines(
                now=now,
                robot_point=(0.30, 0.0),
                max_urgency=0.5,
                max_step=0.02,
                max_consecutive_delay=0.10,
            )
            self.assertEqual((shift, exhausted), (0.0, False))
            self.assertEqual(trajectory.snapshot(now).remain_time[0], round(0.5 - now, 9))

        # The original deadline rolls normally; pacing must not make it move
        # into the future after the robot has passed it.
        trajectory.snapshot(0.50)
        self.assertAlmostEqual(trajectory._points[0].deadline, 1.0)

    def test_replan_rejects_default_speed_lateral_turn_without_mutation(self) -> None:
        trajectory = PersistentTrajectory(limits(replan_join_distance=0.30))
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
            start_point=(0.0, 0.0),
        )
        before = trajectory.snapshot(0.1).points

        adopted = trajectory.adopt_replan(
            Polyline([(0.0, 0.0), (0.0, 5.0)]),
            now=0.1,
        )

        self.assertFalse(adopted)
        self.assertEqual(trajectory.snapshot(0.1).points, before)

    def test_braking_keeps_first_committed_target_then_decelerates(self) -> None:
        trajectory = PersistentTrajectory(limits(replan_commit_time=0.5))
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )
        first = trajectory.snapshot(0.1).points[0]

        trajectory.begin_braking(now=0.1)
        points = trajectory.snapshot(0.1).points

        self.assertEqual(points[0], first)
        self.assertEqual(trajectory.mode, "BRAKING")
        self.assertTrue(
            all(first.speed + 1e-6 >= second.speed for first, second in zip(points, points[1:]))
        )

    def test_emergency_rebase_braking_does_not_rewind_past_actual_progress(self) -> None:
        trajectory = PersistentTrajectory(limits(replan_commit_time=0.5))
        trajectory.activate(
            Polyline([(0.0, 0.0), (5.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )
        old_anchor = trajectory.snapshot(0.0).points[0]
        actual_progress = old_anchor.progress + 0.05

        trajectory.begin_braking(
            now=0.0,
            actual_progress=actual_progress,
            actual_speed=0.4,
        )
        points = trajectory.snapshot(0.0).points

        self.assertEqual(trajectory.mode, trajectory.BRAKING)
        self.assertEqual([point.deadline for point in points], [0.5, 1.0, 1.5, 2.0, 2.5])
        self.assertGreaterEqual(points[0].progress, actual_progress - 1e-9)
        self.assertLessEqual(points[0].speed, 0.4 + 1e-9)
        self.assertTrue(
            all(previous.speed + 1e-9 >= current.speed for previous, current in zip(points, points[1:]))
        )

    def test_emergency_rebase_after_endpoint_keeps_extrapolated_progress(self) -> None:
        trajectory = PersistentTrajectory(limits(replan_commit_time=0.5))
        trajectory.activate(
            Polyline([(0.0, 0.0), (1.0, 0.0)]),
            now=0.0,
            start_progress=0.0,
            start_speed=0.4,
            start_yaw=0.0,
        )
        actual_progress = 1.2

        trajectory.begin_braking(
            now=0.0,
            actual_progress=actual_progress,
            actual_speed=0.2,
        )
        points = trajectory.snapshot(0.0).points

        self.assertGreaterEqual(points[0].progress, actual_progress - 1e-9)
        self.assertGreaterEqual(points[0].pose.x, 1.2 - 1e-9)
        self.assertTrue(
            all(previous.progress <= current.progress + 1e-9 for previous, current in zip(points, points[1:]))
        )


class RollingDeadlineClockTests(unittest.TestCase):
    def test_standing_deadlines_count_down_without_reanchoring(self) -> None:
        clock = RollingDeadlineClock(0.5, 5)

        self.assertEqual(clock.snapshot(10.0), (0.5, 1.0, 1.5, 2.0, 2.5))
        self.assertEqual(clock.snapshot(10.2), (0.3, 0.8, 1.3, 1.8, 2.3))

    def test_standing_deadlines_roll_forward_after_expiry(self) -> None:
        clock = RollingDeadlineClock(0.5, 5)
        clock.snapshot(0.0)

        self.assertEqual(clock.snapshot(0.51), (0.49, 0.99, 1.49, 1.99, 2.49))

    def test_reset_starts_a_new_standing_time_axis(self) -> None:
        clock = RollingDeadlineClock(0.5, 5)
        clock.snapshot(0.0)
        clock.reset()

        self.assertEqual(clock.snapshot(3.0), (0.5, 1.0, 1.5, 2.0, 2.5))


if __name__ == "__main__":
    unittest.main()
