#!/usr/bin/python3

import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from nav2_waypoint_sampling import (  # noqa: E402
    CostmapView,
    CanonicalPath,
    HybridFeasibilityConfig,
    GoalSettleTracker,
    NAVFN_XY_LEGACY,
    Pose2D,
    PoseSample,
    Polyline,
    SMAC_HYBRID_XY_FORWARD,
    SMAC_LATTICE_FULL_SE2,
    SMAC_TERMINAL_YAW,
    STARTUP_PROFILE_CONTRACTS,
    STARTUP_PROFILE_PLANNER_IDS,
    YawGoalSettleTracker,
    adaptive_samples,
    adaptive_pose_samples,
    pose_yaw_step_within_limit,
    validate_raw_curvature,
    validate_raw_full_se2_feasibility,
    validate_raw_forward_no_reverse,
    validate_sampled_full_se2_constraints,
    validate_sampled_local_constraints,
)


def empty_costmap(size: int = 120, resolution: float = 0.05) -> CostmapView:
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


class SamplingTests(unittest.TestCase):
    @staticmethod
    def costmap_message(**overrides):
        metadata = SimpleNamespace(
            resolution=0.1,
            size_x=2,
            size_y=2,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
        for field, value in overrides.items():
            if field in ("origin_x", "origin_y"):
                setattr(metadata.origin.position, field.removeprefix("origin_"), value)
            elif field.startswith("quaternion_"):
                setattr(metadata.origin.orientation, field.removeprefix("quaternion_"), value)
            else:
                setattr(metadata, field, value)
        return SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            metadata=metadata,
            data=[0, 0, 0, 0],
        )

    def test_from_message_rejects_nonfinite_costmap_metadata(self) -> None:
        invalid_metadata = (
            {"resolution": math.nan},
            {"resolution": math.inf},
            {"origin_x": math.nan},
            {"origin_x": math.inf},
            {"origin_y": math.nan},
            {"origin_y": -math.inf},
            {"quaternion_z": math.nan},
            {"quaternion_w": math.inf},
        )
        for overrides in invalid_metadata:
            with self.subTest(overrides=overrides), self.assertRaisesRegex(
                ValueError, "costmap"
            ):
                CostmapView.from_message(
                    self.costmap_message(**overrides),
                    collision_threshold=253,
                    unknown_is_collision=True,
                )

    def test_from_message_rejects_nonfinite_derived_origin_yaw(self) -> None:
        message = self.costmap_message()
        with mock.patch(
            "nav2_waypoint_sampling.yaw_from_quaternion", return_value=math.nan
        ):
            with self.assertRaisesRegex(ValueError, "yaw"):
                CostmapView.from_message(message, 253, True)

    def test_straight_path_prefers_longest_spacing(self) -> None:
        path = Polyline([(0.0, 0.0), (4.0, 0.0)])
        samples = adaptive_samples(
            path,
            empty_costmap(),
            (0.0, 0.0),
            0.0,
            [0.6, 0.4, 0.2],
            5,
        )
        self.assertEqual([round(sample.point[0], 3) for sample in samples], [0.6, 1.2, 1.8, 2.4, 3.0])
        self.assertTrue(all(math.isclose(sample.chosen_spacing, 0.6) for sample in samples))

    def test_corner_chord_falls_back_from_point_six_to_point_four(self) -> None:
        size = 80
        resolution = 0.05
        data = [0] * (size * size)
        origin = -1.0
        # Occupy a small patch crossed by the 0.6 m diagonal shortcut but not
        # by the 0.4 m first leg of the original L-shaped path.
        for x in (0.18, 0.23):
            for y in (0.08, 0.13):
                mx = math.floor((x - origin) / resolution)
                my = math.floor((y - origin) / resolution)
                data[my * size + mx] = 253
        costmap = CostmapView(
            frame_id="map",
            resolution=resolution,
            size_x=size,
            size_y=size,
            origin_x=origin,
            origin_y=origin,
            origin_yaw=0.0,
            data=data,
            collision_threshold=253,
            unknown_is_collision=True,
        )
        path = Polyline([(0.0, 0.0), (0.4, 0.0), (0.4, 1.0)])
        samples = adaptive_samples(
            path,
            costmap,
            (0.0, 0.0),
            0.0,
            [0.6, 0.4, 0.2],
            1,
        )
        self.assertAlmostEqual(samples[0].chosen_spacing, 0.4)
        self.assertAlmostEqual(samples[0].point[0], 0.4)
        self.assertAlmostEqual(samples[0].point[1], 0.0)

    def test_corner_chord_falls_back_to_point_two_when_larger_chords_collide(self) -> None:
        costmap = empty_costmap(size=80, resolution=0.05)
        data = list(costmap.data)
        for x, y in ((0.05, 0.10), (0.10, 0.10)):
            mx = math.floor((x - costmap.origin_x) / costmap.resolution)
            my = math.floor((y - costmap.origin_y) / costmap.resolution)
            data[my * costmap.size_x + mx] = 253
        costmap = CostmapView(**{**costmap.__dict__, "data": data})

        samples = adaptive_samples(
            Polyline([(0.0, 0.0), (0.2, 0.0), (0.2, 1.0)]),
            costmap,
            (0.0, 0.0),
            0.0,
            [0.6, 0.4, 0.2],
            1,
        )

        self.assertAlmostEqual(samples[0].chosen_spacing, 0.2)
        self.assertAlmostEqual(samples[0].point[0], 0.2)
        self.assertAlmostEqual(samples[0].point[1], 0.0)

    def test_samples_encode_as_five_xyz_triples_with_tangent_heading(self) -> None:
        path = Polyline([(0.0, 0.0), (0.5, 0.0), (0.5, 2.0)])
        samples = adaptive_samples(
            path,
            empty_costmap(),
            (0.0, 0.0),
            0.0,
            [0.6, 0.4, 0.2],
            5,
        )

        flattened = [value for sample in samples for value in (*sample.point, sample.yaw)]
        self.assertEqual(len(samples), 5)
        self.assertEqual(len(flattened), 15)
        self.assertAlmostEqual(samples[0].point[0], 0.5)
        self.assertAlmostEqual(samples[0].point[1], 0.1)
        self.assertAlmostEqual(samples[0].yaw, math.pi / 2.0)

    def test_unknown_and_outside_are_collisions(self) -> None:
        costmap = empty_costmap(size=10, resolution=0.1)
        data = list(costmap.data)
        data[5 * 10 + 5] = 255
        costmap = CostmapView(**{**costmap.__dict__, "data": data})
        self.assertTrue(costmap.point_is_collision((-0.45, -0.45)))
        self.assertTrue(costmap.point_is_collision((10.0, 10.0)))

    def test_rotated_origin_maps_world_points_to_cells(self) -> None:
        data = [0] * 16
        data[1 * 4 + 2] = 77
        costmap = CostmapView(
            frame_id="map",
            resolution=1.0,
            size_x=4,
            size_y=4,
            origin_x=1.0,
            origin_y=2.0,
            origin_yaw=math.pi / 2.0,
            data=data,
            collision_threshold=253,
            unknown_is_collision=True,
        )
        self.assertEqual(costmap.cost_at((-0.25, 4.25)), 77)

    def test_costmap_floor_boundaries(self) -> None:
        costmap = CostmapView(
            frame_id="map",
            resolution=1.0,
            size_x=2,
            size_y=2,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            data=[1, 2, 3, 4],
            collision_threshold=253,
            unknown_is_collision=True,
        )
        self.assertEqual(costmap.cost_at((0.0, 0.0)), 1)
        self.assertEqual(costmap.cost_at((1.999999, 1.0)), 4)
        self.assertIsNone(costmap.cost_at((-0.000001, 0.0)))
        self.assertIsNone(costmap.cost_at((2.0, 1.0)))

    def test_unknown_can_be_allowed(self) -> None:
        costmap = CostmapView(
            frame_id="map",
            resolution=0.1,
            size_x=3,
            size_y=3,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            data=[0, 0, 0, 0, 255, 0, 0, 0, 0],
            collision_threshold=253,
            unknown_is_collision=False,
        )
        self.assertFalse(costmap.point_is_collision((0.15, 0.15)))

    def test_one_cell_fallback_succeeds_after_point_two_rejected(self) -> None:
        costmap = empty_costmap(size=40, resolution=0.05)
        data = list(costmap.data)
        mx = math.floor((0.10 - costmap.origin_x) / costmap.resolution)
        my = math.floor((0.0 - costmap.origin_y) / costmap.resolution)
        data[my * costmap.size_x + mx] = 253
        costmap = CostmapView(**{**costmap.__dict__, "data": data})
        samples = adaptive_samples(
            Polyline([(0.0, 0.0), (1.0, 0.0)]),
            costmap,
            (0.0, 0.0),
            0.0,
            [0.2],
            1,
        )
        self.assertAlmostEqual(samples[0].chosen_spacing, 0.05)
        self.assertAlmostEqual(samples[0].point[0], 0.05)

    def test_one_cell_fallback_failure_raises(self) -> None:
        costmap = empty_costmap(size=40, resolution=0.05)
        data = list(costmap.data)
        mx = math.floor((0.05 - costmap.origin_x) / costmap.resolution)
        my = math.floor((0.0 - costmap.origin_y) / costmap.resolution)
        data[my * costmap.size_x + mx] = 253
        costmap = CostmapView(**{**costmap.__dict__, "data": data})
        with self.assertRaisesRegex(RuntimeError, "one-cell path step"):
            adaptive_samples(
                Polyline([(0.0, 0.0), (1.0, 0.0)]),
                costmap,
                (0.0, 0.0),
                0.0,
                [0.2],
                1,
            )

    def test_path_end_is_repeated_to_fill_window(self) -> None:
        samples = adaptive_samples(
            Polyline([(0.0, 0.0), (0.3, 0.0)]),
            empty_costmap(),
            (0.0, 0.0),
            0.0,
            [0.6, 0.4, 0.2],
            5,
        )
        self.assertEqual(len(samples), 5)
        self.assertTrue(all(sample.point == (0.3, 0.0) for sample in samples))
        self.assertEqual(
            [sample.chosen_spacing for sample in samples],
            [0.3, 0.0, 0.0, 0.0, 0.0],
        )

    def test_goal_settle_requires_continuous_position_and_speed_hold(self) -> None:
        tracker = GoalSettleTracker()
        arguments = {
            "position_tolerance": 0.10,
            "speed_tolerance": 0.10,
            "hold_time": 0.30,
        }
        self.assertFalse(tracker.update(0, 0.10, 0.10, **arguments))
        self.assertFalse(tracker.update(299_999_999, 0.10, 0.10, **arguments))
        self.assertTrue(tracker.update(300_000_000, 0.10, 0.10, **arguments))

        self.assertFalse(tracker.update(350_000_000, 0.10, 0.11, **arguments))
        self.assertFalse(tracker.update(650_000_000, 0.10, 0.10, **arguments))
        self.assertTrue(tracker.update(950_000_000, 0.10, 0.10, **arguments))

    def test_goal_settle_rejects_missing_speed_and_position_excursions(self) -> None:
        tracker = GoalSettleTracker()
        arguments = {
            "position_tolerance": 0.10,
            "speed_tolerance": 0.10,
            "hold_time": 0.30,
        }
        self.assertFalse(tracker.update(0, 0.05, None, **arguments))
        self.assertFalse(tracker.update(100_000_000, 0.05, 0.05, **arguments))
        self.assertFalse(tracker.update(250_000_000, 0.11, 0.05, **arguments))
        self.assertFalse(tracker.update(550_000_000, 0.05, 0.05, **arguments))
        self.assertTrue(tracker.update(850_000_000, 0.05, 0.05, **arguments))


class CanonicalPoseSamplingTests(unittest.TestCase):
    def test_command_domain_yaw_step_includes_robot_to_first_target(self) -> None:
        robot = Pose2D(0.0, 0.0, -0.20)

        self.assertTrue(
            pose_yaw_step_within_limit(
                robot,
                PoseSample(0.1, Pose2D(0.0, 0.0, 0.15), 0.1),
                0.35,
            )
        )
        self.assertFalse(
            pose_yaw_step_within_limit(
                robot,
                PoseSample(0.1, Pose2D(0.0, 0.0, 0.16), 0.1),
                0.35,
            )
        )

    def test_rotation_sampling_shortens_first_step_for_live_yaw_lag(self) -> None:
        path = CanonicalPath(
            [
                Pose2D(0.0, 0.0, 0.20),
                Pose2D(0.0, 0.0, 1.00),
            ]
        )
        robot = Pose2D(0.0, 0.0, 0.0)

        samples = adaptive_pose_samples(
            path,
            empty_costmap(),
            robot,
            0.0,
            5,
            max_yaw_step=0.35,
            validator=lambda previous, sample: pose_yaw_step_within_limit(
                previous,
                sample,
                0.35,
            ),
        )

        previous_yaw = robot.yaw
        for sample in samples:
            self.assertLessEqual(
                abs(math.remainder(sample.yaw - previous_yaw, 2.0 * math.pi)),
                0.35 + 1e-9,
            )
            previous_yaw = sample.yaw
        self.assertAlmostEqual(samples[0].chosen_spacing, 0.05)

    def test_yaw_unwrap_is_continuous(self) -> None:
        path = CanonicalPath(
            [
                Pose2D(0.0, 0.0, math.radians(170.0)),
                Pose2D(1.0, 0.0, math.radians(-170.0)),
            ]
        )

        self.assertGreater(path.poses[1].yaw, math.pi)
        self.assertAlmostEqual(path.poses[1].yaw, math.radians(190.0))
        self.assertAlmostEqual(path.yaw_at(path.total_progress), math.radians(190.0))

    def test_translation_progress_ignores_segment_yaw_change(self) -> None:
        path = CanonicalPath(
            [
                Pose2D(0.0, 0.0, 0.0),
                Pose2D(1.0, 0.0, 2.0),
            ],
            rotation_weight=10.0,
        )

        self.assertAlmostEqual(path.total_progress, 1.0)
        midpoint = path.pose_at(0.5)
        self.assertAlmostEqual(midpoint.x, 0.5)
        self.assertAlmostEqual(midpoint.yaw, 1.0)

    def test_duplicate_xy_rotation_segment_is_retained_and_projected(self) -> None:
        path = CanonicalPath(
            [
                Pose2D(0.0, 0.0, 0.0),
                Pose2D(0.0, 0.0, math.pi / 2.0),
                Pose2D(1.0, 0.0, math.pi / 2.0),
            ]
        )

        self.assertEqual(len(path.segments), 2)
        projection = path.project((0.0, 0.0), yaw=math.pi / 4.0)
        self.assertAlmostEqual(projection.progress, math.pi / 4.0)
        self.assertAlmostEqual(projection.pose.x, 0.0)
        self.assertAlmostEqual(projection.pose.yaw, math.pi / 4.0)

    def test_projection_window_does_not_jump_to_later_loop_segment(self) -> None:
        path = CanonicalPath(
            [
                Pose2D(-0.025, -0.025, math.pi / 2.0),
                Pose2D(-0.025, -0.025, 2.0),
                Pose2D(-0.3, 0.6, 2.0),
                Pose2D(0.0, 0.0, math.pi),
            ]
        )

        unrestricted = path.project((0.0, 0.0), yaw=math.pi / 2.0)
        bounded = path.project(
            (0.0, 0.0),
            yaw=math.pi / 2.0,
            max_progress=0.35,
        )

        self.assertGreater(unrestricted.progress, 0.60)
        self.assertLessEqual(bounded.progress, 0.35)
        self.assertLess(bounded.yaw_error, unrestricted.yaw_error)

    def test_rotation_sampling_caps_yaw_step_and_preserves_goal_yaw(self) -> None:
        path = CanonicalPath(
            [
                Pose2D(0.0, 0.0, 0.0),
                Pose2D(0.0, 0.0, 1.0),
            ]
        )

        samples = adaptive_pose_samples(
            path,
            empty_costmap(),
            Pose2D(0.0, 0.0, 0.0),
            0.0,
            4,
            max_yaw_step=0.35,
        )

        self.assertEqual([round(sample.yaw, 2) for sample in samples], [0.35, 0.7, 1.0, 1.0])
        self.assertEqual([sample.point for sample in samples], [(0.0, 0.0)] * 4)
        self.assertAlmostEqual(samples[-1].chosen_spacing, 0.0)

    def test_rotation_sampling_keeps_measured_xy_when_path_is_grid_centered(self) -> None:
        path = CanonicalPath(
            [
                Pose2D(-0.025, -0.025, 0.0),
                Pose2D(-0.025, -0.025, 0.5),
            ]
        )
        robot = Pose2D(0.0, 0.0, 0.0)

        samples = adaptive_pose_samples(
            path,
            empty_costmap(),
            robot,
            0.0,
            1,
            max_yaw_step=0.35,
        )

        self.assertEqual(samples[0].point, robot.point)
        self.assertAlmostEqual(samples[0].yaw, 0.35)

    def test_pose_samples_retry_shorter_when_candidate_crosses_rotation_cap(self) -> None:
        path = CanonicalPath(
            [
                Pose2D(0.0, 0.0, 0.0),
                Pose2D(0.5, 0.0, 0.0),
                Pose2D(0.5, 0.0, 1.0),
            ]
        )

        samples = adaptive_pose_samples(
            path,
            empty_costmap(),
            Pose2D(0.4, 0.0, 0.0),
            0.4,
            1,
            max_yaw_step=0.35,
        )

        self.assertAlmostEqual(samples[0].chosen_spacing, 0.4)
        self.assertAlmostEqual(samples[0].progress, 0.8)
        self.assertAlmostEqual(samples[0].yaw, 0.3)

    def test_pose_samples_can_disable_yaw_cap(self) -> None:
        path = CanonicalPath(
            [
                Pose2D(0.0, 0.0, 0.0),
                Pose2D(0.5, 0.0, 0.0),
                Pose2D(0.5, 0.0, 1.0),
            ]
        )

        samples = adaptive_pose_samples(
            path,
            empty_costmap(),
            Pose2D(0.4, 0.0, 0.0),
            0.4,
            1,
            max_yaw_step=None,
        )

        self.assertAlmostEqual(samples[0].chosen_spacing, 0.6)
        self.assertAlmostEqual(samples[0].yaw, 0.5)

    def test_pose_samples_try_shorter_spacing_before_validator_failure(self) -> None:
        path = CanonicalPath(
            [
                Pose2D(0.0, 0.0, 0.0),
                Pose2D(2.0, 0.0, 0.25),
            ]
        )
        contexts = []

        samples = adaptive_pose_samples(
            path,
            empty_costmap(),
            Pose2D(0.0, 0.0, 0.0),
            0.0,
            1,
            validator=lambda previous, sample: (
                contexts.append((previous, sample)) or sample.progress <= 0.41
            ),
        )

        self.assertAlmostEqual(samples[0].chosen_spacing, 0.4)
        self.assertAlmostEqual(samples[0].yaw, path.yaw_at(samples[0].progress))
        self.assertTrue(all(isinstance(previous, Pose2D) for previous, _ in contexts))

    def test_startup_profile_mapping_is_immutable(self) -> None:
        self.assertEqual(
            STARTUP_PROFILE_PLANNER_IDS,
            {
                NAVFN_XY_LEGACY: "GridBased",
                SMAC_HYBRID_XY_FORWARD: "SmacHybrid",
                SMAC_TERMINAL_YAW: "SmacHybrid",
                SMAC_LATTICE_FULL_SE2: "SmacLattice",
            },
        )
        self.assertTrue(STARTUP_PROFILE_CONTRACTS[SMAC_TERMINAL_YAW].terminal_yaw_required)
        with self.assertRaises(TypeError):
            STARTUP_PROFILE_PLANNER_IDS[NAVFN_XY_LEGACY] = "Other"  # type: ignore[index]

    def test_forward_curvature_and_first_bearing_failures(self) -> None:
        config = HybridFeasibilityConfig()
        reverse = validate_raw_forward_no_reverse(
            [Pose2D(0.0, 0.0, 0.0), Pose2D(-0.10, 0.0, 0.0)],
            config,
        )
        self.assertFalse(reverse.ok)
        self.assertIn("reverse", reverse.reason)

        curvature = validate_raw_curvature(
            [
                Pose2D(0.0, 0.0, 0.0),
                Pose2D(0.5, 0.0, 0.0),
                Pose2D(0.5, 0.5, math.pi / 2.0),
            ],
            config,
        )
        self.assertFalse(curvature.ok)
        self.assertIn("heading", curvature.reason)

        bearing = validate_sampled_local_constraints(
            Pose2D(0.0, 0.0, 0.0),
            [
                adaptive_pose_samples(
                    CanonicalPath([Pose2D(0.0, 0.0, 0.0), Pose2D(0.1, 0.1, 0.0)]),
                    empty_costmap(),
                    Pose2D(0.0, 0.0, 0.0),
                    0.0,
                    1,
                    spacings=[0.15],
                )[0]
            ],
            config,
        )
        self.assertFalse(bearing.ok)
        self.assertIn("bearing", bearing.reason)

    def test_raw_curvature_uses_exact_chord_radius(self) -> None:
        config = HybridFeasibilityConfig(
            max_chord_heading_delta=2.0,
            min_turning_radius=0.70,
        )

        result = validate_raw_curvature(
            [Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 1.5)],
            config,
        )

        self.assertTrue(result.ok)

    def test_raw_hybrid_curvature_uses_xy_geometry_across_quantized_yaw(self) -> None:
        # First three poses from a Humble SmacHybrid path configured with a
        # 1.0 m minimum turning radius. Pose yaw advances in 5 degree bins,
        # while the XY chord headings describe the true one-meter curve.
        poses = [
            Pose2D(3.2750001680, 4.5250001866, 0.0),
            Pose2D(3.3485231183, 4.5277063329, math.radians(5.0)),
            Pose2D(3.4216478142, 4.5358110388, math.radians(10.0)),
            Pose2D(3.4939782902, 4.5492708169, math.radians(15.0)),
        ]

        result = validate_raw_curvature(
            poses,
            HybridFeasibilityConfig(min_turning_radius=0.95),
        )

        self.assertTrue(result.ok, result.reason)

    def test_raw_hybrid_curvature_rejects_tight_geometric_curve(self) -> None:
        result = validate_raw_curvature(
            [
                Pose2D(0.5, 0.0, math.pi / 2.0),
                Pose2D(0.0, 0.5, math.pi),
                Pose2D(-0.5, 0.0, -math.pi / 2.0),
            ],
            HybridFeasibilityConfig(max_chord_heading_delta=2.0),
        )

        self.assertFalse(result.ok)
        self.assertIn("curvature", result.reason)

    def test_raw_full_se2_uses_geometry_across_quantized_headings(self) -> None:
        poses = [
            Pose2D(math.cos(angle), math.sin(angle), yaw)
            for angle, yaw in (
                (0.0, math.pi / 2.0),
                (0.1, math.pi / 2.0),
                (0.2, math.pi / 2.0 + 0.4),
                (0.3, math.pi / 2.0 + 0.4),
            )
        ]

        result = validate_raw_full_se2_feasibility(poses)

        self.assertTrue(result.ok, result.reason)

    def test_raw_full_se2_resets_curvature_at_in_place_rotation(self) -> None:
        poses = [
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(1.0, 0.0, 0.0),
            Pose2D(1.0, 0.0, math.pi / 2.0),
            Pose2D(1.0, 1.0, math.pi / 2.0),
            Pose2D(1.0, 2.0, math.pi / 2.0),
        ]

        result = validate_raw_full_se2_feasibility(poses)

        self.assertTrue(result.ok, result.reason)

    def test_raw_full_se2_rejects_tight_geometric_curve(self) -> None:
        result = validate_raw_full_se2_feasibility(
            [
                Pose2D(0.5, 0.0, math.pi / 2.0),
                Pose2D(0.0, 0.5, math.pi),
                Pose2D(-0.5, 0.0, -math.pi / 2.0),
            ]
        )

        self.assertFalse(result.ok)
        self.assertIn("curvature", result.reason)

    def test_sampled_constraints_use_chord_headings_and_radius(self) -> None:
        config = HybridFeasibilityConfig()
        heading = validate_sampled_local_constraints(
            Pose2D(0.0, 0.0, 0.0),
            [
                PoseSample(0.5, Pose2D(0.5, 0.0, 0.0), 0.5),
                PoseSample(1.0, Pose2D(0.5, 0.5, 0.0), 0.5),
            ],
            config,
        )
        self.assertFalse(heading.ok)
        self.assertIn("chord heading", heading.reason)

        radius = validate_sampled_local_constraints(
            Pose2D(0.0, 0.0, 0.0),
            [
                PoseSample(0.5, Pose2D(0.5, 0.0, 0.0), 0.5),
                PoseSample(0.6, Pose2D(0.587758, 0.047943, 0.0), 0.1),
                PoseSample(0.7, Pose2D(0.641788, 0.132090, 0.0), 0.1),
            ],
            config,
        )
        self.assertFalse(radius.ok)
        self.assertIn("curvature", radius.reason)

    def test_full_se2_sample_guard_resets_heading_after_in_place_rotation(self) -> None:
        robot = Pose2D(0.0, 0.0, 0.0)
        samples = [
            PoseSample(0.35, Pose2D(0.0, 0.0, math.pi / 2.0), 0.35),
            PoseSample(0.95, Pose2D(0.0, 0.6, math.pi / 2.0), 0.60),
        ]

        result = validate_sampled_full_se2_constraints(robot, samples)

        self.assertTrue(result.ok, result.reason)

    def test_full_se2_sample_guard_rejects_reverse_after_rotation(self) -> None:
        robot = Pose2D(0.0, 0.0, 0.0)
        samples = [
            PoseSample(0.35, Pose2D(0.0, 0.0, math.pi / 2.0), 0.35),
            PoseSample(0.95, Pose2D(0.0, -0.6, math.pi / 2.0), 0.60),
        ]

        result = validate_sampled_full_se2_constraints(robot, samples)

        self.assertFalse(result.ok)
        self.assertIn("reverses", result.reason)

    def test_yaw_goal_settle_requires_all_values_and_resets_on_stale_time(self) -> None:
        tracker = YawGoalSettleTracker()
        arguments = {
            "position_tolerance": 0.10,
            "speed_tolerance": 0.10,
            "yaw_tolerance": 0.05,
            "angular_speed_tolerance": 0.10,
            "hold_time": 0.30,
        }
        self.assertFalse(tracker.update(0, 0.05, 0.05, None, 0.05, **arguments))
        self.assertFalse(tracker.update(100_000_000, 0.05, 0.05, 0.04, 0.05, **arguments))
        self.assertTrue(tracker.update(400_000_000, 0.05, 0.05, 0.04, 0.05, **arguments))

        self.assertFalse(tracker.update(350_000_000, 0.05, 0.05, 0.04, 0.05, **arguments))
        self.assertFalse(tracker.update(650_000_000, 0.05, 0.05, 0.06, 0.05, **arguments))
        self.assertFalse(tracker.update(900_000_000, 0.05, 0.05, 0.04, 0.11, **arguments))
        self.assertFalse(tracker.update(950_000_000, 0.05, 0.05, 0.04, -0.11, **arguments))
        self.assertFalse(tracker.update(1_000_000_000, 0.05, 0.05, 0.04, 0.05, **arguments))
        self.assertTrue(tracker.update(1_300_000_000, 0.05, 0.05, 0.04, 0.05, **arguments))


if __name__ == "__main__":
    unittest.main()
