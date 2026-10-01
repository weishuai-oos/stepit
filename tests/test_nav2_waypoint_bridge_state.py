#!/usr/bin/env python3
"""Behavior-level regression tests for the Nav2 waypoint bridge state machine."""

from __future__ import annotations

import sys
import unittest
import math
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMessage


REPO_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_DIR / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import nav2_global_goal_to_waypoints as bridge_module  # noqa: E402
from nav2_global_goal_to_waypoints import Nav2GlobalGoalToWaypoints  # noqa: E402
from nav2_waypoint_sampling import (  # noqa: E402
    NAVFN_XY_LEGACY,
    Polyline,
    Pose2D,
    SMAC_HYBRID_XY_FORWARD,
    YawGoalSettleTracker,
)
from nav2_waypoint_trajectory import (  # noqa: E402
    PersistentTrajectory,
    RollingDeadlineClock,
    TrajectoryLimits,
    TrajectorySnapshot,
)


class FakeNow:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds

    def to_msg(self) -> TimeMessage:
        seconds, nanoseconds = divmod(self.nanoseconds, 1_000_000_000)
        return TimeMessage(sec=seconds, nanosec=nanoseconds)


class FakeClock:
    def __init__(self, nanoseconds: int = 1_000_000_000, step_ns: int = 0) -> None:
        self.nanoseconds = nanoseconds
        self.step_ns = step_ns

    def now(self) -> FakeNow:
        current = FakeNow(self.nanoseconds)
        self.nanoseconds += self.step_ns
        return current

    def advance(self, seconds: float) -> None:
        self.nanoseconds += int(seconds * 1_000_000_000)


class PendingFuture:
    def add_done_callback(self, callback) -> None:
        self.callback = callback


class RecordingActionClient:
    def __init__(self) -> None:
        self.requests = []

    def send_goal_async(self, request):
        self.requests.append(request)
        return PendingFuture()


class FakeCostmap:
    def __init__(self, collision: bool) -> None:
        self.frame_id = "map"
        self.collision = collision

    def point_is_collision(self, _point) -> bool:
        return self.collision


class RecordingPublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def trajectory_limits(**overrides) -> TrajectoryLimits:
    values = dict(
        waypoint_interval=0.5,
        num_waypoints=5,
        cruise_speed=0.3,
        max_acceleration=0.8,
        max_deceleration=1.0,
        terminal_deceleration=1.0,
        max_lateral_acceleration=0.25,
        max_yaw_rate=0.8,
        integration_step=0.02,
        curvature_window=0.15,
        replan_commit_time=0.5,
        replan_join_distance=0.35,
        replan_max_waypoint_shift=0.30,
        replan_max_heading_shift=0.35,
    )
    values.update(overrides)
    return TrajectoryLimits(**values)


def bare_bridge() -> Nav2GlobalGoalToWaypoints:
    return object.__new__(Nav2GlobalGoalToWaypoints)


def bridge_with_valid_parameters() -> Nav2GlobalGoalToWaypoints:
    node = bare_bridge()
    node.publish_rate = 50.0
    node.replan_period = 1.0
    node.replan_deviation = 0.3
    node.goal_tolerance = 0.15
    node.goal_linear_speed_tolerance = 0.1
    node.goal_yaw_tolerance = 0.1
    node.goal_angular_speed_tolerance = 0.15
    node.goal_hold_time = 0.5
    node.goal_capture_release_tolerance = 0.3
    node.handover_hold_time = 0.2
    node.odom_timeout = 0.5
    node.odom_speed_filter_window = 2
    node.costmap_timeout = 1.0
    node.trajectory_limits = trajectory_limits()
    node.max_tracking_urgency = 0.0
    node.terminal_response_time = 0.2
    node.terminal_braking_margin = 0.1
    node.emergency_speed_upper_bound = 1.2
    node.tracking_pacing_max_delay = 0.75
    node.num_waypoints = 5
    node.transient_failure_timeout = 0.3
    node.collision_threshold = 253
    return node


class Nav2WaypointBridgeStateTests(unittest.TestCase):
    def test_smac_hybrid_uses_faster_limits_without_changing_navfn(self) -> None:
        parameters = {
            "waypoint_interval": 0.5,
            "cruise_speed": 0.30,
            "max_acceleration": 0.80,
            "max_deceleration": 1.00,
            "terminal_deceleration": 1.00,
            "max_lateral_acceleration": 0.25,
            "max_tracking_urgency": 0.0,
            "smac_hybrid_cruise_speed": 0.8,
            "smac_hybrid_max_acceleration": 1.20,
            "smac_hybrid_terminal_deceleration": 0.50,
            "smac_hybrid_max_lateral_acceleration": 0.30,
            "smac_hybrid_max_tracking_urgency": 0.0,
            "max_yaw_rate": 0.80,
            "integration_step": 0.02,
            "curvature_window": 0.15,
            "replan_commit_time": 0.50,
            "replan_join_distance": 0.35,
            "replan_max_waypoint_shift": 0.30,
            "replan_max_heading_shift": 0.35,
        }
        node = bare_bridge()
        node.num_waypoints = 5
        node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

        node.planner_profile = NAVFN_XY_LEGACY
        navfn_limits = node._load_trajectory_limits()
        node.planner_profile = SMAC_HYBRID_XY_FORWARD
        smac_limits = node._load_trajectory_limits()

        self.assertEqual(navfn_limits.cruise_speed, 0.30)
        self.assertEqual(navfn_limits.max_acceleration, 0.80)
        self.assertEqual(navfn_limits.max_lateral_acceleration, 0.25)
        self.assertEqual(smac_limits.cruise_speed, 0.8)
        self.assertEqual(smac_limits.max_acceleration, 1.20)
        self.assertEqual(smac_limits.max_lateral_acceleration, 0.30)
        self.assertEqual(smac_limits.max_deceleration, navfn_limits.max_deceleration)
        self.assertEqual(navfn_limits.terminal_deceleration, 1.0)
        self.assertEqual(smac_limits.terminal_deceleration, 0.5)

    def test_smac_default_does_not_enable_deadline_pacing(self) -> None:
        node = bare_bridge()
        node.max_tracking_urgency = 0.0
        self.assertFalse(node._deadline_pacing_enabled())

        node.max_tracking_urgency = 1.0
        self.assertTrue(node._deadline_pacing_enabled())

    def test_planner_pose_yaw_is_normalized_to_one_xy_bridge_contract(self) -> None:
        node = bare_bridge()
        node.global_frame = "map"
        node.profile_contract = mock.Mock(terminal_yaw_required=False)
        message = PathMessage()
        message.header.frame_id = "map"
        first = PoseStamped()
        first.pose.orientation = bridge_module.quaternion_from_yaw(math.pi / 2.0)
        second = PoseStamped()
        second.pose.position.x = 1.0
        second.pose.orientation = bridge_module.quaternion_from_yaw(-math.pi / 2.0)
        message.poses = [first, second]

        parsed = node._parse_planner_path(message)

        self.assertIsInstance(parsed.model, Polyline)
        self.assertAlmostEqual(parsed.model.tangent_yaw(0.0), 0.0)

    def test_xy_normalization_still_rejects_non_finite_planner_yaw(self) -> None:
        node = bare_bridge()
        node.global_frame = "map"
        node.profile_contract = mock.Mock(terminal_yaw_required=False)
        message = PathMessage()
        first = PoseStamped()
        first.pose.orientation.w = float("nan")
        second = PoseStamped()
        second.pose.position.x = 1.0
        second.pose.orientation.w = 1.0
        message.poses = [first, second]

        with self.assertRaisesRegex(ValueError, "non-finite pose"):
            node._parse_planner_path(message)

    def test_runtime_parameters_freeze_policy_waypoint_abi(self) -> None:
        node = bridge_with_valid_parameters()
        node._validate_parameters()

        node.num_waypoints = 4
        with self.assertRaisesRegex(ValueError, "num_waypoints must remain 5"):
            node._validate_parameters()

        node.num_waypoints = 5
        node.trajectory_limits = trajectory_limits(waypoint_interval=0.1)
        with self.assertRaisesRegex(ValueError, "waypoint_interval must remain 0.5"):
            node._validate_parameters()

    def test_runtime_parameters_reject_unbrakeable_waypoint_horizon(self) -> None:
        node = bridge_with_valid_parameters()
        node.trajectory_limits = trajectory_limits(cruise_speed=2.1)
        node.emergency_speed_upper_bound = 2.1

        with self.assertRaisesRegex(ValueError, "uncommitted waypoint horizon"):
            node._validate_parameters()

    def test_runtime_parameters_reject_unbrakeable_emergency_speed_bound(self) -> None:
        node = bridge_with_valid_parameters()
        node.emergency_speed_upper_bound = 2.6

        with self.assertRaisesRegex(ValueError, "rebased waypoint horizon"):
            node._validate_parameters()

    def test_runtime_parameters_require_two_odom_filter_samples(self) -> None:
        node = bridge_with_valid_parameters()
        node.odom_speed_filter_window = 1

        with self.assertRaisesRegex(ValueError, "at least two samples"):
            node._validate_parameters()

    def test_runtime_parameter_callback_rejects_policy_contract_changes(self) -> None:
        node = bridge_with_valid_parameters()
        node.planner_profile = NAVFN_XY_LEGACY

        changes = (
            ("planner_profile", "smac_hybrid_xy_forward"),
            ("num_waypoints", 4),
            ("waypoint_interval", 0.1),
        )
        for name, value in changes:
            with self.subTest(name=name):
                result = node._reject_runtime_contract_change(
                    [SimpleNamespace(name=name, value=value)]
                )
                self.assertFalse(result.successful)
                self.assertIn(f"{name} is immutable", result.reason)

        unchanged = node._reject_runtime_contract_change(
            [
                SimpleNamespace(name="planner_profile", value=NAVFN_XY_LEGACY),
                SimpleNamespace(name="num_waypoints", value=5),
                SimpleNamespace(name="waypoint_interval", value=0.5),
            ]
        )
        self.assertTrue(unchanged.successful)

    def test_each_plan_request_gets_a_fresh_goal_stamp(self) -> None:
        node = bare_bridge()
        clock = FakeClock(step_ns=1_000_000_000)
        node.global_frame = "map"
        node.planner_id = "SmacHybrid"
        node.plan_pending = False
        node.plan_sequence = 0
        node.active_plan_goal_handle = None
        node.active_plan_sequence = None
        node.last_plan_request_ns = 0
        node.action_client = RecordingActionClient()
        node.get_clock = lambda: clock
        node._cancel_active_plan = lambda: None
        node._set_status = lambda _status: None

        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.header.stamp.sec = 123
        goal.pose.position.x = 2.0
        goal.pose.position.y = -1.0
        goal.pose.orientation.w = 1.0

        node._send_plan_goal(goal)
        node._send_plan_goal(goal)

        first = node.action_client.requests[0].goal
        second = node.action_client.requests[1].goal
        first_ns = first.header.stamp.sec * 1_000_000_000 + first.header.stamp.nanosec
        second_ns = second.header.stamp.sec * 1_000_000_000 + second.header.stamp.nanosec
        self.assertGreater(second_ns, first_ns)
        self.assertNotEqual(first.header.stamp.sec, 123)
        self.assertEqual(goal.header.stamp.sec, 123)
        self.assertEqual(first.header.frame_id, "odom")
        self.assertAlmostEqual(first.pose.position.x, 2.0)

    def test_start_collision_enters_emergency_stop_even_with_active_reference(self) -> None:
        node = bare_bridge()
        clock = FakeClock(nanoseconds=5_000_000_000)
        trajectory = mock.Mock()
        trajectory.active = True
        node.global_frame = "map"
        node.costmap = FakeCostmap(collision=True)
        node.costmap_timeout = 1.0
        node.last_costmap_receive_ns = 5_000_000_000
        node.trajectory = trajectory
        node.last_plan_request_ns = 0
        node.get_clock = lambda: clock
        node._base_pose_in = lambda _frame: ((1.0, 2.0), 0.0)
        node._enter_fault = mock.Mock()

        self.assertTrue(node._planning_start_is_blocked())
        node._enter_fault.assert_called_once_with(
            "EMERGENCY_STOP_START_IN_COLLISION", 5.0
        )
        trajectory.begin_braking.assert_not_called()
        self.assertEqual(node.last_plan_request_ns, 5_000_000_000)

    def test_start_collision_emergency_stops_an_idle_bridge(self) -> None:
        node = bare_bridge()
        clock = FakeClock(nanoseconds=5_000_000_000)
        node.global_frame = "map"
        node.costmap = FakeCostmap(collision=True)
        node.costmap_timeout = 1.0
        node.last_costmap_receive_ns = 5_000_000_000
        node.trajectory = mock.Mock(active=False)
        node.last_plan_request_ns = 0
        node.get_clock = lambda: clock
        node._base_pose_in = lambda _frame: ((1.0, 2.0), 0.0)
        node._enter_fault = mock.Mock()

        self.assertTrue(node._planning_start_is_blocked())
        node._enter_fault.assert_called_once_with(
            "EMERGENCY_STOP_START_IN_COLLISION", 5.0
        )

    def test_costmap_freshness_accepts_current_and_exact_timeout_boundary(self) -> None:
        node = bare_bridge()
        node.costmap = mock.Mock()
        node.costmap_timeout = 1.0
        node.last_costmap_receive_ns = 5_000_000_000

        self.assertTrue(node._costmap_is_fresh(5_000_000_000))
        self.assertTrue(node._costmap_is_fresh(6_000_000_000))

    def test_costmap_freshness_rejects_expired_future_and_missing_samples(self) -> None:
        node = bare_bridge()
        node.costmap = mock.Mock()
        node.costmap_timeout = 1.0
        node.last_costmap_receive_ns = 5_000_000_000

        self.assertFalse(node._costmap_is_fresh(6_000_000_001))
        self.assertFalse(node._costmap_is_fresh(4_999_999_999))
        node.last_costmap_receive_ns = 0
        self.assertFalse(node._costmap_is_fresh(5_000_000_000))

    def test_trajectory_collision_check_rejects_path_costmap_frame_mismatch(self) -> None:
        node = bare_bridge()
        node.costmap = FakeCostmap(collision=False)
        snapshot = TrajectorySnapshot((), (), PersistentTrajectory.TRACKING)

        reason = node._trajectory_collision_reason(
            snapshot, (0.0, 0.0), 0.0, "odom"
        )

        self.assertIn("frame mismatch", reason or "")

    def test_costmap_frame_switch_is_rejected_while_path_is_active(self) -> None:
        node = bare_bridge()
        old_costmap = mock.Mock(frame_id="map")
        candidate = mock.Mock(frame_id="odom")
        node.costmap = old_costmap
        node.collision_threshold = 253
        node.unknown_is_collision = True
        node.trajectory = mock.Mock(active=True)
        node.path_message = PathMessage()
        node.path_message.header.frame_id = "map"
        node.get_clock = lambda: FakeClock()
        node._set_status = mock.Mock()

        with mock.patch.object(
            bridge_module.CostmapView,
            "from_message",
            return_value=candidate,
        ):
            node._on_costmap(mock.Mock())

        self.assertIs(node.costmap, old_costmap)
        self.assertTrue(
            any("frame mismatch" in call.args[0] for call in node._set_status.call_args_list)
        )

    def test_active_replan_waits_for_fresh_costmap_without_adopting_path(self) -> None:
        node = bare_bridge()
        node.get_clock = lambda: FakeClock(nanoseconds=5_000_000_000)
        node._costmap_is_fresh = lambda _now_ns: False
        node.trajectory = mock.Mock(active=True)
        node._set_status = mock.Mock()
        path_message = PathMessage()
        path_message.header.frame_id = "map"
        parsed = bridge_module.ParsedPath(
            path_message,
            Polyline([(0.0, 0.0), (1.0, 0.0)]),
            [],
        )

        node._accept_path(parsed)

        self.assertIs(node.pending_path, parsed)
        node.trajectory.begin_braking.assert_called_once_with(now=5.0)
        node.trajectory.adopt_replan.assert_not_called()

    def test_send_goal_sync_exception_resets_pending_flag(self) -> None:
        node = bare_bridge()
        node.global_frame = "map"
        node.planner_id = "GridBased"
        node.plan_pending = False
        node.plan_sequence = 0
        node.active_plan_goal_handle = None
        node.active_plan_sequence = None
        node.last_plan_request_ns = 0
        node.get_clock = lambda: FakeClock()
        node.action_client = mock.Mock()
        node.action_client.send_goal_async.side_effect = RuntimeError("transport")
        node._cancel_active_plan = mock.Mock()
        node._set_status = mock.Mock()
        node.trajectory = mock.Mock(active=False)
        node._clear_path = mock.Mock()
        node.pending_path = None
        node.planner_profile = NAVFN_XY_LEGACY

        node._send_plan_goal(PoseStamped())

        self.assertFalse(node.plan_pending)
        self.assertTrue(
            any("PLAN_SEND_ERROR" in call.args[0] for call in node._set_status.call_args_list)
        )

    def test_real_goal_is_transformed_into_planner_path_frame(self) -> None:
        node = bare_bridge()
        node.global_frame = "map"
        node.goal = PoseStamped()
        node.goal.header.frame_id = "map"
        node.goal.pose.position.x = 1.0
        node.goal.pose.position.y = 0.0
        node.goal_yaw = 0.0
        rotation = mock.Mock(x=0.0, y=0.0, z=2.0 ** -0.5, w=2.0 ** -0.5)
        translation = mock.Mock(x=1.0, y=2.0, z=0.0)
        transform = mock.Mock()
        transform.transform.translation = translation
        transform.transform.rotation = rotation
        node.tf_buffer = mock.Mock()
        node.tf_buffer.lookup_transform.return_value = transform

        goal_in_odom = node._goal_pose_in("odom")

        self.assertAlmostEqual(goal_in_odom.x, 1.0)
        self.assertAlmostEqual(goal_in_odom.y, 3.0)
        self.assertAlmostEqual(goal_in_odom.yaw, 1.5707963267948966)
        node.tf_buffer.lookup_transform.assert_called_once()

    def test_short_tf_outage_extrapolates_pose_using_odom_delta(self) -> None:
        node = bare_bridge()
        node.transient_failure_timeout = 0.3
        node.odom_timeout = 0.5
        node.trajectory_limits = trajectory_limits()
        node.last_tracking_frame = "map"
        node.last_tracking_pose = bridge_module.Pose2D(10.0, 20.0, 0.4)
        node.last_tracking_tf_ns = 10_000_000_000
        node.last_tracking_odom_pose = bridge_module.Pose2D(1.0, 2.0, 0.1)
        node.last_odom_pose = bridge_module.Pose2D(1.05, 2.0, 0.2)
        node.last_odom_pose_ns = 10_100_000_000

        pose = node._extrapolated_tracking_pose("map", 10_200_000_000)

        self.assertIsNotNone(pose)
        assert pose is not None
        self.assertAlmostEqual(pose.x, 10.0 + 0.05 * math.cos(0.3))
        self.assertAlmostEqual(pose.y, 20.0 + 0.05 * math.sin(0.3))
        self.assertAlmostEqual(pose.yaw, 0.5)

    def test_tf_fallback_rejects_an_implausible_odom_jump(self) -> None:
        node = bare_bridge()
        node.transient_failure_timeout = 0.3
        node.odom_timeout = 0.5
        node.trajectory_limits = trajectory_limits()
        node.last_tracking_frame = "map"
        node.last_tracking_pose = bridge_module.Pose2D(10.0, 20.0, 0.4)
        node.last_tracking_tf_ns = 10_000_000_000
        node.last_tracking_odom_pose = bridge_module.Pose2D(1.0, 2.0, 0.1)
        node.last_odom_pose = bridge_module.Pose2D(2.0, 2.0, 0.2)
        node.last_odom_pose_ns = 10_100_000_000

        self.assertIsNone(
            node._extrapolated_tracking_pose("map", 10_200_000_000)
        )

    def test_plan_failure_keeps_active_reference_and_path(self) -> None:
        node = bare_bridge()
        statuses = []
        original_path = Polyline([(0.0, 0.0), (1.0, 0.0)])
        node.trajectory = mock.Mock(active=True)
        node.path = original_path
        node.pending_path = object()
        node._clear_path = mock.Mock()
        node._set_status = statuses.append

        node._handle_plan_failure("NO_PATH")

        self.assertIs(node.path, original_path)
        self.assertIsNotNone(node.pending_path)
        node._clear_path.assert_not_called()
        self.assertEqual(statuses, ["NO_PATH"])

    def test_plan_failure_clears_only_an_idle_bridge(self) -> None:
        node = bare_bridge()
        node.trajectory = mock.Mock(active=False)
        node.pending_path = object()
        node._clear_path = mock.Mock()
        node._set_status = lambda _status: None

        node._handle_plan_failure("NO_PATH")

        node._clear_path.assert_called_once_with()
        self.assertIsNone(node.pending_path)

    def test_pending_activation_tf_failure_preserves_stopped_reference(self) -> None:
        node = bare_bridge()
        clock = FakeClock(nanoseconds=10_000_000_000)
        limits = trajectory_limits()
        old_path = Polyline([(0.0, 0.0), (1.0, 0.0)])
        new_path = Polyline([(0.0, 0.0), (0.0, 1.0)])
        trajectory = PersistentTrajectory(limits)
        trajectory.activate(
            old_path,
            now=0.0,
            start_progress=0.0,
            start_speed=0.0,
            start_yaw=0.0,
        )
        trajectory.begin_braking(now=0.0)
        old_snapshot = trajectory.snapshot(10.0)
        old_message = PathMessage()
        old_message.header.frame_id = "map"
        new_message = PathMessage()
        new_message.header.frame_id = "map"
        node.trajectory = trajectory
        node.path = old_path
        node.path_message = old_message
        node.pending_path = bridge_module.ParsedPath(
            new_message,
            new_path,
            [],
        )
        node.costmap = FakeCostmap(collision=False)
        node.global_frame = "map"
        node.get_clock = lambda: clock
        node._raw_path_collision_reason = lambda _parsed: None
        node._base_pose_in = mock.Mock(
            side_effect=bridge_module.TransformException("missing transform")
        )
        node._set_status = lambda _status: None

        self.assertFalse(node._activate_pending_path())

        self.assertIs(node.path, old_path)
        self.assertIs(node.path_message, old_message)
        self.assertIsNotNone(node.pending_path)
        self.assertEqual(node.trajectory.snapshot(10.0).points, old_snapshot.points)

    def test_replan_is_gated_only_by_xy_deviation(self) -> None:
        node = bare_bridge()
        node.path = Polyline([(0.0, 0.0), (2.0, 0.0)])
        node.planner_profile = NAVFN_XY_LEGACY
        node.replan_deviation = 0.30

        self.assertFalse(node._path_deviation_exceeds_limits((0.5, 0.20), 2.0))
        self.assertTrue(node._path_deviation_exceeds_limits((0.5, 0.31), 0.0))

    def test_timer_turns_a_stale_costmap_into_braking_without_zero_flash(self) -> None:
        node = bare_bridge()
        clock = FakeClock(nanoseconds=5_000_000_000)
        path = Polyline([(0.0, 0.0), (2.0, 0.0)])
        trajectory = PersistentTrajectory(trajectory_limits())
        trajectory.activate(
            path,
            now=5.0,
            start_progress=0.0,
            start_speed=0.3,
            start_yaw=0.0,
        )
        path_message = PathMessage()
        path_message.header.frame_id = "map"
        node.goal = PoseStamped()
        node.goal_reached = False
        node.trajectory = trajectory
        node.pending_path = None
        node.path = path
        node.path_message = path_message
        node.global_frame = "map"
        node.aligning_yaw = False
        node.planner_profile = NAVFN_XY_LEGACY
        node.costmap = FakeCostmap(collision=False)
        node.costmap_timeout = 1.0
        node.last_costmap_receive_ns = 3_000_000_000
        node.num_waypoints = 5
        node.waypoints_publisher = RecordingPublisher()
        node.remain_time_publisher = RecordingPublisher()
        node.sampled_path_publisher = RecordingPublisher()
        node.get_clock = lambda: clock
        node._tracking_pose_in = lambda _frame, _now_ns: ((0.0, 0.0), 0.0, False)
        node._plan_retry_due = lambda _now_ns: False
        node._update_goal_state = mock.Mock()
        node._set_status = mock.Mock()
        node._publish_zero_waypoints = mock.Mock()

        node._on_timer()

        self.assertEqual(node.trajectory.mode, PersistentTrajectory.BRAKING)
        node._publish_zero_waypoints.assert_not_called()
        self.assertEqual(len(node.waypoints_publisher.messages), 1)
        self.assertTrue(any(node.waypoints_publisher.messages[0].data))
        self.assertEqual(
            list(node.remain_time_publisher.messages[0].data),
            [0.5, 1.0, 1.5, 2.0, 2.5],
        )
        node._set_status.assert_called_with("BRAKING_NO_FRESH_COSTMAP")

    def test_zero_waypoint_deadlines_count_down(self) -> None:
        node = bare_bridge()
        node.num_waypoints = 5
        node.standing_clock = RollingDeadlineClock(0.5, 5)
        node.waypoints_publisher = RecordingPublisher()
        node.remain_time_publisher = RecordingPublisher()
        node.last_waypoints = [0.0] * 15

        node._publish_zero_waypoints(10.0)
        node._publish_zero_waypoints(10.2)

        self.assertEqual(
            list(node.remain_time_publisher.messages[0].data),
            [0.5, 1.0, 1.5, 2.0, 2.5],
        )
        self.assertEqual(
            [round(value, 3) for value in node.remain_time_publisher.messages[1].data],
            [0.3, 0.8, 1.3, 1.8, 2.3],
        )

    def test_publish_rejects_invalid_policy_abi(self) -> None:
        node = bare_bridge()
        node.num_waypoints = 5
        node.waypoints_publisher = RecordingPublisher()
        node.remain_time_publisher = RecordingPublisher()
        node.last_waypoints = [0.0] * 15

        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            node._publish_waypoints([0.0] * 15, [0.5, 1.0, 1.0, 2.0, 2.5])
        with self.assertRaisesRegex(ValueError, "finite"):
            node._publish_waypoints([float("nan")] + [0.0] * 14, [0.5, 1.0, 1.5, 2.0, 2.5])

    def test_odom_speed_is_derived_from_pose_not_twist(self) -> None:
        node = bare_bridge()
        clock = FakeClock(nanoseconds=1_000_000_000, step_ns=100_000_000)
        node.get_clock = lambda: clock
        node.odom_timeout = 0.5
        node.linear_speed = None
        node.angular_speed = None
        node.last_linear_odom_ns = 0
        node.last_angular_odom_ns = 0
        node.last_odom_pose = None
        node.last_odom_pose_ns = 0

        first = Odometry()
        first.pose.pose.orientation.w = 1.0
        first.twist.twist.linear.x = 99.0
        second = Odometry()
        second.pose.pose.position.x = 0.1
        second.pose.pose.orientation.w = 1.0
        second.twist.twist.linear.x = 99.0

        node._on_odom(first)
        node._on_odom(second)

        self.assertAlmostEqual(node.linear_speed, 1.0)
        self.assertAlmostEqual(node.angular_speed, 0.0)

    def test_odom_speed_filter_rejects_one_frame_pose_spike(self) -> None:
        node = bare_bridge()
        clock = FakeClock(nanoseconds=1_000_000_000, step_ns=100_000_000)
        node.get_clock = lambda: clock
        node.odom_timeout = 0.5
        node.odom_speed_filter_window = 2
        node.linear_speed = None
        node.angular_speed = None
        node.last_linear_odom_ns = 0
        node.last_angular_odom_ns = 0
        node.last_odom_pose = None
        node.last_odom_pose_ns = 0

        for x in (0.0, 0.1, 0.2, 1.1, 1.2, 1.3):
            message = Odometry()
            message.pose.pose.position.x = x
            message.pose.pose.orientation.w = 1.0
            node._on_odom(message)

        self.assertAlmostEqual(node.linear_speed, 1.0)
        self.assertAlmostEqual(node.angular_speed, 0.0)

    def test_odom_speed_filter_raises_after_two_sustained_fast_samples(self) -> None:
        node = bare_bridge()
        clock = FakeClock(nanoseconds=1_000_000_000, step_ns=100_000_000)
        node.get_clock = lambda: clock
        node.odom_timeout = 0.5
        node.odom_speed_filter_window = 2
        node.linear_speed = None
        node.angular_speed = None
        node.last_linear_odom_ns = 0
        node.last_angular_odom_ns = 0
        node.last_odom_pose = None
        node.last_odom_pose_ns = 0

        for x in (0.0, 0.0, 0.0, 0.0, 0.08, 0.16):
            message = Odometry()
            message.pose.pose.position.x = x
            message.pose.pose.orientation.w = 1.0
            node._on_odom(message)

        self.assertAlmostEqual(node.linear_speed, 0.8)
        self.assertAlmostEqual(node.angular_speed, 0.0)

    def test_odom_speed_filter_uses_latest_consecutive_pair(self) -> None:
        # One interval cannot raise or lower the physical-speed estimate.
        self.assertAlmostEqual(
            Nav2GlobalGoalToWaypoints._conservative_speed_estimate(
                (0.8, 0.1),
                previous=0.8,
            ),
            0.8,
        )
        self.assertAlmostEqual(
            Nav2GlobalGoalToWaypoints._conservative_speed_estimate(
                (0.1, 0.1),
                previous=0.8,
            ),
            0.1,
        )
        self.assertAlmostEqual(
            Nav2GlobalGoalToWaypoints._conservative_speed_estimate(
                (0.1, 0.8),
                previous=0.1,
            ),
            0.1,
        )
        self.assertAlmostEqual(
            Nav2GlobalGoalToWaypoints._conservative_speed_estimate(
                (0.8, 0.8),
                previous=0.1,
            ),
            0.8,
        )

    def test_reset_speed_filter_clears_scalar_estimate_and_freshness(self) -> None:
        node = bare_bridge()
        node.linear_speed = 0.8
        node.angular_speed = 0.2
        node.last_linear_odom_ns = 123
        node.last_angular_odom_ns = 456
        node._linear_speed_samples = deque((0.8, 0.8), maxlen=5)
        node._angular_speed_samples = deque((0.2, 0.2), maxlen=5)

        node._reset_speed_filter()

        self.assertIsNone(node.linear_speed)
        self.assertIsNone(node.angular_speed)
        self.assertEqual(node.last_linear_odom_ns, 0)
        self.assertEqual(node.last_angular_odom_ns, 0)
        self.assertFalse(node._linear_speed_samples)
        self.assertFalse(node._angular_speed_samples)

    def test_smac_safety_braking_rebases_past_committed_endpoint(self) -> None:
        node = bare_bridge()
        node.planner_profile = SMAC_HYBRID_XY_FORWARD
        node.path = Polyline([(0.0, 0.0), (1.0, 0.0)])
        node.trajectory = mock.Mock()
        node.trajectory.mode = PersistentTrajectory.TRACKING
        node.trajectory_limits = trajectory_limits(cruise_speed=0.8)
        node.emergency_speed_upper_bound = 1.2
        node._fresh_linear_speed = mock.Mock(return_value=0.8)

        node._begin_tracking_safety_braking(5_000_000_000, (1.25, 0.0))

        node.trajectory.begin_braking.assert_called_once_with(
            now=5.0,
            actual_speed=0.8,
            actual_progress=1.25,
        )

    def test_navfn_safety_braking_keeps_legacy_committed_prefix_call(self) -> None:
        node = bare_bridge()
        node.planner_profile = NAVFN_XY_LEGACY
        node.trajectory = mock.Mock()

        node._begin_tracking_safety_braking(5_000_000_000, (1.25, 0.0))

        node.trajectory.begin_braking.assert_called_once_with(now=5.0)

    def test_reference_stop_waits_for_fresh_actual_motion_hold(self) -> None:
        node = bare_bridge()
        node.goal_linear_speed_tolerance = 0.10
        node.goal_angular_speed_tolerance = 0.15
        node.handover_hold_time = 0.20
        node.handover_settle_tracker = YawGoalSettleTracker()
        snapshot = TrajectorySnapshot(
            tuple(SimpleNamespace(speed=0.0) for _ in range(5)),
            (0.5, 1.0, 1.5, 2.0, 2.5),
            PersistentTrajectory.BRAKING,
        )

        node._fresh_odom_speeds = lambda _now_ns: (0.20, 0.30)
        self.assertFalse(node._reference_and_robot_stopped(snapshot, 1_000_000_000))

        node._fresh_odom_speeds = lambda _now_ns: (0.05, 0.05)
        self.assertFalse(node._reference_and_robot_stopped(snapshot, 1_100_000_000))
        self.assertTrue(node._reference_and_robot_stopped(snapshot, 1_300_000_000))

    def test_pending_activation_enforces_smac_actual_stop_gate_internally(self) -> None:
        node = bare_bridge()
        node.pending_path = object()
        node.planner_profile = SMAC_HYBRID_XY_FORWARD
        node.profile_contract = mock.Mock(terminal_yaw_required=False)
        node.trajectory = mock.Mock(active=True)
        node.trajectory.snapshot.return_value = TrajectorySnapshot(
            tuple(SimpleNamespace(speed=0.0) for _ in range(5)),
            (0.5, 1.0, 1.5, 2.0, 2.5),
            PersistentTrajectory.BRAKING,
        )
        node.get_clock = lambda: FakeClock(nanoseconds=5_000_000_000)
        node._reference_and_robot_stopped = mock.Mock(return_value=False)
        node._costmap_is_fresh = mock.Mock()
        node._set_status = mock.Mock()

        self.assertFalse(node._activate_pending_path())

        node._reference_and_robot_stopped.assert_called_once()
        node._costmap_is_fresh.assert_not_called()
        node._set_status.assert_called_with("REPLAN_WAITING_FOR_ROBOT_STOP")

    def test_smac_terminal_approach_uses_actual_speed_braking_distance(self) -> None:
        node = bare_bridge()
        limits = trajectory_limits(
            cruise_speed=0.8,
            terminal_deceleration=0.5,
        )
        trajectory = PersistentTrajectory(limits)
        path = Polyline([(0.0, 0.0), (0.9, 0.0)])
        trajectory.activate(
            path,
            now=5.0,
            start_progress=0.0,
            start_speed=0.8,
            start_yaw=0.0,
            start_point=(0.0, 0.0),
        )
        node.planner_profile = SMAC_HYBRID_XY_FORWARD
        node.profile_contract = mock.Mock(terminal_yaw_required=False)
        node.terminal_approach_active = False
        node.terminal_emergency_braking_active = False
        node.xy_goal_capture_active = False
        node.aligning_yaw = False
        node.reference_matches_goal = True
        node.reference_goal_pose = Pose2D(0.9, 0.0, 0.0)
        node.path = path
        node.trajectory = trajectory
        node.trajectory_limits = limits
        node.terminal_response_time = 0.20
        node.terminal_braking_margin = 0.10
        node.max_tracking_urgency = 1.0
        node.emergency_speed_upper_bound = 1.2
        node.plan_sequence = 3
        node.plan_pending = True
        node.last_plan_request_ns = 123
        node.pending_path = object()
        node.handover_settle_tracker = mock.Mock()
        node._fresh_linear_speed = lambda _now_ns: 0.8
        node._cancel_active_plan = mock.Mock()
        node._set_status = mock.Mock()

        with mock.patch.object(
            trajectory,
            "begin_terminal_approach",
            wraps=trajectory.begin_terminal_approach,
        ) as begin_terminal_approach:
            self.assertTrue(
                node._maybe_start_terminal_approach(
                    5_000_000_000,
                    (0.0, 0.0),
                )
            )

        self.assertTrue(node.terminal_approach_active)
        self.assertEqual(node.trajectory.mode, PersistentTrajectory.TERMINAL_APPROACH)
        self.assertIsNone(node.pending_path)
        self.assertFalse(node.plan_pending)
        self.assertFalse(node.terminal_emergency_braking_active)
        self.assertAlmostEqual(node.terminal_closest_goal_distance, 0.9)
        self.assertAlmostEqual(node._terminal_braking_distance(0.8), 0.9)
        begin_terminal_approach.assert_called_once_with(
            now=5.0,
            actual_speed=0.8,
            response_time=0.2,
            actual_progress=0.0,
        )

    def test_smac_terminal_overspeed_uses_measured_bounded_braking(self) -> None:
        node = bare_bridge()
        limits = trajectory_limits(
            cruise_speed=0.8,
            terminal_deceleration=0.5,
        )
        trajectory = PersistentTrajectory(limits)
        path = Polyline([(0.0, 0.0), (1.0, 0.0)])
        trajectory.activate(
            path,
            now=5.0,
            start_progress=0.0,
            start_speed=0.8,
            start_yaw=0.0,
            start_point=(0.0, 0.0),
        )
        node.planner_profile = SMAC_HYBRID_XY_FORWARD
        node.profile_contract = mock.Mock(terminal_yaw_required=False)
        node.terminal_approach_active = False
        node.terminal_emergency_braking_active = False
        node.xy_goal_capture_active = False
        node.aligning_yaw = False
        node.reference_matches_goal = True
        node.reference_goal_pose = Pose2D(1.0, 0.0, 0.0)
        node.path = path
        node.trajectory = trajectory
        node.trajectory_limits = limits
        node.terminal_response_time = 0.20
        node.terminal_braking_margin = 0.10
        node.plan_sequence = 3
        node.plan_pending = True
        node.last_plan_request_ns = 123
        node.pending_path = object()
        node.handover_settle_tracker = mock.Mock()
        node._fresh_linear_speed = lambda _now_ns: 1.4
        node._cancel_active_plan = mock.Mock()
        node._set_status = mock.Mock()

        with mock.patch.object(
            trajectory,
            "begin_braking",
            wraps=trajectory.begin_braking,
        ) as begin_braking:
            self.assertTrue(
                node._maybe_start_terminal_approach(
                    5_000_000_000,
                    (0.0, 0.0),
                )
            )

        self.assertTrue(node.terminal_approach_active)
        self.assertTrue(node.terminal_emergency_braking_active)
        self.assertAlmostEqual(node.terminal_closest_goal_distance, 1.0)
        self.assertEqual(node.trajectory.mode, PersistentTrajectory.BRAKING)
        begin_braking.assert_called_once_with(
            now=5.0,
            actual_speed=1.4,
            actual_progress=0.0,
        )
        self.assertTrue(
            any(
                "BRAKING_TERMINAL_OVERSPEED" in call.args[0]
                for call in node._set_status.call_args_list
            )
        )

    def test_terminal_miss_switches_comfortable_approach_to_bounded_braking(
        self,
    ) -> None:
        node = bare_bridge()
        node.terminal_approach_active = True
        node.terminal_emergency_braking_active = False
        node.terminal_closest_goal_distance = 0.20
        node.xy_goal_capture_active = False
        node.reference_goal_pose = Pose2D(1.0, 0.0, 0.0)
        node.goal_capture_release_tolerance = 0.30
        node.path = Polyline([(0.0, 0.0), (1.0, 0.0)])
        node.trajectory = mock.Mock()
        node.handover_settle_tracker = mock.Mock()
        node._fresh_linear_speed = mock.Mock(return_value=0.40)
        node._set_status = mock.Mock()

        self.assertTrue(
            node._monitor_terminal_miss(
                5_000_000_000,
                (1.35, 0.0),
            )
        )

        self.assertTrue(node.terminal_emergency_braking_active)
        self.assertAlmostEqual(node.terminal_closest_goal_distance, 0.20)
        node.trajectory.begin_braking.assert_called_once_with(
            now=5.0,
            actual_speed=0.40,
            actual_progress=1.35,
        )
        node.handover_settle_tracker.reset.assert_called_once_with()
        self.assertIn(
            "BRAKING_TERMINAL_MISSED",
            node._set_status.call_args.args[0],
        )

    def test_terminal_miss_with_stale_odom_uses_fail_closed_speed(self) -> None:
        node = bare_bridge()
        node.terminal_approach_active = True
        node.terminal_emergency_braking_active = False
        node.terminal_closest_goal_distance = 0.20
        node.xy_goal_capture_active = False
        node.reference_goal_pose = Pose2D(1.0, 0.0, 0.0)
        node.goal_capture_release_tolerance = 0.30
        node.path = Polyline([(0.0, 0.0), (1.0, 0.0)])
        node.trajectory_limits = trajectory_limits(cruise_speed=0.8)
        node.max_tracking_urgency = 1.0
        node.emergency_speed_upper_bound = 1.2
        node.linear_speed = 0.4  # stale, but still the last finite observation
        node.trajectory = mock.Mock()
        node.handover_settle_tracker = mock.Mock()
        node._fresh_linear_speed = mock.Mock(return_value=None)
        node._set_status = mock.Mock()

        self.assertTrue(
            node._monitor_terminal_miss(
                5_000_000_000,
                (1.35, 0.0),
            )
        )

        node.trajectory.begin_braking.assert_called_once_with(
            now=5.0,
            actual_speed=1.2,
            actual_progress=1.35,
        )
        status = node._set_status.call_args.args[0]
        self.assertIn("BRAKING_TERMINAL_MISSED_STALE_ODOM", status)
        self.assertIn("speed=1.200", status)

    def test_terminal_miss_monitor_tracks_closest_distance_while_approaching(
        self,
    ) -> None:
        node = bare_bridge()
        node.terminal_approach_active = True
        node.terminal_emergency_braking_active = False
        node.terminal_closest_goal_distance = 0.50
        node.xy_goal_capture_active = False
        node.reference_goal_pose = Pose2D(1.0, 0.0, 0.0)
        node.goal_capture_release_tolerance = 0.30
        node.trajectory = mock.Mock()
        node.handover_settle_tracker = mock.Mock()
        node._fresh_linear_speed = mock.Mock()
        node._set_status = mock.Mock()

        self.assertFalse(
            node._monitor_terminal_miss(
                5_000_000_000,
                (0.60, 0.0),
            )
        )

        self.assertAlmostEqual(node.terminal_closest_goal_distance, 0.40)
        node.trajectory.begin_braking.assert_not_called()
        node._fresh_linear_speed.assert_not_called()

    def test_stopped_terminal_replans_if_goal_is_not_reached(self) -> None:
        node = bare_bridge()
        snapshot = TrajectorySnapshot(
            tuple(SimpleNamespace(speed=0.0) for _ in range(5)),
            (0.5, 1.0, 1.5, 2.0, 2.5),
            PersistentTrajectory.BRAKING,
        )
        node.terminal_emergency_braking_active = False
        node.terminal_approach_active = True
        node.terminal_closest_goal_distance = 0.40
        node.reference_goal_pose = Pose2D(1.0, 0.0, 0.0)
        node.goal_tolerance = 0.15
        node.trajectory = mock.Mock()
        node._clear_path = mock.Mock()
        node.standing_clock = mock.Mock()
        node.goal_settle_tracker = mock.Mock()
        node.yaw_goal_settle_tracker = mock.Mock()
        node.handover_settle_tracker = mock.Mock()
        node._reference_and_robot_stopped = mock.Mock(return_value=True)
        node._reset_speed_filter = mock.Mock()
        node._publish_zero_waypoints = mock.Mock()
        node._set_status = mock.Mock()
        node._request_plan = mock.Mock()

        self.assertTrue(
            node._recover_stopped_terminal(
                snapshot,
                5_000_000_000,
                (0.5, 0.0),
            )
        )

        node.trajectory.clear.assert_called_once_with()
        node._clear_path.assert_called_once_with(publish_empty=True)
        node.standing_clock.reset.assert_called_once_with()
        self.assertFalse(node.terminal_approach_active)
        self.assertFalse(node.terminal_emergency_braking_active)
        self.assertIsNone(node.terminal_closest_goal_distance)
        node._reset_speed_filter.assert_called_once_with()
        node._publish_zero_waypoints.assert_called_once_with(5.0)
        node._request_plan.assert_called_once_with(bridge_module.PLAN_SAFETY)

    def test_periodic_discontinuous_replan_keeps_active_reference(self) -> None:
        node = bare_bridge()
        old_path = Polyline([(0.0, 0.0), (1.0, 0.0)])
        old_message = PathMessage()
        old_message.header.frame_id = "map"
        new_message = PathMessage()
        new_message.header.frame_id = "map"
        parsed = bridge_module.ParsedPath(
            new_message,
            Polyline([(0.0, 0.1), (1.0, 0.0)]),
            [Pose2D(0.0, 0.1, 0.0), Pose2D(1.0, 0.0, 0.0)],
        )
        node.planner_profile = SMAC_HYBRID_XY_FORWARD
        node.profile_contract = mock.Mock(terminal_yaw_required=False)
        node.plan_request_reason = bridge_module.PLAN_PERIODIC
        node.trajectory = mock.Mock(active=True)
        node.trajectory.mode = PersistentTrajectory.TRACKING
        node.trajectory.adopt_replan.return_value = False
        node.path = old_path
        node.path_message = old_message
        node.costmap = FakeCostmap(collision=False)
        node.goal_tolerance = 0.15
        node.handover_settle_tracker = mock.Mock()
        node.get_clock = lambda: FakeClock(nanoseconds=5_000_000_000)
        node._costmap_is_fresh = lambda _now_ns: True
        node._raw_path_collision_reason = lambda _parsed: None
        node._goal_pose_in = lambda _frame: Pose2D(1.0, 0.0, 0.0)
        node._set_status = mock.Mock()

        node._accept_path(parsed)

        self.assertIs(node.path, old_path)
        self.assertIs(node.path_message, old_message)
        self.assertIsNone(node.pending_path)
        node.trajectory.begin_braking.assert_not_called()
        node._set_status.assert_called_with("PATH_REPLAN_REJECTED_CONTINUITY")

    def test_navfn_periodic_replan_keeps_legacy_braking_fallback(self) -> None:
        node = bare_bridge()
        parsed = mock.Mock()
        parsed.message.header.frame_id = "map"
        parsed.poses = [Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0)]
        node.planner_profile = NAVFN_XY_LEGACY
        node.profile_contract = mock.Mock(terminal_yaw_required=False)
        node.plan_request_reason = bridge_module.PLAN_PERIODIC
        node.trajectory = mock.Mock(active=True)
        node.trajectory.mode = PersistentTrajectory.TRACKING
        node.trajectory.adopt_replan.return_value = False
        node.costmap = FakeCostmap(collision=False)
        node.goal_tolerance = 0.15
        node.handover_settle_tracker = mock.Mock()
        node.get_clock = lambda: FakeClock(nanoseconds=5_000_000_000)
        node._costmap_is_fresh = lambda _now_ns: True
        node._raw_path_collision_reason = lambda _parsed: None
        node._goal_pose_in = lambda _frame: Pose2D(1.0, 0.0, 0.0)
        node._set_status = mock.Mock()

        node._accept_path(parsed)

        node.trajectory.adopt_replan.assert_called_once_with(
            parsed.model,
            now=5.0,
            strict_continuity=False,
        )
        node.trajectory.begin_braking.assert_called_once_with(now=5.0)
        node._set_status.assert_called_with("REPLAN_WAITING_FOR_BRAKE")

    def test_terminal_alignment_uses_a_persistent_timed_rotation_path(self) -> None:
        node = bare_bridge()
        node.goal = PoseStamped()
        node.goal.header.frame_id = "map"
        node.goal.pose.position.x = 1.0
        node.goal.pose.position.y = 2.0
        node.goal.pose.orientation.z = 0.5 ** 0.5
        node.goal.pose.orientation.w = 0.5 ** 0.5
        node.goal_yaw = 0.5 * 3.141592653589793
        node.global_frame = "map"
        node.trajectory = PersistentTrajectory(trajectory_limits())
        node.standing_clock = RollingDeadlineClock(0.5, 5)
        node.goal_settle_tracker = mock.Mock()
        node.yaw_goal_settle_tracker = mock.Mock()
        node.handover_settle_tracker = mock.Mock()
        node.pending_path = None
        node.get_clock = lambda: FakeClock()
        node._set_status = lambda _status: None

        node._start_terminal_alignment((0.9, 1.9), 0.0, 10.0)
        snapshot = node.trajectory.snapshot(10.2)

        self.assertTrue(node.aligning_yaw)
        self.assertTrue(node.reference_matches_goal)
        self.assertEqual(snapshot.remain_time, (0.3, 0.8, 1.3, 1.8, 2.3))
        self.assertTrue(all(point.pose.point == (0.9, 1.9) for point in snapshot.points))
        self.assertTrue(all(
            first.pose.yaw <= second.pose.yaw
            for first, second in zip(snapshot.points, snapshot.points[1:])
        ))
        self.assertEqual(node.reference_goal_pose.point, (1.0, 2.0))

        node.profile_contract = mock.Mock(terminal_yaw_required=True)
        node.goal_tolerance = 0.15
        node.goal_linear_speed_tolerance = 0.1
        node.goal_yaw_tolerance = 0.1
        node.goal_angular_speed_tolerance = 0.15
        node.goal_hold_time = 0.5
        node._fresh_odom_speeds = lambda _now_ns: (0.0, 0.0)
        node.yaw_goal_settle_tracker.update.return_value = False
        node._finish_goal = mock.Mock()

        node._update_goal_state(10_200_000_000, (0.9, 1.9), 0.0)

        settle_position_error = node.yaw_goal_settle_tracker.update.call_args.args[1]
        self.assertAlmostEqual(settle_position_error, math.hypot(0.1, 0.1))

    def test_smac_xy_goal_capture_uses_real_bounded_position_and_motion_settle(
        self,
    ) -> None:
        node = bare_bridge()
        node.planner_profile = SMAC_HYBRID_XY_FORWARD
        node.profile_contract = mock.Mock(terminal_yaw_required=False)
        node.reference_matches_goal = True
        node.reference_goal_pose = Pose2D(1.0, 0.0, 0.0)
        node.goal = PoseStamped()
        node.path = Polyline([(0.0, 0.0), (1.0, 0.0)])
        node.xy_goal_capture_active = False
        node.goal_tolerance = 0.15
        node.goal_capture_release_tolerance = 0.30
        node.goal_linear_speed_tolerance = 0.10
        node.goal_angular_speed_tolerance = 0.15
        node.goal_hold_time = 0.50
        node.goal_settle_tracker = mock.Mock()
        node.yaw_goal_settle_tracker = mock.Mock()
        node.yaw_goal_settle_tracker.update.return_value = False
        node._fresh_odom_speeds = lambda _now_ns: (0.20, 0.0)
        node._finish_goal = mock.Mock()
        node._set_status = mock.Mock()

        def latch_capture(
            _now: float,
            *,
            actual_speed=None,
            actual_progress=None,
        ) -> None:
            node.xy_goal_capture_active = True

        node._start_xy_goal_capture = mock.Mock(side_effect=latch_capture)
        node._update_goal_state(1_000_000_000, (0.90, 0.0), 1.0)

        node._start_xy_goal_capture.assert_called_once_with(
            1.0,
            actual_speed=0.20,
            actual_progress=0.90,
        )
        self.assertAlmostEqual(
            node.yaw_goal_settle_tracker.update.call_args.args[1], 0.10
        )
        node._finish_goal.assert_not_called()

        node.yaw_goal_settle_tracker.update.reset_mock()
        node._fresh_odom_speeds = lambda _now_ns: (0.02, 0.02)
        node._update_goal_state(1_600_000_000, (1.35, 0.0), 2.0)

        node.yaw_goal_settle_tracker.update.assert_not_called()
        self.assertTrue(
            any(
                "GOAL_CAPTURE_DRIFTED" in call.args[0]
                for call in node._set_status.call_args_list
            )
        )
        node._start_xy_goal_capture.assert_called_once()
        node._finish_goal.assert_not_called()

        node.yaw_goal_settle_tracker.update.return_value = True
        node._update_goal_state(2_200_000_000, (0.78, 0.0), 2.0)

        node._finish_goal.assert_called_once_with(2.2)
        self.assertAlmostEqual(
            node.yaw_goal_settle_tracker.update.call_args.args[1],
            0.22,
        )
        self.assertAlmostEqual(
            node.yaw_goal_settle_tracker.update.call_args.args[5],
            0.30,
        )

    def test_smac_xy_capture_cannot_finish_without_entering_goal_tolerance(self) -> None:
        node = bare_bridge()
        node.planner_profile = SMAC_HYBRID_XY_FORWARD
        node.profile_contract = mock.Mock(terminal_yaw_required=False)
        node.reference_matches_goal = True
        node.reference_goal_pose = Pose2D(1.0, 0.0, 0.0)
        node.goal = PoseStamped()
        node.xy_goal_capture_active = False
        node.goal_tolerance = 0.15
        node.goal_capture_release_tolerance = 0.30
        node.goal_settle_tracker = mock.Mock()
        node.yaw_goal_settle_tracker = mock.Mock()
        node._fresh_odom_speeds = lambda _now_ns: (0.0, 0.0)
        node._start_xy_goal_capture = mock.Mock()
        node._finish_goal = mock.Mock()
        node._set_status = mock.Mock()

        # 0.15--0.30 m is the release band only after a real <=0.15 m latch.
        node._update_goal_state(1_000_000_000, (0.80, 0.0), 0.0)

        node._start_xy_goal_capture.assert_not_called()
        node._finish_goal.assert_not_called()
        node.yaw_goal_settle_tracker.update.assert_not_called()

    def test_path_progress_preserves_forward_overshoot_beyond_polyline_endpoint(self) -> None:
        node = bare_bridge()
        node.path = Polyline([(0.0, 0.0), (1.0, 0.0)])

        self.assertAlmostEqual(node._path_progress_at_point((1.25, 0.0)), 1.25)
        # A perpendicular offset is not forward progress along the terminal
        # tangent and therefore remains clamped at the endpoint.
        self.assertAlmostEqual(node._path_progress_at_point((1.0, 0.25)), 1.0)

    def test_stopped_goal_capture_beyond_release_replans_from_rest(self) -> None:
        node = bare_bridge()
        node.xy_goal_capture_active = True
        node.terminal_approach_active = False
        node.terminal_emergency_braking_active = False
        node.terminal_closest_goal_distance = None
        node.reference_matches_goal = True
        node.reference_goal_pose = Pose2D(1.0, 0.0, 0.0)
        node.goal_capture_release_tolerance = 0.30
        node.trajectory = mock.Mock()
        node._clear_path = mock.Mock()
        node.standing_clock = mock.Mock()
        node.pending_path = object()
        node.goal_settle_tracker = mock.Mock()
        node.yaw_goal_settle_tracker = mock.Mock()
        node.handover_settle_tracker = mock.Mock()
        node._reset_speed_filter = mock.Mock()
        node._publish_zero_waypoints = mock.Mock()
        node._set_status = mock.Mock()
        node._request_plan = mock.Mock()

        self.assertTrue(
            node._recover_stopped_goal_capture(
                5_000_000_000,
                (0.65, 0.0),
                robot_stopped=True,
            )
        )

        node.trajectory.clear.assert_called_once_with()
        node._clear_path.assert_called_once_with(publish_empty=True)
        self.assertFalse(node.xy_goal_capture_active)
        self.assertFalse(node.reference_matches_goal)
        self.assertIsNone(node.reference_goal_pose)
        self.assertIsNone(node.pending_path)
        node._reset_speed_filter.assert_called_once_with()
        node._publish_zero_waypoints.assert_called_once_with(5.0)
        node._set_status.assert_called_once_with(
            "GOAL_CAPTURE_DRIFTED_REPLANNING"
        )
        node._request_plan.assert_called_once_with(bridge_module.PLAN_SAFETY)

    def test_start_xy_goal_capture_stops_planning_and_brakes_active_horizon(self) -> None:
        node = bare_bridge()
        node.xy_goal_capture_active = False
        node.terminal_approach_active = True
        node.terminal_emergency_braking_active = False
        node.plan_sequence = 4
        node.plan_pending = True
        node.last_plan_request_ns = 123
        node.pending_path = object()
        node.trajectory = mock.Mock()
        node.aligning_yaw = False
        node.alignment_goal_point = None
        node.alignment_goal_yaw = None
        node.goal_settle_tracker = mock.Mock()
        node.yaw_goal_settle_tracker = mock.Mock()
        node.handover_settle_tracker = mock.Mock()
        node.standing_clock = mock.Mock()
        node._cancel_active_plan = mock.Mock()
        node._set_status = mock.Mock()

        node._start_xy_goal_capture(
            3.5,
            actual_speed=0.25,
            actual_progress=0.70,
        )

        self.assertTrue(node.xy_goal_capture_active)
        self.assertFalse(node.terminal_approach_active)
        self.assertFalse(node.terminal_emergency_braking_active)
        self.assertEqual(node.plan_sequence, 5)
        self.assertFalse(node.plan_pending)
        self.assertEqual(node.last_plan_request_ns, 0)
        self.assertIsNone(node.pending_path)
        node._cancel_active_plan.assert_called_once_with()
        node.trajectory.begin_braking.assert_called_once_with(
            now=3.5,
            actual_speed=0.25,
            actual_progress=0.70,
        )
        node.trajectory.clear.assert_not_called()
        node.goal_settle_tracker.reset.assert_called_once_with()
        node.yaw_goal_settle_tracker.reset.assert_called_once_with()
        node.handover_settle_tracker.reset.assert_called_once_with()
        node.standing_clock.reset.assert_not_called()
        node._set_status.assert_called_once_with("GOAL_CAPTURE_BRAKING")

    def test_active_tracking_requests_periodic_replan_without_changing_horizon(self) -> None:
        node = bare_bridge()
        clock = FakeClock(nanoseconds=5_000_000_000)
        path = Polyline([(0.0, 0.0), (2.0, 0.0)])
        trajectory = PersistentTrajectory(trajectory_limits())
        trajectory.activate(
            path,
            now=5.0,
            start_progress=0.0,
            start_speed=0.3,
            start_yaw=0.0,
            start_point=(0.0, 0.0),
        )
        before = trajectory.snapshot(5.0)
        path_message = PathMessage()
        path_message.header.frame_id = "map"
        node.goal = PoseStamped()
        node.goal_reached = False
        node.trajectory = trajectory
        node.pending_path = None
        node.path = path
        node.path_message = path_message
        node.global_frame = "map"
        node.aligning_yaw = False
        node.plan_pending = False
        node.replan_period = 1.0
        node.last_plan_request_ns = 4_200_000_000
        node.costmap = FakeCostmap(collision=False)
        node.costmap_timeout = 1.0
        node.last_costmap_receive_ns = 5_000_000_000
        node.get_clock = lambda: clock
        node._tracking_pose_in = lambda _frame, _now_ns: ((0.0, 0.0), 0.0, False)
        node._trajectory_collision_reason = lambda *_args: None
        node._path_deviation_exceeds_limits = lambda *_args: False
        node._publish_waypoints = mock.Mock()
        node._publish_sampled_path = mock.Mock()
        node._update_goal_state = mock.Mock()
        node._set_status = mock.Mock()
        node._request_plan = mock.Mock()

        node._on_timer()
        node._request_plan.assert_not_called()

        clock.advance(0.25)
        node._on_timer()

        node._request_plan.assert_called_once_with(bridge_module.PLAN_PERIODIC)
        self.assertEqual(trajectory.snapshot(5.25).points, before.points)

    def test_periodic_replan_respects_alignment_pending_and_inflight_guards(self) -> None:
        node = bare_bridge()
        path = Polyline([(0.0, 0.0), (2.0, 0.0)])
        trajectory = PersistentTrajectory(trajectory_limits())
        trajectory.activate(
            path,
            now=5.0,
            start_progress=0.0,
            start_speed=0.3,
            start_yaw=0.0,
            start_point=(0.0, 0.0),
        )
        path_message = PathMessage()
        path_message.header.frame_id = "map"
        node.goal = PoseStamped()
        node.goal_reached = False
        node.trajectory = trajectory
        node.path = path
        node.path_message = path_message
        node.global_frame = "map"
        node.replan_period = 1.0
        node.last_plan_request_ns = 3_000_000_000
        node.costmap = FakeCostmap(collision=False)
        node.costmap_timeout = 1.0
        node.last_costmap_receive_ns = 5_000_000_000
        node.get_clock = lambda: FakeClock(nanoseconds=5_000_000_000)
        node._tracking_pose_in = lambda _frame, _now_ns: ((0.0, 0.0), 0.0, False)
        node._trajectory_collision_reason = lambda *_args: None
        node._path_deviation_exceeds_limits = lambda *_args: False
        node._publish_waypoints = mock.Mock()
        node._publish_sampled_path = mock.Mock()
        node._update_goal_state = mock.Mock()
        node._set_status = mock.Mock()
        node._request_plan = mock.Mock()

        guarded_states = (
            (True, None, False),
            (False, object(), False),
            (False, None, True),
        )
        for aligning_yaw, pending_path, plan_pending in guarded_states:
            with self.subTest(
                aligning_yaw=aligning_yaw,
                has_pending=pending_path is not None,
                plan_pending=plan_pending,
            ):
                node.aligning_yaw = aligning_yaw
                node.pending_path = pending_path
                node.plan_pending = plan_pending
                node._on_timer()

        node._request_plan.assert_not_called()

    def test_base_pose_rejects_non_finite_tf_values(self) -> None:
        node = bare_bridge()
        node.base_frame = "base_link"
        node.tf_buffer = mock.Mock()
        transform = mock.Mock()
        transform.transform.translation.x = float("nan")
        transform.transform.translation.y = 1.0
        transform.transform.rotation = bridge_module.quaternion_from_yaw(0.0)
        node.tf_buffer.lookup_transform.return_value = transform

        with self.assertRaisesRegex(bridge_module.TransformException, "non-finite"):
            node._base_pose_in("map")

    def test_goal_pose_rejects_non_finite_tf_values(self) -> None:
        node = bare_bridge()
        node.global_frame = "map"
        node.goal = PoseStamped()
        node.goal.header.frame_id = "odom"
        node.goal.pose.orientation.w = 1.0
        node.goal_yaw = 0.0
        node.tf_buffer = mock.Mock()
        transform = mock.Mock()
        transform.transform.translation.x = float("inf")
        transform.transform.translation.y = 0.0
        transform.transform.rotation = bridge_module.quaternion_from_yaw(0.0)
        node.tf_buffer.lookup_transform.return_value = transform

        with self.assertRaisesRegex(bridge_module.TransformException, "non-finite"):
            node._goal_pose_in("map")

    def test_planner_endpoint_is_not_treated_as_real_goal(self) -> None:
        """A truncated planner path must not make the user goal look reached."""
        node = bare_bridge()
        node.planner_profile = NAVFN_XY_LEGACY
        node.profile_contract = mock.Mock(terminal_yaw_required=False)
        node.reference_matches_goal = True
        node.goal = PoseStamped()
        node.goal.pose.position.x = 10.0
        node.goal.pose.position.y = 0.0
        node.path = Polyline([(0.0, 0.0), (2.0, 0.0)])
        node.aligning_yaw = False
        node.goal_settle_tracker = mock.Mock()
        node.goal_settle_tracker.update.return_value = True
        node.yaw_goal_settle_tracker = mock.Mock()
        node._fresh_odom_speeds = lambda _now_ns: (0.0, 0.0)
        node._finish_goal = mock.Mock()
        node.goal_tolerance = 0.1
        node.goal_linear_speed_tolerance = 0.1
        node.goal_hold_time = 0.5

        node._update_goal_state(1_000_000_000, (2.0, 0.0), 0.0)

        node._finish_goal.assert_not_called()

    def test_every_terminal_yaw_profile_enters_independent_alignment(self) -> None:
        node = bare_bridge()
        node.profile_contract = mock.Mock(terminal_yaw_required=True)
        node.reference_matches_goal = True
        node.reference_goal_pose = bridge_module.Pose2D(1.0, 2.0, 1.0)
        node.goal = PoseStamped()
        node.aligning_yaw = False
        node.goal_tolerance = 0.15
        node.goal_linear_speed_tolerance = 0.1
        node.goal_hold_time = 0.5
        node.goal_settle_tracker = mock.Mock()
        node.goal_settle_tracker.update.return_value = True
        node.yaw_goal_settle_tracker = mock.Mock()
        node._fresh_odom_speeds = lambda _now_ns: (0.0, 0.0)
        node.path_message = PathMessage()
        node.path_message.header.frame_id = "map"
        node.global_frame = "map"
        node._start_terminal_alignment = mock.Mock()

        node._update_goal_state(2_000_000_000, (1.0, 2.0), 0.2)

        node._start_terminal_alignment.assert_called_once()

    def test_pending_activation_tf_failure_preserves_pending_reference(self) -> None:
        node = bare_bridge()
        pending = mock.Mock()
        node.pending_path = pending
        node.costmap = mock.Mock()
        node._raw_path_collision_reason = lambda _parsed: None
        node._base_pose_in = mock.Mock(
            side_effect=bridge_module.TransformException("TF unavailable")
        )
        node._set_status = mock.Mock()
        node.get_clock = lambda: FakeClock()
        node.trajectory = mock.Mock(active=True)

        self.assertFalse(node._activate_pending_path())
        self.assertIs(node.pending_path, pending)
        node.trajectory.clear.assert_not_called()

    def test_initial_activation_rejects_a_path_far_from_the_robot(self) -> None:
        node = bare_bridge()
        clock = FakeClock(nanoseconds=5_000_000_000)
        message = PathMessage()
        message.header.frame_id = "map"
        poses = [
            bridge_module.Pose2D(0.0, 0.0, 0.0),
            bridge_module.Pose2D(1.0, 0.0, 0.0),
        ]
        node.pending_path = bridge_module.ParsedPath(
            message,
            Polyline([pose.point for pose in poses]),
            poses,
        )
        node.costmap = FakeCostmap(collision=False)
        node.costmap_timeout = 1.0
        node.last_costmap_receive_ns = 5_000_000_000
        node.global_frame = "map"
        node.planner_profile = NAVFN_XY_LEGACY
        node.trajectory_limits = trajectory_limits()
        node.trajectory = mock.Mock(active=False)
        node.goal = PoseStamped()
        node.goal.header.frame_id = "map"
        node.goal.pose.position.x = 1.0
        node.goal.pose.orientation.w = 1.0
        node.goal_yaw = 0.0
        node.goal_tolerance = 0.15
        node.get_clock = lambda: clock
        node._raw_path_collision_reason = lambda _parsed: None
        node._base_pose_in = lambda _frame: ((0.0, 1.0), 0.0)
        node._clear_path = mock.Mock()
        node._set_status = mock.Mock()

        self.assertFalse(node._activate_pending_path())

        node.trajectory.activate.assert_not_called()
        self.assertIsNone(node.pending_path)
        self.assertTrue(
            any(
                "robot is" in call.args[0]
                for call in node._set_status.call_args_list
            )
        )

    def test_initial_activation_accepts_reverse_motion_and_passes_robot_origin(self) -> None:
        node = bare_bridge()
        clock = FakeClock(nanoseconds=5_000_000_000)
        message = PathMessage()
        message.header.frame_id = "map"
        poses = [
            bridge_module.Pose2D(0.0, 0.0, 0.0),
            bridge_module.Pose2D(1.0, 0.0, 0.0),
        ]
        node.pending_path = bridge_module.ParsedPath(
            message,
            Polyline([pose.point for pose in poses]),
            poses,
        )
        node.costmap = FakeCostmap(collision=False)
        node.global_frame = "map"
        node.trajectory_limits = trajectory_limits()
        node.trajectory = mock.Mock(active=False)
        node.goal = PoseStamped()
        node.goal.header.frame_id = "map"
        node.goal.pose.position.x = 1.0
        node.goal.pose.orientation.w = 1.0
        node.goal_yaw = 0.0
        node.goal_tolerance = 0.15
        node.get_clock = lambda: clock
        node._costmap_is_fresh = lambda _now_ns: True
        node._raw_path_collision_reason = lambda _parsed: None
        node._base_pose_in = lambda _frame: ((0.0, 0.0), math.pi)
        node._goal_pose_in = lambda _frame: bridge_module.Pose2D(1.0, 0.0, 0.0)
        node._fresh_linear_speed = lambda _now_ns: 0.0
        node.standing_clock = mock.Mock()
        node.raw_path_publisher = RecordingPublisher()
        node._set_status = mock.Mock()

        self.assertTrue(node._activate_pending_path())

        node.trajectory.activate.assert_called_once_with(
            node.path,
            now=5.0,
            start_progress=0.0,
            start_speed=0.0,
            start_yaw=math.pi,
            start_point=(0.0, 0.0),
        )
        node._set_status.assert_called_with("PATH_READY")

    def test_shutdown_take_message_runtime_error_is_suppressed(self) -> None:
        fake_node = mock.Mock()
        shutdown_error = RuntimeError(
            "Unable to convert call argument to Python object "
            "(compile in debug mode for details)"
        )
        with (
            mock.patch.object(bridge_module.rclpy, "init"),
            mock.patch.object(
                bridge_module,
                "Nav2GlobalGoalToWaypoints",
                return_value=fake_node,
            ),
            mock.patch.object(bridge_module.rclpy, "spin", side_effect=shutdown_error),
            mock.patch.object(bridge_module.rclpy, "ok", return_value=False),
            mock.patch.object(bridge_module.rclpy, "shutdown") as shutdown,
        ):
            bridge_module.main()

        fake_node.destroy_node.assert_called_once_with()
        shutdown.assert_not_called()

    def test_unexpected_runtime_error_is_not_suppressed(self) -> None:
        fake_node = mock.Mock()
        with (
            mock.patch.object(bridge_module.rclpy, "init"),
            mock.patch.object(
                bridge_module,
                "Nav2GlobalGoalToWaypoints",
                return_value=fake_node,
            ),
            mock.patch.object(
                bridge_module.rclpy,
                "spin",
                side_effect=RuntimeError("unexpected failure"),
            ),
            mock.patch.object(bridge_module.rclpy, "ok", return_value=True),
            mock.patch.object(bridge_module.rclpy, "shutdown"),
        ):
            with self.assertRaisesRegex(RuntimeError, "unexpected failure"):
                bridge_module.main()

        fake_node.destroy_node.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
