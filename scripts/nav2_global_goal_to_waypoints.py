#!/usr/bin/python3
"""Bridge Nav2 paths to StepIt's persistent, time-parameterized waypoint queue."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from functools import partial
from typing import Optional, Sequence, Union

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

from nav2_waypoint_sampling import (
    CanonicalPath,
    CostmapView,
    GoalSettleTracker,
    NAVFN_XY_LEGACY,
    Point2,
    Polyline,
    Pose2D,
    SMAC_HYBRID_XY_FORWARD,
    SMAC_LATTICE_FULL_SE2,
    SMAC_TERMINAL_YAW,
    STARTUP_PROFILE_CONTRACTS,
    distance,
    wrap_to_pi,
    yaw_from_quaternion,
    YawGoalSettleTracker,
)
from nav2_waypoint_trajectory import (
    PersistentTrajectory,
    RollingDeadlineClock,
    TrajectoryLimits,
    TrajectorySnapshot,
)
from nav2_waypoint_profiles import (
    approach_yaw_candidates,
    path_centerline_collision_reason,
    relative_waypoints,
)


PathModel = Union[Polyline, CanonicalPath]
POLICY_NUM_WAYPOINTS = 5
POLICY_WAYPOINT_INTERVAL = 0.5
PLAN_INITIAL = "INITIAL"
PLAN_NEW_GOAL = "NEW_GOAL"
PLAN_PERIODIC = "PERIODIC"
PLAN_DEVIATION = "DEVIATION"
PLAN_SAFETY = "SAFETY"


@dataclass(frozen=True)
class ParsedPath:
    message: Path
    model: PathModel
    poses: list[Pose2D]


def quaternion_from_yaw(yaw: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(0.5 * yaw)
    quaternion.w = math.cos(0.5 * yaw)
    return quaternion


class Nav2GlobalGoalToWaypoints(Node):
    def __init__(self) -> None:
        super().__init__("nav2_global_goal_to_waypoints")

        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("waypoints_topic", "/waypoints_b")
        self.declare_parameter("remain_time_topic", "/remain_time")
        self.declare_parameter("status_topic", "/nav2_waypoint_status")
        self.declare_parameter("raw_path_topic", "/nav2_global_path")
        self.declare_parameter("sampled_path_topic", "/nav2_sampled_waypoints_path")
        self.declare_parameter("compute_path_action", "/compute_path_to_pose")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap_raw")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("planner_profile", NAVFN_XY_LEGACY)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("replan_period", 1.0)
        self.declare_parameter("replan_deviation", 0.30)
        self.declare_parameter("goal_tolerance", 0.15)
        self.declare_parameter("goal_linear_speed_tolerance", 0.10)
        self.declare_parameter("goal_yaw_tolerance", 0.10)
        self.declare_parameter("goal_angular_speed_tolerance", 0.15)
        self.declare_parameter("goal_hold_time", 0.50)
        self.declare_parameter("goal_capture_release_tolerance", 0.30)
        self.declare_parameter("handover_hold_time", 0.20)
        self.declare_parameter("odom_timeout", 0.50)
        self.declare_parameter("odom_speed_filter_window", 2)
        self.declare_parameter("costmap_timeout", 1.00)
        self.declare_parameter("num_waypoints", POLICY_NUM_WAYPOINTS)
        self.declare_parameter("waypoint_interval", POLICY_WAYPOINT_INTERVAL)
        self.declare_parameter("cruise_speed", 0.3)
        self.declare_parameter("max_acceleration", 0.8)
        self.declare_parameter("max_deceleration", 1.0)
        self.declare_parameter("terminal_deceleration", 1.0)
        self.declare_parameter("max_lateral_acceleration", 0.25)
        self.declare_parameter("max_tracking_urgency", 0.0)
        self.declare_parameter("smac_hybrid_cruise_speed", 0.8)
        self.declare_parameter("smac_hybrid_max_acceleration", 1.20)
        self.declare_parameter("smac_hybrid_terminal_deceleration", 0.50)
        self.declare_parameter("smac_hybrid_max_lateral_acceleration", 0.30)
        # Optional experiment knob.  Keep the safe default disabled: delaying
        # absolute waypoint deadlines can make the policy chase stale targets.
        self.declare_parameter("smac_hybrid_max_tracking_urgency", 0.0)
        self.declare_parameter("terminal_response_time", 0.20)
        self.declare_parameter("terminal_braking_margin", 0.10)
        self.declare_parameter("emergency_speed_upper_bound", 1.20)
        self.declare_parameter("tracking_pacing_max_delay", 0.75)
        self.declare_parameter("integration_step", 0.02)
        self.declare_parameter("curvature_window", 0.15)
        self.declare_parameter("replan_commit_time", 0.5)
        self.declare_parameter("replan_join_distance", 0.35)
        self.declare_parameter("replan_max_waypoint_shift", 0.30)
        self.declare_parameter("replan_max_heading_shift", 0.35)
        self.declare_parameter("transient_failure_timeout", 0.3)
        self.declare_parameter("collision_cost_threshold", 253)
        self.declare_parameter("unknown_is_collision", True)
        self.declare_parameter("hold_goal", True)
        self.declare_parameter("max_yaw_rate", 0.80)

        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.waypoints_topic = str(self.get_parameter("waypoints_topic").value)
        self.remain_time_topic = str(self.get_parameter("remain_time_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.raw_path_topic = str(self.get_parameter("raw_path_topic").value)
        self.sampled_path_topic = str(self.get_parameter("sampled_path_topic").value)
        self.action_name = str(self.get_parameter("compute_path_action").value)
        self.costmap_topic = str(self.get_parameter("costmap_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.planner_profile = str(self.get_parameter("planner_profile").value)
        if self.planner_profile not in STARTUP_PROFILE_CONTRACTS:
            allowed = ", ".join(sorted(STARTUP_PROFILE_CONTRACTS))
            raise ValueError(f"planner_profile must be one of: {allowed}")
        self.profile_contract = STARTUP_PROFILE_CONTRACTS[self.planner_profile]
        self.planner_id = self.profile_contract.planner_id
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.replan_period = float(self.get_parameter("replan_period").value)
        self.replan_deviation = float(self.get_parameter("replan_deviation").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.goal_linear_speed_tolerance = float(
            self.get_parameter("goal_linear_speed_tolerance").value
        )
        self.goal_yaw_tolerance = float(self.get_parameter("goal_yaw_tolerance").value)
        self.goal_angular_speed_tolerance = float(
            self.get_parameter("goal_angular_speed_tolerance").value
        )
        self.goal_hold_time = float(self.get_parameter("goal_hold_time").value)
        self.goal_capture_release_tolerance = float(
            self.get_parameter("goal_capture_release_tolerance").value
        )
        self.handover_hold_time = float(
            self.get_parameter("handover_hold_time").value
        )
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.odom_speed_filter_window = int(
            self.get_parameter("odom_speed_filter_window").value
        )
        self.costmap_timeout = float(self.get_parameter("costmap_timeout").value)
        self.num_waypoints = int(self.get_parameter("num_waypoints").value)
        self.trajectory_limits = self._load_trajectory_limits()
        urgency_prefix = (
            "smac_hybrid_"
            if self.planner_profile == SMAC_HYBRID_XY_FORWARD
            else ""
        )
        self.max_tracking_urgency = float(
            self.get_parameter(f"{urgency_prefix}max_tracking_urgency").value
        )
        self.terminal_response_time = float(
            self.get_parameter("terminal_response_time").value
        )
        self.terminal_braking_margin = float(
            self.get_parameter("terminal_braking_margin").value
        )
        self.emergency_speed_upper_bound = float(
            self.get_parameter("emergency_speed_upper_bound").value
        )
        self.tracking_pacing_max_delay = float(
            self.get_parameter("tracking_pacing_max_delay").value
        )
        self.transient_failure_timeout = float(
            self.get_parameter("transient_failure_timeout").value
        )
        self.collision_threshold = int(self.get_parameter("collision_cost_threshold").value)
        self.unknown_is_collision = bool(self.get_parameter("unknown_is_collision").value)
        self.hold_goal = bool(self.get_parameter("hold_goal").value)
        self._validate_parameters()
        self.add_on_set_parameters_callback(self._reject_runtime_contract_change)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, ComputePathToPose, self.action_name)

        self.waypoints_publisher = self.create_publisher(
            Float32MultiArray, self.waypoints_topic, 10
        )
        self.remain_time_publisher = self.create_publisher(
            Float32MultiArray, self.remain_time_topic, 10
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, latched_qos
        )
        self.raw_path_publisher = self.create_publisher(
            Path, self.raw_path_topic, latched_qos
        )
        self.sampled_path_publisher = self.create_publisher(
            Path, self.sampled_path_topic, 10
        )
        costmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Costmap,
            self.costmap_topic,
            self._on_costmap,
            costmap_qos,
        )

        self.goal: Optional[PoseStamped] = None
        self.goal_yaw = 0.0
        self.goal_reached = False
        self.path_message: Optional[Path] = None
        self.path: Optional[PathModel] = None
        self.costmap: Optional[CostmapView] = None
        self.last_costmap_receive_ns = 0
        self.trajectory = PersistentTrajectory(self.trajectory_limits)
        self.standing_clock = RollingDeadlineClock(
            self.trajectory_limits.waypoint_interval,
            self.num_waypoints,
        )
        self.pending_path: Optional[ParsedPath] = None
        self.reference_matches_goal = False
        self.reference_goal_pose: Optional[Pose2D] = None
        self.linear_speed: Optional[float] = None
        self.angular_speed: Optional[float] = None
        self._linear_speed_samples: deque[float] = deque(
            maxlen=self.odom_speed_filter_window
        )
        self._angular_speed_samples: deque[float] = deque(
            maxlen=self.odom_speed_filter_window
        )
        self.last_linear_odom_ns = 0
        self.last_angular_odom_ns = 0
        self.last_odom_pose: Optional[Pose2D] = None
        self.last_odom_pose_ns = 0
        self.last_tracking_frame = ""
        self.last_tracking_pose: Optional[Pose2D] = None
        self.last_tracking_odom_pose: Optional[Pose2D] = None
        self.last_tracking_tf_ns = 0
        self.goal_settle_tracker = GoalSettleTracker()
        self.yaw_goal_settle_tracker = YawGoalSettleTracker()
        self.handover_settle_tracker = YawGoalSettleTracker()
        self.plan_pending = False
        self.plan_sequence = 0
        self.active_plan_goal_handle = None
        self.active_plan_sequence: Optional[int] = None
        self.last_plan_request_ns = 0
        self.last_status = ""
        self.aligning_yaw = False
        self.terminal_approach_active = False
        self.terminal_emergency_braking_active = False
        self.terminal_closest_goal_distance: Optional[float] = None
        self.xy_goal_capture_active = False
        self.plan_request_reason = PLAN_INITIAL
        self.alignment_goal_point: Optional[Point2] = None
        self.alignment_goal_yaw: Optional[float] = None
        self.alignment_frame = self.global_frame
        self.terminal_candidate_queue: list[PoseStamped] = []
        self.terminal_successes: list[ParsedPath] = []
        self.terminal_failures: list[str] = []

        self.create_timer(1.0 / self.publish_rate, self._on_timer)
        self._set_status("WAITING_GOAL")
        self.get_logger().info(
            f"Nav2 waypoint bridge ready: profile={self.planner_profile}, "
            f"planner_id={self.planner_id}, route={self.goal_topic} -> "
            f"{self.action_name} -> {self.waypoints_topic}; "
            f"trajectory=(interval={self.trajectory_limits.waypoint_interval:.2f}s, "
            f"speed={self.trajectory_limits.cruise_speed:.2f}m/s), collision_cost>="
            f"{self.collision_threshold}; stop=(position<={self.goal_tolerance:.2f} m, "
            f"planar_speed<={self.goal_linear_speed_tolerance:.2f} m/s for "
            f"{self.goal_hold_time:.2f} s), odom={self.odom_topic}"
        )

    def _load_trajectory_limits(self) -> TrajectoryLimits:
        speed_prefix = (
            "smac_hybrid_"
            if self.planner_profile == SMAC_HYBRID_XY_FORWARD
            else ""
        )
        return TrajectoryLimits(
            waypoint_interval=float(self.get_parameter("waypoint_interval").value),
            num_waypoints=self.num_waypoints,
            cruise_speed=float(
                self.get_parameter(f"{speed_prefix}cruise_speed").value
            ),
            max_acceleration=float(
                self.get_parameter(f"{speed_prefix}max_acceleration").value
            ),
            max_deceleration=float(self.get_parameter("max_deceleration").value),
            terminal_deceleration=float(
                self.get_parameter(f"{speed_prefix}terminal_deceleration").value
            ),
            max_lateral_acceleration=float(
                self.get_parameter(
                    f"{speed_prefix}max_lateral_acceleration"
                ).value
            ),
            max_yaw_rate=float(self.get_parameter("max_yaw_rate").value),
            integration_step=float(self.get_parameter("integration_step").value),
            curvature_window=float(self.get_parameter("curvature_window").value),
            replan_commit_time=float(self.get_parameter("replan_commit_time").value),
            replan_join_distance=float(self.get_parameter("replan_join_distance").value),
            replan_max_waypoint_shift=float(
                self.get_parameter("replan_max_waypoint_shift").value
            ),
            replan_max_heading_shift=float(
                self.get_parameter("replan_max_heading_shift").value
            ),
        )

    def _validate_parameters(self) -> None:
        if not math.isfinite(self.publish_rate) or self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be finite and positive")
        if not math.isfinite(self.replan_period) or self.replan_period <= 0.0:
            raise ValueError("replan_period must be finite and positive")
        if not math.isfinite(self.replan_deviation) or self.replan_deviation <= 0.0:
            raise ValueError("replan_deviation must be finite and positive")
        if not math.isfinite(self.goal_tolerance) or self.goal_tolerance <= 0.0:
            raise ValueError("goal_tolerance must be finite and positive")
        if (
            not math.isfinite(self.goal_linear_speed_tolerance)
            or self.goal_linear_speed_tolerance < 0.0
        ):
            raise ValueError(
                "goal_linear_speed_tolerance must be finite and non-negative"
            )
        if not math.isfinite(self.goal_yaw_tolerance) or self.goal_yaw_tolerance < 0.0:
            raise ValueError("goal_yaw_tolerance must be finite and non-negative")
        if (
            not math.isfinite(self.goal_angular_speed_tolerance)
            or self.goal_angular_speed_tolerance < 0.0
        ):
            raise ValueError(
                "goal_angular_speed_tolerance must be finite and non-negative"
            )
        if not math.isfinite(self.goal_hold_time) or self.goal_hold_time <= 0.0:
            raise ValueError("goal_hold_time must be finite and positive")
        if (
            not math.isfinite(self.goal_capture_release_tolerance)
            or self.goal_capture_release_tolerance < self.goal_tolerance
        ):
            raise ValueError(
                "goal_capture_release_tolerance must be finite and at least "
                "goal_tolerance"
            )
        if (
            not math.isfinite(self.handover_hold_time)
            or self.handover_hold_time <= 0.0
        ):
            raise ValueError("handover_hold_time must be finite and positive")
        if not math.isfinite(self.odom_timeout) or self.odom_timeout <= 0.0:
            raise ValueError("odom_timeout must be finite and positive")
        if self.odom_speed_filter_window < 2:
            raise ValueError("odom_speed_filter_window must retain at least two samples")
        if not math.isfinite(self.costmap_timeout) or self.costmap_timeout <= 0.0:
            raise ValueError("costmap_timeout must be finite and positive")
        if (
            not math.isfinite(self.max_tracking_urgency)
            or self.max_tracking_urgency < 0.0
        ):
            raise ValueError("max_tracking_urgency must be finite and non-negative")
        if 0.0 < self.max_tracking_urgency < self.trajectory_limits.cruise_speed:
            raise ValueError(
                "max_tracking_urgency must be zero (disabled) or at least cruise_speed"
            )
        if (
            not math.isfinite(self.emergency_speed_upper_bound)
            or self.emergency_speed_upper_bound < self.trajectory_limits.cruise_speed
        ):
            raise ValueError(
                "emergency_speed_upper_bound must be finite and at least cruise_speed"
            )
        for name, value in (
            ("terminal_response_time", self.terminal_response_time),
            ("terminal_braking_margin", self.terminal_braking_margin),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if (
            not math.isfinite(self.tracking_pacing_max_delay)
            or self.tracking_pacing_max_delay <= 0.0
        ):
            raise ValueError(
                "tracking_pacing_max_delay must be finite and positive"
            )
        if self.num_waypoints != POLICY_NUM_WAYPOINTS:
            raise ValueError(
                f"num_waypoints must remain {POLICY_NUM_WAYPOINTS} for the policy ABI"
            )
        if not math.isclose(
            self.trajectory_limits.waypoint_interval,
            POLICY_WAYPOINT_INTERVAL,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError(
                "waypoint_interval must remain "
                f"{POLICY_WAYPOINT_INTERVAL:.1f} s for the policy ABI"
            )
        available_braking_time = (
            self.num_waypoints * self.trajectory_limits.waypoint_interval
            - self.trajectory_limits.replan_commit_time
        )
        stopping_time = (
            self.trajectory_limits.cruise_speed
            / self.trajectory_limits.max_deceleration
        )
        if stopping_time > available_braking_time + 1e-9:
            raise ValueError(
                "cruise_speed must support a complete bounded stop inside the "
                "uncommitted waypoint horizon"
            )
        emergency_stopping_time = (
            self.emergency_speed_upper_bound
            / self.trajectory_limits.max_deceleration
        )
        if emergency_stopping_time > available_braking_time + 1e-9:
            raise ValueError(
                "emergency_speed_upper_bound must support a complete bounded "
                "stop inside the rebased waypoint horizon"
            )
        terminal_stopping_time = (
            self.trajectory_limits.cruise_speed
            / self.trajectory_limits.terminal_deceleration
        )
        terminal_margin_time = (
            self.terminal_braking_margin
            / self.trajectory_limits.cruise_speed
        )
        if (
            terminal_stopping_time
            + self.terminal_response_time
            + terminal_margin_time
            > available_braking_time + 1e-9
        ):
            raise ValueError(
                "terminal_deceleration must support a complete cruise-speed "
                "stop with response and margin reserve inside the uncommitted "
                "waypoint horizon"
            )
        if (
            not math.isfinite(self.transient_failure_timeout)
            or self.transient_failure_timeout <= 0.0
        ):
            raise ValueError("transient_failure_timeout must be finite and positive")
        if not 1 <= self.collision_threshold <= 254:
            raise ValueError("collision_cost_threshold must be in [1, 254]")

    def _reject_runtime_contract_change(self, parameters) -> SetParametersResult:
        immutable_values = {
            "planner_profile": self.planner_profile,
            "num_waypoints": self.num_waypoints,
            "waypoint_interval": self.trajectory_limits.waypoint_interval,
        }
        for parameter in parameters:
            if parameter.name not in immutable_values:
                continue
            expected = immutable_values[parameter.name]
            if parameter.value != expected:
                return SetParametersResult(
                    successful=False,
                    reason=f"{parameter.name} is immutable after startup",
                )
        return SetParametersResult(successful=True)

    def _set_status(self, status: str) -> None:
        if not status.startswith(f"{self.planner_profile}:"):
            status = f"{self.planner_profile}:{status}"
        if status == self.last_status:
            return
        self.last_status = status
        message = String()
        message.data = status
        self.status_publisher.publish(message)
        self.get_logger().info(f"state={status}")

    def _on_costmap(self, message: Costmap) -> None:
        previous = self.costmap
        previous_receive_ns = getattr(self, "last_costmap_receive_ns", 0)
        try:
            candidate = CostmapView.from_message(
                message,
                self.collision_threshold,
                self.unknown_is_collision,
            )
            if not candidate.frame_id:
                raise ValueError("costmap frame_id is empty")
            if self.trajectory.active and self.path_message is not None:
                path_frame = self.path_message.header.frame_id or self.global_frame
                if candidate.frame_id != path_frame:
                    raise ValueError(
                        "active path/costmap frame mismatch: "
                        f"path={path_frame}, costmap={candidate.frame_id}"
                    )
            self.costmap = candidate
            self.last_costmap_receive_ns = self.get_clock().now().nanoseconds
        except ValueError as error:
            self.costmap = previous
            self.last_costmap_receive_ns = previous_receive_ns
            self._set_status(f"COSTMAP_ERROR: {error}")

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        now_ns = self.get_clock().now().nanoseconds
        if not all(math.isfinite(value) for value in (position.x, position.y, yaw)):
            self.linear_speed = None
            self.angular_speed = None
            self._reset_speed_filter()
            self.last_linear_odom_ns = 0
            self.last_angular_odom_ns = 0
            self.last_odom_pose = None
            self.last_odom_pose_ns = 0
            return

        pose = Pose2D(float(position.x), float(position.y), yaw)
        if self.last_odom_pose is not None and now_ns > self.last_odom_pose_ns:
            dt = (now_ns - self.last_odom_pose_ns) * 1e-9
            if dt <= 2.0 * self.odom_timeout:
                raw_linear_speed = (
                    distance(self.last_odom_pose.point, pose.point) / dt
                )
                raw_angular_speed = (
                    wrap_to_pi(yaw - self.last_odom_pose.yaw) / dt
                )
                sample_count = getattr(self, "odom_speed_filter_window", 1)
                linear_samples = getattr(self, "_linear_speed_samples", None)
                angular_samples = getattr(self, "_angular_speed_samples", None)
                if linear_samples is None or linear_samples.maxlen != sample_count:
                    linear_samples = deque(maxlen=sample_count)
                    angular_samples = deque(maxlen=sample_count)
                    self._linear_speed_samples = linear_samples
                    self._angular_speed_samples = angular_samples
                assert angular_samples is not None
                linear_samples.append(raw_linear_speed)
                angular_samples.append(abs(raw_angular_speed))
                self.linear_speed = self._conservative_speed_estimate(
                    linear_samples,
                    self.linear_speed,
                )
                self.angular_speed = self._conservative_speed_estimate(
                    angular_samples,
                    self.angular_speed,
                )
                self.last_linear_odom_ns = now_ns
                self.last_angular_odom_ns = now_ns
            else:
                self.linear_speed = None
                self.angular_speed = None
                self._reset_speed_filter()
                self.last_linear_odom_ns = 0
                self.last_angular_odom_ns = 0
        self.last_odom_pose = pose
        self.last_odom_pose_ns = now_ns

    @staticmethod
    def _conservative_speed_estimate(
        samples: Sequence[float],
        previous: Optional[float] = None,
    ) -> float:
        """Require two consecutive intervals before raising or lowering speed."""
        ordered = [abs(float(sample)) for sample in samples]
        if not ordered:
            raise ValueError("speed estimate requires at least one sample")
        if len(ordered) == 1 or previous is None or not math.isfinite(previous):
            return ordered[0]
        baseline = abs(float(previous))
        latest_pair = ordered[-2:]
        if all(sample > baseline for sample in latest_pair):
            return min(latest_pair)
        if all(sample < baseline for sample in latest_pair):
            return max(latest_pair)
        return baseline

    def _reset_speed_filter(self) -> None:
        for name in ("_linear_speed_samples", "_angular_speed_samples"):
            samples = getattr(self, name, None)
            if samples is not None:
                samples.clear()
        # Clearing the history must also invalidate the scalar estimate and
        # its freshness timestamps; otherwise a new goal or a terminal miss
        # can accidentally consume speed from the previous execution.
        self.linear_speed = None
        self.angular_speed = None
        self.last_linear_odom_ns = 0
        self.last_angular_odom_ns = 0

    def _reset_goal_execution(self, *, clear_goal: bool) -> None:
        """Invalidate every asynchronous artifact from the current command."""
        if clear_goal:
            self.goal = None
            self.goal_yaw = 0.0
        self.goal_reached = False
        self.goal_settle_tracker.reset()
        self.yaw_goal_settle_tracker.reset()
        self.handover_settle_tracker.reset()
        self._reset_speed_filter()
        self._clear_path()
        self.trajectory.clear()
        self.standing_clock.reset()
        self.pending_path = None
        self.reference_matches_goal = False
        self.reference_goal_pose = None
        self.aligning_yaw = False
        self.terminal_approach_active = False
        self.terminal_emergency_braking_active = False
        self.terminal_closest_goal_distance = None
        self.xy_goal_capture_active = False
        self.alignment_goal_point = None
        self.alignment_goal_yaw = None
        self.alignment_frame = self.global_frame
        self._reset_terminal_planning_batch()
        self.plan_sequence += 1
        self.plan_pending = False
        self.plan_request_reason = PLAN_INITIAL
        self.last_plan_request_ns = 0
        self._cancel_active_plan()

    def _on_goal(self, message: PoseStamped) -> None:
        values = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            self._reset_goal_execution(clear_goal=True)
            self._set_status("INVALID_GOAL")
            self._publish_zero_waypoints()
            return
        if not message.header.frame_id:
            message.header.frame_id = self.global_frame

        now = self.get_clock().now().nanoseconds * 1e-9
        was_moving = self.trajectory.active
        self.plan_sequence += 1
        self.plan_pending = False
        self.last_plan_request_ns = 0
        self._cancel_active_plan()
        self._reset_terminal_planning_batch()
        self.goal = message
        self.goal_yaw = yaw_from_quaternion(message.pose.orientation)
        self.goal_reached = False
        self.goal_settle_tracker.reset()
        self.yaw_goal_settle_tracker.reset()
        self.handover_settle_tracker.reset()
        self._reset_speed_filter()
        self.pending_path = None
        self.reference_matches_goal = False
        self.reference_goal_pose = None
        self.aligning_yaw = False
        self.terminal_approach_active = False
        self.terminal_emergency_braking_active = False
        self.terminal_closest_goal_distance = None
        self.xy_goal_capture_active = False
        self.alignment_goal_point = None
        self.alignment_goal_yaw = None
        self.alignment_frame = self.global_frame
        if was_moving:
            self.trajectory.begin_braking(now=now)
            self._set_status("NEW_GOAL_BRAKING")
        else:
            self._clear_path()
            self.trajectory.clear()
            self.standing_clock.reset()
            self._set_status("GOAL_RECEIVED")
        self._request_plan(PLAN_NEW_GOAL)

    def _cancel_active_plan(self) -> None:
        goal_handle = self.active_plan_goal_handle
        self.active_plan_goal_handle = None
        self.active_plan_sequence = None
        if goal_handle is None:
            return
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception as error:
            self.get_logger().warn(f"failed to cancel previous plan request: {error}")
            return
        cancel_future.add_done_callback(self._on_cancel_complete)

    def _on_cancel_complete(self, future) -> None:
        try:
            future.result()
        except Exception as error:
            self.get_logger().warn(f"plan cancel request failed: {error}")

    def _request_plan(self, reason: Optional[str] = None) -> None:
        if reason is not None:
            self.plan_request_reason = reason
        if (
            self.goal is None
            or self.goal_reached
            or self.plan_pending
            or getattr(self, "xy_goal_capture_active", False)
            or getattr(self, "terminal_approach_active", False)
        ):
            return
        if not self.action_client.server_is_ready():
            self.last_plan_request_ns = self.get_clock().now().nanoseconds
            self._set_status("WAITING_FOR_NAV2")
            return
        if self._planning_start_is_blocked():
            return

        if self.planner_profile == SMAC_TERMINAL_YAW:
            if not self.terminal_candidate_queue and not self.terminal_failures:
                if self._prepare_terminal_translation_candidates():
                    return
            if self.terminal_candidate_queue:
                self._send_plan_goal(self.terminal_candidate_queue.pop(0))
                return
            self._set_status(
                "UNTRACKABLE_PATH: terminal yaw translation candidates exhausted"
            )
            return

        self._send_plan_goal(self.goal)

    def _send_plan_goal(self, goal: PoseStamped) -> None:
        self.plan_pending = True
        self.plan_sequence += 1
        sequence = self.plan_sequence
        self._cancel_active_plan()
        request = ComputePathToPose.Goal()
        # RViz stamps a goal once when it is clicked. Reusing that stamp for a
        # later replan eventually asks TF for data older than its cache. The
        # coordinates remain in the same frame, but every action request must
        # represent the current planning transaction.
        request.goal = self._fresh_plan_goal(goal)
        request.planner_id = self.planner_id
        request.use_start = False
        self.last_plan_request_ns = self.get_clock().now().nanoseconds
        self._set_status("PLANNING")
        try:
            future = self.action_client.send_goal_async(request)
        except Exception as error:
            self.plan_pending = False
            if self._handle_terminal_candidate_failure(f"send error {error}"):
                return
            self._handle_plan_failure(f"PLAN_SEND_ERROR: {error}")
            return
        future.add_done_callback(partial(self._on_plan_goal_response, sequence=sequence))

    def _fresh_plan_goal(self, goal: PoseStamped) -> PoseStamped:
        candidate = PoseStamped()
        candidate.header.frame_id = goal.header.frame_id or self.global_frame
        candidate.header.stamp = self.get_clock().now().to_msg()
        candidate.pose.position.x = goal.pose.position.x
        candidate.pose.position.y = goal.pose.position.y
        candidate.pose.position.z = goal.pose.position.z
        candidate.pose.orientation.x = goal.pose.orientation.x
        candidate.pose.orientation.y = goal.pose.orientation.y
        candidate.pose.orientation.z = goal.pose.orientation.z
        candidate.pose.orientation.w = goal.pose.orientation.w
        return candidate

    def _planning_start_is_blocked(self) -> bool:
        """Fail closed before asking Nav2 to plan without a safe current start."""
        now_ns = self.get_clock().now().nanoseconds
        self.last_plan_request_ns = now_ns
        if not self._costmap_is_fresh(now_ns):
            if self.trajectory.active:
                self.trajectory.begin_braking(now=now_ns * 1e-9)
                self._set_status("BRAKING_WAITING_FOR_FRESH_COSTMAP")
            else:
                self._set_status("WAITING_FOR_FRESH_COSTMAP")
                self._publish_zero_waypoints(now_ns * 1e-9)
            return True
        assert self.costmap is not None
        costmap_frame = self.costmap.frame_id or self.global_frame
        try:
            robot_point, _ = self._base_pose_in(costmap_frame)
        except TransformException as error:
            self._set_status(f"TF_ERROR: {error}")
            if not self.trajectory.active:
                self._publish_zero_waypoints(now_ns * 1e-9)
            return True
        if not self.costmap.point_is_collision(robot_point):
            return False
        self._enter_fault("EMERGENCY_STOP_START_IN_COLLISION", now_ns * 1e-9)
        return True

    def _prepare_terminal_translation_candidates(self) -> bool:
        if self.goal is None:
            return False
        goal_frame = self.goal.header.frame_id or self.global_frame
        try:
            robot_point, robot_yaw = self._base_pose_in(goal_frame)
        except TransformException as error:
            self._set_status(f"TF_ERROR: {error}")
            return True
        goal_point = (self.goal.pose.position.x, self.goal.pose.position.y)
        if distance(robot_point, goal_point) <= self.goal_tolerance:
            now_ns = self.get_clock().now().nanoseconds
            speed = self._fresh_linear_speed(now_ns)
            if self.trajectory.active and (
                speed is None or speed > self.goal_linear_speed_tolerance
            ):
                self.trajectory.begin_braking(now=now_ns * 1e-9)
                self._set_status("GOAL_BRAKING")
            else:
                self._start_terminal_alignment(
                    robot_point,
                    robot_yaw,
                    now_ns * 1e-9,
                    goal_pose=Pose2D(goal_point[0], goal_point[1], self.goal_yaw),
                    frame_id=goal_frame,
                )
            return True

        candidates = approach_yaw_candidates(robot_point, robot_yaw, goal_point)
        self.terminal_candidate_queue = [
            self._copy_goal_with_yaw(self.goal, candidate_yaw)
            for candidate_yaw in candidates
        ]
        self._set_status(f"PLANNING_TERMINAL_TRANSLATION candidates={len(candidates)}")
        return False

    def _copy_goal_with_yaw(self, goal: PoseStamped, yaw: float) -> PoseStamped:
        candidate = PoseStamped()
        candidate.header.frame_id = goal.header.frame_id or self.global_frame
        candidate.header.stamp = goal.header.stamp
        candidate.pose.position.x = goal.pose.position.x
        candidate.pose.position.y = goal.pose.position.y
        candidate.pose.position.z = goal.pose.position.z
        candidate.pose.orientation = quaternion_from_yaw(yaw)
        return candidate

    def _reset_terminal_planning_batch(self) -> None:
        self.terminal_candidate_queue = []
        self.terminal_successes = []
        self.terminal_failures = []

    def _handle_terminal_candidate_failure(self, reason: str) -> bool:
        if self.planner_profile != SMAC_TERMINAL_YAW:
            return False
        self.terminal_failures.append(reason)
        if self.terminal_candidate_queue:
            self._request_plan()
            return True
        if self._select_terminal_translation_path():
            return True
        failures = "; ".join(self.terminal_failures) or "no feasible candidates"
        self._handle_plan_failure(f"UNTRACKABLE_PATH: {failures}")
        return True

    def _handle_plan_failure(self, status: str) -> None:
        """Keep an active reference; only an idle bridge falls back to standing."""
        if not self.trajectory.active:
            self._clear_path()
            self.pending_path = None
        self._set_status(status)

    def _on_plan_goal_response(self, future, sequence: int) -> None:
        if sequence != self.plan_sequence:
            try:
                goal_handle = future.result()
            except Exception as error:
                self.get_logger().debug(
                    f"stale plan request completed with an error: {error}"
                )
                return
            if goal_handle.accepted:
                try:
                    goal_handle.cancel_goal_async()
                except Exception as error:
                    self.get_logger().warn(f"failed to cancel stale plan request: {error}")
            return
        try:
            goal_handle = future.result()
        except Exception as error:  # rclpy transports several exception types.
            self.plan_pending = False
            if self._handle_terminal_candidate_failure(f"request error {error}"):
                return
            self._handle_plan_failure(f"PLAN_REQUEST_ERROR: {error}")
            return
        if not goal_handle.accepted:
            self.plan_pending = False
            if self._handle_terminal_candidate_failure("goal rejected"):
                return
            self._handle_plan_failure("PLAN_REJECTED")
            return
        self.active_plan_goal_handle = goal_handle
        self.active_plan_sequence = sequence
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(self._on_plan_result, sequence=sequence))

    def _on_plan_result(self, future, sequence: int) -> None:
        if sequence != self.plan_sequence:
            return
        self.plan_pending = False
        if self.active_plan_sequence == sequence:
            self.active_plan_goal_handle = None
            self.active_plan_sequence = None
        try:
            wrapped_result = future.result()
        except Exception as error:
            if self._handle_terminal_candidate_failure(f"result error {error}"):
                return
            self._handle_plan_failure(f"PLAN_RESULT_ERROR: {error}")
            return
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            if self._handle_terminal_candidate_failure(
                f"status_{wrapped_result.status}"
            ):
                return
            self._handle_plan_failure(f"NO_PATH_STATUS_{wrapped_result.status}")
            return

        path_message = wrapped_result.result.path
        try:
            parsed = self._parse_planner_path(path_message)
        except ValueError as error:
            if self._handle_terminal_candidate_failure(str(error)):
                return
            self._handle_plan_failure(f"INVALID_PATH: {error}")
            return

        collision_reason = None
        if self._costmap_is_fresh(self.get_clock().now().nanoseconds):
            collision_reason = self._raw_path_collision_reason(parsed)
        if collision_reason is not None:
            if self._handle_terminal_candidate_failure(collision_reason):
                return
            self._handle_plan_failure(f"UNTRACKABLE_PATH: raw {collision_reason}")
            return

        if self.planner_profile == SMAC_TERMINAL_YAW:
            self.terminal_successes.append(parsed)
            if self.terminal_candidate_queue:
                self._request_plan()
                return
            if not self._select_terminal_translation_path():
                failures = "; ".join(self.terminal_failures) or "no feasible candidates"
                self._handle_plan_failure(f"UNTRACKABLE_PATH: {failures}")
                return
            return

        self._accept_path(parsed)

    def _clear_path(self, *, publish_empty: bool = True) -> None:
        self.path = None
        self.path_message = None
        if not publish_empty:
            return
        stamp = self.get_clock().now().to_msg()
        for publisher in (self.raw_path_publisher, self.sampled_path_publisher):
            message = Path()
            message.header.frame_id = self.global_frame
            message.header.stamp = stamp
            publisher.publish(message)

    def _parse_planner_path(self, path_message: Path) -> ParsedPath:
        if not path_message.header.frame_id:
            path_message.header.frame_id = self.global_frame
        poses = [
            Pose2D(
                pose.pose.position.x,
                pose.pose.position.y,
                yaw_from_quaternion(pose.pose.orientation),
            )
            for pose in path_message.poses
        ]
        if any(
            not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw))
            for pose in poses
        ):
            raise ValueError("path contains a non-finite pose")
        # The frozen policy consumes movement-direction headings derived from
        # XY targets.  Planner pose orientations are therefore metadata, not
        # a second yaw trajectory.  Normalize every translating planner path
        # to the same XY polyline contract; retain a CanonicalPath only for an
        # explicit same-position rotation request.
        try:
            model: PathModel = Polyline((pose.x, pose.y) for pose in poses)
        except ValueError:
            if not self.profile_contract.terminal_yaw_required:
                raise
            model = CanonicalPath(poses)
        return ParsedPath(path_message, model, poses)

    def _raw_path_collision_reason(self, parsed: ParsedPath) -> Optional[str]:
        if self.costmap is None:
            return None
        path_frame = parsed.message.header.frame_id or self.global_frame
        if self.costmap.frame_id and self.costmap.frame_id != path_frame:
            return f"frame mismatch path={path_frame}, costmap={self.costmap.frame_id}"
        return path_centerline_collision_reason(parsed.poses, self.costmap)

    def _accept_path(self, parsed: ParsedPath) -> None:
        now_ns = self.get_clock().now().nanoseconds
        now = now_ns * 1e-9
        request_reason = getattr(self, "plan_request_reason", PLAN_INITIAL)
        self.pending_path = parsed
        if not self._costmap_is_fresh(now_ns):
            if self.trajectory.active:
                self.trajectory.begin_braking(now=now)
                self._set_status("REPLAN_WAITING_FOR_FRESH_COSTMAP")
            else:
                self._set_status("PATH_WAITING_FOR_FRESH_COSTMAP")
            return

        collision_reason = self._raw_path_collision_reason(parsed)
        if collision_reason is not None:
            self.pending_path = None
            self._handle_plan_failure(f"UNTRACKABLE_PATH: raw {collision_reason}")
            return

        path_frame = parsed.message.header.frame_id or self.global_frame
        try:
            goal_pose = self._goal_pose_in(path_frame)
        except TransformException as error:
            if self.trajectory.active:
                self.trajectory.begin_braking(now=now)
            self._set_status(f"PATH_WAITING_FOR_GOAL_TF: {error}")
            return
        endpoint_error = distance(parsed.poses[-1].point, goal_pose.point)
        if endpoint_error > self.goal_tolerance + 1e-6:
            self.pending_path = None
            self._handle_plan_failure(
                "UNTRACKABLE_PATH: planner endpoint is "
                f"{endpoint_error:.3f} m from the requested goal"
            )
            return

        smac_xy = self._smac_xy_bridge_enabled()
        if self.trajectory.active:
            if smac_xy and self.trajectory.mode == PersistentTrajectory.BRAKING:
                self._set_status("REPLAN_WAITING_FOR_ROBOT_STOP")
                return
            if self.trajectory.adopt_replan(
                parsed.model,
                now=now,
                strict_continuity=smac_xy,
            ):
                self.path_message = parsed.message
                self.path = parsed.model
                self.raw_path_publisher.publish(parsed.message)
                self.pending_path = None
                self.reference_matches_goal = True
                self.reference_goal_pose = goal_pose
                self.aligning_yaw = False
                self.terminal_approach_active = False
                self.terminal_emergency_braking_active = False
                self.terminal_closest_goal_distance = None
                self.alignment_goal_point = None
                self.alignment_goal_yaw = None
                self._set_status("PATH_REPLAN_BLENDED")
            else:
                if smac_xy and request_reason == PLAN_PERIODIC:
                    # A periodic refresh is optional.  If it cannot replace
                    # the policy horizon continuously, keep the still-safe
                    # active reference and retry on the next period instead
                    # of injecting a stop/start cycle.
                    self.pending_path = None
                    self._set_status("PATH_REPLAN_REJECTED_CONTINUITY")
                    return
                self.trajectory.begin_braking(now=now)
                self.handover_settle_tracker.reset()
                self._set_status("REPLAN_WAITING_FOR_BRAKE")
            return
        self._activate_pending_path()

    def _activate_pending_path(self) -> bool:
        parsed = self.pending_path
        if parsed is None:
            return False
        now_ns = self.get_clock().now().nanoseconds
        if self.trajectory.active:
            snapshot = self.trajectory.snapshot(now_ns * 1e-9)
            if snapshot.mode != PersistentTrajectory.BRAKING:
                self._set_status("REPLAN_WAITING_FOR_BRAKE")
                return False
            if self._smac_xy_bridge_enabled():
                handover_ready = self._reference_and_robot_stopped(
                    snapshot,
                    now_ns,
                )
            else:
                handover_ready = self._reference_stopped(snapshot)
            if not handover_ready:
                self._set_status("REPLAN_WAITING_FOR_ROBOT_STOP")
                return False
        if not self._costmap_is_fresh(now_ns):
            self._set_status("PATH_WAITING_FOR_FRESH_COSTMAP")
            return False
        assert self.costmap is not None
        collision_reason = self._raw_path_collision_reason(parsed)
        if collision_reason is not None:
            self.pending_path = None
            self._handle_plan_failure(f"UNTRACKABLE_PATH: raw {collision_reason}")
            return False
        path_frame = parsed.message.header.frame_id or self.global_frame
        try:
            robot_point, robot_yaw = self._base_pose_in(path_frame)
            goal_pose = self._goal_pose_in(path_frame)
            endpoint_error = distance(parsed.poses[-1].point, goal_pose.point)
            if endpoint_error > self.goal_tolerance + 1e-6:
                self.pending_path = None
                self._handle_plan_failure(
                    "UNTRACKABLE_PATH: planner endpoint is "
                    f"{endpoint_error:.3f} m from the requested goal"
                )
                return False
            if isinstance(parsed.model, Polyline):
                projection = parsed.model.project(robot_point, 0.0)
                start_progress = projection.arc_length
                join_distance = projection.distance
            else:
                projection = parsed.model.project(
                    robot_point,
                    # Planner pose yaw is not the policy command heading.
                    # Initial progress is selected from XY geometry only.
                    yaw=None,
                    min_progress=0.0,
                    max_progress=parsed.model.total_progress,
                )
                start_progress = projection.progress
                join_distance = projection.distance
        except TransformException as error:
            self._set_status(f"PATH_WAITING_FOR_TF: {error}")
            return False

        if join_distance > self.trajectory_limits.replan_join_distance:
            self.pending_path = None
            self._handle_plan_failure(
                "UNTRACKABLE_PATH: robot is "
                f"{join_distance:.3f} m from the planner path"
            )
            return False
        start_speed = min(
            self._fresh_linear_speed(now_ns) or 0.0,
            self.trajectory_limits.cruise_speed,
        )
        self.trajectory.activate(
            parsed.model,
            now=now_ns * 1e-9,
            start_progress=start_progress,
            start_speed=start_speed,
            start_yaw=robot_yaw,
            start_point=robot_point,
        )
        self.path_message = parsed.message
        self.path = parsed.model
        self.pending_path = None
        self.reference_matches_goal = True
        self.reference_goal_pose = goal_pose
        self.aligning_yaw = False
        self.terminal_approach_active = False
        self.terminal_emergency_braking_active = False
        self.terminal_closest_goal_distance = None
        self.alignment_goal_point = None
        self.alignment_goal_yaw = None
        self.standing_clock.reset()
        self.raw_path_publisher.publish(parsed.message)
        self._set_status("PATH_READY")
        return True

    def _select_terminal_translation_path(self) -> bool:
        if not self.terminal_successes:
            return False

        def translation_length(parsed: ParsedPath) -> float:
            return sum(
                distance(first.point, second.point)
                for first, second in zip(parsed.poses, parsed.poses[1:])
            )

        selected = min(self.terminal_successes, key=translation_length)
        self._reset_terminal_planning_batch()
        self._accept_path(selected)
        return True

    def _base_pose_in(self, frame_id: str) -> tuple[Point2, float]:
        transform = self.tf_buffer.lookup_transform(frame_id, self.base_frame, Time())
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = yaw_from_quaternion(rotation)
        transform_values = (
            translation.x,
            translation.y,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
            yaw,
        )
        if not all(math.isfinite(value) for value in transform_values):
            raise TransformException(
                f"non-finite base transform from {self.base_frame} to {frame_id}"
            )
        return (float(translation.x), float(translation.y)), float(yaw)

    def _goal_pose_in(self, frame_id: str) -> Pose2D:
        """Return the real user goal in the same frame as the active path."""
        if self.goal is None:
            raise TransformException("goal is not available")
        global_frame = getattr(self, "global_frame", "map")
        target_frame = frame_id or global_frame
        source_frame = self.goal.header.frame_id or global_frame
        goal_x = float(self.goal.pose.position.x)
        goal_y = float(self.goal.pose.position.y)
        goal_yaw = float(
            getattr(
                self,
                "goal_yaw",
                yaw_from_quaternion(self.goal.pose.orientation),
            )
        )
        if source_frame == target_frame:
            return Pose2D(goal_x, goal_y, goal_yaw)

        transform = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        transform_yaw = yaw_from_quaternion(rotation)
        transform_values = (
            translation.x,
            translation.y,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
            transform_yaw,
        )
        if not all(math.isfinite(value) for value in transform_values):
            raise TransformException(
                f"non-finite goal transform from {source_frame} to {target_frame}"
            )
        cosine = math.cos(transform_yaw)
        sine = math.sin(transform_yaw)
        return Pose2D(
            translation.x + cosine * goal_x - sine * goal_y,
            translation.y + sine * goal_x + cosine * goal_y,
            wrap_to_pi(transform_yaw + goal_yaw),
        )

    def _costmap_is_fresh(self, now_ns: int) -> bool:
        if self.costmap is None or getattr(self, "last_costmap_receive_ns", 0) <= 0:
            return False
        age = (now_ns - self.last_costmap_receive_ns) * 1e-9
        return 0.0 <= age <= self.costmap_timeout

    def _tracking_pose_in(
        self,
        frame_id: str,
        now_ns: int,
    ) -> tuple[Point2, float, bool]:
        """Use TF normally and bounded odometry extrapolation during a short outage."""
        try:
            point, yaw = self._base_pose_in(frame_id)
        except TransformException:
            fallback = self._extrapolated_tracking_pose(frame_id, now_ns)
            if fallback is None:
                raise
            return fallback.point, fallback.yaw, True

        pose = Pose2D(point[0], point[1], yaw)
        self.last_tracking_frame = frame_id
        self.last_tracking_pose = pose
        self.last_tracking_tf_ns = now_ns
        self.last_tracking_odom_pose = self.last_odom_pose
        return pose.point, pose.yaw, False

    def _extrapolated_tracking_pose(
        self,
        frame_id: str,
        now_ns: int,
    ) -> Optional[Pose2D]:
        if (
            self.last_tracking_frame != frame_id
            or self.last_tracking_pose is None
            or self.last_tracking_odom_pose is None
            or self.last_odom_pose is None
            or self.last_tracking_tf_ns <= 0
            or self.last_odom_pose_ns <= 0
        ):
            return None
        tf_age = (now_ns - self.last_tracking_tf_ns) * 1e-9
        odom_age = (now_ns - self.last_odom_pose_ns) * 1e-9
        if (
            tf_age < 0.0
            or tf_age > self.transient_failure_timeout
            or odom_age < 0.0
            or odom_age > self.odom_timeout
        ):
            return None

        cached_odom = self.last_tracking_odom_pose
        current_odom = self.last_odom_pose
        delta_distance = distance(cached_odom.point, current_odom.point)
        max_delta = (
            self.trajectory_limits.cruise_speed * tf_age
            + 0.5 * self.trajectory_limits.max_acceleration * tf_age**2
            + 0.05
        )
        odom_yaw_delta = wrap_to_pi(current_odom.yaw - cached_odom.yaw)
        max_yaw_delta = self.trajectory_limits.max_yaw_rate * tf_age + 0.10
        if (
            delta_distance > max_delta + 1e-9
            or abs(odom_yaw_delta) > max_yaw_delta + 1e-9
        ):
            return None
        frame_rotation = self.last_tracking_pose.yaw - cached_odom.yaw
        delta_x = current_odom.x - cached_odom.x
        delta_y = current_odom.y - cached_odom.y
        cosine = math.cos(frame_rotation)
        sine = math.sin(frame_rotation)
        return Pose2D(
            self.last_tracking_pose.x + cosine * delta_x - sine * delta_y,
            self.last_tracking_pose.y + sine * delta_x + cosine * delta_y,
            wrap_to_pi(
                self.last_tracking_pose.yaw
                + odom_yaw_delta
            ),
        )

    def _fresh_linear_speed(self, now_ns: int) -> Optional[float]:
        if self.linear_speed is None or self.last_linear_odom_ns <= 0:
            return None
        age = (now_ns - self.last_linear_odom_ns) * 1e-9
        if age < 0.0 or age > self.odom_timeout:
            return None
        return self.linear_speed

    def _fresh_odom_speeds(self, now_ns: int) -> Optional[tuple[float, float]]:
        if (
            self.linear_speed is None
            or self.angular_speed is None
            or self.last_linear_odom_ns <= 0
            or self.last_angular_odom_ns <= 0
        ):
            return None
        linear_age = (now_ns - self.last_linear_odom_ns) * 1e-9
        angular_age = (now_ns - self.last_angular_odom_ns) * 1e-9
        if (
            linear_age < 0.0
            or angular_age < 0.0
            or linear_age > self.odom_timeout
            or angular_age > self.odom_timeout
        ):
            return None
        return self.linear_speed, self.angular_speed

    def _plan_retry_due(self, now_ns: int) -> bool:
        if self.plan_pending:
            return False
        if self.last_plan_request_ns <= 0:
            return True
        return (now_ns - self.last_plan_request_ns) * 1e-9 >= self.replan_period

    def _smac_xy_bridge_enabled(self) -> bool:
        return (
            getattr(self, "planner_profile", "") == SMAC_HYBRID_XY_FORWARD
            and not self.profile_contract.terminal_yaw_required
        )

    @staticmethod
    def _reference_stopped(snapshot: TrajectorySnapshot) -> bool:
        return all(point.speed <= 1e-4 for point in snapshot.points)

    def _reference_and_robot_stopped(
        self,
        snapshot: TrajectorySnapshot,
        now_ns: int,
    ) -> bool:
        """Gate state handovers on both the reference and fresh odometry."""
        if not self._reference_stopped(snapshot):
            self.handover_settle_tracker.reset()
            return False
        return self._robot_motion_settled(now_ns)

    def _robot_motion_settled(self, now_ns: int) -> bool:
        """Require fresh quiet linear and angular odometry for a short hold."""
        speeds = self._fresh_odom_speeds(now_ns)
        if speeds is None:
            self.handover_settle_tracker.reset()
            return False
        linear_speed, angular_speed = speeds
        return self.handover_settle_tracker.update(
            now_ns,
            0.0,
            linear_speed,
            0.0,
            angular_speed,
            0.0,
            self.goal_linear_speed_tolerance,
            0.0,
            self.goal_angular_speed_tolerance,
            self.handover_hold_time,
        )

    def _terminal_braking_distance(self, actual_speed: float) -> float:
        speed = max(0.0, float(actual_speed))
        return (
            self._terminal_minimum_comfort_distance(speed)
            + self.terminal_braking_margin
        )

    def _terminal_minimum_comfort_distance(self, actual_speed: float) -> float:
        speed = max(0.0, float(actual_speed))
        return (
            speed * speed / (2.0 * self.trajectory_limits.terminal_deceleration)
            + speed * self.terminal_response_time
        )

    def _goal_capture_status(self, robot_point: Point2) -> str:
        goal_pose = self.reference_goal_pose
        if goal_pose is None:
            return "GOAL_CAPTURE_BRAKING"
        position_error = distance(robot_point, goal_pose.point)
        if position_error > self.goal_capture_release_tolerance:
            return f"GOAL_CAPTURE_DRIFTED position_error={position_error:.3f}"
        if position_error > self.goal_tolerance:
            return (
                "GOAL_CAPTURE_OUTSIDE_TOLERANCE "
                f"position_error={position_error:.3f}"
            )
        return "GOAL_CAPTURE_BRAKING"

    def _terminal_emergency_speed(self) -> float:
        """Return a fail-closed speed when terminal odometry is stale."""
        candidates = [
            self.trajectory_limits.cruise_speed,
            self.emergency_speed_upper_bound,
        ]
        last_known = getattr(self, "linear_speed", None)
        if last_known is not None and math.isfinite(last_known) and last_known >= 0.0:
            candidates.append(float(last_known))
        return max(float(candidate) for candidate in candidates)

    def _begin_tracking_safety_braking(
        self,
        now_ns: int,
        robot_point: Point2,
    ) -> None:
        """Rebase Smac safety braking at the measured state without changing NavFn."""
        now = now_ns * 1e-9
        if getattr(self, "planner_profile", "") != SMAC_HYBRID_XY_FORWARD:
            self.trajectory.begin_braking(now=now)
            return
        # Rebase once when entering the safety stop. Re-anchoring every 50 Hz
        # cycle would turn the braking horizon back into a robot-following
        # moving target and break the persistent-reference contract.
        if self.trajectory.mode == PersistentTrajectory.BRAKING:
            return
        actual_speed = self._fresh_linear_speed(now_ns)
        if actual_speed is None:
            actual_speed = self._terminal_emergency_speed()
        actual_progress = self._path_progress_at_point(robot_point)
        if actual_progress is None:
            self.trajectory.begin_braking(now=now, actual_speed=actual_speed)
            return
        self.trajectory.begin_braking(
            now=now,
            actual_speed=actual_speed,
            actual_progress=actual_progress,
        )

    def _path_progress_at_point(self, robot_point: Point2) -> Optional[float]:
        path = getattr(self, "path", None)
        if isinstance(path, Polyline):
            projection = path.project(robot_point, 0.0)
            if projection.arc_length >= path.total_length - 1e-9:
                last_x, last_y = path.points[-1]
                previous_x, previous_y = path.points[-2]
                tangent_x = last_x - previous_x
                tangent_y = last_y - previous_y
                segment_length = math.hypot(tangent_x, tangent_y)
                if segment_length > 1e-9:
                    extra = (
                        (robot_point[0] - last_x) * tangent_x
                        + (robot_point[1] - last_y) * tangent_y
                    ) / segment_length
                    if extra > 0.0:
                        return path.total_length + extra
            return projection.arc_length
        if isinstance(path, CanonicalPath):
            return path.project(
                robot_point,
                yaw=None,
                min_progress=0.0,
                max_progress=path.total_progress,
            ).progress
        return None

    def _monitor_terminal_miss(
        self,
        now_ns: int,
        robot_point: Point2,
    ) -> bool:
        """Brake if a terminal approach crosses its capture envelope."""
        if (
            not getattr(self, "terminal_approach_active", False)
            or getattr(self, "terminal_emergency_braking_active", False)
            or getattr(self, "xy_goal_capture_active", False)
            or self.reference_goal_pose is None
        ):
            return False

        goal_distance = distance(robot_point, self.reference_goal_pose.point)
        closest_distance = getattr(
            self,
            "terminal_closest_goal_distance",
            None,
        )
        if closest_distance is None:
            closest_distance = goal_distance
        self.terminal_closest_goal_distance = min(
            closest_distance,
            goal_distance,
        )
        if (
            closest_distance > self.goal_capture_release_tolerance
            or goal_distance <= self.goal_capture_release_tolerance
        ):
            return False

        actual_speed = self._fresh_linear_speed(now_ns)
        speed_is_stale = actual_speed is None
        if speed_is_stale:
            # Never let PersistentTrajectory silently fall back to its
            # reference speed on a stale terminal miss.  Use a conservative
            # bounded estimate and expose the stale-odom condition explicitly.
            actual_speed = self._terminal_emergency_speed()
        self.terminal_emergency_braking_active = True
        self.trajectory.begin_braking(
            now=now_ns * 1e-9,
            actual_speed=actual_speed,
            actual_progress=self._path_progress_at_point(robot_point),
        )
        self.handover_settle_tracker.reset()
        status_prefix = (
            "BRAKING_TERMINAL_MISSED_STALE_ODOM"
            if speed_is_stale
            else "BRAKING_TERMINAL_MISSED"
        )
        self._set_status(
            f"{status_prefix} "
            f"closest_distance={closest_distance:.3f} "
            f"distance={goal_distance:.3f} speed={actual_speed:.3f}"
        )
        return True

    def _recover_stopped_goal_capture(
        self,
        now_ns: int,
        robot_point: Point2,
        *,
        robot_stopped: bool = False,
    ) -> bool:
        """Recover a latched capture that stopped outside its release bound."""
        if (
            not getattr(self, "xy_goal_capture_active", False)
            or self.reference_goal_pose is None
            or distance(robot_point, self.reference_goal_pose.point)
            <= self.goal_capture_release_tolerance
            or (not robot_stopped and not self._robot_motion_settled(now_ns))
        ):
            return False
        self.trajectory.clear()
        self._clear_path(publish_empty=True)
        self.standing_clock.reset()
        self.pending_path = None
        self.reference_matches_goal = False
        self.reference_goal_pose = None
        self.xy_goal_capture_active = False
        self.terminal_approach_active = False
        self.terminal_emergency_braking_active = False
        self.terminal_closest_goal_distance = None
        self.goal_settle_tracker.reset()
        self.yaw_goal_settle_tracker.reset()
        self.handover_settle_tracker.reset()
        self._reset_speed_filter()
        self._publish_zero_waypoints(now_ns * 1e-9)
        self._set_status("GOAL_CAPTURE_DRIFTED_REPLANNING")
        self._request_plan(PLAN_SAFETY)
        return True

    def _recover_stopped_terminal(
        self,
        snapshot: TrajectorySnapshot,
        now_ns: int,
        robot_point: Point2,
    ) -> bool:
        """Replan from rest if terminal braking stopped outside the goal."""
        if (
            not getattr(self, "terminal_approach_active", False)
            or self.reference_goal_pose is None
            or distance(robot_point, self.reference_goal_pose.point)
            <= self.goal_tolerance
            or not self._reference_and_robot_stopped(snapshot, now_ns)
        ):
            return False
        self.trajectory.clear()
        self._clear_path(publish_empty=True)
        self.standing_clock.reset()
        self.pending_path = None
        self.reference_matches_goal = False
        self.reference_goal_pose = None
        self.terminal_approach_active = False
        self.terminal_emergency_braking_active = False
        self.terminal_closest_goal_distance = None
        self.xy_goal_capture_active = False
        self.goal_settle_tracker.reset()
        self.yaw_goal_settle_tracker.reset()
        self.handover_settle_tracker.reset()
        self._reset_speed_filter()
        self._publish_zero_waypoints(now_ns * 1e-9)
        self._set_status("TERMINAL_STOPPED_OUTSIDE_GOAL_REPLANNING")
        self._request_plan(PLAN_SAFETY)
        return True

    def _maybe_start_terminal_approach(
        self,
        now_ns: int,
        robot_point: Point2,
    ) -> bool:
        """Commit to a gradual XY stop before entering the goal tolerance."""
        if (
            getattr(self, "planner_profile", "") != SMAC_HYBRID_XY_FORWARD
            or self.profile_contract.terminal_yaw_required
            or self.terminal_approach_active
            or self.xy_goal_capture_active
            or self.aligning_yaw
            or not self.reference_matches_goal
            or self.reference_goal_pose is None
            or not self.trajectory.active
            or self.trajectory.mode != PersistentTrajectory.TRACKING
        ):
            return False
        actual_speed = self._fresh_linear_speed(now_ns)
        if actual_speed is None:
            return False
        goal_distance = distance(robot_point, self.reference_goal_pose.point)
        braking_distance = self._terminal_braking_distance(actual_speed)
        if goal_distance > braking_distance:
            return False

        comfortable_distance = self._terminal_minimum_comfort_distance(
            actual_speed
        )
        actual_progress = self._path_progress_at_point(robot_point)
        comfortable_horizon = self.trajectory.terminal_approach_feasible(
            now=now_ns * 1e-9,
            actual_speed=actual_speed,
            response_time=self.terminal_response_time,
            actual_progress=actual_progress,
        )

        self.terminal_approach_active = True
        self.terminal_closest_goal_distance = goal_distance
        self.plan_sequence += 1
        self.plan_pending = False
        self.last_plan_request_ns = 0
        self._cancel_active_plan()
        self.pending_path = None
        self.handover_settle_tracker.reset()
        if goal_distance + 1e-6 >= comfortable_distance and comfortable_horizon:
            self.terminal_emergency_braking_active = False
            self.trajectory.begin_terminal_approach(
                now=now_ns * 1e-9,
                actual_speed=actual_speed,
                response_time=self.terminal_response_time,
                actual_progress=actual_progress,
            )
            self._set_status(
                "TERMINAL_APPROACH "
                f"distance={goal_distance:.3f} "
                f"brake_distance={braking_distance:.3f}"
            )
        else:
            # The measured state cannot be represented by the comfortable
            # suffix without running past its horizon (or the goal).  Rebase
            # the strongest configured bounded stop at the measured progress
            # instead of keeping an already unreachable committed anchor.
            self.terminal_emergency_braking_active = True
            self.trajectory.begin_braking(
                now=now_ns * 1e-9,
                actual_speed=actual_speed,
                actual_progress=actual_progress,
            )
            reason = (
                "distance"
                if goal_distance + 1e-6 < comfortable_distance
                else "horizon"
            )
            self._set_status(
                "BRAKING_TERMINAL_OVERSPEED "
                f"reason={reason} speed={actual_speed:.3f} "
                f"distance={goal_distance:.3f} "
                f"comfortable_distance={comfortable_distance:.3f}"
            )
        return True

    def _path_deviation_exceeds_limits(
        self,
        robot_point: Point2,
        _robot_yaw: float,
    ) -> bool:
        if self.path is None:
            return False
        if isinstance(self.path, Polyline):
            projection = self.path.project(robot_point, 0.0)
            lateral_error = projection.distance
        else:
            projection = self.path.project(
                robot_point,
                yaw=None,
                min_progress=0.0,
                max_progress=self.path.total_progress,
            )
            lateral_error = projection.distance
        # The policy heading is a desired movement direction generated from
        # the emitted XY targets, not the robot body's yaw.  Heading-only
        # deviation therefore must not churn the global plan.
        return lateral_error > self.replan_deviation

    def _trajectory_collision_reason(
        self,
        snapshot: TrajectorySnapshot,
        robot_point: Point2,
        robot_yaw: float,
        path_frame: str,
    ) -> Optional[str]:
        if self.costmap is None:
            return None
        if self.costmap.frame_id != path_frame:
            return (
                "frame mismatch path="
                f"{path_frame}, costmap={self.costmap.frame_id}"
            )
        if self.costmap.point_is_collision(robot_point):
            return "robot cell is in collision"
        poses = [Pose2D(robot_point[0], robot_point[1], robot_yaw)]
        poses.extend(point.pose for point in snapshot.points)
        return path_centerline_collision_reason(poses, self.costmap)

    def _start_terminal_alignment(
        self,
        robot_point: Point2,
        robot_yaw: float,
        now: float,
        *,
        goal_pose: Optional[Pose2D] = None,
        frame_id: Optional[str] = None,
    ) -> None:
        if self.goal is None:
            return
        if frame_id is None:
            frame_id = self.goal.header.frame_id or self.global_frame
        if goal_pose is None:
            try:
                goal_pose = self._goal_pose_in(frame_id)
            except TransformException as error:
                self._set_status(f"ALIGNING_YAW_WAITING_FOR_GOAL_TF: {error}")
                return
        final_yaw = robot_yaw + wrap_to_pi(goal_pose.yaw - robot_yaw)
        self.aligning_yaw = True
        self.terminal_approach_active = False
        self.terminal_emergency_braking_active = False
        self.terminal_closest_goal_distance = None
        self.xy_goal_capture_active = False
        self.reference_matches_goal = True
        self.reference_goal_pose = goal_pose
        self.alignment_goal_point = goal_pose.point
        self.alignment_goal_yaw = final_yaw
        self.alignment_frame = frame_id
        self.pending_path = None
        self.goal_settle_tracker.reset()
        self.yaw_goal_settle_tracker.reset()
        self.handover_settle_tracker.reset()
        self.trajectory.clear()
        self.path = None

        alignment_path = Path()
        alignment_path.header.frame_id = frame_id
        alignment_path.header.stamp = self.get_clock().now().to_msg()
        self.path_message = alignment_path
        if abs(final_yaw - robot_yaw) <= 1e-6:
            self.standing_clock.reset()
            self._set_status("ALIGNING_YAW_SETTLING")
            return

        poses = [
            Pose2D(robot_point[0], robot_point[1], robot_yaw),
            Pose2D(robot_point[0], robot_point[1], final_yaw),
        ]
        model = CanonicalPath(poses)
        self.path = model
        for target in poses:
            pose = PoseStamped()
            pose.header = alignment_path.header
            pose.pose.position.x = target.x
            pose.pose.position.y = target.y
            pose.pose.orientation = quaternion_from_yaw(target.yaw)
            alignment_path.poses.append(pose)
        self.trajectory.activate(
            model,
            now=now,
            start_progress=0.0,
            start_speed=0.0,
            start_yaw=robot_yaw,
            start_point=robot_point,
        )
        self.standing_clock.reset()
        self._set_status("ALIGNING_YAW")

    def _start_xy_goal_capture(
        self,
        now: float,
        *,
        actual_speed: Optional[float] = None,
        actual_progress: Optional[float] = None,
    ) -> None:
        """Latch a reached XY goal and stop instead of planning a return loop."""
        if self.xy_goal_capture_active:
            return
        self.xy_goal_capture_active = True
        self.terminal_approach_active = False
        self.terminal_emergency_braking_active = False
        self.terminal_closest_goal_distance = None
        self.plan_sequence += 1
        self.plan_pending = False
        self.last_plan_request_ns = 0
        self._cancel_active_plan()
        self.pending_path = None
        # Capture can occur after the robot has already passed a committed
        # reference point.  Rebase the stop at the measured path progress so
        # no stale prefix remains ahead of (or behind) the real robot and
        # continues driving it through the capture envelope.
        self.trajectory.begin_braking(
            now=now,
            actual_speed=actual_speed,
            actual_progress=actual_progress,
        )
        self.aligning_yaw = False
        self.alignment_goal_point = None
        self.alignment_goal_yaw = None
        self.goal_settle_tracker.reset()
        self.yaw_goal_settle_tracker.reset()
        self.handover_settle_tracker.reset()
        self._set_status("GOAL_CAPTURE_BRAKING")

    def _update_goal_state(
        self,
        now_ns: int,
        robot_point: Point2,
        robot_yaw: float,
    ) -> None:
        if not self.reference_matches_goal:
            return
        goal_pose = getattr(self, "reference_goal_pose", None)
        if goal_pose is None and self.goal is not None:
            frame_id = (
                self.path_message.header.frame_id
                if getattr(self, "path_message", None) is not None
                else (
                    self.goal.header.frame_id
                    or getattr(self, "global_frame", "map")
                )
            )
            try:
                goal_pose = self._goal_pose_in(frame_id)
            except TransformException:
                return
        if goal_pose is None:
            return
        position_error = distance(robot_point, goal_pose.point)
        captures_xy_goal = (
            getattr(self, "planner_profile", "") == SMAC_HYBRID_XY_FORWARD
            and not self.profile_contract.terminal_yaw_required
        )
        capture_is_latched = (
            captures_xy_goal
            and getattr(self, "xy_goal_capture_active", False)
        )
        position_tolerance = (
            self.goal_capture_release_tolerance
            if capture_is_latched
            else self.goal_tolerance
        )
        if position_error > position_tolerance:
            self.goal_settle_tracker.reset()
            self.yaw_goal_settle_tracker.reset()
            if capture_is_latched:
                self._set_status(
                    "GOAL_CAPTURE_DRIFTED "
                    f"position_error={position_error:.3f}"
                )
            return
        if capture_is_latched and position_error > self.goal_tolerance:
            self._set_status(
                "GOAL_CAPTURE_OUTSIDE_TOLERANCE "
                f"position_error={position_error:.3f}"
            )
        speeds = self._fresh_odom_speeds(now_ns)
        linear_speed = None if speeds is None else speeds[0]
        angular_speed = None if speeds is None else speeds[1]

        if self.profile_contract.terminal_yaw_required:
            if not self.aligning_yaw:
                translation_settled = self.goal_settle_tracker.update(
                    now_ns,
                    position_error,
                    linear_speed,
                    self.goal_tolerance,
                    self.goal_linear_speed_tolerance,
                    self.goal_hold_time,
                )
                if translation_settled:
                    self._start_terminal_alignment(
                        robot_point,
                        robot_yaw,
                        now_ns * 1e-9,
                        goal_pose=goal_pose,
                        frame_id=(
                            self.path_message.header.frame_id
                            if self.path_message is not None
                            else self.global_frame
                        ),
                    )
                return
            yaw_error = wrap_to_pi(goal_pose.yaw - robot_yaw)
            settled = self.yaw_goal_settle_tracker.update(
                now_ns,
                position_error,
                linear_speed,
                yaw_error,
                angular_speed,
                self.goal_tolerance,
                self.goal_linear_speed_tolerance,
                self.goal_yaw_tolerance,
                self.goal_angular_speed_tolerance,
                self.goal_hold_time,
            )
        else:
            if captures_xy_goal:
                if not self.xy_goal_capture_active:
                    self._start_xy_goal_capture(
                        now_ns * 1e-9,
                        actual_speed=linear_speed,
                        actual_progress=self._path_progress_at_point(robot_point),
                    )
                # XY-only Smac does not align to the RViz goal yaw, but a
                # stable arrival still requires the body to stop rotating.
                settled = self.yaw_goal_settle_tracker.update(
                    now_ns,
                    position_error,
                    linear_speed,
                    0.0,
                    angular_speed,
                    # Release tolerance is valid only after the robot has
                    # genuinely entered goal_tolerance and latched capture.
                    position_tolerance,
                    self.goal_linear_speed_tolerance,
                    0.0,
                    self.goal_angular_speed_tolerance,
                    self.goal_hold_time,
                )
            else:
                settled = self.goal_settle_tracker.update(
                    now_ns,
                    position_error,
                    linear_speed,
                    self.goal_tolerance,
                    self.goal_linear_speed_tolerance,
                    self.goal_hold_time,
                )
        if settled:
            self._finish_goal(now_ns * 1e-9)

    def _enter_fault(self, status: str, now: float) -> None:
        self.plan_sequence += 1
        self.plan_pending = False
        self._cancel_active_plan()
        self.trajectory.clear()
        self.pending_path = None
        self.reference_matches_goal = False
        self.reference_goal_pose = None
        self.aligning_yaw = False
        self.terminal_approach_active = False
        self.terminal_emergency_braking_active = False
        self.terminal_closest_goal_distance = None
        self.xy_goal_capture_active = False
        self.alignment_goal_point = None
        self.alignment_goal_yaw = None
        self.alignment_frame = self.global_frame
        self.goal_settle_tracker.reset()
        self.yaw_goal_settle_tracker.reset()
        self.handover_settle_tracker.reset()
        self._reset_speed_filter()
        self._clear_path(publish_empty=True)
        self.standing_clock.reset()
        self._set_status(status)
        self._publish_zero_waypoints(now)

    def _deadline_pacing_enabled(self) -> bool:
        """Return whether optional deadline pacing was explicitly enabled."""
        return getattr(self, "max_tracking_urgency", 0.0) > 0.0

    def _on_timer(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        now = now_ns * 1e-9
        if self.goal is None or self.goal_reached:
            self._publish_zero_waypoints(now)
            return

        if not self.trajectory.active:
            if getattr(self, "xy_goal_capture_active", False):
                self._publish_zero_waypoints(now)
                path_frame = (
                    self.path_message.header.frame_id
                    if self.path_message is not None
                    else self.global_frame
                )
                try:
                    robot_point, robot_yaw, _ = self._tracking_pose_in(
                        path_frame,
                        now_ns,
                    )
                except TransformException as error:
                    self._set_status(f"GOAL_CAPTURE_WAITING_FOR_TF: {error}")
                    return
                if self._recover_stopped_goal_capture(
                    now_ns,
                    robot_point,
                ):
                    return
                self._update_goal_state(now_ns, robot_point, robot_yaw)
                return
            if self.pending_path is not None:
                self._activate_pending_path()
            if not self.trajectory.active:
                self._publish_zero_waypoints(now)
                if self.aligning_yaw:
                    try:
                        robot_point, robot_yaw = self._base_pose_in(
                            self.alignment_frame
                        )
                    except TransformException as error:
                        self._set_status(f"ALIGNING_YAW_WAITING_FOR_TF: {error}")
                        return
                    self._update_goal_state(now_ns, robot_point, robot_yaw)
                    return
                if self.pending_path is None and self._plan_retry_due(now_ns):
                    self._request_plan()
                return

        path_frame = (
            self.path_message.header.frame_id
            if self.path_message is not None
            else self.global_frame
        )
        try:
            robot_point, robot_yaw, using_odom_fallback = self._tracking_pose_in(
                path_frame,
                now_ns,
            )
            if using_odom_fallback:
                self._begin_tracking_safety_braking(now_ns, robot_point)
                self.terminal_approach_active = False
                self.terminal_emergency_braking_active = False
                self.terminal_closest_goal_distance = None
        except TransformException as error:
            self._enter_fault(f"EMERGENCY_STOP_TF_UNAVAILABLE: {error}", now)
            return

        # Pace before snapshot() advances expired targets.  This gives a
        # lagging robot one bounded chance to catch the committed target
        # instead of silently dropping it and increasing catch-up urgency.
        pacing_shift = 0.0
        pacing_exhausted = False
        if (
            self._deadline_pacing_enabled()
            and not using_odom_fallback
            and not self.aligning_yaw
            and not self.xy_goal_capture_active
            and self.trajectory.mode
            in (
                PersistentTrajectory.TRACKING,
                PersistentTrajectory.TERMINAL_APPROACH,
            )
        ):
            pacing_shift, pacing_exhausted = self.trajectory.pace_deadlines(
                now=now,
                robot_point=robot_point,
                max_urgency=self.max_tracking_urgency,
                max_step=1.0 / self.publish_rate,
                max_consecutive_delay=self.tracking_pacing_max_delay,
            )
            if pacing_exhausted:
                self._begin_tracking_safety_braking(now_ns, robot_point)
                self.terminal_approach_active = False
                self.terminal_emergency_braking_active = False
                self.terminal_closest_goal_distance = None
                self.handover_settle_tracker.reset()

        snapshot = self.trajectory.snapshot(now)
        if (
            self.pending_path is not None
            and snapshot.mode == PersistentTrajectory.BRAKING
        ):
            if self._activate_pending_path():
                path_frame = (
                    self.path_message.header.frame_id
                    if self.path_message is not None
                    else self.global_frame
                )
                try:
                    robot_point, robot_yaw, using_odom_fallback = (
                        self._tracking_pose_in(path_frame, now_ns)
                    )
                except TransformException as error:
                    self._enter_fault(
                        f"EMERGENCY_STOP_TF_UNAVAILABLE: {error}", now
                    )
                    return
                snapshot = self.trajectory.snapshot(now)

        if self._maybe_start_terminal_approach(now_ns, robot_point):
            snapshot = self.trajectory.snapshot(now)

        if self._monitor_terminal_miss(now_ns, robot_point):
            snapshot = self.trajectory.snapshot(now)

        if self._recover_stopped_terminal(
            snapshot,
            now_ns,
            robot_point,
        ):
            return

        if (
            getattr(self, "xy_goal_capture_active", False)
            and self._reference_and_robot_stopped(snapshot, now_ns)
        ):
            if self._recover_stopped_goal_capture(
                now_ns,
                robot_point,
                robot_stopped=True,
            ):
                return
            self.trajectory.clear()
            self.standing_clock.reset()
            self._publish_zero_waypoints(now)
            self._set_status("GOAL_CAPTURE_SETTLING")
            self._update_goal_state(now_ns, robot_point, robot_yaw)
            return

        if using_odom_fallback:
            mode_status = "BRAKING_TF_ODOM_FALLBACK"
        elif getattr(self, "xy_goal_capture_active", False):
            mode_status = self._goal_capture_status(robot_point)
        elif self.aligning_yaw:
            mode_status = "ALIGNING_YAW"
        elif getattr(self, "terminal_emergency_braking_active", False):
            mode_status = "BRAKING_TERMINAL_OVERSPEED"
        elif pacing_exhausted:
            mode_status = "BRAKING_TRACKING_LAG"
        elif snapshot.mode == PersistentTrajectory.TERMINAL_APPROACH:
            mode_status = (
                "TERMINAL_APPROACH_PACING"
                if pacing_shift > 0.0
                else "TERMINAL_APPROACH"
            )
        elif pacing_shift > 0.0:
            mode_status = "ACTIVE_PACING"
        elif snapshot.mode == PersistentTrajectory.TRACKING:
            mode_status = "ACTIVE"
        else:
            mode_status = "BRAKING"
        request_replan_reason: Optional[str] = (
            PLAN_SAFETY if pacing_exhausted else None
        )
        if not self._costmap_is_fresh(now_ns):
            self._begin_tracking_safety_braking(now_ns, robot_point)
            self.terminal_approach_active = False
            self.terminal_emergency_braking_active = False
            self.terminal_closest_goal_distance = None
            snapshot = self.trajectory.snapshot(now)
            mode_status = "BRAKING_NO_FRESH_COSTMAP"
        else:
            assert self.costmap is not None
            collision_reason = self._trajectory_collision_reason(
                snapshot,
                robot_point,
                robot_yaw,
                path_frame,
            )
            if collision_reason is not None:
                if self.costmap.point_is_collision(robot_point):
                    self._enter_fault(
                        "EMERGENCY_STOP_START_IN_COLLISION",
                        now,
                    )
                    return
                self._begin_tracking_safety_braking(now_ns, robot_point)
                self.terminal_approach_active = False
                self.terminal_emergency_braking_active = False
                self.terminal_closest_goal_distance = None
                snapshot = self.trajectory.snapshot(now)
                if not self.aligning_yaw:
                    request_replan_reason = PLAN_SAFETY
                braking_reason = self._trajectory_collision_reason(
                    snapshot,
                    robot_point,
                    robot_yaw,
                    path_frame,
                )
                if braking_reason is not None:
                    self._enter_fault(
                        f"EMERGENCY_STOP_COLLISION_AHEAD: {braking_reason}",
                        now,
                    )
                    return
                mode_status = "BRAKING_COLLISION_AHEAD"

        targets = [point.pose for point in snapshot.points]
        values = self._encode_samples_in_base(targets, robot_point, robot_yaw)
        self._publish_waypoints(values, snapshot.remain_time)
        self._publish_sampled_path(path_frame, targets)
        self._set_status(mode_status)

        self._update_goal_state(now_ns, robot_point, robot_yaw)
        if self.goal_reached or getattr(self, "xy_goal_capture_active", False):
            return

        if (
            self.aligning_yaw
            or getattr(self, "terminal_approach_active", False)
            or self.pending_path is not None
            or not self._plan_retry_due(now_ns)
        ):
            return
        if request_replan_reason is None and snapshot.mode == PersistentTrajectory.BRAKING:
            request_replan_reason = PLAN_SAFETY
        if request_replan_reason is None and self._path_deviation_exceeds_limits(
            robot_point, robot_yaw
        ):
            request_replan_reason = PLAN_DEVIATION
        if (
            request_replan_reason is None
            and snapshot.mode == PersistentTrajectory.TRACKING
        ):
            request_replan_reason = PLAN_PERIODIC
        if request_replan_reason is not None:
            self._request_plan(request_replan_reason)

    def _finish_goal(self, now: Optional[float] = None) -> None:
        if now is None:
            now = self.get_clock().now().nanoseconds * 1e-9
        self.goal_reached = True
        self.plan_sequence += 1
        self.plan_pending = False
        self._cancel_active_plan()
        self.trajectory.clear()
        self.pending_path = None
        self.reference_matches_goal = False
        self.reference_goal_pose = None
        self.aligning_yaw = False
        self.terminal_approach_active = False
        self.terminal_emergency_braking_active = False
        self.terminal_closest_goal_distance = None
        self.xy_goal_capture_active = False
        self.alignment_goal_point = None
        self.alignment_goal_yaw = None
        self.alignment_frame = self.global_frame
        self.goal_settle_tracker.reset()
        self.yaw_goal_settle_tracker.reset()
        self.handover_settle_tracker.reset()
        self._reset_speed_filter()
        self._clear_path(publish_empty=True)
        self.standing_clock.reset()
        self._set_status("GOAL_REACHED")
        self._publish_zero_waypoints(now)
        if not self.hold_goal:
            self.goal = None

    def _encode_samples_in_base(
        self,
        samples: Sequence[Pose2D],
        robot_point: Point2,
        robot_yaw: float,
    ) -> list[float]:
        return relative_waypoints(
            samples,
            Pose2D(robot_point[0], robot_point[1], robot_yaw),
        )

    def _publish_waypoints(
        self,
        values: Sequence[float],
        remain_time: Sequence[float],
    ) -> None:
        if len(values) != self.num_waypoints * 3:
            raise ValueError("waypoint output length does not match num_waypoints")
        if len(remain_time) != self.num_waypoints:
            raise ValueError("remain_time output length does not match num_waypoints")
        if not all(math.isfinite(float(value)) for value in (*values, *remain_time)):
            raise ValueError("waypoint output must contain only finite values")
        if any(float(value) <= 0.0 for value in remain_time):
            raise ValueError("remain_time values must be positive")
        if any(
            float(second) <= float(first)
            for first, second in zip(remain_time, remain_time[1:])
        ):
            raise ValueError("remain_time values must be strictly increasing")
        waypoints_message = Float32MultiArray()
        waypoints_message.data = [float(value) for value in values]
        remain_time_message = Float32MultiArray()
        remain_time_message.data = [float(value) for value in remain_time]
        self.waypoints_publisher.publish(waypoints_message)
        self.remain_time_publisher.publish(remain_time_message)

    def _publish_zero_waypoints(self, now: Optional[float] = None) -> None:
        if now is None:
            now = self.get_clock().now().nanoseconds * 1e-9
        remain = self.standing_clock.snapshot(now)
        self._publish_waypoints([0.0] * (self.num_waypoints * 3), remain)

    def _publish_sampled_path(
        self,
        frame_id: str,
        samples: Sequence[Pose2D],
    ) -> None:
        message = Path()
        message.header.frame_id = frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        for sample in samples:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = sample.point[0]
            pose.pose.position.y = sample.point[1]
            pose.pose.orientation = quaternion_from_yaw(sample.yaw)
            message.poses.append(pose)
        self.sampled_path_publisher.publish(message)


def _is_expected_shutdown_runtime_error(error: RuntimeError) -> bool:
    return (
        not rclpy.ok()
        and "Unable to convert call argument to Python object" in str(error)
    )


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[Nav2GlobalGoalToWaypoints] = None
    try:
        node = Nav2GlobalGoalToWaypoints()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as error:
        if not _is_expected_shutdown_runtime_error(error):
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
