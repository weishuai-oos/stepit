#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import Any, Optional

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, Quaternion
    from nav_msgs.msg import Odometry, Path
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray
except ImportError:
    rclpy = None
    PoseStamped = Any
    Quaternion = Any
    Odometry = Any
    Path = Any
    Node = object
    Float32MultiArray = Any


DEFAULT_REMAIN_TIME = (0.5, 1.0, 1.5, 2.0, 2.5)
PATH_TYPES = ("LSL", "RSR", "LSR", "RSL")


@dataclass(frozen=True)
class LocalPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class DubinsPath:
    path_type: str
    lengths: tuple[float, ...]
    radius: float

    @property
    def total_length(self) -> float:
        return sum(self.lengths)

    @property
    def max_curvature(self) -> float:
        return 1.0 / self.radius


@dataclass(frozen=True)
class PlanResult:
    command_poses: list[LocalPose]
    debug_path: list[LocalPose]
    path: DubinsPath
    speed: float
    constrained: bool


@dataclass(frozen=True)
class PlannerConfig:
    remain_time: tuple[float, ...] = DEFAULT_REMAIN_TIME
    v_des: float = 0.50
    min_speed: float = 0.08
    target_tolerance: float = 0.08
    heading_tolerance: float = 0.10
    final_heading_distance: float = 0.80
    yaw_rate_des: float = 0.80
    max_tracking_yaw_rate: float = 0.45
    min_turn_radius: float = 1.00
    max_path_length_ratio: float = 2.50
    max_guarded_turn_angle: float = 1.20
    max_first_distance: float = 0.60
    max_segment_speed: float = 1.20
    max_segment_acceleration: float = 4.00
    max_waypoint_distance: float = 1.35
    max_heading_error: float = 1.20
    min_heading_segment: float = 0.05
    debug_path_spacing: float = 0.05
    max_debug_path_points: int = 220
    feasibility_iterations: int = 12

    @property
    def max_time(self) -> float:
        return max(self.remain_time)

    @property
    def max_curvature(self) -> float:
        return 1.0 / self.min_turn_radius

    @property
    def effective_v_des(self) -> float:
        return min(
            self.v_des,
            self.max_segment_speed,
            self.max_segment_acceleration * self.remain_time[0],
            self.max_first_distance / self.remain_time[0],
            self.max_waypoint_distance / self.max_time,
        )

    @property
    def effective_min_speed(self) -> float:
        return min(self.min_speed, self.effective_v_des)


def yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def mod2pi(angle: float) -> float:
    return angle % (2.0 * math.pi)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def smoothstep(x: float) -> float:
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def angle_lerp(start: float, end: float, ratio: float) -> float:
    return wrap_to_pi(start + wrap_to_pi(end - start) * ratio)


def parse_float_sequence(value: Any) -> tuple[float, ...]:
    return tuple(float(item) for item in value)


def validate_config(config: PlannerConfig) -> None:
    if len(config.remain_time) != 5:
        raise ValueError("remain_time must contain exactly 5 values for g1_traj_finetune_v2.")

    finite_values = (
        ("v_des", config.v_des),
        ("min_speed", config.min_speed),
        ("target_tolerance", config.target_tolerance),
        ("heading_tolerance", config.heading_tolerance),
        ("final_heading_distance", config.final_heading_distance),
        ("yaw_rate_des", config.yaw_rate_des),
        ("max_tracking_yaw_rate", config.max_tracking_yaw_rate),
        ("min_turn_radius", config.min_turn_radius),
        ("max_path_length_ratio", config.max_path_length_ratio),
        ("max_guarded_turn_angle", config.max_guarded_turn_angle),
        ("max_first_distance", config.max_first_distance),
        ("max_segment_speed", config.max_segment_speed),
        ("max_segment_acceleration", config.max_segment_acceleration),
        ("max_waypoint_distance", config.max_waypoint_distance),
        ("max_heading_error", config.max_heading_error),
        ("min_heading_segment", config.min_heading_segment),
        ("debug_path_spacing", config.debug_path_spacing),
    )
    for name, value in finite_values:
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite.")

    if config.v_des < 0.0:
        raise ValueError("v_des must be non-negative.")
    if config.min_speed < 0.0:
        raise ValueError("min_speed must be non-negative.")
    for name, value in finite_values[2:]:
        if value <= 0.0:
            raise ValueError(f"{name} must be positive.")

    previous_time = 0.0
    for index, time_value in enumerate(config.remain_time):
        if not math.isfinite(time_value):
            raise ValueError("remain_time values must be finite.")
        if time_value <= previous_time:
            raise ValueError(
                "remain_time must be strictly increasing and positive; "
                f"index {index} has {time_value:.3f} after {previous_time:.3f}."
            )
        previous_time = time_value

    if config.max_debug_path_points < 2:
        raise ValueError("max_debug_path_points must be at least 2.")
    if config.feasibility_iterations < 1:
        raise ValueError("feasibility_iterations must be positive.")


class ArcWaypointGenerator:
    def __init__(self, config: PlannerConfig) -> None:
        validate_config(config)
        self.config = config

    def plan(self, local_goal: LocalPose, use_goal_heading: bool) -> PlanResult:
        distance = math.hypot(local_goal.x, local_goal.y)
        if distance <= self.config.target_tolerance:
            return self.plan_goal_pose(local_goal, use_goal_heading)

        goal = self.planning_goal(local_goal, use_goal_heading)
        path = self.shortest_dubins_path(goal)
        if self.should_use_guarded_path(local_goal, path):
            path = self.guarded_path(local_goal)
        speed = self.initial_speed(path.total_length)
        constrained = False
        command_poses = self.sample_command(path, speed)

        for _ in range(self.config.feasibility_iterations):
            max_speed, max_acceleration, max_heading_error = self.command_metrics(command_poses)
            if (
                max_speed <= self.config.max_segment_speed + 1e-6
                and max_acceleration <= self.config.max_segment_acceleration + 1e-6
                and max_heading_error <= self.config.max_heading_error + 1e-6
            ):
                break

            constrained = True
            ratio = max(
                max_speed / self.config.max_segment_speed,
                math.sqrt(max_acceleration / self.config.max_segment_acceleration),
                max_heading_error / self.config.max_heading_error,
                1.05,
            )
            next_speed = max(0.02, speed / ratio)
            if next_speed >= speed * 0.98:
                next_speed = speed * 0.80
            speed = next_speed
            command_poses = self.sample_command(path, speed)

        return PlanResult(
            command_poses=command_poses,
            debug_path=self.debug_path(path),
            path=path,
            speed=speed,
            constrained=constrained,
        )

    def plan_goal_pose(self, local_goal: LocalPose, use_goal_heading: bool) -> PlanResult:
        command_poses: list[LocalPose] = []
        for time_value in self.config.remain_time:
            if use_goal_heading:
                yaw = math.copysign(min(abs(local_goal.yaw), self.config.yaw_rate_des * time_value), local_goal.yaw)
            else:
                yaw = 0.0
            command_poses.append(LocalPose(local_goal.x, local_goal.y, yaw))

        zero_path = DubinsPath("S", (0.0, 0.0, 0.0), self.config.min_turn_radius)
        return PlanResult(
            command_poses=command_poses,
            debug_path=[LocalPose(0.0, 0.0, 0.0), local_goal],
            path=zero_path,
            speed=0.0,
            constrained=False,
        )

    def initial_speed(self, path_length: float) -> float:
        speed_scale = min(1.0, path_length / max(self.config.min_turn_radius, 1e-6))
        return min(
            self.config.effective_v_des,
            max(self.config.effective_min_speed, self.config.effective_v_des * speed_scale),
        )

    def planning_goal(self, local_goal: LocalPose, use_goal_heading: bool) -> LocalPose:
        line_heading = math.atan2(local_goal.y, local_goal.x)
        if not use_goal_heading:
            return LocalPose(local_goal.x, local_goal.y, line_heading)

        distance = math.hypot(local_goal.x, local_goal.y)
        heading_ratio = smoothstep(
            (self.config.final_heading_distance - distance) / self.config.final_heading_distance
        )
        yaw = angle_lerp(line_heading, local_goal.yaw, heading_ratio)
        return LocalPose(local_goal.x, local_goal.y, yaw)

    def should_use_guarded_path(self, local_goal: LocalPose, path: DubinsPath) -> bool:
        distance = math.hypot(local_goal.x, local_goal.y)
        max_reasonable_length = max(
            self.config.min_turn_radius * 2.0,
            distance * self.config.max_path_length_ratio,
        )
        return path.total_length > max_reasonable_length

    def guarded_path(self, local_goal: LocalPose) -> DubinsPath:
        distance = math.hypot(local_goal.x, local_goal.y)
        if distance <= 1e-9:
            return DubinsPath("S", (0.0,), self.config.min_turn_radius)

        bearing = wrap_to_pi(math.atan2(local_goal.y, local_goal.x))
        if abs(bearing) <= 1e-3:
            return DubinsPath("S", (distance,), self.config.min_turn_radius)

        turn_angle = min(abs(bearing), self.config.max_guarded_turn_angle)
        mode = "L" if bearing > 0.0 else "R"
        arc_length = self.config.min_turn_radius * turn_angle
        x, y, yaw = propagate_segment(0.0, 0.0, 0.0, mode, arc_length, self.config.min_turn_radius)

        straight_length = 0.0
        if turn_angle >= abs(bearing) - 1e-6:
            dx = local_goal.x - x
            dy = local_goal.y - y
            straight_length = max(0.0, dx * math.cos(yaw) + dy * math.sin(yaw))

        if straight_length > 1e-3:
            return DubinsPath(mode + "S", (arc_length, straight_length), self.config.min_turn_radius)
        return DubinsPath(mode, (arc_length,), self.config.min_turn_radius)

    def shortest_dubins_path(self, goal: LocalPose) -> DubinsPath:
        radius = self.config.min_turn_radius
        dx = goal.x / radius
        dy = goal.y / radius
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        alpha = mod2pi(-theta)
        beta = mod2pi(goal.yaw - theta)

        candidates: list[DubinsPath] = []
        for path_type in PATH_TYPES:
            normalized = dubins_word(path_type, alpha, beta, d)
            if normalized is None:
                continue
            lengths = tuple(value * radius for value in normalized)
            candidates.append(DubinsPath(path_type, lengths, radius))

        if not candidates:
            raise RuntimeError("No curvature-limited Dubins path found for goal.")

        return min(candidates, key=lambda path: path.total_length)

    def sample_command(self, path: DubinsPath, speed: float) -> list[LocalPose]:
        poses: list[LocalPose] = []
        for time_value in self.config.remain_time:
            pose = self.pose_at(path, min(path.total_length, speed * time_value))
            poses.append(self.limit_tracking_heading(pose, time_value))
        return poses

    def limit_tracking_heading(self, pose: LocalPose, time_value: float) -> LocalPose:
        yaw_rate_limit = min(self.config.yaw_rate_des, self.config.max_tracking_yaw_rate)
        yaw_limit = min(yaw_rate_limit * time_value, self.config.max_heading_error)
        wrapped_yaw = wrap_to_pi(pose.yaw)
        if wrapped_yaw <= -math.pi + 1e-6 and (pose.yaw > 0.0 or pose.y > 0.0):
            wrapped_yaw = math.pi
        if abs(wrapped_yaw) > yaw_limit and abs(pose.y) > 1e-4:
            wrapped_yaw = math.copysign(abs(wrapped_yaw), pose.y)
        yaw = clamp(wrapped_yaw, -yaw_limit, yaw_limit)
        return LocalPose(pose.x, pose.y, yaw)

    def debug_path(self, path: DubinsPath) -> list[LocalPose]:
        if path.total_length <= 1e-9:
            return [LocalPose(0.0, 0.0, 0.0)]

        count = min(
            self.config.max_debug_path_points,
            max(2, int(math.ceil(path.total_length / self.config.debug_path_spacing)) + 1),
        )
        return [self.pose_at(path, path.total_length * index / (count - 1)) for index in range(count)]

    def pose_at(self, path: DubinsPath, target_s: float) -> LocalPose:
        x = 0.0
        y = 0.0
        yaw = 0.0
        remaining = target_s

        for mode, length in zip(path.path_type, path.lengths):
            step = min(remaining, length)
            x, y, yaw = propagate_segment(x, y, yaw, mode, step, path.radius)
            remaining -= step
            if remaining <= 1e-9:
                break

        return LocalPose(x, y, wrap_to_pi(yaw))

    def command_metrics(self, poses: list[LocalPose]) -> tuple[float, float, float]:
        previous_pos = (0.0, 0.0)
        previous_time = 0.0
        previous_velocity = (0.0, 0.0)
        max_speed = 0.0
        max_acceleration = 0.0
        max_heading_error = 0.0

        for pose, time_value in zip(poses, self.config.remain_time):
            dt = time_value - previous_time
            dx = pose.x - previous_pos[0]
            dy = pose.y - previous_pos[1]
            segment_length = math.hypot(dx, dy)
            speed = segment_length / dt
            velocity = (dx / dt, dy / dt)
            acceleration = math.hypot(
                velocity[0] - previous_velocity[0],
                velocity[1] - previous_velocity[1],
            ) / dt

            if segment_length >= self.config.min_heading_segment:
                chord_heading = math.atan2(dy, dx)
                max_heading_error = max(max_heading_error, abs(wrap_to_pi(pose.yaw - chord_heading)))

            max_speed = max(max_speed, speed)
            max_acceleration = max(max_acceleration, acceleration)
            previous_pos = (pose.x, pose.y)
            previous_time = time_value
            previous_velocity = velocity

        return max_speed, max_acceleration, max_heading_error


def dubins_word(path_type: str, alpha: float, beta: float, d: float) -> Optional[tuple[float, float, float]]:
    if path_type == "LSL":
        return dubins_lsl(alpha, beta, d)
    if path_type == "RSR":
        return dubins_rsr(alpha, beta, d)
    if path_type == "LSR":
        return dubins_lsr(alpha, beta, d)
    if path_type == "RSL":
        return dubins_rsl(alpha, beta, d)
    if path_type == "RLR":
        return dubins_rlr(alpha, beta, d)
    if path_type == "LRL":
        return dubins_lrl(alpha, beta, d)
    raise ValueError(f"Unknown Dubins path type: {path_type}")


def dubins_lsl(alpha: float, beta: float, d: float) -> Optional[tuple[float, float, float]]:
    tmp = d + math.sin(alpha) - math.sin(beta)
    p_sq = 2.0 + d**2 - 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(alpha) - math.sin(beta))
    if p_sq < 0.0:
        return None
    angle = math.atan2(math.cos(beta) - math.cos(alpha), tmp)
    return mod2pi(-alpha + angle), math.sqrt(p_sq), mod2pi(beta - angle)


def dubins_rsr(alpha: float, beta: float, d: float) -> Optional[tuple[float, float, float]]:
    tmp = d - math.sin(alpha) + math.sin(beta)
    p_sq = 2.0 + d**2 - 2.0 * math.cos(alpha - beta) + 2.0 * d * (-math.sin(alpha) + math.sin(beta))
    if p_sq < 0.0:
        return None
    angle = math.atan2(math.cos(alpha) - math.cos(beta), tmp)
    return mod2pi(alpha - angle), math.sqrt(p_sq), mod2pi(-beta + angle)


def dubins_lsr(alpha: float, beta: float, d: float) -> Optional[tuple[float, float, float]]:
    p_sq = -2.0 + d**2 + 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(alpha) + math.sin(beta))
    if p_sq < 0.0:
        return None
    p = math.sqrt(p_sq)
    angle = math.atan2(-math.cos(alpha) - math.cos(beta), d + math.sin(alpha) + math.sin(beta)) - math.atan2(-2.0, p)
    return mod2pi(-alpha + angle), p, mod2pi(-beta + angle)


def dubins_rsl(alpha: float, beta: float, d: float) -> Optional[tuple[float, float, float]]:
    p_sq = -2.0 + d**2 + 2.0 * math.cos(alpha - beta) - 2.0 * d * (math.sin(alpha) + math.sin(beta))
    if p_sq < 0.0:
        return None
    p = math.sqrt(p_sq)
    angle = math.atan2(math.cos(alpha) + math.cos(beta), d - math.sin(alpha) - math.sin(beta)) - math.atan2(2.0, p)
    return mod2pi(alpha - angle), p, mod2pi(beta - angle)


def dubins_rlr(alpha: float, beta: float, d: float) -> Optional[tuple[float, float, float]]:
    value = (6.0 - d**2 + 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(alpha) - math.sin(beta))) / 8.0
    if abs(value) > 1.0:
        return None
    p = mod2pi(2.0 * math.pi - math.acos(value))
    angle = math.atan2(math.cos(alpha) - math.cos(beta), d - math.sin(alpha) + math.sin(beta))
    t = mod2pi(alpha - angle + p / 2.0)
    q = mod2pi(alpha - beta - t + p)
    return t, p, q


def dubins_lrl(alpha: float, beta: float, d: float) -> Optional[tuple[float, float, float]]:
    value = (6.0 - d**2 + 2.0 * math.cos(alpha - beta) + 2.0 * d * (-math.sin(alpha) + math.sin(beta))) / 8.0
    if abs(value) > 1.0:
        return None
    p = mod2pi(2.0 * math.pi - math.acos(value))
    angle = math.atan2(math.cos(alpha) - math.cos(beta), d + math.sin(alpha) - math.sin(beta))
    t = mod2pi(-alpha - angle + p / 2.0)
    q = mod2pi(beta - alpha - t + p)
    return t, p, q


def propagate_segment(x: float, y: float, yaw: float, mode: str, length: float, radius: float) -> tuple[float, float, float]:
    if length <= 0.0:
        return x, y, yaw

    if mode == "S":
        return x + length * math.cos(yaw), y + length * math.sin(yaw), yaw

    delta = length / radius
    if mode == "L":
        center_x = x - radius * math.sin(yaw)
        center_y = y + radius * math.cos(yaw)
        new_yaw = yaw + delta
        return center_x + radius * math.sin(new_yaw), center_y - radius * math.cos(new_yaw), new_yaw

    if mode == "R":
        center_x = x + radius * math.sin(yaw)
        center_y = y - radius * math.cos(yaw)
        new_yaw = yaw - delta
        return center_x - radius * math.sin(new_yaw), center_y + radius * math.cos(new_yaw), new_yaw

    raise ValueError(f"Unknown segment mode: {mode}")


class ArcGlobalGoalToWaypoints(Node):
    def __init__(self) -> None:
        super().__init__("arc_global_goal_to_waypoints")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("waypoints_topic", "/waypoints_b")
        self.declare_parameter("remain_time_topic", "/remain_time")
        self.declare_parameter("debug_path_topic", "/arc_global_goal_waypoints_path")
        self.declare_parameter("v_des", 0.50)
        self.declare_parameter("min_speed", 0.08)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("target_tolerance", 0.08)
        self.declare_parameter("heading_tolerance", 0.10)
        self.declare_parameter("final_heading_distance", 0.80)
        self.declare_parameter("yaw_rate_des", 0.80)
        self.declare_parameter("max_tracking_yaw_rate", 0.45)
        self.declare_parameter("min_turn_radius", 1.00)
        self.declare_parameter("max_path_length_ratio", 2.50)
        self.declare_parameter("max_guarded_turn_angle", 1.20)
        self.declare_parameter("max_first_distance", 0.60)
        self.declare_parameter("max_segment_speed", 1.20)
        self.declare_parameter("max_segment_acceleration", 4.00)
        self.declare_parameter("max_waypoint_distance", 1.35)
        self.declare_parameter("max_heading_error", 1.20)
        self.declare_parameter("min_heading_segment", 0.05)
        self.declare_parameter("debug_path_spacing", 0.05)
        self.declare_parameter("max_debug_path_points", 220)
        self.declare_parameter("feasibility_iterations", 12)
        self.declare_parameter("remain_time", list(DEFAULT_REMAIN_TIME))
        self.declare_parameter("use_goal_heading", True)
        self.declare_parameter("hold_goal", True)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.goal_topic = self.get_parameter("goal_topic").value
        self.waypoints_topic = self.get_parameter("waypoints_topic").value
        self.remain_time_topic = self.get_parameter("remain_time_topic").value
        self.debug_path_topic = self.get_parameter("debug_path_topic").value
        self.use_goal_heading = bool(self.get_parameter("use_goal_heading").value)
        self.hold_goal = bool(self.get_parameter("hold_goal").value)

        config = PlannerConfig(
            remain_time=parse_float_sequence(self.get_parameter("remain_time").value),
            v_des=float(self.get_parameter("v_des").value),
            min_speed=float(self.get_parameter("min_speed").value),
            target_tolerance=float(self.get_parameter("target_tolerance").value),
            heading_tolerance=float(self.get_parameter("heading_tolerance").value),
            final_heading_distance=float(self.get_parameter("final_heading_distance").value),
            yaw_rate_des=float(self.get_parameter("yaw_rate_des").value),
            max_tracking_yaw_rate=float(self.get_parameter("max_tracking_yaw_rate").value),
            min_turn_radius=float(self.get_parameter("min_turn_radius").value),
            max_path_length_ratio=float(self.get_parameter("max_path_length_ratio").value),
            max_guarded_turn_angle=float(self.get_parameter("max_guarded_turn_angle").value),
            max_first_distance=float(self.get_parameter("max_first_distance").value),
            max_segment_speed=float(self.get_parameter("max_segment_speed").value),
            max_segment_acceleration=float(self.get_parameter("max_segment_acceleration").value),
            max_waypoint_distance=float(self.get_parameter("max_waypoint_distance").value),
            max_heading_error=float(self.get_parameter("max_heading_error").value),
            min_heading_segment=float(self.get_parameter("min_heading_segment").value),
            debug_path_spacing=float(self.get_parameter("debug_path_spacing").value),
            max_debug_path_points=int(self.get_parameter("max_debug_path_points").value),
            feasibility_iterations=int(self.get_parameter("feasibility_iterations").value),
        )
        self.generator = ArcWaypointGenerator(config)

        self.current_xy: Optional[tuple[float, float]] = None
        self.current_yaw: Optional[float] = None
        self.odom_frame: Optional[str] = None
        self.goal_xy: Optional[tuple[float, float]] = None
        self.goal_heading: Optional[float] = None
        self.goal_frame: Optional[str] = None
        self.frame_warned = False
        self.reached_reported = False
        self.constraint_warned = False
        self.last_path_type: Optional[str] = None

        self.waypoints_pub = self.create_publisher(Float32MultiArray, self.waypoints_topic, 10)
        self.remain_time_pub = self.create_publisher(Float32MultiArray, self.remain_time_topic, 10)
        self.debug_path_pub = self.create_publisher(Path, self.debug_path_topic, 10)

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)

        publish_rate = float(self.get_parameter("publish_rate").value)
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError("publish_rate must be finite and positive.")
        self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f"Publishing curvature-limited waypoints to {self.waypoints_topic}; "
            f"min_turn_radius={config.min_turn_radius:.2f} m, "
            f"max_curvature={config.max_curvature:.3f} 1/m, "
            f"effective_v_des={config.effective_v_des:.2f} m/s"
        )

    def odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            self.get_logger().warn("Ignoring odometry with non-finite pose values.")
            return

        self.current_xy = (x, y)
        self.current_yaw = yaw
        self.odom_frame = msg.header.frame_id or "odom"

    def goal_callback(self, msg: PoseStamped) -> None:
        if self.current_xy is None:
            self.get_logger().warn("Ignoring goal: no odometry received yet.")
            return

        goal_yaw = yaw_from_quat(msg.pose.orientation)
        if not (
            math.isfinite(msg.pose.position.x)
            and math.isfinite(msg.pose.position.y)
            and math.isfinite(goal_yaw)
        ):
            self.get_logger().warn("Ignoring goal with non-finite pose values.")
            return

        self.goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self.goal_heading = goal_yaw
        self.goal_frame = msg.header.frame_id or self.odom_frame
        self.frame_warned = False
        self.reached_reported = False
        self.constraint_warned = False
        self.last_path_type = None

        assert self.current_xy is not None
        distance = math.hypot(self.goal_xy[0] - self.current_xy[0], self.goal_xy[1] - self.current_xy[1])
        self.get_logger().info(
            f"New arc goal: robot=({self.current_xy[0]:.3f}, {self.current_xy[1]:.3f}), "
            f"goal=({self.goal_xy[0]:.3f}, {self.goal_xy[1]:.3f}), distance={distance:.3f} m, "
            f"goal_yaw={self.goal_heading:.3f} rad"
        )

    def clear_goal(self) -> None:
        self.goal_xy = None
        self.goal_heading = None
        self.goal_frame = None
        self.frame_warned = False
        self.reached_reported = False
        self.constraint_warned = False
        self.last_path_type = None

    def timer_callback(self) -> None:
        self.publish_remain_time()

        if self.current_xy is None or self.current_yaw is None:
            self.publish_zero_waypoints()
            self.publish_debug_path([])
            return

        if self.goal_xy is None or self.goal_heading is None:
            self.publish_zero_waypoints()
            self.publish_debug_path([])
            return

        if self.goal_frame and self.odom_frame and self.goal_frame != self.odom_frame and not self.frame_warned:
            self.get_logger().warn(
                f"Goal frame '{self.goal_frame}' differs from odom frame '{self.odom_frame}'. "
                "This node assumes both are already in the same frame."
            )
            self.frame_warned = True

        plan = self.compute_plan()
        self.publish_waypoints(flatten_poses(plan.command_poses))
        self.publish_debug_path([self.local_pose_to_world(pose) for pose in plan.debug_path])

        if plan.path.path_type != self.last_path_type:
            self.get_logger().info(
                f"Arc planner selected {plan.path.path_type}; length={plan.path.total_length:.3f} m, "
                f"max_curvature={plan.path.max_curvature:.3f} 1/m"
            )
            self.last_path_type = plan.path.path_type

        if plan.constrained and not self.constraint_warned:
            self.get_logger().warn(
                f"Arc planner reduced speed to {plan.speed:.3f} m/s to satisfy waypoint constraints."
            )
            self.constraint_warned = True

        local_goal = self.goal_to_local()
        yaw_reached = (not self.use_goal_heading) or abs(local_goal.yaw) <= self.generator.config.heading_tolerance
        reached = math.hypot(local_goal.x, local_goal.y) <= self.generator.config.target_tolerance and yaw_reached

        if reached:
            if not self.reached_reported:
                self.get_logger().info("Goal pose reached; holding final pose.")
                self.reached_reported = True
            if not self.hold_goal:
                self.clear_goal()

    def compute_plan(self) -> PlanResult:
        local_goal = self.goal_to_local()
        return self.generator.plan(local_goal, self.use_goal_heading)

    def goal_to_local(self) -> LocalPose:
        assert self.current_xy is not None
        assert self.current_yaw is not None
        assert self.goal_xy is not None
        assert self.goal_heading is not None

        dx = self.goal_xy[0] - self.current_xy[0]
        dy = self.goal_xy[1] - self.current_xy[1]
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        x_b = cos_yaw * dx + sin_yaw * dy
        y_b = -sin_yaw * dx + cos_yaw * dy
        heading_b = wrap_to_pi(self.goal_heading - self.current_yaw)
        return LocalPose(x_b, y_b, heading_b)

    def local_pose_to_world(self, pose: LocalPose) -> tuple[float, float, float]:
        assert self.current_xy is not None
        assert self.current_yaw is not None

        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        x_w = self.current_xy[0] + cos_yaw * pose.x - sin_yaw * pose.y
        y_w = self.current_xy[1] + sin_yaw * pose.x + cos_yaw * pose.y
        yaw_w = wrap_to_pi(self.current_yaw + pose.yaw)
        return x_w, y_w, yaw_w

    def publish_waypoints(self, waypoints: list[float]) -> None:
        msg = Float32MultiArray()
        msg.data = waypoints
        self.waypoints_pub.publish(msg)

    def publish_zero_waypoints(self) -> None:
        self.publish_waypoints([0.0] * 15)

    def publish_remain_time(self) -> None:
        msg = Float32MultiArray()
        msg.data = list(self.generator.config.remain_time)
        self.remain_time_pub.publish(msg)

    def publish_debug_path(self, poses: list[tuple[float, float, float]]) -> None:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame or self.goal_frame or "odom"

        for x, y, yaw in poses:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation = quat_from_yaw(yaw)
            msg.poses.append(pose)

        self.debug_path_pub.publish(msg)


def flatten_poses(poses: list[LocalPose]) -> list[float]:
    values: list[float] = []
    for pose in poses:
        values.extend([pose.x, pose.y, wrap_to_pi(pose.yaw)])
    return values


def parse_goal(text: str) -> LocalPose:
    parts = [float(part.strip()) for part in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("--goal must be formatted as x,y,heading.")
    return LocalPose(parts[0], parts[1], parts[2])


def run_dry() -> int:
    parser = argparse.ArgumentParser(description="Dry-run the curvature-limited waypoint generator without ROS.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--goal", type=parse_goal, default=LocalPose(2.0, 1.0, 0.5))
    parser.add_argument("--v-des", type=float, default=0.50)
    parser.add_argument("--min-turn-radius", type=float, default=1.00)
    parser.add_argument("--use-goal-heading", action="store_true", default=True)
    parser.add_argument("--ignore-goal-heading", action="store_false", dest="use_goal_heading")
    args = parser.parse_args()

    config = PlannerConfig(v_des=args.v_des, min_turn_radius=args.min_turn_radius)
    generator = ArcWaypointGenerator(config)
    plan = generator.plan(args.goal, args.use_goal_heading)
    waypoints = flatten_poses(plan.command_poses)

    print("waypoints_b:", ",".join(f"{value:.6f}" for value in waypoints))
    print("remain_time:", ",".join(f"{value:.3f}" for value in config.remain_time))
    print(f"path_type: {plan.path.path_type}")
    print(f"path_length: {plan.path.total_length:.3f}")
    print(f"min_turn_radius: {plan.path.radius:.3f}")
    print(f"max_curvature: {plan.path.max_curvature:.3f}")
    print(f"speed: {plan.speed:.3f}")
    print(f"constrained: {str(plan.constrained).lower()}")
    return 0


def run_ros() -> None:
    if rclpy is None:
        raise RuntimeError("ROS mode requires rclpy and ROS message packages.")

    rclpy.init()
    node = ArcGlobalGoalToWaypoints()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    if "--dry-run" in sys.argv:
        return run_dry()
    run_ros()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
