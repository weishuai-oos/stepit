#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from typing import Sequence


FLOAT_RE = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")


@dataclass(frozen=True)
class Limits:
    num_waypoints: int
    expected_dt: float
    time_tolerance: float
    max_first_distance: float
    hard_first_distance: float
    max_horizon_distance: float
    max_segment_speed: float
    max_segment_acceleration: float
    strict: bool


@dataclass(frozen=True)
class Issue:
    severity: str
    message: str


@dataclass(frozen=True)
class ValidationResult:
    issues: list[Issue]
    distances: list[float]
    segment_lengths: list[float]
    segment_speeds: list[float]
    segment_accelerations: list[float]

    @property
    def has_errors(self) -> bool:
        return any(issue.severity == "ERROR" for issue in self.issues)

    @property
    def has_warnings(self) -> bool:
        return any(issue.severity == "WARN" for issue in self.issues)


def parse_float_list(text: str) -> list[float]:
    values = [float(token) for token in FLOAT_RE.findall(text)]
    return values


def load_float_list(value: str | None, path: str | None) -> list[float]:
    if value is None and path is None:
        return []
    if value is not None and path is not None:
        raise ValueError("Provide either an inline value or a file path, not both.")
    if path is not None:
        with open(path, "r", encoding="utf-8") as file:
            value = file.read()
    assert value is not None
    if "data:" in value:
        value = value.split("data:", 1)[1]
    return parse_float_list(value)


def triples(values: Sequence[float]) -> list[tuple[float, float, float]]:
    return [(values[i], values[i + 1], values[i + 2]) for i in range(0, len(values), 3)]


def norm2(x: float, y: float) -> float:
    return math.hypot(x, y)


def validate_waypoints(
    waypoints_flat: Sequence[float],
    remain_time: Sequence[float],
    limits: Limits,
) -> ValidationResult:
    issues: list[Issue] = []
    distances: list[float] = []
    segment_lengths: list[float] = []
    segment_speeds: list[float] = []
    segment_accelerations: list[float] = []

    expected_waypoint_len = limits.num_waypoints * 3
    if len(waypoints_flat) != expected_waypoint_len:
        issues.append(
            Issue(
                "ERROR",
                f"waypoints_b must contain {expected_waypoint_len} floats, got {len(waypoints_flat)}.",
            )
        )
        return ValidationResult(issues, distances, segment_lengths, segment_speeds, segment_accelerations)

    if len(remain_time) != limits.num_waypoints:
        issues.append(
            Issue(
                "ERROR",
                f"remain_time must contain {limits.num_waypoints} floats, got {len(remain_time)}.",
            )
        )
        return ValidationResult(issues, distances, segment_lengths, segment_speeds, segment_accelerations)

    all_values = list(waypoints_flat) + list(remain_time)
    non_finite = [index for index, value in enumerate(all_values) if not math.isfinite(value)]
    if non_finite:
        issues.append(Issue("ERROR", f"All waypoint/time values must be finite; bad indices: {non_finite}."))
        return ValidationResult(issues, distances, segment_lengths, segment_speeds, segment_accelerations)

    wps = triples(waypoints_flat)

    for index, (x, y, heading) in enumerate(wps):
        distance = norm2(x, y)
        distances.append(distance)
        if abs(heading) > math.pi + 1e-3:
            issues.append(
                Issue(
                    "ERROR",
                    f"wp{index + 1} heading {heading:.3f} rad is outside [-pi, pi].",
                )
            )
        if distance > limits.max_horizon_distance:
            issues.append(
                Issue(
                    "WARN",
                    f"wp{index + 1} distance {distance:.3f} m exceeds horizon recommendation "
                    f"{limits.max_horizon_distance:.3f} m.",
                )
            )

    first_distance = distances[0]
    if first_distance > limits.hard_first_distance:
        issues.append(
            Issue(
                "ERROR",
                f"Current waypoint distance {first_distance:.3f} m exceeds hard limit "
                f"{limits.hard_first_distance:.3f} m.",
            )
        )
    elif first_distance > limits.max_first_distance:
        issues.append(
            Issue(
                "WARN",
                f"Current waypoint distance {first_distance:.3f} m exceeds recommended first-point "
                f"distance {limits.max_first_distance:.3f} m.",
            )
        )

    for index, time_value in enumerate(remain_time):
        if time_value <= 0.0:
            issues.append(Issue("ERROR", f"remain_time[{index}] must be positive, got {time_value:.3f}."))
        if index > 0 and time_value <= remain_time[index - 1]:
            issues.append(
                Issue(
                    "ERROR",
                    f"remain_time must be strictly increasing; index {index - 1}->{index} is "
                    f"{remain_time[index - 1]:.3f}->{time_value:.3f}.",
                )
            )
    if remain_time and remain_time[0] > limits.expected_dt + limits.time_tolerance:
        issues.append(
            Issue(
                "WARN",
                f"remain_time[0]={remain_time[0]:.3f}s exceeds one waypoint interval "
                f"({limits.expected_dt:.3f}s).",
            )
        )
    for index, (first, second) in enumerate(zip(remain_time, remain_time[1:])):
        interval = second - first
        if abs(interval - limits.expected_dt) > limits.time_tolerance:
            issues.append(
                Issue(
                    "WARN",
                    f"deadline interval {index}->{index + 1} is {interval:.3f}s; "
                    f"expected {limits.expected_dt:.3f}s ± {limits.time_tolerance:.3f}s.",
                )
            )

    prev_pos = (0.0, 0.0)
    prev_time = 0.0
    prev_vel = (0.0, 0.0)
    valid_time_sequence = not any(issue.severity == "ERROR" and "remain_time" in issue.message for issue in issues)
    if valid_time_sequence:
        for index, ((x, y, heading), time_value) in enumerate(zip(wps, remain_time)):
            dt = time_value - prev_time
            dx = x - prev_pos[0]
            dy = y - prev_pos[1]
            segment_length = norm2(dx, dy)
            segment_lengths.append(segment_length)
            speed = segment_length / dt
            segment_speeds.append(speed)
            vel = (dx / dt, dy / dt)
            acc = norm2(vel[0] - prev_vel[0], vel[1] - prev_vel[1]) / dt
            segment_accelerations.append(acc)

            if speed > limits.max_segment_speed + 1e-6:
                issues.append(
                    Issue(
                        "ERROR",
                        f"segment {index + 1} speed {speed:.3f} m/s exceeds "
                        f"{limits.max_segment_speed:.3f} m/s.",
                    )
                )
            if acc > limits.max_segment_acceleration + 1e-6:
                issues.append(
                    Issue(
                        "ERROR",
                        f"segment {index + 1} acceleration {acc:.3f} m/s^2 exceeds "
                        f"{limits.max_segment_acceleration:.3f} m/s^2.",
                    )
                )

            prev_pos = (x, y)
            prev_time = time_value
            prev_vel = vel

    return ValidationResult(issues, distances, segment_lengths, segment_speeds, segment_accelerations)


def format_float_list(values: Sequence[float], unit: str = "") -> str:
    if not values:
        return "[]"
    suffix = unit if unit else ""
    return "[" + ", ".join(f"{value:.3f}{suffix}" for value in values) + "]"


def format_report(result: ValidationResult, limits: Limits) -> str:
    status = "FAIL" if result.has_errors or (limits.strict and result.has_warnings) else "PASS"
    lines = [f"Waypoint condition check: {status}"]
    lines.append(f"distances_from_robot_m: {format_float_list(result.distances)}")
    lines.append(f"segment_lengths_m: {format_float_list(result.segment_lengths)}")
    lines.append(f"segment_speeds_mps: {format_float_list(result.segment_speeds)}")
    lines.append(f"segment_accelerations_mps2: {format_float_list(result.segment_accelerations)}")
    if result.issues:
        lines.append("issues:")
        for issue in result.issues:
            lines.append(f"  {issue.severity}: {issue.message}")
    else:
        lines.append("issues: none")
    return "\n".join(lines)


def run_ros(args: argparse.Namespace, limits: Limits) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray
    except ImportError as exc:
        print(f"ERROR: ROS mode requires rclpy and std_msgs: {exc}", file=sys.stderr)
        return 2

    class ValidatorNode(Node):
        def __init__(self) -> None:
            super().__init__("waypoint_condition_validator")
            self.waypoints: list[float] | None = None
            self.remain_time: list[float] | None = None
            self.last_report_time = 0.0
            self.exit_code = 0
            self.create_subscription(Float32MultiArray, args.waypoints_topic, self.on_waypoints, 10)
            self.create_subscription(Float32MultiArray, args.remain_time_topic, self.on_remain_time, 10)
            self.get_logger().info(
                f"Validating {args.waypoints_topic} and {args.remain_time_topic}; once={args.once}"
            )

        def on_waypoints(self, msg: Float32MultiArray) -> None:
            self.waypoints = list(msg.data)
            self.maybe_report()

        def on_remain_time(self, msg: Float32MultiArray) -> None:
            self.remain_time = list(msg.data)
            self.maybe_report()

        def maybe_report(self) -> None:
            if self.waypoints is None or self.remain_time is None:
                return
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if not args.once and now_sec - self.last_report_time < args.report_period:
                return
            self.last_report_time = now_sec

            result = validate_waypoints(self.waypoints, self.remain_time, limits)
            print(format_report(result, limits), flush=True)
            if result.has_errors or (limits.strict and result.has_warnings):
                self.exit_code = 1
            if args.once:
                rclpy.shutdown()

    rclpy.init()
    node = ValidatorNode()
    try:
        rclpy.spin(node)
        return node.exit_code
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate g1_traj_finetune_v2 waypoint_b/remain_time samples."
    )
    parser.add_argument("--waypoints", help="Inline 15-float waypoints_b list.")
    parser.add_argument("--waypoints-file", help="File containing a 15-float waypoints_b list or ROS echo data.")
    parser.add_argument("--remain-time", default="0.5,1.0,1.5,2.0,2.5", help="Inline 5-float remain_time list.")
    parser.add_argument("--remain-time-file", help="File containing a 5-float remain_time list or ROS echo data.")
    parser.add_argument("--ros", action="store_true", help="Subscribe to ROS2 Float32MultiArray topics.")
    parser.add_argument("--once", action="store_true", help="In ROS mode, validate once after both topics arrive.")
    parser.add_argument("--waypoints-topic", default="/waypoints_b")
    parser.add_argument("--remain-time-topic", default="/remain_time")
    parser.add_argument("--report-period", type=float, default=1.0)
    parser.add_argument("--strict", action="store_true", help="Return non-zero when warnings are present.")

    parser.add_argument("--num-waypoints", type=int, default=5)
    parser.add_argument("--expected-dt", type=float, default=0.5)
    parser.add_argument("--time-tolerance", type=float, default=0.05)
    parser.add_argument("--max-first-distance", type=float, default=0.6)
    parser.add_argument("--hard-first-distance", type=float, default=2.0)
    parser.add_argument("--max-horizon-distance", type=float, default=3.0)
    parser.add_argument("--max-segment-speed", type=float, default=1.2)
    parser.add_argument("--max-segment-acceleration", type=float, default=4.0)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    limits = Limits(
        num_waypoints=args.num_waypoints,
        expected_dt=args.expected_dt,
        time_tolerance=args.time_tolerance,
        max_first_distance=args.max_first_distance,
        hard_first_distance=args.hard_first_distance,
        max_horizon_distance=args.max_horizon_distance,
        max_segment_speed=args.max_segment_speed,
        max_segment_acceleration=args.max_segment_acceleration,
        strict=args.strict,
    )

    if args.ros:
        return run_ros(args, limits)

    try:
        waypoints = load_float_list(args.waypoints, args.waypoints_file)
        remain_time = load_float_list(args.remain_time, args.remain_time_file)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if not waypoints:
        parser.error("Provide --waypoints/--waypoints-file, or use --ros.")

    result = validate_waypoints(waypoints, remain_time, limits)
    print(format_report(result, limits))
    return 1 if result.has_errors or (limits.strict and result.has_warnings) else 0


if __name__ == "__main__":
    raise SystemExit(main())
