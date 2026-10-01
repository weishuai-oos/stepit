#!/usr/bin/python3
"""Pure profile helpers for the Nav2-to-StepIt bridge."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from nav2_waypoint_sampling import CostmapView, Point2, Pose2D, distance, wrap_to_pi


def dedupe_angles(angles: Sequence[float], tolerance: float = 1e-6) -> list[float]:
    unique: list[float] = []
    for angle in angles:
        wrapped = wrap_to_pi(float(angle))
        if not math.isfinite(wrapped):
            continue
        if all(abs(wrap_to_pi(wrapped - existing)) > tolerance for existing in unique):
            unique.append(wrapped)
    return unique


def approach_yaw_candidates(
    robot_point: Point2,
    robot_yaw: float,
    goal_point: Point2,
) -> list[float]:
    bearing = math.atan2(goal_point[1] - robot_point[1], goal_point[0] - robot_point[0])
    return dedupe_angles(
        [
            bearing,
            robot_yaw,
            bearing + math.pi / 4.0,
            bearing - math.pi / 4.0,
            bearing + math.pi / 2.0,
            bearing - math.pi / 2.0,
        ]
    )


@dataclass(frozen=True)
class AlignmentTarget:
    x: float
    y: float
    relative_yaw: float


def relative_waypoints(
    targets: Sequence[Pose2D],
    robot_pose: Pose2D,
) -> list[float]:
    """Encode map-frame path poses as StepIt's base-frame waypoint triples."""
    cosine = math.cos(robot_pose.yaw)
    sine = math.sin(robot_pose.yaw)
    values: list[float] = []
    for target in targets:
        dx = target.x - robot_pose.x
        dy = target.y - robot_pose.y
        values.extend(
            (
                cosine * dx + sine * dy,
                -sine * dx + cosine * dy,
                wrap_to_pi(target.yaw - robot_pose.yaw),
            )
        )
    return values


def alignment_targets(
    goal_in_base: Point2,
    current_yaw: float,
    final_yaw: float,
    remain_times: Sequence[float],
    max_yaw_rate: float,
) -> list[AlignmentTarget]:
    if max_yaw_rate <= 0.0 or not math.isfinite(max_yaw_rate):
        raise ValueError("max_yaw_rate must be finite and positive")
    yaw_error = wrap_to_pi(final_yaw - current_yaw)
    targets: list[AlignmentTarget] = []
    for remain_time in remain_times:
        if remain_time < 0.0 or not math.isfinite(remain_time):
            raise ValueError("remain_times must be finite and non-negative")
        limit = max_yaw_rate * remain_time
        if abs(yaw_error) <= limit:
            relative_yaw = yaw_error
        else:
            relative_yaw = math.copysign(limit, yaw_error)
        targets.append(AlignmentTarget(goal_in_base[0], goal_in_base[1], relative_yaw))
    return targets


def path_centerline_collision_reason(
    poses: Sequence[Pose2D],
    costmap: CostmapView,
) -> str | None:
    if not poses:
        return "path contains no poses"
    for index, pose in enumerate(poses):
        if costmap.point_is_collision(pose.point):
            return f"path pose {index} is in collision"
    for index, (first, second) in enumerate(zip(poses, poses[1:])):
        if distance(first.point, second.point) <= 1e-9:
            continue
        if not costmap.line_is_collision_free(first.point, second.point):
            return f"path segment {index} is in collision"
    return None
