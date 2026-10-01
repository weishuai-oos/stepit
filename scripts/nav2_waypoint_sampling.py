#!/usr/bin/python3
"""Pure waypoint sampling helpers for the Nav2-to-StepIt bridge."""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import MappingProxyType
from typing import Callable, Iterable, Mapping, Optional, Sequence


Point2 = tuple[float, float]

NAVFN_XY_LEGACY = "navfn_xy_legacy"
SMAC_HYBRID_XY_FORWARD = "smac_hybrid_xy_forward"
SMAC_TERMINAL_YAW = "smac_terminal_yaw"
SMAC_LATTICE_FULL_SE2 = "smac_lattice_full_se2"
STARTUP_PROFILE_PLANNER_IDS: Mapping[str, str] = MappingProxyType(
    {
        NAVFN_XY_LEGACY: "GridBased",
        SMAC_HYBRID_XY_FORWARD: "SmacHybrid",
        SMAC_TERMINAL_YAW: "SmacHybrid",
        SMAC_LATTICE_FULL_SE2: "SmacLattice",
    }
)


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(quaternion) -> float:
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def distance(first: Point2, second: Point2) -> float:
    return math.hypot(second[0] - first[0], second[1] - first[1])


@dataclass(frozen=True)
class Projection:
    arc_length: float
    point: Point2
    distance: float


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    @property
    def point(self) -> Point2:
        return (self.x, self.y)


@dataclass(frozen=True)
class PoseProjection:
    progress: float
    pose: Pose2D
    distance: float
    yaw_error: float


@dataclass(frozen=True)
class _PoseSegment:
    start: Pose2D
    end: Pose2D
    translation: float
    dyaw: float
    progress_length: float


class CanonicalPath:
    """SE(2) path queries with preserved duplicate-XY rotation segments."""

    def __init__(
        self,
        poses: Iterable[Pose2D],
        rotation_weight: float = 1.0,
    ) -> None:
        if rotation_weight < 0.0 or not math.isfinite(rotation_weight):
            raise ValueError("rotation_weight must be finite and non-negative")

        filtered: list[Pose2D] = []
        previous_yaw: Optional[float] = None
        for pose in poses:
            current = Pose2D(float(pose.x), float(pose.y), float(pose.yaw))
            if not (
                math.isfinite(current.x)
                and math.isfinite(current.y)
                and math.isfinite(current.yaw)
            ):
                raise ValueError("path contains a non-finite pose")
            if previous_yaw is not None:
                current = Pose2D(
                    current.x,
                    current.y,
                    previous_yaw + wrap_to_pi(current.yaw - previous_yaw),
                )
            previous_yaw = current.yaw

            if not filtered:
                filtered.append(current)
                continue

            last = filtered[-1]
            if (
                distance(last.point, current.point) <= 1e-6
                and abs(current.yaw - last.yaw) <= 1e-9
            ):
                continue
            filtered.append(current)

        if len(filtered) < 2:
            raise ValueError("path must contain at least two distinct poses")

        segments: list[_PoseSegment] = []
        cumulative = [0.0]
        for first, second in zip(filtered, filtered[1:]):
            translation = distance(first.point, second.point)
            dyaw = second.yaw - first.yaw
            progress_length = (
                translation if translation > 1e-9 else rotation_weight * abs(dyaw)
            )
            if progress_length <= 1e-9:
                continue
            segments.append(
                _PoseSegment(first, second, translation, dyaw, progress_length)
            )
            cumulative.append(cumulative[-1] + progress_length)

        if not segments:
            raise ValueError("path must contain translation or rotation progress")

        self.poses = filtered
        self.segments = segments
        self.cumulative = cumulative
        self.rotation_weight = float(rotation_weight)
        self.total_progress = cumulative[-1]
        self.total_length = self.total_progress

    def pose_at(self, progress: float) -> Pose2D:
        target = max(0.0, min(float(progress), self.total_progress))
        for index, segment in enumerate(self.segments):
            start_s = self.cumulative[index]
            end_s = self.cumulative[index + 1]
            if target <= end_s or index == len(self.segments) - 1:
                ratio = 0.0
                if segment.progress_length > 1e-9:
                    ratio = (target - start_s) / segment.progress_length
                ratio = max(0.0, min(1.0, ratio))
                return Pose2D(
                    segment.start.x + ratio * (segment.end.x - segment.start.x),
                    segment.start.y + ratio * (segment.end.y - segment.start.y),
                    segment.start.yaw + ratio * segment.dyaw,
                )
        return self.segments[-1].end

    def point_at(self, progress: float) -> Point2:
        return self.pose_at(progress).point

    def yaw_at(self, progress: float) -> float:
        return self.pose_at(progress).yaw

    def tangent_yaw(self, progress: float) -> float:
        target = max(0.0, min(float(progress), self.total_progress))
        for index, segment in enumerate(self.segments):
            if target <= self.cumulative[index + 1] or index == len(self.segments) - 1:
                if segment.translation > 1e-9:
                    return math.atan2(
                        segment.end.y - segment.start.y,
                        segment.end.x - segment.start.x,
                    )
                return segment.start.yaw
        last = self.segments[-1]
        if last.translation > 1e-9:
            return math.atan2(last.end.y - last.start.y, last.end.x - last.start.x)
        return last.end.yaw

    def project(
        self,
        point: Point2,
        yaw: Optional[float] = None,
        min_progress: float = 0.0,
        max_progress: Optional[float] = None,
    ) -> PoseProjection:
        minimum = max(0.0, min(float(min_progress), self.total_progress))
        maximum = self.total_progress
        if max_progress is not None:
            maximum = max(0.0, min(float(max_progress), self.total_progress))
        maximum = max(minimum, maximum)
        best: Optional[PoseProjection] = None

        for index, segment in enumerate(self.segments):
            segment_start_s = self.cumulative[index]
            segment_end_s = self.cumulative[index + 1]
            if segment_end_s + 1e-9 < minimum:
                continue
            if segment_start_s - 1e-9 > maximum:
                break

            minimum_ratio = max(
                0.0,
                min(1.0, (minimum - segment_start_s) / segment.progress_length),
            )
            maximum_ratio = max(
                0.0,
                min(1.0, (maximum - segment_start_s) / segment.progress_length),
            )
            if segment.translation > 1e-9:
                dx = segment.end.x - segment.start.x
                dy = segment.end.y - segment.start.y
                length_squared = dx * dx + dy * dy
                ratio = (
                    (point[0] - segment.start.x) * dx
                    + (point[1] - segment.start.y) * dy
                ) / length_squared
                ratio = max(minimum_ratio, min(maximum_ratio, ratio))
            else:
                ratio = minimum_ratio
                if yaw is not None and abs(segment.dyaw) > 1e-9:
                    yaw_delta = _continuous_yaw_delta(segment.start.yaw, yaw)
                    ratio = yaw_delta / segment.dyaw
                    ratio = max(minimum_ratio, min(maximum_ratio, ratio))

            progress = segment_start_s + ratio * segment.progress_length
            pose = self.pose_at(progress)
            yaw_error = 0.0 if yaw is None else abs(wrap_to_pi(yaw - pose.yaw))
            candidate = PoseProjection(
                progress,
                pose,
                distance(point, pose.point),
                yaw_error,
            )
            if best is None or (candidate.distance, candidate.yaw_error) < (
                best.distance,
                best.yaw_error,
            ):
                best = candidate

        if best is None:
            pose = self.pose_at(maximum)
            return PoseProjection(
                maximum,
                pose,
                distance(point, pose.point),
                0.0 if yaw is None else abs(wrap_to_pi(yaw - pose.yaw)),
            )
        return best


def _continuous_yaw_delta(start_yaw: float, yaw: float) -> float:
    return wrap_to_pi(yaw - start_yaw)


class Polyline:
    """Arc-length queries and nearest-point projection for a 2-D polyline."""

    def __init__(self, points: Iterable[Point2]) -> None:
        filtered: list[Point2] = []
        for point in points:
            x, y = float(point[0]), float(point[1])
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError("path contains a non-finite point")
            if not filtered or distance(filtered[-1], (x, y)) > 1e-6:
                filtered.append((x, y))
        if len(filtered) < 2:
            raise ValueError("path must contain at least two distinct points")

        cumulative = [0.0]
        for first, second in zip(filtered, filtered[1:]):
            cumulative.append(cumulative[-1] + distance(first, second))
        self.points = filtered
        self.cumulative = cumulative
        self.total_length = cumulative[-1]

    def point_at(self, arc_length: float) -> Point2:
        target = max(0.0, min(float(arc_length), self.total_length))
        for index in range(len(self.points) - 1):
            start_s = self.cumulative[index]
            end_s = self.cumulative[index + 1]
            if target <= end_s or index == len(self.points) - 2:
                segment_length = end_s - start_s
                if segment_length <= 1e-9:
                    return self.points[index + 1]
                ratio = (target - start_s) / segment_length
                first = self.points[index]
                second = self.points[index + 1]
                return (
                    first[0] + ratio * (second[0] - first[0]),
                    first[1] + ratio * (second[1] - first[1]),
                )
        return self.points[-1]

    def tangent_yaw(self, arc_length: float) -> float:
        target = max(0.0, min(float(arc_length), self.total_length))
        for index in range(len(self.points) - 1):
            if target <= self.cumulative[index + 1] or index == len(self.points) - 2:
                first = self.points[index]
                second = self.points[index + 1]
                return math.atan2(second[1] - first[1], second[0] - first[0])
        first = self.points[-2]
        second = self.points[-1]
        return math.atan2(second[1] - first[1], second[0] - first[0])

    def project(self, point: Point2, minimum_arc_length: float = 0.0) -> Projection:
        minimum = max(0.0, min(minimum_arc_length, self.total_length))
        best: Optional[Projection] = None
        for index, (first, second) in enumerate(zip(self.points, self.points[1:])):
            segment_start_s = self.cumulative[index]
            segment_end_s = self.cumulative[index + 1]
            if segment_end_s + 1e-9 < minimum:
                continue

            dx = second[0] - first[0]
            dy = second[1] - first[1]
            length_squared = dx * dx + dy * dy
            if length_squared <= 1e-12:
                continue
            segment_length = math.sqrt(length_squared)
            ratio = ((point[0] - first[0]) * dx + (point[1] - first[1]) * dy) / length_squared
            minimum_ratio = max(0.0, (minimum - segment_start_s) / segment_length)
            ratio = max(minimum_ratio, min(1.0, ratio))
            projected = (first[0] + ratio * dx, first[1] + ratio * dy)
            arc_length = segment_start_s + ratio * segment_length
            candidate = Projection(arc_length, projected, distance(point, projected))
            if best is None or candidate.distance < best.distance:
                best = candidate

        if best is None:
            end = self.points[-1]
            return Projection(self.total_length, end, distance(point, end))
        return best


@dataclass(frozen=True)
class CostmapView:
    frame_id: str
    resolution: float
    size_x: int
    size_y: int
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: Sequence[int]
    collision_threshold: int
    unknown_is_collision: bool

    @classmethod
    def from_message(
        cls,
        message,
        collision_threshold: int,
        unknown_is_collision: bool,
    ) -> "CostmapView":
        metadata = message.metadata
        resolution = float(metadata.resolution)
        origin_x = float(metadata.origin.position.x)
        origin_y = float(metadata.origin.position.y)
        orientation = metadata.origin.orientation
        quaternion = tuple(
            float(getattr(orientation, component))
            for component in ("x", "y", "z", "w")
        )
        if not math.isfinite(resolution) or resolution <= 0.0:
            raise ValueError("costmap resolution must be finite and positive")
        if not math.isfinite(origin_x) or not math.isfinite(origin_y):
            raise ValueError("costmap origin position must be finite")
        if not all(math.isfinite(value) for value in quaternion):
            raise ValueError("costmap origin quaternion must be finite")
        origin_yaw = yaw_from_quaternion(orientation)
        if not math.isfinite(origin_yaw):
            raise ValueError("costmap origin yaw must be finite")
        expected_size = int(metadata.size_x) * int(metadata.size_y)
        if expected_size <= 0:
            raise ValueError("costmap metadata is empty or invalid")
        if len(message.data) != expected_size:
            raise ValueError(
                f"costmap contains {len(message.data)} cells, expected {expected_size}"
            )
        return cls(
            frame_id=message.header.frame_id,
            resolution=resolution,
            size_x=int(metadata.size_x),
            size_y=int(metadata.size_y),
            origin_x=origin_x,
            origin_y=origin_y,
            origin_yaw=origin_yaw,
            data=message.data,
            collision_threshold=int(collision_threshold),
            unknown_is_collision=bool(unknown_is_collision),
        )

    def cost_at(self, point: Point2) -> Optional[int]:
        dx = point[0] - self.origin_x
        dy = point[1] - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        mx = math.floor(local_x / self.resolution)
        my = math.floor(local_y / self.resolution)
        if mx < 0 or my < 0 or mx >= self.size_x or my >= self.size_y:
            return None
        return int(self.data[my * self.size_x + mx])

    def point_is_collision(self, point: Point2) -> bool:
        cost = self.cost_at(point)
        if cost is None:
            return True
        if cost == 255:
            return self.unknown_is_collision
        return cost >= self.collision_threshold

    def line_is_collision_free(self, start: Point2, end: Point2) -> bool:
        segment_length = distance(start, end)
        # Half-cell steps prevent a thin occupied cell from being skipped.
        step = max(0.005, 0.5 * self.resolution)
        samples = max(1, math.ceil(segment_length / step))
        for index in range(samples + 1):
            ratio = index / samples
            point = (
                start[0] + ratio * (end[0] - start[0]),
                start[1] + ratio * (end[1] - start[1]),
            )
            if self.point_is_collision(point):
                return False
        return True


@dataclass(frozen=True)
class SafeSample:
    arc_length: float
    point: Point2
    yaw: float
    chosen_spacing: float


@dataclass(frozen=True)
class PoseSample:
    progress: float
    pose: Pose2D
    chosen_spacing: float

    @property
    def point(self) -> Point2:
        return self.pose.point

    @property
    def yaw(self) -> float:
        return self.pose.yaw


PoseSampleValidator = Callable[[Pose2D, PoseSample], bool]


def pose_yaw_step_within_limit(
    previous_pose: Pose2D,
    sample: PoseSample,
    max_yaw_step: float,
) -> bool:
    """Check the command-domain yaw step, including robot-to-first target."""
    if not math.isfinite(max_yaw_step) or max_yaw_step <= 0.0:
        raise ValueError("max_yaw_step must be finite and positive")
    return (
        abs(wrap_to_pi(sample.pose.yaw - previous_pose.yaw))
        <= max_yaw_step + 1e-9
    )


@dataclass(frozen=True)
class StartupProfileContract:
    name: str
    planner_id: str
    terminal_yaw_required: bool


STARTUP_PROFILE_CONTRACTS: Mapping[str, StartupProfileContract] = MappingProxyType(
    {
        NAVFN_XY_LEGACY: StartupProfileContract(
            NAVFN_XY_LEGACY,
            STARTUP_PROFILE_PLANNER_IDS[NAVFN_XY_LEGACY],
            False,
        ),
        SMAC_HYBRID_XY_FORWARD: StartupProfileContract(
            SMAC_HYBRID_XY_FORWARD,
            STARTUP_PROFILE_PLANNER_IDS[SMAC_HYBRID_XY_FORWARD],
            False,
        ),
        SMAC_TERMINAL_YAW: StartupProfileContract(
            SMAC_TERMINAL_YAW,
            STARTUP_PROFILE_PLANNER_IDS[SMAC_TERMINAL_YAW],
            True,
        ),
        SMAC_LATTICE_FULL_SE2: StartupProfileContract(
            SMAC_LATTICE_FULL_SE2,
            STARTUP_PROFILE_PLANNER_IDS[SMAC_LATTICE_FULL_SE2],
            True,
        ),
    }
)


@dataclass(frozen=True)
class HybridFeasibilityConfig:
    reverse_x_tolerance: float = -0.05
    max_first_bearing: float = 0.45
    max_chord_heading_delta: float = 0.60
    min_turning_radius: float = 1.0


@dataclass(frozen=True)
class FeasibilityResult:
    ok: bool
    reason: str = ""


@dataclass
class GoalSettleTracker:
    """Require position and speed limits to hold continuously before stopping."""

    condition_start_ns: Optional[int] = None

    def reset(self) -> None:
        self.condition_start_ns = None

    def update(
        self,
        now_ns: int,
        position_error: float,
        linear_speed: Optional[float],
        position_tolerance: float,
        speed_tolerance: float,
        hold_time: float,
    ) -> bool:
        values = (position_error, position_tolerance, speed_tolerance, hold_time)
        condition_met = (
            linear_speed is not None
            and all(math.isfinite(value) for value in values)
            and math.isfinite(linear_speed)
            and position_error <= position_tolerance
            and linear_speed <= speed_tolerance
        )
        if not condition_met:
            self.reset()
            return False

        if self.condition_start_ns is None or now_ns < self.condition_start_ns:
            self.condition_start_ns = now_ns
        elapsed = (now_ns - self.condition_start_ns) * 1e-9
        return elapsed >= hold_time


@dataclass
class YawGoalSettleTracker:
    """Require position, yaw, and speed limits to hold continuously."""

    condition_start_ns: Optional[int] = None

    def reset(self) -> None:
        self.condition_start_ns = None

    def update(
        self,
        now_ns: int,
        position_error: Optional[float],
        linear_speed: Optional[float],
        yaw_error: Optional[float],
        angular_speed: Optional[float],
        position_tolerance: float,
        speed_tolerance: float,
        yaw_tolerance: float,
        angular_speed_tolerance: float,
        hold_time: float,
    ) -> bool:
        values = (
            position_error,
            linear_speed,
            yaw_error,
            angular_speed,
            position_tolerance,
            speed_tolerance,
            yaw_tolerance,
            angular_speed_tolerance,
            hold_time,
        )
        condition_met = (
            all(value is not None and math.isfinite(value) for value in values)
            and position_error <= position_tolerance
            and linear_speed <= speed_tolerance
            and abs(yaw_error) <= yaw_tolerance
            and abs(angular_speed) <= angular_speed_tolerance
        )
        if not condition_met:
            self.reset()
            return False

        if self.condition_start_ns is None or now_ns < self.condition_start_ns:
            self.condition_start_ns = now_ns
        elapsed = (now_ns - self.condition_start_ns) * 1e-9
        return elapsed >= hold_time


def adaptive_samples(
    path: Polyline,
    costmap: CostmapView,
    robot_point: Point2,
    start_arc_length: float,
    spacings: Sequence[float],
    count: int,
) -> list[SafeSample]:
    if count <= 0:
        raise ValueError("sample count must be positive")
    candidates = sorted({float(value) for value in spacings if value > 0.0}, reverse=True)
    if not candidates:
        raise ValueError("at least one positive sampling spacing is required")

    samples: list[SafeSample] = []
    cursor = max(0.0, min(start_arc_length, path.total_length))
    previous_point = robot_point
    while len(samples) < count:
        remaining = path.total_length - cursor
        if remaining <= 1e-6:
            samples.append(
                SafeSample(path.total_length, path.points[-1], path.tangent_yaw(path.total_length), 0.0)
            )
            continue

        accepted: Optional[SafeSample] = None
        tried_arc_lengths: set[float] = set()
        for spacing in candidates:
            target_arc = min(path.total_length, cursor + spacing)
            rounded_target = round(target_arc, 9)
            if rounded_target in tried_arc_lengths:
                continue
            tried_arc_lengths.add(rounded_target)
            target_point = path.point_at(target_arc)
            if costmap.line_is_collision_free(previous_point, target_point):
                accepted = SafeSample(
                    target_arc,
                    target_point,
                    path.tangent_yaw(target_arc),
                    target_arc - cursor,
                )
                break

        if accepted is None:
            # The requested 0.2 m minimum may still bridge a very sharp corner.
            # Fall back to one costmap cell along the original Nav2 polyline.
            fallback_step = min(remaining, max(costmap.resolution, 0.02))
            target_arc = cursor + fallback_step
            target_point = path.point_at(target_arc)
            if not costmap.line_is_collision_free(previous_point, target_point):
                raise RuntimeError(
                    "no collision-free chord found, including a one-cell path step"
                )
            accepted = SafeSample(
                target_arc,
                target_point,
                path.tangent_yaw(target_arc),
                fallback_step,
            )

        samples.append(accepted)
        cursor = accepted.arc_length
        previous_point = accepted.point
    return samples


def _segment_index_at(path: CanonicalPath, progress: float) -> int:
    target = max(0.0, min(float(progress), path.total_progress))
    for index in range(len(path.segments)):
        if target <= path.cumulative[index + 1] - 1e-9:
            return index
    return len(path.segments) - 1


def _yaw_step_ok(
    path: CanonicalPath,
    cursor: float,
    target_progress: float,
    max_yaw_step: Optional[float],
) -> bool:
    if max_yaw_step is None:
        return True
    yaw_delta = abs(path.yaw_at(target_progress) - path.yaw_at(cursor))
    return yaw_delta <= max_yaw_step + 1e-9


def adaptive_pose_samples(
    path: CanonicalPath,
    costmap: CostmapView,
    robot_pose: Pose2D,
    start_progress: float,
    count: int,
    spacings: Sequence[float] = (0.60, 0.40, 0.20),
    max_yaw_step: Optional[float] = 0.35,
    validator: Optional[PoseSampleValidator] = None,
) -> list[PoseSample]:
    if count <= 0:
        raise ValueError("sample count must be positive")
    if max_yaw_step is not None and (
        max_yaw_step <= 0.0 or not math.isfinite(max_yaw_step)
    ):
        raise ValueError("max_yaw_step must be finite and positive")
    candidates = sorted({float(value) for value in spacings if value > 0.0}, reverse=True)
    if not candidates:
        raise ValueError("at least one positive sampling spacing is required")

    samples: list[PoseSample] = []
    cursor = max(0.0, min(float(start_progress), path.total_progress))
    previous_pose = robot_pose
    while len(samples) < count:
        remaining = path.total_progress - cursor
        if remaining <= 1e-6:
            samples.append(PoseSample(path.total_progress, path.pose_at(path.total_progress), 0.0))
            continue

        accepted: Optional[PoseSample] = None
        segment_index = _segment_index_at(path, cursor)
        segment = path.segments[segment_index]
        segment_end = path.cumulative[segment_index + 1]
        if segment.translation <= 1e-9 and abs(segment.dyaw) > 1e-9:
            yaw_progress_step = (
                segment_end - cursor
                if max_yaw_step is None
                else path.rotation_weight * max_yaw_step
            )
            target_progress = min(path.total_progress, segment_end, cursor + yaw_progress_step)
            target_pose = path.pose_at(target_progress)
            candidate = PoseSample(
                target_progress,
                Pose2D(previous_pose.x, previous_pose.y, target_pose.yaw),
                abs(path.yaw_at(target_progress) - path.yaw_at(cursor)),
            )
            if costmap.line_is_collision_free(previous_pose.point, candidate.point) and (
                validator is None or validator(previous_pose, candidate)
            ):
                accepted = candidate
        else:
            tried_progresses: set[float] = set()
            for spacing in candidates:
                target_progress = min(path.total_progress, cursor + spacing)
                rounded_target = round(target_progress, 9)
                if rounded_target in tried_progresses:
                    continue
                tried_progresses.add(rounded_target)
                candidate = PoseSample(
                    target_progress,
                    path.pose_at(target_progress),
                    target_progress - cursor,
                )
                if (
                    _yaw_step_ok(path, cursor, target_progress, max_yaw_step)
                    and costmap.line_is_collision_free(previous_pose.point, candidate.point)
                    and (validator is None or validator(previous_pose, candidate))
                ):
                    accepted = candidate
                    break

        if accepted is None:
            fallback_step = min(remaining, max(costmap.resolution, 0.02))
            if segment.translation <= 1e-9 and abs(segment.dyaw) > 1e-9:
                fallback_step = min(fallback_step, segment_end - cursor)
                if max_yaw_step is not None:
                    fallback_step = min(fallback_step, path.rotation_weight * max_yaw_step)
            target_progress = cursor + fallback_step
            target_pose = path.pose_at(target_progress)
            if segment.translation <= 1e-9 and abs(segment.dyaw) > 1e-9:
                target_pose = Pose2D(previous_pose.x, previous_pose.y, target_pose.yaw)
            candidate = PoseSample(
                target_progress,
                target_pose,
                fallback_step,
            )
            if (
                not _yaw_step_ok(path, cursor, target_progress, max_yaw_step)
                or not costmap.line_is_collision_free(previous_pose.point, candidate.point)
                or (validator is not None and not validator(previous_pose, candidate))
            ):
                raise RuntimeError(
                    "no collision-free pose step found, including a one-cell path step"
                )
            accepted = candidate

        samples.append(accepted)
        cursor = accepted.progress
        previous_pose = accepted.pose
    return samples


def _local_delta(reference: Pose2D, point: Point2) -> Point2:
    dx = point[0] - reference.x
    dy = point[1] - reference.y
    cosine = math.cos(reference.yaw)
    sine = math.sin(reference.yaw)
    return (cosine * dx + sine * dy, -sine * dx + cosine * dy)


def _chord_radius(chord: float, heading_delta: float) -> float:
    delta = abs(heading_delta)
    if delta <= 1e-9:
        return math.inf
    sine = math.sin(delta / 2.0)
    if chord <= 1e-9 or sine <= 1e-12:
        return 0.0
    return chord / (2.0 * sine)


def _circumradius(first: Point2, middle: Point2, last: Point2) -> float:
    """Return the radius through three path points, or infinity if collinear."""
    first_middle = distance(first, middle)
    middle_last = distance(middle, last)
    first_last = distance(first, last)
    double_area = abs(
        (middle[0] - first[0]) * (last[1] - first[1])
        - (middle[1] - first[1]) * (last[0] - first[0])
    )
    if double_area <= 1e-9:
        return math.inf
    return first_middle * middle_last * first_last / (2.0 * double_area)


def validate_raw_forward_no_reverse(
    poses: Sequence[Pose2D],
    config: HybridFeasibilityConfig = HybridFeasibilityConfig(),
) -> FeasibilityResult:
    for first, second in zip(poses, poses[1:]):
        local_x, _ = _local_delta(first, second.point)
        if local_x < config.reverse_x_tolerance:
            return FeasibilityResult(False, "reverse motion exceeds tolerance")
    return FeasibilityResult(True)


def validate_raw_curvature(
    poses: Sequence[Pose2D],
    config: HybridFeasibilityConfig = HybridFeasibilityConfig(),
) -> FeasibilityResult:
    """Validate path geometry without treating quantized pose yaw as curvature.

    Smac Hybrid orientations are discretized into angle bins.  Adjacent XY
    chords still describe the configured motion radius, but the corresponding
    pose yaw can jump by a full bin and make ``chord / yaw_delta`` report a
    radius smaller than the actual curve.  Three translated points provide a
    stable geometric radius and keep the bridge's check aligned with the path
    Nav2 actually returned.
    """
    translated: list[Pose2D] = []
    for pose in poses:
        if not translated or distance(translated[-1].point, pose.point) > 1e-9:
            translated.append(pose)

    for first, middle, last in zip(
        translated,
        translated[1:],
        translated[2:],
    ):
        first_heading = math.atan2(
            middle.y - first.y,
            middle.x - first.x,
        )
        second_heading = math.atan2(
            last.y - middle.y,
            last.x - middle.x,
        )
        heading_delta = abs(wrap_to_pi(second_heading - first_heading))
        if heading_delta > config.max_chord_heading_delta:
            return FeasibilityResult(False, "chord heading delta exceeds limit")
        radius = _circumradius(first.point, middle.point, last.point)
        if radius + 1e-6 < config.min_turning_radius:
            return FeasibilityResult(False, "curvature exceeds minimum turning radius")
    return FeasibilityResult(True)


def validate_raw_hybrid_feasibility(
    poses: Sequence[Pose2D],
    config: HybridFeasibilityConfig = HybridFeasibilityConfig(),
) -> FeasibilityResult:
    forward = validate_raw_forward_no_reverse(poses, config)
    if not forward.ok:
        return forward
    return validate_raw_curvature(poses, config)


def validate_raw_full_se2_feasibility(
    poses: Sequence[Pose2D],
    config: HybridFeasibilityConfig = HybridFeasibilityConfig(),
) -> FeasibilityResult:
    """Validate a lattice path while preserving its in-place rotations.

    Smac Lattice reports quantized pose headings at motion-primitive boundaries,
    so adjacent-pose yaw deltas do not reliably represent geometric curvature.
    Curvature is therefore measured from three translated path points.  An
    explicit duplicate-XY rotation starts a new translational segment, avoiding
    a false sharp-corner measurement across a valid in-place turn.
    """
    forward = validate_raw_forward_no_reverse(poses, config)
    if not forward.ok:
        return forward
    if not poses:
        return FeasibilityResult(False, "path contains no poses")

    translated_segment: list[Pose2D] = [poses[0]]
    previous = poses[0]
    for current in poses[1:]:
        if distance(previous.point, current.point) <= 1e-9:
            translated_segment = [current]
            previous = current
            continue

        translated_segment.append(current)
        if len(translated_segment) >= 3:
            radius = _circumradius(
                translated_segment[-3].point,
                translated_segment[-2].point,
                translated_segment[-1].point,
            )
            if radius + 1e-6 < config.min_turning_radius:
                return FeasibilityResult(False, "curvature exceeds minimum turning radius")
        previous = current
    return FeasibilityResult(True)


def validate_sampled_local_constraints(
    robot_pose: Pose2D,
    samples: Sequence[PoseSample],
    config: HybridFeasibilityConfig = HybridFeasibilityConfig(),
) -> FeasibilityResult:
    if not samples:
        return FeasibilityResult(False, "no samples to validate")
    previous_pose = robot_pose
    previous_chord_heading: Optional[float] = None
    translation_chords = 0
    for sample in samples:
        local_x, _ = _local_delta(previous_pose, sample.point)
        if local_x < config.reverse_x_tolerance:
            return FeasibilityResult(False, "sample reverses behind previous pose")
        chord = distance(previous_pose.point, sample.point)
        if chord > 1e-9:
            chord_heading = math.atan2(
                sample.point[1] - previous_pose.y,
                sample.point[0] - previous_pose.x,
            )
            if previous_chord_heading is None:
                first_bearing = abs(wrap_to_pi(chord_heading - robot_pose.yaw))
                if first_bearing > config.max_first_bearing:
                    return FeasibilityResult(False, "first bearing exceeds limit")
            else:
                heading_delta = abs(wrap_to_pi(chord_heading - previous_chord_heading))
                if heading_delta > config.max_chord_heading_delta:
                    return FeasibilityResult(False, "sample chord heading delta exceeds limit")
                # The first chord starts at the measured robot pose, which may
                # be a few centimeters off the grid-centered path projection.
                # Do not infer curvature from that convergence chord.
                if (
                    translation_chords >= 2
                    and _chord_radius(chord, heading_delta) < config.min_turning_radius
                ):
                    return FeasibilityResult(
                        False,
                        "sample curvature exceeds minimum turning radius",
                    )
            previous_chord_heading = chord_heading
            translation_chords += 1
        previous_pose = sample.pose
    return FeasibilityResult(True)


def validate_sampled_full_se2_constraints(
    robot_pose: Pose2D,
    samples: Sequence[PoseSample],
    config: HybridFeasibilityConfig = HybridFeasibilityConfig(),
) -> FeasibilityResult:
    """Validate forward chords while allowing explicit in-place rotations.

    A zero-translation sample resets the chord-heading history. The next
    translation is checked against the new path yaw instead of being treated
    as an instantaneous bend from the chord before the rotation.
    """
    if not samples:
        return FeasibilityResult(False, "no samples to validate")
    previous_pose = robot_pose
    previous_chord_heading: Optional[float] = None
    translation_chords = 0
    for sample in samples:
        chord = distance(previous_pose.point, sample.point)
        if chord <= 1e-9:
            previous_pose = sample.pose
            previous_chord_heading = None
            translation_chords = 0
            continue

        local_x, _ = _local_delta(previous_pose, sample.point)
        if local_x < config.reverse_x_tolerance:
            return FeasibilityResult(False, "sample reverses behind previous pose")
        chord_heading = math.atan2(
            sample.point[1] - previous_pose.y,
            sample.point[0] - previous_pose.x,
        )
        if previous_chord_heading is None:
            departure = abs(wrap_to_pi(chord_heading - previous_pose.yaw))
            if departure > config.max_first_bearing:
                return FeasibilityResult(False, "departure bearing exceeds limit")
        else:
            heading_delta = abs(wrap_to_pi(chord_heading - previous_chord_heading))
            if heading_delta > config.max_chord_heading_delta:
                return FeasibilityResult(False, "sample chord heading delta exceeds limit")
            if (
                translation_chords >= 2
                and _chord_radius(chord, heading_delta) < config.min_turning_radius
            ):
                return FeasibilityResult(
                    False,
                    "sample curvature exceeds minimum turning radius",
                )
        previous_chord_heading = chord_heading
        translation_chords += 1
        previous_pose = sample.pose
    return FeasibilityResult(True)
