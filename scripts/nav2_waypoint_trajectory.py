#!/usr/bin/python3
"""Persistent, time-parameterized waypoint reference for the StepIt policy.

This module is deliberately ROS-free.  A planner supplies path geometry; this
module owns the dynamic contract seen by the policy: five persistent world
targets, fixed absolute target times, bounded speed/acceleration, and a
counting-down remain-time vector.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Union

from nav2_waypoint_sampling import (
    CanonicalPath,
    Point2,
    Polyline,
    Pose2D,
    distance,
    wrap_to_pi,
)


PathModel = Union[Polyline, CanonicalPath]

# Keep the bridge's simple-heading fallback identical to the frozen training
# command implementation (`direction_magnitude > 1e-3`).  This is deliberately
# much larger than the numerical epsilons used for clocks/path geometry below.
SIMPLE_HEADING_MIN_DISPLACEMENT = 1e-3


@dataclass(frozen=True)
class TrajectoryLimits:
    waypoint_interval: float
    num_waypoints: int
    cruise_speed: float
    max_acceleration: float
    max_deceleration: float
    terminal_deceleration: float
    max_lateral_acceleration: float
    max_yaw_rate: float
    integration_step: float
    curvature_window: float
    replan_commit_time: float
    replan_join_distance: float
    replan_max_waypoint_shift: float
    replan_max_heading_shift: float

    def __post_init__(self) -> None:
        positive = {
            "waypoint_interval": self.waypoint_interval,
            "cruise_speed": self.cruise_speed,
            "max_acceleration": self.max_acceleration,
            "max_deceleration": self.max_deceleration,
            "terminal_deceleration": self.terminal_deceleration,
            "max_lateral_acceleration": self.max_lateral_acceleration,
            "max_yaw_rate": self.max_yaw_rate,
            "integration_step": self.integration_step,
            "curvature_window": self.curvature_window,
            "replan_join_distance": self.replan_join_distance,
            "replan_max_waypoint_shift": self.replan_max_waypoint_shift,
            "replan_max_heading_shift": self.replan_max_heading_shift,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.num_waypoints <= 0:
            raise ValueError("num_waypoints must be positive")
        if (
            not math.isfinite(self.replan_commit_time)
            or self.replan_commit_time < 0.0
        ):
            raise ValueError("replan_commit_time must be finite and non-negative")
        if self.integration_step > self.waypoint_interval:
            raise ValueError("integration_step cannot exceed waypoint_interval")
        if self.terminal_deceleration > self.max_deceleration:
            raise ValueError("terminal_deceleration cannot exceed max_deceleration")


@dataclass(frozen=True)
class TimedWaypoint:
    deadline: float
    pose: Pose2D
    progress: float
    speed: float


@dataclass(frozen=True)
class TrajectorySnapshot:
    points: tuple[TimedWaypoint, ...]
    remain_time: tuple[float, ...]
    mode: str


class RollingDeadlineClock:
    """Keep a standing command on the same rolling time axis as training."""

    def __init__(self, interval: float, count: int) -> None:
        if not math.isfinite(interval) or interval <= 0.0:
            raise ValueError("interval must be finite and positive")
        if count <= 0:
            raise ValueError("count must be positive")
        self.interval = float(interval)
        self.count = int(count)
        self._deadlines: list[float] = []

    def reset(self) -> None:
        self._deadlines = []

    def snapshot(self, now: float) -> tuple[float, ...]:
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        if not self._deadlines:
            self._deadlines = [
                now + self.interval * (index + 1)
                for index in range(self.count)
            ]
        while self._deadlines[0] <= now + 1e-9:
            self._deadlines.pop(0)
            self._deadlines.append(self._deadlines[-1] + self.interval)
        return tuple(
            round(max(deadline - now, 1e-9), 9)
            for deadline in self._deadlines
        )


@dataclass(frozen=True)
class _PathProjection:
    progress: float
    pose: Pose2D
    distance: float
    yaw_error: float


class PersistentTrajectory:
    """Own a persistent future waypoint queue on one continuous time axis."""

    TRACKING = "TRACKING"
    BRAKING = "BRAKING"
    TERMINAL_APPROACH = "TERMINAL_APPROACH"

    def __init__(self, limits: TrajectoryLimits) -> None:
        self.limits = limits
        self._path: Optional[PathModel] = None
        self._points: list[TimedWaypoint] = []
        self._tail_progress = 0.0
        self._tail_speed = 0.0
        self._tail_yaw = 0.0
        self._mode = self.TRACKING
        self._pace_delay_total = 0.0

    @property
    def active(self) -> bool:
        return self._path is not None and bool(self._points)

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def path(self) -> Optional[PathModel]:
        return self._path

    def clear(self) -> None:
        self._path = None
        self._points = []
        self._tail_progress = 0.0
        self._tail_speed = 0.0
        self._tail_yaw = 0.0
        self._mode = self.TRACKING
        self._pace_delay_total = 0.0

    def activate(
        self,
        path: PathModel,
        *,
        now: float,
        start_progress: float,
        start_speed: float,
        start_yaw: float,
        start_point: Optional[Point2] = None,
    ) -> None:
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        total = _path_total(path)
        if total <= 0.0:
            raise ValueError("path must have positive progress")
        self._path = path
        self._points = []
        self._tail_progress = max(0.0, min(float(start_progress), total))
        if not math.isfinite(start_speed) or start_speed < 0.0:
            raise ValueError("start_speed must be finite and non-negative")
        if not math.isfinite(start_yaw):
            raise ValueError("start_yaw must be finite")
        # The bridge's cruise speed is the policy reference limit, not an
        # estimate of the robot's physical velocity.  Measured overspeed is
        # handled explicitly by the terminal/emergency braking paths instead
        # of turning it into a faster tracking command.
        self._tail_speed = min(float(start_speed), self.limits.cruise_speed)
        self._tail_yaw = float(start_yaw)
        self._mode = self.TRACKING
        self._pace_delay_total = 0.0
        deadline = float(now)
        for _ in range(self.limits.num_waypoints):
            deadline += self.limits.waypoint_interval
            self._points.append(self._next_point(deadline, self.limits.waypoint_interval))
        if not _path_is_pure_rotation(path):
            origin = (
                start_point
                if start_point is not None
                else self._points[0].pose.point
            )
            self._assign_simple_headings(origin, start_yaw)
            self._tail_yaw = self._points[-1].pose.yaw

    def snapshot(self, now: float) -> TrajectorySnapshot:
        self._advance(float(now))
        remain_time = tuple(
            round(max(point.deadline - now, 1e-9), 9)
            for point in self._points
        )
        return TrajectorySnapshot(tuple(self._points), remain_time, self._mode)

    def adopt_replan(
        self,
        path: PathModel,
        *,
        now: float,
        strict_continuity: bool = True,
    ) -> bool:
        """Replace only the uncommitted suffix when a new path joins smoothly."""
        self._advance(float(now))
        if not self.active:
            return False

        preserved = [
            point
            for point in self._points
            if point.deadline - now <= self.limits.replan_commit_time + 1e-9
        ]
        if not preserved:
            preserved = [self._points[0]]
        anchor = preserved[-1]
        projection = _project_path(path, anchor.pose)
        if projection.distance > self.limits.replan_join_distance:
            return False
        old_deadlines = [point.deadline for point in self._points]
        candidate = list(preserved)
        progress = projection.progress
        speed = min(anchor.speed, self.limits.cruise_speed)
        yaw = anchor.pose.yaw
        previous_deadline = anchor.deadline
        for deadline in old_deadlines[len(preserved) :]:
            duration = deadline - previous_deadline
            progress, speed, pose = self._integrate(
                path,
                progress,
                speed,
                yaw,
                duration,
                braking=False,
            )
            candidate.append(TimedWaypoint(deadline, pose, progress, speed))
            yaw = pose.yaw
            previous_deadline = deadline

        if len(candidate) != self.limits.num_waypoints:
            return False
        # If every currently queued target is committed there is no safe
        # suffix on which to switch the path/progress coordinate system.
        if len(candidate) == len(preserved):
            return False
        if not _path_is_pure_rotation(path):
            self._assign_suffix_headings(candidate, len(preserved))
        # Smac uses the stricter all-horizon contract.  The legacy mode keeps
        # the original first-suffix XY check so NavFn behavior is unchanged.
        suffix_indexes = range(len(preserved), len(candidate))
        if not strict_continuity:
            suffix_indexes = range(len(preserved), len(preserved) + 1)
        for index in suffix_indexes:
            previous = candidate[index - 1]
            current = candidate[index]
            duration = current.deadline - previous.deadline
            if not _transition_is_reachable(
                previous,
                current,
                duration,
                self.limits,
                check_heading=strict_continuity,
            ):
                return False
            if not strict_continuity:
                continue
            old = self._points[index]
            if (
                distance(old.pose.point, current.pose.point)
                > self.limits.replan_max_waypoint_shift + 1e-6
            ):
                return False
            if (
                abs(wrap_to_pi(old.pose.yaw - current.pose.yaw))
                > self.limits.replan_max_heading_shift + 1e-6
            ):
                return False
        self._path = path
        self._points = candidate
        tail = candidate[-1]
        self._tail_progress = tail.progress
        self._tail_speed = tail.speed
        self._tail_yaw = tail.pose.yaw
        self._mode = self.TRACKING
        self._pace_delay_total = 0.0
        return True

    def terminal_approach_feasible(
        self,
        *,
        now: float,
        actual_speed: float,
        response_time: float,
        actual_progress: Optional[float] = None,
    ) -> bool:
        """Whether measured speed can reach zero inside the mutable horizon."""
        values = (now, actual_speed, response_time)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("terminal feasibility inputs must be finite")
        if actual_speed < 0.0 or response_time < 0.0:
            raise ValueError("terminal speed and response_time must be non-negative")
        self._advance(float(now))
        if not self.active:
            return False
        preserved = self._preserved_points(now)
        if len(preserved) == len(self._points):
            return False
        anchor = preserved[-1]
        if actual_progress is not None:
            if not math.isfinite(actual_progress):
                raise ValueError("actual_progress must be finite")
            path = self._path
            assert path is not None
            actual_progress = max(0.0, min(float(actual_progress), _path_total(path)))
            # The committed anchor is the first immutable target.  If the
            # measured robot progress has already passed it, rebuilding the
            # suffix from that anchor would necessarily command a backwards
            # temporal jump or let the robot overshoot it before its deadline.
            if actual_progress > anchor.progress + 1e-9:
                return False
            duration_to_anchor = max(0.0, anchor.deadline - now)
            response_phase = min(response_time, duration_to_anchor)
            deceleration_phase = min(
                duration_to_anchor - response_phase,
                actual_speed / self.limits.terminal_deceleration,
            )
            speed_at_anchor = max(
                0.0,
                actual_speed
                - self.limits.terminal_deceleration * deceleration_phase,
            )
            predicted_to_anchor = (
                actual_speed * response_phase
                + 0.5 * (actual_speed + speed_at_anchor) * deceleration_phase
            )
            if predicted_to_anchor > anchor.progress - actual_progress + 1e-9:
                return False
        duration_to_anchor = max(0.0, anchor.deadline - now)
        deceleration_before_anchor = max(
            0.0,
            duration_to_anchor - response_time,
        )
        deceleration_before_anchor = min(
            deceleration_before_anchor,
            actual_speed / self.limits.terminal_deceleration,
        )
        speed_at_anchor = max(
            0.0,
            actual_speed
            - self.limits.terminal_deceleration * deceleration_before_anchor,
        )
        response_after_anchor = max(0.0, response_time - duration_to_anchor)
        available = (
            self._points[-1].deadline
            - anchor.deadline
            - response_after_anchor
        )
        path = self._path
        assert path is not None
        remaining_path = max(0.0, _path_total(path) - anchor.progress)
        required_path = (
            speed_at_anchor * response_after_anchor
            + speed_at_anchor * speed_at_anchor
            / (2.0 * self.limits.terminal_deceleration)
        )
        return (
            speed_at_anchor
            <= self.limits.terminal_deceleration * available + 1e-9
            and required_path <= remaining_path + 1e-9
        )

    def begin_terminal_approach(
        self,
        *,
        now: float,
        actual_speed: float,
        response_time: float,
        actual_progress: Optional[float] = None,
    ) -> bool:
        """Rebuild only the uncommitted suffix using the measured approach speed."""
        values = (actual_speed, response_time)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("terminal approach inputs must be finite")
        if actual_speed < 0.0 or response_time < 0.0:
            raise ValueError("terminal speed and response_time must be non-negative")
        if not self.terminal_approach_feasible(
            now=now,
            actual_speed=actual_speed,
            response_time=response_time,
            actual_progress=actual_progress,
        ):
            return False
        preserved = self._preserved_points(now)
        if len(preserved) == len(self._points):
            self._mode = self.TERMINAL_APPROACH
            return True
        anchor = preserved[-1]
        deceleration_time_to_anchor = max(
            0.0,
            anchor.deadline - now - response_time,
        )
        speed = max(
            0.0,
            actual_speed
            - self.limits.terminal_deceleration
            * deceleration_time_to_anchor,
        )
        candidate = list(preserved)
        progress, yaw = anchor.progress, anchor.pose.yaw
        previous_deadline = anchor.deadline
        path = self._path
        assert path is not None
        for deadline in [
            point.deadline for point in self._points[len(preserved) :]
        ]:
            duration = deadline - previous_deadline
            progress, speed, pose = self._integrate(
                path,
                progress,
                speed,
                yaw,
                duration,
                braking=False,
                deceleration_limit=self.limits.terminal_deceleration,
                allow_acceleration=False,
            )
            candidate.append(TimedWaypoint(deadline, pose, progress, speed))
            yaw, previous_deadline = pose.yaw, deadline
        if not _path_is_pure_rotation(path):
            self._assign_suffix_headings(candidate, len(preserved))
        self._points = candidate
        tail = candidate[-1]
        self._tail_progress = tail.progress
        self._tail_speed = tail.speed
        self._tail_yaw = tail.pose.yaw
        self._mode = self.TERMINAL_APPROACH
        self._pace_delay_total = 0.0
        return True

    def _preserved_points(self, now: float) -> list[TimedWaypoint]:
        preserved = [
            point
            for point in self._points
            if point.deadline - now <= self.limits.replan_commit_time + 1e-9
        ]
        return preserved if preserved else [self._points[0]]

    def pace_deadlines(
        self,
        *,
        now: float,
        robot_point: Point2,
        max_urgency: float,
        max_step: float,
        max_consecutive_delay: float,
    ) -> tuple[float, bool]:
        """Pause the common time axis when the first target requires catch-up."""
        if (
            self._mode not in (self.TRACKING, self.TERMINAL_APPROACH)
            or not self.active
        ):
            return 0.0, False
        values = (
            now,
            robot_point[0],
            robot_point[1],
            max_urgency,
            max_step,
            max_consecutive_delay,
        )
        if (
            not all(math.isfinite(value) for value in values)
            or max_urgency <= 0.0
            or max_step <= 0.0
            or max_consecutive_delay <= 0.0
        ):
            raise ValueError("pace parameters must be finite and positive")
        first = self._points[0]
        remain = first.deadline - now
        # Pacing is only valid while the first target is still ahead on the
        # path.  Euclidean distance alone is misleading once the robot has
        # crossed that target: the distance then grows again and the bridge
        # can keep shifting every 20 ms, effectively chasing a stale target
        # and exhausting the delay budget.  Use the path coordinate to make
        # this decision; retain the distance-based check for targets that are
        # still ahead (including lateral tracking error).
        path = self._path
        assert path is not None
        projection = _project_path(
            path,
            Pose2D(robot_point[0], robot_point[1], first.pose.yaw),
        )
        if projection.progress > first.progress + 1e-6:
            self._pace_delay_total = 0.0
            return 0.0, False
        required_remain = max(
            distance(robot_point, first.pose.point) / max_urgency,
            1e-6,
        )
        required = required_remain - remain
        if required <= 1e-9:
            self._pace_delay_total = 0.0
            return 0.0, False
        available = max(0.0, max_consecutive_delay - self._pace_delay_total)
        shift = min(required, max_step, available)
        if shift <= 1e-12:
            return 0.0, True
        self._points = [
            TimedWaypoint(
                point.deadline + shift,
                point.pose,
                point.progress,
                point.speed,
            )
            for point in self._points
        ]
        self._pace_delay_total += shift
        exhausted = (
            remain + shift <= 1e-9
            or (
                required - shift > 1e-9
                and self._pace_delay_total >= max_consecutive_delay - 1e-9
            )
        )
        return shift, exhausted

    def begin_braking(
        self,
        *,
        now: float,
        actual_progress: Optional[float] = None,
        actual_speed: Optional[float] = None,
    ) -> None:
        """Keep the committed prefix and replace the suffix with bounded braking."""
        if actual_speed is not None and (
            not math.isfinite(actual_speed) or actual_speed < 0.0
        ):
            raise ValueError("actual_speed must be finite and non-negative")
        self._advance(float(now))
        if not self.active:
            return
        if actual_progress is not None:
            if not math.isfinite(actual_progress) or actual_progress < 0.0:
                raise ValueError("actual_progress must be finite and non-negative")
            path = self._path
            assert path is not None
            # Emergency braking may be requested after the robot has already
            # passed the planner endpoint.  Keep the overrun progress intact;
            # _path_pose provides the final-tangent extrapolation needed to
            # avoid sending the robot back toward the endpoint.
            actual_progress = float(actual_progress)
            origin = _path_pose(path, actual_progress, self.limits.curvature_window)
            speed = (
                float(actual_speed)
                if actual_speed is not None
                else self._points[0].speed
            )
            deadlines = [point.deadline for point in self._points]
            candidate: list[TimedWaypoint] = []
            progress = actual_progress
            yaw = origin.yaw
            previous_deadline = float(now)
            for deadline in deadlines:
                duration = max(0.0, deadline - previous_deadline)
                progress, speed, pose = self._integrate(
                    path,
                    progress,
                    speed,
                    yaw,
                    duration,
                    braking=True,
                )
                candidate.append(TimedWaypoint(deadline, pose, progress, speed))
                yaw, previous_deadline = pose.yaw, deadline
            if not _path_is_pure_rotation(path):
                original_points = self._points
                self._points = candidate
                self._assign_simple_headings(origin.point, origin.yaw)
                candidate = self._points
                self._points = original_points
            self._points = candidate
            tail = candidate[-1]
            self._tail_progress = tail.progress
            self._tail_speed = tail.speed
            self._tail_yaw = tail.pose.yaw
            self._mode = self.BRAKING
            self._pace_delay_total = 0.0
            return
        preserved = [
            point
            for point in self._points
            if point.deadline - now <= self.limits.replan_commit_time + 1e-9
        ]
        if not preserved:
            preserved = [self._points[0]]

        old_deadlines = [point.deadline for point in self._points]
        candidate = list(preserved)
        anchor = preserved[-1]
        progress = anchor.progress
        speed = anchor.speed
        if actual_speed is not None:
            duration_to_anchor = max(0.0, anchor.deadline - now)
            speed = max(
                speed,
                actual_speed
                - self.limits.max_deceleration * duration_to_anchor,
            )
        yaw = anchor.pose.yaw
        previous_deadline = anchor.deadline
        path = self._path
        assert path is not None
        for deadline in old_deadlines[len(preserved) :]:
            duration = deadline - previous_deadline
            progress, speed, pose = self._integrate(
                path,
                progress,
                speed,
                yaw,
                duration,
                braking=True,
            )
            candidate.append(TimedWaypoint(deadline, pose, progress, speed))
            yaw = pose.yaw
            previous_deadline = deadline

        if not _path_is_pure_rotation(path):
            self._assign_suffix_headings(candidate, len(preserved))
        self._points = candidate
        tail = candidate[-1]
        self._tail_progress = tail.progress
        self._tail_speed = tail.speed
        self._tail_yaw = tail.pose.yaw
        self._mode = self.BRAKING
        self._pace_delay_total = 0.0

    def _advance(self, now: float) -> None:
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        if not self.active:
            return
        while self._points and self._points[0].deadline <= now + 1e-9:
            self._points.pop(0)
            last_deadline = self._points[-1].deadline if self._points else now
            next_deadline = last_deadline + self.limits.waypoint_interval
            self._points.append(
                self._next_point(next_deadline, self.limits.waypoint_interval)
            )
            if self._path is not None and not _path_is_pure_rotation(self._path):
                self._assign_suffix_headings(self._points, len(self._points) - 1)
                self._tail_yaw = self._points[-1].pose.yaw

    def _next_point(self, deadline: float, duration: float) -> TimedWaypoint:
        path = self._path
        assert path is not None
        progress, speed, pose = self._integrate(
            path,
            self._tail_progress,
            self._tail_speed,
            self._tail_yaw,
            duration,
            braking=self._mode == self.BRAKING,
            deceleration_limit=(
                self.limits.terminal_deceleration
                if self._mode == self.TERMINAL_APPROACH
                else self.limits.max_deceleration
            ),
            allow_acceleration=self._mode != self.TERMINAL_APPROACH,
        )
        self._tail_progress = progress
        self._tail_speed = speed
        self._tail_yaw = pose.yaw
        return TimedWaypoint(deadline, pose, progress, speed)

    def _assign_simple_headings(self, start_point: Point2, start_yaw: float) -> None:
        if not self._points:
            return
        heading = start_yaw
        first = self._points[0]
        if (
            distance(start_point, first.pose.point)
            > SIMPLE_HEADING_MIN_DISPLACEMENT
        ):
            heading = math.atan2(
                first.pose.y - start_point[1], first.pose.x - start_point[0]
            )
        self._points[0] = TimedWaypoint(
            first.deadline,
            Pose2D(first.pose.x, first.pose.y, heading),
            first.progress,
            first.speed,
        )
        for index in range(1, len(self._points) - 1):
            point = self._points[index]
            next_point = self._points[index + 1]
            if (
                distance(point.pose.point, next_point.pose.point)
                > SIMPLE_HEADING_MIN_DISPLACEMENT
            ):
                heading = math.atan2(
                    next_point.pose.y - point.pose.y,
                    next_point.pose.x - point.pose.x,
                )
            self._points[index] = TimedWaypoint(
                point.deadline,
                Pose2D(point.pose.x, point.pose.y, heading),
                point.progress,
                point.speed,
            )
        if len(self._points) > 1:
            last = self._points[-1]
            self._points[-1] = TimedWaypoint(
                last.deadline,
                Pose2D(last.pose.x, last.pose.y, heading),
                last.progress,
                last.speed,
            )

    def _assign_suffix_headings(self, points: list[TimedWaypoint], start: int) -> None:
        if start >= len(points):
            return
        heading = points[start - 1].pose.yaw if start > 0 else 0.0
        for index in range(start, len(points)):
            previous = points[index - 1].pose.point
            current = points[index].pose.point
            if distance(previous, current) > SIMPLE_HEADING_MIN_DISPLACEMENT:
                heading = math.atan2(current[1] - previous[1], current[0] - previous[0])
            point = points[index]
            points[index] = TimedWaypoint(
                point.deadline,
                Pose2D(point.pose.x, point.pose.y, heading),
                point.progress,
                point.speed,
            )

    def _integrate(
        self,
        path: PathModel,
        progress: float,
        speed: float,
        yaw: float,
        duration: float,
        *,
        braking: bool,
        deceleration_limit: Optional[float] = None,
        allow_acceleration: bool = True,
    ) -> tuple[float, float, Pose2D]:
        if deceleration_limit is None:
            deceleration_limit = self.limits.max_deceleration
        if not math.isfinite(deceleration_limit) or deceleration_limit <= 0.0:
            raise ValueError("deceleration_limit must be finite and positive")
        remaining_time = max(0.0, duration)
        while remaining_time > 1e-12:
            dt = min(self.limits.integration_step, remaining_time)
            if braking:
                target_speed = 0.0
            else:
                target_speed = self._speed_limit(path, progress)
                if not allow_acceleration:
                    target_speed = min(target_speed, speed)
                    remaining_path = max(0.0, _path_total(path) - progress)
                    stop_with_one_step_reserve = (
                        speed * dt
                        + speed * speed / (2.0 * deceleration_limit)
                    )
                    if stop_with_one_step_reserve >= remaining_path - 1e-12:
                        target_speed = 0.0
            desired_acceleration = (target_speed - speed) / dt
            acceleration = max(
                -deceleration_limit,
                min(self.limits.max_acceleration, desired_acceleration),
            )
            if acceleration < 0.0 and speed + acceleration * dt < 0.0:
                stop_time = speed / -acceleration
                progress += speed * stop_time + 0.5 * acceleration * stop_time**2
                speed = 0.0
            else:
                progress += speed * dt + 0.5 * acceleration * dt**2
                speed = max(0.0, speed + acceleration * dt)
            progress = max(0.0, progress)
            remaining_time -= dt

        target_pose = _path_pose(path, progress, self.limits.curvature_window)
        if _path_has_translation_at(path, progress):
            # Translational path pose supplies the XY integration result;
            # emitted simple-heading yaw is assigned after the target queue
            # is generated, so it remains decoupled from this internal yaw.
            yaw = target_pose.yaw
        else:
            max_yaw_delta = self.limits.max_yaw_rate * max(0.0, duration)
            yaw_delta = wrap_to_pi(target_pose.yaw - yaw)
            yaw = yaw + max(-max_yaw_delta, min(max_yaw_delta, yaw_delta))
        return progress, speed, Pose2D(target_pose.x, target_pose.y, yaw)

    def _speed_limit(self, path: PathModel, progress: float) -> float:
        remaining = max(0.0, _path_total(path) - progress)
        stopping_limit = math.sqrt(2.0 * self.limits.terminal_deceleration * remaining)
        curvature = (
            _path_curvature(path, progress, self.limits.curvature_window)
            if _path_has_translation_at(path, progress)
            else 0.0
        )
        if curvature <= 1e-9:
            curvature_limit = self.limits.cruise_speed
        else:
            curvature_limit = min(
                math.sqrt(self.limits.max_lateral_acceleration / curvature),
                self.limits.max_yaw_rate / curvature,
            )
        if not _path_has_translation_at(path, progress):
            rotation_rate_limit = self.limits.cruise_speed
            if isinstance(path, CanonicalPath):
                rotation_rate_limit = (
                    self.limits.max_yaw_rate * path.rotation_weight
                )
            return min(rotation_rate_limit, stopping_limit)
        return max(
            0.0,
            min(self.limits.cruise_speed, stopping_limit, curvature_limit),
        )


def _path_total(path: PathModel) -> float:
    return float(path.total_length)


def _path_pose(
    path: PathModel,
    progress: float,
    heading_window: float = 0.15,
) -> Pose2D:
    if isinstance(path, CanonicalPath):
        if progress > path.total_progress:
            last = path.segments[-1]
            if last.translation > 1e-9:
                extra = progress - path.total_progress
                heading = math.atan2(
                    last.end.y - last.start.y,
                    last.end.x - last.start.x,
                )
                return Pose2D(
                    last.end.x + extra * math.cos(heading),
                    last.end.y + extra * math.sin(heading),
                    last.end.yaw,
                )
            return last.end
        return path.pose_at(progress)
    if progress > path.total_length:
        first = path.points[-2]
        last = path.points[-1]
        heading = math.atan2(last[1] - first[1], last[0] - first[0])
        extra = progress - path.total_length
        return Pose2D(
            last[0] + extra * math.cos(heading),
            last[1] + extra * math.sin(heading),
            heading,
        )
    point = path.point_at(progress)
    return Pose2D(point[0], point[1], path.tangent_yaw(progress))


def _project_path(path: PathModel, pose: Pose2D) -> _PathProjection:
    if isinstance(path, CanonicalPath):
        projection = path.project(
            pose.point,
            yaw=pose.yaw,
            min_progress=0.0,
            max_progress=path.total_progress,
        )
        return _PathProjection(
            projection.progress,
            projection.pose,
            projection.distance,
            projection.yaw_error,
        )
    projection = path.project(pose.point, 0.0)
    projected_pose = _path_pose(path, projection.arc_length)
    return _PathProjection(
        projection.arc_length,
        projected_pose,
        projection.distance,
        abs(wrap_to_pi(projected_pose.yaw - pose.yaw)),
    )


def _path_has_translation_at(path: PathModel, progress: float) -> bool:
    if isinstance(path, Polyline):
        return True
    target = max(0.0, min(float(progress), path.total_progress))
    for index, segment in enumerate(path.segments):
        if target <= path.cumulative[index + 1] or index == len(path.segments) - 1:
            return segment.translation > 1e-9
    return path.segments[-1].translation > 1e-9


def _path_is_pure_rotation(path: PathModel) -> bool:
    return isinstance(path, CanonicalPath) and all(
        segment.translation <= 1e-9 for segment in path.segments
    )


def _path_curvature(path: PathModel, progress: float, window: float) -> float:
    total = _path_total(path)
    start = max(0.0, progress - window)
    end = min(total, progress + window)
    if end - start <= 1e-6:
        return 0.0
    start_yaw = _path_pose(path, start, window).yaw
    end_yaw = _path_pose(path, end, window).yaw
    return abs(wrap_to_pi(end_yaw - start_yaw)) / (end - start)


def _transition_is_reachable(
    anchor: TimedWaypoint,
    target: TimedWaypoint,
    duration: float,
    limits: TrajectoryLimits,
    *,
    check_heading: bool = True,
) -> bool:
    """Check a new target's displacement and optional heading continuity."""
    if not math.isfinite(duration) or duration <= 0.0:
        return False
    if check_heading and (
        abs(wrap_to_pi(target.pose.yaw - anchor.pose.yaw))
        > limits.max_yaw_rate * duration + 1e-6
    ):
        return False
    displacement = distance(anchor.pose.point, target.pose.point)
    max_displacement = (
        anchor.speed * duration
        + 0.5 * limits.max_acceleration * duration * duration
    )
    if displacement > max_displacement + 1e-6:
        return False
    # Treat the committed command heading as the initial XY velocity
    # direction and solve the first suffix chord as a constant-acceleration
    # step: a_req = 2*(delta - v0*dt)/dt^2.  This catches lateral jumps and
    # reversals that a scalar distance bound cannot see.  The norm bound is
    # conservative; longitudinal braking retains the tighter deceleration
    # limit below.
    direction = (math.cos(anchor.pose.yaw), math.sin(anchor.pose.yaw))
    delta_x = target.pose.x - anchor.pose.x
    delta_y = target.pose.y - anchor.pose.y
    v0_x = anchor.speed * direction[0]
    v0_y = anchor.speed * direction[1]
    acceleration_x = 2.0 * (delta_x - v0_x * duration) / (duration * duration)
    acceleration_y = 2.0 * (delta_y - v0_y * duration) / (duration * duration)
    acceleration_norm = math.hypot(acceleration_x, acceleration_y)
    if acceleration_norm > limits.max_acceleration + 1e-6:
        return False
    longitudinal_acceleration = (
        acceleration_x * direction[0] + acceleration_y * direction[1]
    )
    if longitudinal_acceleration < -limits.max_deceleration - 1e-6:
        return False
    return True
