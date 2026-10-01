#!/usr/bin/python3
"""Pure helpers for Nav2-to-policy bridge diagnostics."""

from __future__ import annotations

import math
from collections import Counter, defaultdict
from typing import Mapping, Optional, Sequence


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Return intrinsic XYZ roll, pitch and yaw for a quaternion."""
    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)

    sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
    return roll, pitch, yaw


def select_measurement_time(
    message_stamp_s: Optional[float], receive_monotonic_s: float
) -> tuple[float, str]:
    """Prefer a valid message stamp and otherwise use a monotonic receive clock."""
    if message_stamp_s is not None and math.isfinite(message_stamp_s) and message_stamp_s > 0.0:
        return float(message_stamp_s), "message_stamp"
    return float(receive_monotonic_s), "receive_monotonic_clock"


def percentile(values: Sequence[float], fraction: float) -> Optional[float]:
    finite = sorted(float(value) for value in values if math.isfinite(value))
    if not finite:
        return None
    if not 0.0 <= fraction <= 1.0:
        raise ValueError("fraction must be in [0, 1]")
    position = fraction * (len(finite) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return finite[lower]
    weight = position - lower
    return finite[lower] * (1.0 - weight) + finite[upper] * weight


def metric_summary(values: Sequence[float]) -> dict[str, Optional[float] | int]:
    finite = [float(value) for value in values if math.isfinite(value)]
    if not finite:
        return {"count": 0, "mean": None, "p95": None, "max": None}
    return {
        "count": len(finite),
        "mean": sum(finite) / len(finite),
        "p95": percentile(finite, 0.95),
        "max": max(finite),
    }


def _state_name(status: str) -> str:
    details = status.split(":", 1)[1] if ":" in status else status
    return details.split(":", 1)[0].strip().split(" ", 1)[0]


def _indexed_jumps(
    previous: Optional[Sequence[float]],
    current: Optional[Sequence[float]],
) -> tuple[list[float], list[float]]:
    if previous is None or current is None or len(previous) != len(current) or len(current) % 3:
        return [], []
    xy_jumps: list[float] = []
    heading_jumps: list[float] = []
    for offset in range(0, len(current), 3):
        xy_jumps.append(
            math.hypot(current[offset] - previous[offset], current[offset + 1] - previous[offset + 1])
        )
        heading_jumps.append(abs(wrap_to_pi(current[offset + 2] - previous[offset + 2])))
    return xy_jumps, heading_jumps


class DiagnosticsMetrics:
    """Collect compact metrics while the recorder writes full-fidelity data to disk."""

    def __init__(self) -> None:
        self.sample_count = 0
        self.topic_counts: Counter[str] = Counter()
        self.status_entries: Counter[str] = Counter()
        self.status_samples: Counter[str] = Counter()
        self.status_timeline: list[dict[str, object]] = []
        self.replan_entries = 0
        self.replans_after_goal_capture = 0
        self.goal_count = 0
        self.capture_active = False

        self.motion: dict[str, list[float]] = defaultdict(list)
        self.topic_ages: dict[str, list[float]] = defaultdict(list)
        self.waypoint_xy_jumps: list[float] = []
        self.waypoint_heading_jumps: list[float] = []
        self.sampled_xy_jumps: list[float] = []
        self.sampled_heading_jumps: list[float] = []
        self.action_norms: list[float] = []
        self.action_max_abs: list[float] = []
        self.action_delta_norms: list[float] = []
        self.previous_waypoints: Optional[list[float]] = None
        self.previous_sampled: Optional[list[float]] = None
        self.previous_action: Optional[list[float]] = None
        self.remain_interval_errors: list[float] = []
        self.remain_negative_values = 0
        self.remain_order_violation_samples = 0
        self.raw_path_messages = 0
        self.raw_path_geometry_changes = 0
        self.previous_raw_path_hash: Optional[str] = None

    def count_topic(self, name: str) -> None:
        self.topic_counts[name] += 1

    def observe_goal(self) -> None:
        self.goal_count += 1
        self.capture_active = False

    def observe_status(self, status: str, elapsed_s: float) -> None:
        state = _state_name(status)
        self.status_entries[state] += 1
        self.status_timeline.append({"elapsed_s": elapsed_s, "status": status, "state": state})
        if state.startswith("GOAL_CAPTURE_"):
            self.capture_active = True
        elif state == "GOAL_REACHED":
            self.capture_active = False
        if "PLANNING" in state:
            self.replan_entries += 1
            if self.capture_active:
                self.replans_after_goal_capture += 1

    def observe_raw_path(self, geometry_hash: str) -> None:
        self.raw_path_messages += 1
        if self.previous_raw_path_hash is not None and geometry_hash != self.previous_raw_path_hash:
            self.raw_path_geometry_changes += 1
        self.previous_raw_path_hash = geometry_hash

    def observe_action(self, action: Sequence[float]) -> None:
        values = [float(value) for value in action]
        if not values or not all(math.isfinite(value) for value in values):
            return
        self.action_norms.append(math.sqrt(sum(value * value for value in values)))
        self.action_max_abs.append(max(abs(value) for value in values))
        if self.previous_action is not None and len(values) == len(self.previous_action):
            self.action_delta_norms.append(
                math.sqrt(sum((value - old) ** 2 for value, old in zip(values, self.previous_action)))
            )
        self.previous_action = values

    def observe_waypoints(self, waypoints: Sequence[float]) -> None:
        values = [float(value) for value in waypoints]
        xy_deltas, heading_deltas = _indexed_jumps(self.previous_waypoints, values)
        self.waypoint_xy_jumps.extend(xy_deltas)
        self.waypoint_heading_jumps.extend(heading_deltas)
        self.previous_waypoints = values

    def observe_sampled_waypoints(self, waypoints: Sequence[float]) -> None:
        values = [float(value) for value in waypoints]
        xy_deltas, heading_deltas = _indexed_jumps(self.previous_sampled, values)
        self.sampled_xy_jumps.extend(xy_deltas)
        self.sampled_heading_jumps.extend(heading_deltas)
        self.previous_sampled = values

    def observe_remain_times(self, remain_times: Sequence[float]) -> None:
        values = [float(value) for value in remain_times]
        self.remain_negative_values += sum(value < -1e-6 for value in values)
        if any(new <= old for old, new in zip(values, values[1:])):
            self.remain_order_violation_samples += 1
        self.remain_interval_errors.extend(
            abs((new - old) - 0.5) for old, new in zip(values, values[1:])
        )

    def observe_sample(
        self,
        *,
        status: str,
        motion: Mapping[str, Optional[float]],
        topic_ages: Mapping[str, Optional[float]],
    ) -> None:
        self.sample_count += 1
        if status:
            self.status_samples[_state_name(status)] += 1
        for name, value in motion.items():
            if value is not None and math.isfinite(value):
                self.motion[name].append(abs(float(value)))
        for name, value in topic_ages.items():
            if value is not None and math.isfinite(value):
                self.topic_ages[name].append(max(0.0, float(value)))

    def summary(self, duration_s: float) -> dict[str, object]:
        return {
            "schema_version": 1,
            "duration_s": max(0.0, duration_s),
            "sample_count": self.sample_count,
            "topic_message_counts": dict(sorted(self.topic_counts.items())),
            "topic_message_rates_hz": {
                name: (count / duration_s if duration_s > 0.0 else None)
                for name, count in sorted(self.topic_counts.items())
            },
            "goals": {"count": self.goal_count},
            "status": {
                "transition_count": len(self.status_timeline),
                "state_entry_counts": dict(sorted(self.status_entries.items())),
                "sampled_state_counts": dict(sorted(self.status_samples.items())),
                "planning_entries": self.replan_entries,
                "planning_entries_after_goal_capture": self.replans_after_goal_capture,
                "timeline": self.status_timeline,
            },
            "motion_absolute": {
                name: metric_summary(values) for name, values in sorted(self.motion.items())
            },
            "continuity": {
                "body_waypoint_same_index_delta_m": metric_summary(self.waypoint_xy_jumps),
                "body_waypoint_same_index_heading_delta_rad": metric_summary(
                    self.waypoint_heading_jumps
                ),
                "sampled_world_same_index_delta_m": metric_summary(self.sampled_xy_jumps),
                "sampled_world_same_index_heading_delta_rad": metric_summary(
                    self.sampled_heading_jumps
                ),
            },
            "continuity_interpretation": (
                "Deltas compare the same horizon index in consecutive full-rate messages. "
                "They include normal rolling-horizon and body-frame motion and are not, by "
                "themselves, proof of a command discontinuity. Correlate them with status, "
                "raw-path geometry hashes, robot motion, and the full JSONL streams."
            ),
            "remain_time": {
                "interval_error_from_0_5_s": metric_summary(self.remain_interval_errors),
                "negative_value_count": self.remain_negative_values,
                "order_violation_sample_count": self.remain_order_violation_samples,
            },
            "policy_action": {
                "l2_norm": metric_summary(self.action_norms),
                "max_abs": metric_summary(self.action_max_abs),
                "consecutive_delta_l2": metric_summary(self.action_delta_norms),
            },
            "raw_path": {
                "message_count": self.raw_path_messages,
                "geometry_change_count": self.raw_path_geometry_changes,
            },
            "topic_age_s": {
                name: metric_summary(values) for name, values in sorted(self.topic_ages.items())
            },
        }


def render_summary(summary: Mapping[str, object]) -> str:
    def metric(path: Sequence[str]) -> Mapping[str, object]:
        value: object = summary
        for key in path:
            value = value[key]  # type: ignore[index]
        return value  # type: ignore[return-value]

    def number(value: object, digits: int = 3) -> str:
        return "n/a" if value is None else f"{float(value):.{digits}f}"

    status = summary["status"]  # type: ignore[index]
    raw_path = summary["raw_path"]  # type: ignore[index]
    lines = [
        "Nav2 bridge diagnostics summary",
        f"duration_s: {number(summary['duration_s'])}",
        f"sample_count: {summary['sample_count']}",
        f"goal_count: {summary['goals']['count']}",  # type: ignore[index]
        f"status_transition_count: {status['transition_count']}",  # type: ignore[index]
        f"planning_entries: {status['planning_entries']}",  # type: ignore[index]
        f"planning_entries_after_goal_capture: {status['planning_entries_after_goal_capture']}",  # type: ignore[index]
        f"raw_path_messages: {raw_path['message_count']}",  # type: ignore[index]
        f"raw_path_geometry_changes: {raw_path['geometry_change_count']}",  # type: ignore[index]
        f"topic_message_rates_hz: {summary['topic_message_rates_hz']}",
        "",
        "motion_absolute (mean / p95 / max)",
    ]
    motion = summary["motion_absolute"]  # type: ignore[index]
    for name in sorted(motion):  # type: ignore[arg-type]
        values = motion[name]  # type: ignore[index]
        lines.append(
            f"{name}: {number(values['mean'])} / {number(values['p95'])} / {number(values['max'])}"
        )
    lines.extend(["", "continuity (mean / p95 / max)"])
    continuity = summary["continuity"]  # type: ignore[index]
    for name in sorted(continuity):  # type: ignore[arg-type]
        values = continuity[name]  # type: ignore[index]
        lines.append(
            f"{name}: {number(values['mean'])} / {number(values['p95'])} / {number(values['max'])}"
        )
    lines.extend(["", "policy_action (mean / p95 / max)"])
    actions = summary["policy_action"]  # type: ignore[index]
    for name in sorted(actions):  # type: ignore[arg-type]
        values = actions[name]  # type: ignore[index]
        lines.append(
            f"{name}: {number(values['mean'])} / {number(values['p95'])} / {number(values['max'])}"
        )
    lines.extend(["", f"state_entry_counts: {status['state_entry_counts']}"])  # type: ignore[index]
    warnings = summary.get("warnings", [])
    if warnings:
        lines.extend(["", "warnings"])
        lines.extend(f"- {warning}" for warning in warnings)  # type: ignore[union-attr]
    return "\n".join(lines) + "\n"
