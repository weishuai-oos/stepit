#!/usr/bin/python3
"""One-command recorder for Nav2 bridge, policy, and robot motion diagnostics."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import queue
import re
import subprocess
import sys
import threading
import time
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path as FilePath
from typing import Any, Optional, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, String

from nav2_bridge_diagnostics_common import (
    DiagnosticsMetrics,
    quaternion_to_rpy,
    render_summary,
    select_measurement_time,
    wrap_to_pi,
)


POLICY_WAYPOINT_COUNT = 5
POLICY_WAYPOINT_VALUES = POLICY_WAYPOINT_COUNT * 3


@dataclass
class Latest:
    value: Any = None
    receive_monotonic_s: Optional[float] = None

    def update(self, value: Any, receive_monotonic_s: Optional[float] = None) -> None:
        self.value = value
        self.receive_monotonic_s = (
            time.monotonic() if receive_monotonic_s is None else receive_monotonic_s
        )

    def age(self, now_monotonic_s: float) -> Optional[float]:
        if self.receive_monotonic_s is None:
            return None
        return max(0.0, now_monotonic_s - self.receive_monotonic_s)


def stamp_to_seconds(stamp: Any) -> Optional[float]:
    seconds = int(stamp.sec) + int(stamp.nanosec) * 1e-9
    return seconds if seconds > 0.0 else None


def pose_to_record(pose: Any) -> dict[str, float]:
    orientation = pose.orientation
    roll, pitch, yaw = quaternion_to_rpy(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    return {
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "z": float(pose.position.z),
        "qx": float(orientation.x),
        "qy": float(orientation.y),
        "qz": float(orientation.z),
        "qw": float(orientation.w),
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
    }


def path_to_record(message: Path) -> dict[str, Any]:
    return {
        "frame_id": message.header.frame_id,
        "stamp_s": stamp_to_seconds(message.header.stamp),
        "poses": [pose_to_record(item.pose) for item in message.poses],
    }


def path_geometry_hash(record: dict[str, Any]) -> str:
    geometry = [
        [round(pose["x"], 5), round(pose["y"], 5), round(pose["yaw"], 5)]
        for pose in record["poses"]
    ]
    encoded = json.dumps([record["frame_id"], geometry], separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def create_output_directory(output_root: FilePath, label: str) -> FilePath:
    clean_label = re.sub(r"[^A-Za-z0-9._-]+", "_", label.strip()).strip("._-")
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    stem = f"{timestamp}_{clean_label}" if clean_label else timestamp
    candidate = output_root / stem
    suffix = 1
    while candidate.exists():
        candidate = output_root / f"{stem}_{suffix:02d}"
        suffix += 1
    candidate.mkdir(parents=True)
    return candidate.resolve()


def make_latched_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def sample_columns() -> list[str]:
    columns = [
        "wall_time",
        "ros_time_s",
        "elapsed_s",
        "status",
        "odom_stamp_s",
        "odom_time_source",
        "odom_message_dt_s",
        "odom_receive_dt_s",
        "odom_x_m",
        "odom_y_m",
        "odom_z_m",
        "odom_roll_rad",
        "odom_pitch_rad",
        "odom_yaw_rad",
        "pose_speed_mps",
        "pose_accel_mps2",
        "pose_yaw_rate_radps",
        "twist_vx_mps",
        "twist_vy_mps",
        "twist_vz_mps",
        "twist_planar_speed_mps",
        "twist_wx_radps",
        "twist_wy_radps",
        "twist_wz_radps",
        "imu_roll_rad",
        "imu_pitch_rad",
        "imu_yaw_rad",
        "imu_wx_radps",
        "imu_wy_radps",
        "imu_wz_radps",
        "imu_ax_mps2",
        "imu_ay_mps2",
        "imu_az_mps2",
        "action_size",
        "action_l2",
        "action_max_abs",
        "joint_count",
        "joint_velocity_l2",
        "joint_velocity_max_abs",
        "height_scan_size",
    ]
    for topic in (
        "odom",
        "waypoints",
        "remain_time",
        "sampled_path",
        "imu",
        "action",
        "joint_states",
        "height_scan",
    ):
        columns.append(f"{topic}_age_s")
    for index in range(1, POLICY_WAYPOINT_COUNT + 1):
        columns.extend(
            [
                f"waypoint_{index}_body_x_m",
                f"waypoint_{index}_body_y_m",
                f"waypoint_{index}_body_heading_rad",
                f"waypoint_{index}_remain_s",
                f"sampled_{index}_world_x_m",
                f"sampled_{index}_world_y_m",
                f"sampled_{index}_world_yaw_rad",
            ]
        )
    return columns


class AsyncRecordWriter:
    """Drain all high-rate recorder output on a dedicated bounded writer thread."""

    JSON_STREAMS = (
        "events",
        "raw_paths",
        "sampled_paths",
        "waypoints",
        "remain_time",
        "odometry",
        "imu",
        "actions",
        "joint_states",
        "height_scan",
    )

    def __init__(self, output_directory: FilePath, queue_size: int) -> None:
        self.queue: queue.Queue[Optional[tuple[str, dict[str, Any]]]] = queue.Queue(
            maxsize=queue_size
        )
        self.dropped_records: Counter[str] = Counter()
        self.errors: list[str] = []
        self.files = {
            name: (output_directory / f"{name}.jsonl").open(
                "w", encoding="utf-8", buffering=1
            )
            for name in self.JSON_STREAMS
        }
        self.samples_file = (output_directory / "samples.csv").open(
            "w", newline="", encoding="utf-8", buffering=1
        )
        self.samples_writer = csv.DictWriter(
            self.samples_file, fieldnames=sample_columns()
        )
        self.samples_writer.writeheader()
        self.thread = threading.Thread(
            target=self._run,
            name="nav2-diagnostics-writer",
            daemon=True,
        )
        self.thread.start()

    def _submit(self, stream: str, record: dict[str, Any]) -> None:
        try:
            self.queue.put_nowait((stream, record))
        except queue.Full:
            self.dropped_records[stream] += 1

    def submit_json(self, stream: str, record: dict[str, Any]) -> None:
        if stream not in self.files:
            raise ValueError(f"unknown JSONL stream: {stream}")
        self._submit(stream, record)

    def submit_sample(self, row: dict[str, Any]) -> None:
        self._submit("samples", row)

    def _run(self) -> None:
        while True:
            item = self.queue.get()
            try:
                if item is None:
                    return
                stream, record = item
                if stream == "samples":
                    self.samples_writer.writerow(record)
                else:
                    self.files[stream].write(
                        json.dumps(
                            record,
                            ensure_ascii=False,
                            separators=(",", ":"),
                        )
                        + "\n"
                    )
            except Exception as error:  # keep draining so shutdown cannot deadlock
                self.dropped_records["writer_error"] += 1
                if len(self.errors) < 10:
                    self.errors.append(f"{type(error).__name__}: {error}")
            finally:
                self.queue.task_done()

    def close(self) -> None:
        self.queue.join()
        self.queue.put(None)
        self.queue.join()
        self.thread.join(timeout=10.0)
        if self.thread.is_alive():
            self.errors.append("writer thread did not stop within 10 seconds")
        for output in self.files.values():
            output.close()
        self.samples_file.close()


class Nav2BridgeDiagnosticsRecorder(Node):
    def __init__(self, args: argparse.Namespace, output_directory: FilePath) -> None:
        super().__init__("nav2_bridge_diagnostics_recorder")
        self.args = args
        self.output_directory = output_directory
        self.start_monotonic_s = time.monotonic()
        self.metrics = DiagnosticsMetrics()
        self.closed = False
        self.final_warnings: list[str] = []
        self.metadata_lock = threading.Lock()
        self.parameter_capture_thread: Optional[threading.Thread] = None

        self.odom = Latest()
        self.imu = Latest()
        self.waypoints = Latest()
        self.remain_time = Latest()
        self.status = Latest("")
        self.sampled_path = Latest()
        self.action = Latest()
        self.joint_states = Latest()
        self.height_scan = Latest()
        self.previous_odom: Optional[
            tuple[float, float, float, float, float, str]
        ] = None
        self.previous_pose_speed: Optional[float] = None
        self.writer = AsyncRecordWriter(output_directory, args.writer_queue_size)

        self.metadata: dict[str, Any] = {
            "schema_version": 1,
            "start_time": datetime.now().astimezone().isoformat(),
            "hostname": platform.node(),
            "platform": platform.platform(),
            "python": sys.version,
            "argv": sys.argv,
            "sample_rate_hz": args.sample_rate,
            "output_directory": str(output_directory),
            "topics": {
                "odom": args.odom_topic,
                "imu": args.imu_topic,
                "waypoints": args.waypoints_topic,
                "remain_time": args.remain_time_topic,
                "status": args.status_topic,
                "raw_path": args.raw_path_topic,
                "sampled_path": args.sampled_path_topic,
                "goal": args.goal_topic,
                "action": args.action_topic,
                "joint_states": args.joint_state_topic,
                "height_scan": args.height_scan_topic,
            },
            "writer_queue_size": args.writer_queue_size,
        }
        self._write_metadata()

        latched_qos = make_latched_qos()
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, args.imu_topic, self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, args.waypoints_topic, self._on_waypoints, 50)
        self.create_subscription(Float32MultiArray, args.remain_time_topic, self._on_remain_time, 50)
        self.create_subscription(String, args.status_topic, self._on_status, latched_qos)
        self.create_subscription(Path, args.raw_path_topic, self._on_raw_path, latched_qos)
        self.create_subscription(Path, args.sampled_path_topic, self._on_sampled_path, 50)
        self.create_subscription(PoseStamped, args.goal_topic, self._on_goal, 10)
        self.create_subscription(Float32MultiArray, args.action_topic, self._on_action, 50)
        self.create_subscription(
            JointState,
            args.joint_state_topic,
            self._on_joint_states,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32MultiArray,
            args.height_scan_topic,
            self._on_height_scan,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / args.sample_rate, self._write_sample)
        with self.metadata_lock:
            self.metadata["recording_ready_time"] = (
                datetime.now().astimezone().isoformat()
            )
            self.metadata["recording_ready_elapsed_s"] = self._elapsed()
        self._write_metadata()

    def _elapsed(self) -> float:
        return time.monotonic() - self.start_monotonic_s

    def _ros_time(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _write_metadata(self) -> None:
        with self.metadata_lock:
            contents = json.dumps(self.metadata, ensure_ascii=False, indent=2) + "\n"
        (self.output_directory / "metadata.json").write_text(
            contents,
            encoding="utf-8",
        )

    def _write_event(self, event: str, **payload: Any) -> None:
        self.writer.submit_json(
            "events",
            {
                "wall_time": datetime.now().astimezone().isoformat(),
                "ros_time_s": self._ros_time(),
                "elapsed_s": self._elapsed(),
                "event": event,
                **payload,
            },
        )

    def _on_odom(self, message: Odometry) -> None:
        self.metrics.count_topic("odom")
        receive_monotonic_s = time.monotonic()
        receive_ros_s = self._ros_time()
        message_stamp_s = stamp_to_seconds(message.header.stamp)
        measurement_time_s, time_source = select_measurement_time(
            message_stamp_s, receive_monotonic_s
        )
        pose = pose_to_record(message.pose.pose)
        pose_speed = None
        pose_acceleration = None
        pose_yaw_rate = None
        message_delta_s = None
        receive_delta_s = None
        if self.previous_odom is not None:
            old_time, old_receive, old_x, old_y, old_yaw, old_time_source = (
                self.previous_odom
            )
            message_delta_s = measurement_time_s - old_time
            receive_delta_s = receive_monotonic_s - old_receive
            if old_time_source == time_source and 1e-4 < message_delta_s < 2.0:
                pose_speed = (
                    math.hypot(pose["x"] - old_x, pose["y"] - old_y)
                    / message_delta_s
                )
                pose_yaw_rate = (
                    wrap_to_pi(pose["yaw"] - old_yaw) / message_delta_s
                )
                if self.previous_pose_speed is not None:
                    pose_acceleration = (
                        pose_speed - self.previous_pose_speed
                    ) / message_delta_s
                self.previous_pose_speed = pose_speed
            else:
                self.previous_pose_speed = None
                self._write_event(
                    "odom_time_discontinuity",
                    time_source=time_source,
                    previous_time_source=old_time_source,
                    message_delta_s=message_delta_s,
                    receive_delta_s=receive_delta_s,
                )
        self.previous_odom = (
            measurement_time_s,
            receive_monotonic_s,
            pose["x"],
            pose["y"],
            pose["yaw"],
            time_source,
        )

        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        record = {
            "stamp_s": message_stamp_s,
            "time_source": time_source,
            "message_delta_s": message_delta_s,
            "receive_delta_s": receive_delta_s,
            **pose,
            "pose_speed": pose_speed,
            "pose_acceleration": pose_acceleration,
            "pose_yaw_rate": pose_yaw_rate,
            "twist_vx": float(linear.x),
            "twist_vy": float(linear.y),
            "twist_vz": float(linear.z),
            "twist_planar_speed": math.hypot(linear.x, linear.y),
            "twist_wx": float(angular.x),
            "twist_wy": float(angular.y),
            "twist_wz": float(angular.z),
        }
        self.odom.update(record, receive_monotonic_s)
        self.writer.submit_json(
            "odometry",
            {
                "ros_time_s": receive_ros_s,
                "elapsed_s": self._elapsed(),
                "sequence": self.metrics.topic_counts["odom"],
                **record,
            },
        )

    def _on_imu(self, message: Imu) -> None:
        self.metrics.count_topic("imu")
        receive_monotonic_s = time.monotonic()
        orientation = message.orientation
        roll, pitch, yaw = quaternion_to_rpy(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        record = {
            "stamp_s": stamp_to_seconds(message.header.stamp),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "wx": float(message.angular_velocity.x),
            "wy": float(message.angular_velocity.y),
            "wz": float(message.angular_velocity.z),
            "ax": float(message.linear_acceleration.x),
            "ay": float(message.linear_acceleration.y),
            "az": float(message.linear_acceleration.z),
        }
        self.imu.update(record, receive_monotonic_s)
        self.writer.submit_json(
            "imu",
            {
                "ros_time_s": self._ros_time(),
                "elapsed_s": self._elapsed(),
                "sequence": self.metrics.topic_counts["imu"],
                **record,
            },
        )

    def _on_waypoints(self, message: Float32MultiArray) -> None:
        self.metrics.count_topic("waypoints")
        values = [float(value) for value in message.data]
        if len(values) != POLICY_WAYPOINT_VALUES or not all(math.isfinite(value) for value in values):
            self._write_event("invalid_waypoints", length=len(values), values=values)
            self.waypoints.update(None)
            return
        self.waypoints.update(values)
        self.metrics.observe_waypoints(values)
        self.writer.submit_json(
            "waypoints",
            {
                "ros_time_s": self._ros_time(),
                "elapsed_s": self._elapsed(),
                "sequence": self.metrics.topic_counts["waypoints"],
                "data": values,
            },
        )

    def _on_remain_time(self, message: Float32MultiArray) -> None:
        self.metrics.count_topic("remain_time")
        values = [float(value) for value in message.data]
        if len(values) != POLICY_WAYPOINT_COUNT or not all(math.isfinite(value) for value in values):
            self._write_event("invalid_remain_time", length=len(values), values=values)
            self.remain_time.update(None)
            return
        self.remain_time.update(values)
        self.metrics.observe_remain_times(values)
        self.writer.submit_json(
            "remain_time",
            {
                "ros_time_s": self._ros_time(),
                "elapsed_s": self._elapsed(),
                "sequence": self.metrics.topic_counts["remain_time"],
                "data": values,
            },
        )

    def _on_status(self, message: String) -> None:
        self.metrics.count_topic("status")
        if message.data == self.status.value:
            self.status.update(message.data)
            return
        self.status.update(message.data)
        self.metrics.observe_status(message.data, self._elapsed())
        self._write_event("status_transition", status=message.data)

    def _on_raw_path(self, message: Path) -> None:
        self.metrics.count_topic("raw_path")
        record = path_to_record(message)
        geometry_hash = path_geometry_hash(record)
        self.metrics.observe_raw_path(geometry_hash)
        self.writer.submit_json(
            "raw_paths",
            {
                "wall_time": datetime.now().astimezone().isoformat(),
                "ros_time_s": self._ros_time(),
                "elapsed_s": self._elapsed(),
                "geometry_hash": geometry_hash,
                **record,
            },
        )

    def _on_sampled_path(self, message: Path) -> None:
        self.metrics.count_topic("sampled_path")
        path_record = path_to_record(message)
        values: list[float] = []
        for pose in path_record["poses"]:
            values.extend([pose["x"], pose["y"], pose["yaw"]])
        self.writer.submit_json(
            "sampled_paths",
            {
                "ros_time_s": self._ros_time(),
                "elapsed_s": self._elapsed(),
                "sequence": self.metrics.topic_counts["sampled_path"],
                **path_record,
            },
        )
        if len(values) != POLICY_WAYPOINT_VALUES:
            self.sampled_path.update(None)
            self._write_event("invalid_sampled_path", pose_count=len(message.poses))
            return
        self.sampled_path.update(values)
        self.metrics.observe_sampled_waypoints(values)

    def _on_goal(self, message: PoseStamped) -> None:
        self.metrics.count_topic("goal")
        self.metrics.observe_goal()
        self._write_event(
            "goal",
            frame_id=message.header.frame_id,
            stamp_s=stamp_to_seconds(message.header.stamp),
            pose=pose_to_record(message.pose),
        )

    def _on_action(self, message: Float32MultiArray) -> None:
        self.metrics.count_topic("action")
        values = [float(value) for value in message.data]
        if not values or not all(math.isfinite(value) for value in values):
            self._write_event("invalid_action", length=len(values))
            self.action.update(None)
            return
        norm = math.sqrt(sum(value * value for value in values))
        maximum = max(abs(value) for value in values)
        self.action.update({"values": values, "l2": norm, "max_abs": maximum})
        self.metrics.observe_action(values)
        self.writer.submit_json(
            "actions",
            {
                "ros_time_s": self._ros_time(),
                "elapsed_s": self._elapsed(),
                "sequence": self.metrics.topic_counts["action"],
                "data": values,
            },
        )

    def _on_joint_states(self, message: JointState) -> None:
        self.metrics.count_topic("joint_states")
        positions = [float(value) for value in message.position]
        velocities = [float(value) for value in message.velocity]
        efforts = [float(value) for value in message.effort]
        velocity_l2 = (
            math.sqrt(sum(value * value for value in velocities)) if velocities else None
        )
        velocity_maximum = max((abs(value) for value in velocities), default=None)
        self.joint_states.update(
            {
                "count": len(message.name),
                "velocity_l2": velocity_l2,
                "velocity_max_abs": velocity_maximum,
            }
        )
        self.writer.submit_json(
            "joint_states",
            {
                "ros_time_s": self._ros_time(),
                "elapsed_s": self._elapsed(),
                "sequence": self.metrics.topic_counts["joint_states"],
                "stamp_s": stamp_to_seconds(message.header.stamp),
                "name": list(message.name),
                "position": positions,
                "velocity": velocities,
                "effort": efforts,
            },
        )

    def _on_height_scan(self, message: Float32MultiArray) -> None:
        self.metrics.count_topic("height_scan")
        values = [float(value) for value in message.data]
        valid = bool(values) and all(math.isfinite(value) for value in values)
        self.height_scan.update({"size": len(values)} if valid else None)
        self.writer.submit_json(
            "height_scan",
            {
                "ros_time_s": self._ros_time(),
                "elapsed_s": self._elapsed(),
                "sequence": self.metrics.topic_counts["height_scan"],
                "data": values,
            },
        )
        if not valid or len(values) != 187:
            self._write_event("unexpected_height_scan", length=len(values))

    def _write_sample(self) -> None:
        if self.closed:
            return
        now_monotonic_s = time.monotonic()
        odom = self.odom.value or {}
        imu = self.imu.value or {}
        action = self.action.value or {}
        joint_states = self.joint_states.value or {}
        height_scan = self.height_scan.value or {}
        waypoints = self.waypoints.value
        remain_time = self.remain_time.value
        sampled = self.sampled_path.value
        ages = {
            "odom": self.odom.age(now_monotonic_s),
            "waypoints": self.waypoints.age(now_monotonic_s),
            "remain_time": self.remain_time.age(now_monotonic_s),
            "sampled_path": self.sampled_path.age(now_monotonic_s),
            "imu": self.imu.age(now_monotonic_s),
            "action": self.action.age(now_monotonic_s),
            "joint_states": self.joint_states.age(now_monotonic_s),
            "height_scan": self.height_scan.age(now_monotonic_s),
        }
        motion = {
            "pose_speed_mps": odom.get("pose_speed"),
            "pose_accel_mps2": odom.get("pose_acceleration"),
            "pose_yaw_rate_radps": odom.get("pose_yaw_rate"),
            "twist_planar_speed_mps": odom.get("twist_planar_speed"),
            "twist_wz_radps": odom.get("twist_wz"),
            "odom_roll_rad": odom.get("roll"),
            "odom_pitch_rad": odom.get("pitch"),
            "imu_roll_rad": imu.get("roll"),
            "imu_pitch_rad": imu.get("pitch"),
            "imu_wx_radps": imu.get("wx"),
            "imu_wy_radps": imu.get("wy"),
            "imu_wz_radps": imu.get("wz"),
            "joint_velocity_l2": joint_states.get("velocity_l2"),
            "joint_velocity_max_abs": joint_states.get("velocity_max_abs"),
        }
        self.metrics.observe_sample(
            status=self.status.value or "",
            motion=motion,
            topic_ages=ages,
        )
        row: dict[str, Any] = {
            "wall_time": datetime.now().astimezone().isoformat(),
            "ros_time_s": self._ros_time(),
            "elapsed_s": self._elapsed(),
            "status": self.status.value or "",
            "odom_stamp_s": odom.get("stamp_s"),
            "odom_time_source": odom.get("time_source"),
            "odom_message_dt_s": odom.get("message_delta_s"),
            "odom_receive_dt_s": odom.get("receive_delta_s"),
            "odom_x_m": odom.get("x"),
            "odom_y_m": odom.get("y"),
            "odom_z_m": odom.get("z"),
            "odom_roll_rad": odom.get("roll"),
            "odom_pitch_rad": odom.get("pitch"),
            "odom_yaw_rad": odom.get("yaw"),
            "pose_speed_mps": odom.get("pose_speed"),
            "pose_accel_mps2": odom.get("pose_acceleration"),
            "pose_yaw_rate_radps": odom.get("pose_yaw_rate"),
            "twist_vx_mps": odom.get("twist_vx"),
            "twist_vy_mps": odom.get("twist_vy"),
            "twist_vz_mps": odom.get("twist_vz"),
            "twist_planar_speed_mps": odom.get("twist_planar_speed"),
            "twist_wx_radps": odom.get("twist_wx"),
            "twist_wy_radps": odom.get("twist_wy"),
            "twist_wz_radps": odom.get("twist_wz"),
            "imu_roll_rad": imu.get("roll"),
            "imu_pitch_rad": imu.get("pitch"),
            "imu_yaw_rad": imu.get("yaw"),
            "imu_wx_radps": imu.get("wx"),
            "imu_wy_radps": imu.get("wy"),
            "imu_wz_radps": imu.get("wz"),
            "imu_ax_mps2": imu.get("ax"),
            "imu_ay_mps2": imu.get("ay"),
            "imu_az_mps2": imu.get("az"),
            "action_size": len(action.get("values", [])),
            "action_l2": action.get("l2"),
            "action_max_abs": action.get("max_abs"),
            "joint_count": joint_states.get("count", 0),
            "joint_velocity_l2": joint_states.get("velocity_l2"),
            "joint_velocity_max_abs": joint_states.get("velocity_max_abs"),
            "height_scan_size": height_scan.get("size", 0),
        }
        for topic, age in ages.items():
            row[f"{topic}_age_s"] = age
        for index in range(POLICY_WAYPOINT_COUNT):
            number = index + 1
            offset = index * 3
            if waypoints is not None:
                row[f"waypoint_{number}_body_x_m"] = waypoints[offset]
                row[f"waypoint_{number}_body_y_m"] = waypoints[offset + 1]
                row[f"waypoint_{number}_body_heading_rad"] = waypoints[offset + 2]
            if remain_time is not None:
                row[f"waypoint_{number}_remain_s"] = remain_time[index]
            if sampled is not None:
                row[f"sampled_{number}_world_x_m"] = sampled[offset]
                row[f"sampled_{number}_world_y_m"] = sampled[offset + 1]
                row[f"sampled_{number}_world_yaw_rad"] = sampled[offset + 2]
        self.writer.submit_sample(row)

    def capture_bridge_parameters(self) -> None:
        target = self.output_directory / "bridge_parameters.yaml"
        with self.metadata_lock:
            self.metadata["parameter_dump_start_time"] = (
                datetime.now().astimezone().isoformat()
            )
        try:
            result = subprocess.run(
                ["ros2", "param", "dump", self.args.bridge_node],
                check=False,
                capture_output=True,
                text=True,
                timeout=8.0,
            )
            target.write_text(result.stdout, encoding="utf-8")
            if result.returncode != 0:
                with self.metadata_lock:
                    self.metadata["bridge_parameter_dump_error"] = (
                        result.stderr.strip()
                    )
        except (FileNotFoundError, subprocess.TimeoutExpired) as error:
            with self.metadata_lock:
                self.metadata["bridge_parameter_dump_error"] = str(error)
        with self.metadata_lock:
            self.metadata["parameter_dump_end_time"] = (
                datetime.now().astimezone().isoformat()
            )
        self._write_metadata()

    def start_bridge_parameter_capture(self) -> None:
        self.parameter_capture_thread = threading.Thread(
            target=self.capture_bridge_parameters,
            name="nav2-diagnostics-parameter-dump",
            daemon=True,
        )
        self.parameter_capture_thread.start()

    def close(self) -> None:
        if self.closed:
            return
        self.timer.cancel()
        if rclpy.ok():
            self._write_sample()
        self.closed = True
        self.writer.close()
        if self.parameter_capture_thread is not None:
            self.parameter_capture_thread.join(timeout=9.0)
            if self.parameter_capture_thread.is_alive():
                with self.metadata_lock:
                    self.metadata["bridge_parameter_dump_error"] = (
                        "parameter dump thread did not stop within 9 seconds"
                    )
        duration_s = self._elapsed()
        summary = self.metrics.summary(duration_s)
        expected_topics = (
            "odom",
            "imu",
            "waypoints",
            "remain_time",
            "status",
            "sampled_path",
            "action",
            "joint_states",
            "height_scan",
        )
        missing_topics = [
            name for name in expected_topics if self.metrics.topic_counts[name] == 0
        ]
        if missing_topics:
            self.final_warnings.append(
                "no messages received from required topics: " + ", ".join(missing_topics)
            )
        with self.metadata_lock:
            parameter_error = self.metadata.get("bridge_parameter_dump_error")
        if parameter_error:
            self.final_warnings.append(
                f"bridge parameter snapshot failed: {parameter_error}"
            )
        if self.writer.dropped_records:
            self.final_warnings.append(
                "writer dropped records: "
                + ", ".join(
                    f"{name}={count}"
                    for name, count in sorted(self.writer.dropped_records.items())
                )
            )
        if self.writer.errors:
            self.final_warnings.append(
                "writer errors: " + " | ".join(self.writer.errors)
            )
        summary["writer"] = {
            "dropped_records": dict(sorted(self.writer.dropped_records.items())),
            "errors": self.writer.errors,
        }
        summary["warnings"] = self.final_warnings
        (self.output_directory / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        (self.output_directory / "summary.txt").write_text(
            render_summary(summary), encoding="utf-8"
        )
        with self.metadata_lock:
            self.metadata.update(
                {
                    "end_time": datetime.now().astimezone().isoformat(),
                    "duration_s": duration_s,
                    "completed": True,
                    "writer_dropped_records": dict(
                        sorted(self.writer.dropped_records.items())
                    ),
                    "writer_errors": self.writer.errors,
                }
            )
        self._write_metadata()


def parse_args() -> argparse.Namespace:
    repository_root = FilePath(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Record synchronized Nav2 bridge, policy action, odometry, and IMU diagnostics."
    )
    parser.add_argument(
        "--output-root",
        type=FilePath,
        default=repository_root / "diagnostics" / "nav2_bridge_runs",
    )
    parser.add_argument("--label", default="", help="Optional short label appended to the run directory.")
    parser.add_argument("--duration", type=float, help="Stop automatically after this many seconds.")
    parser.add_argument("--sample-rate", type=float, default=20.0)
    parser.add_argument("--bridge-node", default="/nav2_global_goal_to_waypoints")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--imu-topic", default="/stepit/imu")
    parser.add_argument("--waypoints-topic", default="/waypoints_b")
    parser.add_argument("--remain-time-topic", default="/remain_time")
    parser.add_argument("--status-topic", default="/nav2_waypoint_status")
    parser.add_argument("--raw-path-topic", default="/nav2_global_path")
    parser.add_argument("--sampled-path-topic", default="/nav2_sampled_waypoints_path")
    parser.add_argument("--goal-topic", default="/goal_pose")
    parser.add_argument("--action-topic", default="/stepit/field/action")
    parser.add_argument("--joint-state-topic", default="/stepit/joint_states")
    parser.add_argument("--height-scan-topic", default="/height_scan_array")
    parser.add_argument("--writer-queue-size", type=int, default=20000)
    args = parser.parse_args()
    if not math.isfinite(args.sample_rate) or args.sample_rate <= 0.0:
        parser.error("--sample-rate must be finite and positive")
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0.0):
        parser.error("--duration must be finite and positive")
    if args.writer_queue_size <= 0:
        parser.error("--writer-queue-size must be positive")
    return args


def main() -> int:
    args = parse_args()
    output_directory = create_output_directory(args.output_root.expanduser(), args.label)
    print(f"[diagnostics] recording to: {output_directory}", flush=True)
    print("[diagnostics] reproduce the issue, then press Ctrl-C to finalize.", flush=True)
    rclpy.init()
    node = Nav2BridgeDiagnosticsRecorder(args, output_directory)
    node.start_bridge_parameter_capture()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.duration is not None and node._elapsed() >= args.duration:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"[diagnostics] complete: {output_directory}", flush=True)
    for warning in node.final_warnings:
        print(f"[diagnostics] WARNING: {warning}", flush=True)
    print(f"[diagnostics] share this directory with Codex: {output_directory}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
