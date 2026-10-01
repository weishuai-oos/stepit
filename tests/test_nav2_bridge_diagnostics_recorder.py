#!/usr/bin/python3

import argparse
import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path as FilePath

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, String


SCRIPTS_DIR = FilePath(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from collect_nav2_bridge_diagnostics import (  # noqa: E402
    Nav2BridgeDiagnosticsRecorder,
)


def recorder_args() -> argparse.Namespace:
    return argparse.Namespace(
        sample_rate=20.0,
        bridge_node="/nav2_global_goal_to_waypoints",
        odom_topic="/test/odom",
        imu_topic="/test/imu",
        waypoints_topic="/test/waypoints",
        remain_time_topic="/test/remain_time",
        status_topic="/test/status",
        raw_path_topic="/test/raw_path",
        sampled_path_topic="/test/sampled_path",
        goal_topic="/test/goal",
        action_topic="/test/action",
        joint_state_topic="/test/joint_states",
        height_scan_topic="/test/height_scan",
        writer_queue_size=1000,
    )


class DiagnosticsRecorderTests(unittest.TestCase):
    def test_callbacks_produce_complete_machine_readable_run(self) -> None:
        initialized_here = False
        if not rclpy.ok():
            rclpy.init()
            initialized_here = True
        node = None
        try:
            with tempfile.TemporaryDirectory() as temporary_directory:
                output = FilePath(temporary_directory)
                node = Nav2BridgeDiagnosticsRecorder(recorder_args(), output)

                odom = Odometry()
                odom.pose.pose.orientation.w = 1.0
                odom.twist.twist.linear.x = 0.3
                node._on_odom(odom)

                imu = Imu()
                imu.orientation.w = 1.0
                imu.angular_velocity.z = 0.1
                node._on_imu(imu)
                node._on_waypoints(
                    Float32MultiArray(data=[0.1, 0.0, 0.0] * 5)
                )
                node._on_remain_time(
                    Float32MultiArray(data=[0.5, 1.0, 1.5, 2.0, 2.5])
                )
                node._on_status(String(data="smac_hybrid_xy_forward:ACTIVE"))

                raw_path = Path()
                raw_path.header.frame_id = "map"
                for x in (0.0, 1.0):
                    pose = PoseStamped()
                    pose.pose.position.x = x
                    pose.pose.orientation.w = 1.0
                    raw_path.poses.append(pose)
                node._on_raw_path(raw_path)

                sampled_path = Path()
                sampled_path.header.frame_id = "map"
                for index in range(5):
                    pose = PoseStamped()
                    pose.pose.position.x = 0.1 * (index + 1)
                    pose.pose.orientation.w = 1.0
                    sampled_path.poses.append(pose)
                node._on_sampled_path(sampled_path)

                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.pose.position.x = 1.0
                goal.pose.orientation.w = 1.0
                node._on_goal(goal)
                node._on_action(Float32MultiArray(data=[0.0, 0.1, -0.1]))
                node._on_action(Float32MultiArray(data=[0.1, 0.2, -0.1]))

                joints = JointState()
                joints.name = ["left_hip", "right_hip"]
                joints.position = [0.1, -0.1]
                joints.velocity = [0.2, -0.2]
                joints.effort = [1.0, 1.0]
                node._on_joint_states(joints)
                node._on_height_scan(Float32MultiArray(data=[0.0] * 187))
                node._write_sample()
                node.close()

                summary = json.loads((output / "summary.json").read_text())
                self.assertEqual(summary["warnings"], [])
                self.assertEqual(summary["goals"]["count"], 1)
                self.assertEqual(summary["raw_path"]["message_count"], 1)
                self.assertGreater(summary["sample_count"], 0)
                self.assertGreater(
                    summary["policy_action"]["consecutive_delta_l2"]["max"],
                    0.0,
                )
                with (output / "samples.csv").open(newline="") as samples_file:
                    rows = list(csv.DictReader(samples_file))
                self.assertGreater(len(rows), 0)
                self.assertEqual(rows[-1]["status"], "smac_hybrid_xy_forward:ACTIVE")
                self.assertEqual(rows[-1]["joint_count"], "2")
                self.assertEqual(
                    len((output / "actions.jsonl").read_text().splitlines()), 2
                )
                self.assertEqual(
                    len((output / "joint_states.jsonl").read_text().splitlines()),
                    1,
                )
                self.assertEqual(
                    len((output / "waypoints.jsonl").read_text().splitlines()),
                    1,
                )
                self.assertEqual(
                    len((output / "sampled_paths.jsonl").read_text().splitlines()),
                    1,
                )
                self.assertEqual(
                    len((output / "height_scan.jsonl").read_text().splitlines()),
                    1,
                )
        finally:
            if node is not None:
                node.destroy_node()
            if initialized_here and rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
