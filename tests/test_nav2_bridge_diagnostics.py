#!/usr/bin/python3

import json
import math
import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from nav2_bridge_diagnostics_common import (  # noqa: E402
    DiagnosticsMetrics,
    percentile,
    quaternion_to_rpy,
    render_summary,
    select_measurement_time,
)


class DiagnosticsHelperTests(unittest.TestCase):
    def test_quaternion_to_rpy_recovers_yaw(self) -> None:
        expected = 0.7
        roll, pitch, yaw = quaternion_to_rpy(
            0.0,
            0.0,
            math.sin(expected / 2.0),
            math.cos(expected / 2.0),
        )

        self.assertAlmostEqual(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0)
        self.assertAlmostEqual(yaw, expected)

    def test_percentile_interpolates_and_ignores_non_finite_values(self) -> None:
        self.assertAlmostEqual(percentile([0.0, 10.0, math.nan], 0.95), 9.5)
        self.assertIsNone(percentile([], 0.95))

    def test_measurement_time_falls_back_to_monotonic_receive_clock(self) -> None:
        self.assertEqual(select_measurement_time(12.5, 99.0), (12.5, "message_stamp"))
        self.assertEqual(
            select_measurement_time(None, 99.0),
            (99.0, "receive_monotonic_clock"),
        )

    def test_metrics_detect_waypoint_and_world_path_jumps(self) -> None:
        metrics = DiagnosticsMetrics()
        zero = [0.0, 0.0, 0.0] * 5
        changed = list(zero)
        changed[0] = 0.3
        sampled = list(zero)
        sampled[1] = 0.4
        metrics.observe_waypoints(zero)
        metrics.observe_sampled_waypoints(zero)
        metrics.observe_remain_times([0.5, 1.0, 1.5, 2.0, 2.5])
        metrics.observe_sample(
            status="smac_hybrid_xy_forward:ACTIVE",
            motion={"pose_speed_mps": 0.2},
            topic_ages={"odom": 0.01},
        )
        metrics.observe_waypoints(changed)
        metrics.observe_sampled_waypoints(sampled)
        metrics.observe_remain_times([0.4, 0.9, 1.4, 1.9, 2.4])
        metrics.observe_sample(
            status="smac_hybrid_xy_forward:ACTIVE",
            motion={"pose_speed_mps": 0.4},
            topic_ages={"odom": 0.02},
        )

        summary = metrics.summary(1.0)
        self.assertAlmostEqual(
            summary["continuity"]["body_waypoint_same_index_delta_m"]["max"], 0.3
        )
        self.assertAlmostEqual(
            summary["continuity"]["sampled_world_same_index_delta_m"]["max"], 0.4
        )
        self.assertAlmostEqual(summary["motion_absolute"]["pose_speed_mps"]["mean"], 0.3)
        self.assertAlmostEqual(summary["remain_time"]["interval_error_from_0_5_s"]["max"], 0.0)
        self.assertEqual(summary["remain_time"]["order_violation_sample_count"], 0)

    def test_metrics_flag_planning_after_goal_capture(self) -> None:
        metrics = DiagnosticsMetrics()
        metrics.observe_goal()
        metrics.count_topic("status")
        metrics.observe_status("smac_hybrid_xy_forward:GOAL_CAPTURE_BRAKING", 1.0)
        metrics.count_topic("status")
        metrics.observe_status("smac_hybrid_xy_forward:PLANNING", 1.2)

        summary = metrics.summary(2.0)
        self.assertEqual(summary["status"]["planning_entries"], 1)
        self.assertEqual(summary["status"]["planning_entries_after_goal_capture"], 1)
        self.assertAlmostEqual(summary["topic_message_rates_hz"]["status"], 1.0)

    def test_action_delta_and_raw_path_changes_are_summarized(self) -> None:
        metrics = DiagnosticsMetrics()
        metrics.observe_action([0.0, 0.0])
        metrics.observe_action([3.0, 4.0])
        metrics.observe_raw_path("a")
        metrics.observe_raw_path("a")
        metrics.observe_raw_path("b")

        summary = metrics.summary(1.0)
        self.assertAlmostEqual(summary["policy_action"]["consecutive_delta_l2"]["max"], 5.0)
        self.assertEqual(summary["raw_path"]["message_count"], 3)
        self.assertEqual(summary["raw_path"]["geometry_change_count"], 1)
        json.dumps(summary)
        self.assertIn("planning_entries", render_summary(summary))


if __name__ == "__main__":
    unittest.main()
