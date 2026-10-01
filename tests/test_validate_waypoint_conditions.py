#!/usr/bin/env python3

from __future__ import annotations

import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from validate_waypoint_conditions import Limits, validate_waypoints  # noqa: E402


def limits() -> Limits:
    return Limits(
        num_waypoints=5,
        expected_dt=0.5,
        time_tolerance=0.05,
        max_first_distance=0.5,
        hard_first_distance=1.0,
        max_horizon_distance=2.0,
        max_segment_speed=1.2,
        max_segment_acceleration=4.0,
        strict=False,
    )


class ValidateWaypointConditionsTests(unittest.TestCase):
    def test_counting_down_deadlines_match_the_timed_queue_contract(self) -> None:
        result = validate_waypoints(
            [0.0] * 15,
            [0.3, 0.8, 1.3, 1.8, 2.3],
            limits(),
        )

        time_issues = [
            issue.message
            for issue in result.issues
            if "remain_time" in issue.message or "deadline interval" in issue.message
        ]
        self.assertEqual(time_issues, [])

    def test_reanchored_or_irregular_deadlines_are_reported(self) -> None:
        result = validate_waypoints(
            [0.0] * 15,
            [0.8, 1.0, 1.5, 2.0, 2.5],
            limits(),
        )

        self.assertTrue(any(
            "exceeds one waypoint interval" in issue.message
            for issue in result.issues
        ))
        self.assertTrue(any(
            "deadline interval" in issue.message
            for issue in result.issues
        ))


if __name__ == "__main__":
    unittest.main()
