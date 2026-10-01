import json
import importlib.util
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "publish_traversable_terrain_markers.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("terrain_markers", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class VisualizationTest(unittest.TestCase):
    def test_world_declares_staircase_schema(self):
        world = json.loads(
            (ROOT / "nav2/worlds/simple_columns.json").read_text(encoding="utf-8")
        )
        required_fields = {
            "name",
            "center_x",
            "center_y",
            "yaw",
            "width",
            "step_depth",
            "step_height",
            "up_steps",
            "platform_length",
            "down_steps",
        }

        for staircase in world.get("traversable_staircases", []):
            self.assertLessEqual(required_fields, staircase.keys())

    def test_marker_builder_draws_outline_steps_and_platform(self):
        try:
            module = _load_module()
        except ModuleNotFoundError as error:
            if error.name == "rclpy":
                self.skipTest("ROS 2 Python packages are unavailable in this environment")
            raise
        world = {
            "traversable_staircases": [
                {
                    "name": "test_stairs",
                    "center_x": 1,
                    "center_y": 2,
                    "yaw": 0,
                    "width": 1,
                    "step_depth": 0.3,
                    "step_height": 0.1,
                    "up_steps": 2,
                    "platform_length": 0.6,
                    "down_steps": 2,
                }
            ]
        }

        markers = module.build_markers(world)

        self.assertEqual(len(markers.markers), 1 + 2 + 2 + 2)
        self.assertEqual(markers.markers[0].ns, "test_stairs/outline")
        self.assertTrue(
            all(marker.header.frame_id == "map" for marker in markers.markers)
        )
        self.assertTrue(
            all(
                marker.type == module.Marker.LINE_STRIP
                for marker in markers.markers
            )
        )
        self.assertAlmostEqual(markers.markers[1].points[0].x, 0.4)
        self.assertAlmostEqual(markers.markers[2].points[0].x, 0.7)
        self.assertAlmostEqual(markers.markers[3].points[0].x, 1.6)
        self.assertAlmostEqual(markers.markers[4].points[0].x, 1.9)
        self.assertTrue(
            all(
                marker.ns == "test_stairs/platform"
                for marker in markers.markers[-2:]
            )
        )

    def test_marker_source_uses_latched_qos_and_is_costmap_independent(self):
        source = SCRIPT.read_text(encoding="utf-8")

        self.assertIn("DurabilityPolicy.TRANSIENT_LOCAL", source)
        self.assertIn("ReliabilityPolicy.RELIABLE", source)
        self.assertIn("MarkerArray", source)
        self.assertNotIn("costmap", source.lower())

    def test_rviz_config_separates_map_costmap_and_terrain_topics(self):
        config = yaml.safe_load(
            (ROOT / "nav2/rviz/sim_global_planning.rviz").read_text(
                encoding="utf-8"
            )
        )
        displays = config["Visualization Manager"]["Displays"]
        topics = {
            display["Name"]: display.get("Topic", {}).get("Value")
            for display in displays
        }

        self.assertEqual(topics["Static map"], "/map")
        self.assertEqual(topics["Global costmap"], "/global_costmap/costmap")
        self.assertEqual(
            topics["Traversable terrain"], "/nav2_traversable_terrain"
        )
