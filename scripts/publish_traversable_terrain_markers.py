#!/usr/bin/env python3
"""Publish traversable terrain outlines for RViz as a visualization-only layer."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def _point(x: float, y: float, z: float = 0.03) -> Point:
    point = Point()
    point.x, point.y, point.z = float(x), float(y), float(z)
    return point


def _transform(x: float, y: float, staircase: dict[str, Any]) -> tuple[float, float]:
    angle = float(staircase.get("yaw", 0.0))
    c, s = math.cos(angle), math.sin(angle)
    return (
        float(staircase["center_x"]) + c * x - s * y,
        float(staircase["center_y"]) + s * x + c * y,
    )


def _line_marker(marker_id: int, name: str, points: list[tuple[float, float]],
                 staircase: dict[str, Any], color: tuple[float, float, float],
                 width: float = 0.035) -> Marker:
    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = name
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = width
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, 0.95)
    marker.points = [_point(*_transform(x, y, staircase)) for x, y in points]
    return marker


def build_markers(world: dict[str, Any], frame_id: str = "map") -> MarkerArray:
    """Build a deterministic MarkerArray from ``traversable_staircases``."""
    output = MarkerArray()
    marker_id = 0
    for staircase in world.get("traversable_staircases", []):
        name = str(staircase.get("name", f"staircase_{marker_id}"))
        width = float(staircase["width"])
        depth = float(staircase["step_depth"])
        up_steps = int(staircase["up_steps"])
        down_steps = int(staircase["down_steps"])
        platform = float(staircase["platform_length"])
        run = (up_steps + down_steps) * depth + platform
        outline = [(-run / 2, -width / 2), (run / 2, -width / 2),
                   (run / 2, width / 2), (-run / 2, width / 2),
                   (-run / 2, -width / 2)]
        output.markers.append(_line_marker(marker_id, name + "/outline", outline,
                                            staircase, (0.1, 0.9, 0.2), 0.06))
        marker_id += 1

        # Riser/tread divisions are shown as transverse lines.  Keep the
        # platform gap explicit: it is not a stair tread and has its own
        # boundary marker below.  These markers are visualization only.
        start_x = -run / 2
        for step in range(1, up_steps + 1):
            x = start_x + step * depth
            line = [(x, -width / 2), (x, width / 2)]
            output.markers.append(_line_marker(marker_id, name + "/step", line,
                                                staircase, (1.0, 0.75, 0.1), 0.035))
            marker_id += 1

        platform_start = start_x + up_steps * depth
        platform_end = platform_start + platform
        for step in range(1, down_steps + 1):
            x = platform_end + step * depth
            line = [(x, -width / 2), (x, width / 2)]
            output.markers.append(_line_marker(marker_id, name + "/step", line,
                                                staircase, (1.0, 0.75, 0.1), 0.035))
            marker_id += 1
        for y in (-width / 2, width / 2):
            output.markers.append(_line_marker(
                marker_id, name + "/platform", [(platform_start, y), (platform_end, y)],
                staircase, (0.2, 0.7, 1.0), 0.05))
            marker_id += 1
    for marker in output.markers:
        marker.header.frame_id = frame_id
    return output


class TerrainMarkerPublisher(Node):
    def __init__(self, world_path: Path, topic: str, frame_id: str) -> None:
        super().__init__("traversable_terrain_markers")
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(MarkerArray, topic, qos)
        self._markers = build_markers(json.loads(world_path.read_text(encoding="utf-8")), frame_id)
        self._publisher.publish(self._markers)
        self.get_logger().info(
            f"published {len(self._markers.markers)} traversable terrain markers on {topic}"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    here = Path(__file__).resolve().parents[1]
    parser.add_argument("--world", type=Path, default=here / "nav2" / "worlds" / "simple_columns.json")
    parser.add_argument("--topic", default="/nav2_traversable_terrain")
    parser.add_argument("--frame-id", default="map")
    args = parser.parse_args()
    rclpy.init()
    node = TerrainMarkerPublisher(args.world.resolve(), args.topic, args.frame_id)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
