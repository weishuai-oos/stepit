#!/usr/bin/python3
"""Standalone launch for the static map, planner server, and waypoint bridge."""

from __future__ import annotations

import os
import sys
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PLANNER_PROFILES = {
    "navfn_xy_legacy": "GridBased",
    "smac_hybrid_xy_forward": "SmacHybrid",
    "smac_terminal_yaw": "SmacHybrid",
    "smac_lattice_full_se2": "SmacLattice",
}
DIRECT_LAUNCH_ARGUMENTS = {
    "planner_profile", "publish_map_to_world", "publish_terrain_markers",
    "terrain_markers_topic", "terrain_markers_frame",
}


def _parse_direct_launch_arguments(argv):
    """Seed launch configurations when this file is executed directly."""
    overrides = {}
    passthrough = []
    for argument in argv:
        if ":=" not in argument:
            passthrough.append(argument)
            continue
        name, value = argument.split(":=", 1)
        if name not in DIRECT_LAUNCH_ARGUMENTS:
            allowed = ", ".join(sorted(DIRECT_LAUNCH_ARGUMENTS))
            raise ValueError(f"unknown direct launch argument {name!r}; allowed: {allowed}")
        overrides[name] = value
    return overrides, passthrough


def _launch_nodes(context, nav2_dir: Path, stepit_dir: Path):
    params_file = nav2_dir / "nav2_params.yaml"
    map_yaml = nav2_dir / "maps" / "simple_columns.yaml"
    lattice_file = nav2_dir / "lattice" / "g1_diff_5cm_1m.json"
    world_file = nav2_dir / "worlds" / "simple_columns.json"
    waypoint_node = stepit_dir / "scripts" / "nav2_global_goal_to_waypoints.py"
    trajectory_core = stepit_dir / "scripts" / "nav2_waypoint_trajectory.py"
    terrain_marker_node = stepit_dir / "scripts" / "publish_traversable_terrain_markers.py"

    for required in (
        params_file,
        map_yaml,
        lattice_file,
        waypoint_node,
        trajectory_core,
        terrain_marker_node,
        world_file,
    ):
        if not required.is_file():
            raise FileNotFoundError(f"required Nav2 integration file is missing: {required}")

    planner_profile = LaunchConfiguration("planner_profile").perform(context)
    if planner_profile not in PLANNER_PROFILES:
        allowed = ", ".join(sorted(PLANNER_PROFILES))
        raise ValueError(f"planner_profile must be one of: {allowed}")

    publish_map_to_world = LaunchConfiguration("publish_map_to_world")
    publish_terrain_markers = LaunchConfiguration("publish_terrain_markers")
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_world_static_tf",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "map", "--child-frame-id", "world",
            ],
            condition=IfCondition(publish_map_to_world),
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "/usr/bin/python3", str(terrain_marker_node),
                "--world", str(world_file),
                "--topic", LaunchConfiguration("terrain_markers_topic"),
                "--frame-id", LaunchConfiguration("terrain_markers_frame"),
            ],
            condition=IfCondition(publish_terrain_markers),
            additional_env={"PYTHONUNBUFFERED": "1"},
            output="screen",
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[str(params_file), {"yaml_filename": str(map_yaml)}],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[
                str(params_file),
                {"SmacLattice": {"lattice_filepath": str(lattice_file)}},
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_global_planning",
            output="screen",
            parameters=[str(params_file)],
        ),
        ExecuteProcess(
            cmd=[
                "/usr/bin/python3",
                str(waypoint_node),
                "--ros-args",
                "--params-file",
                str(params_file),
                "-p",
                f"planner_profile:={planner_profile}",
            ],
            additional_env={"PYTHONUNBUFFERED": "1"},
            output="screen",
        ),
    ]


def generate_launch_description(overrides=None) -> LaunchDescription:
    nav2_dir = Path(__file__).resolve().parents[1]
    stepit_dir = nav2_dir.parent

    actions = [
        SetLaunchConfiguration(name, value)
        for name, value in (overrides or {}).items()
    ]
    actions.extend(
        [
            DeclareLaunchArgument(
                "publish_map_to_world",
                default_value=os.environ.get("PUBLISH_MAP_TO_WORLD", "true"),
                choices=["true", "false"],
                description=(
                    "Publish the sim-only identity map -> world transform; "
                    "MuJoCo owns world -> odom -> base_link."
                ),
            ),
            DeclareLaunchArgument(
                "planner_profile",
                default_value="navfn_xy_legacy",
                choices=sorted(PLANNER_PROFILES),
                description=(
                    "Startup-only planner profile. Allowed values: "
                    + ", ".join(sorted(PLANNER_PROFILES))
                ),
            ),
            DeclareLaunchArgument(
                "publish_terrain_markers",
                default_value=os.environ.get("PUBLISH_TERRAIN_MARKERS", "true"),
                choices=["true", "false"],
                description="Publish RViz-only outlines for traversable terrain.",
            ),
            DeclareLaunchArgument(
                "terrain_markers_topic",
                default_value="/nav2_traversable_terrain",
                description="MarkerArray topic for traversable terrain visualization.",
            ),
            DeclareLaunchArgument(
                "terrain_markers_frame",
                default_value="map",
                description="Frame used by traversable terrain markers.",
            ),
            OpaqueFunction(function=_launch_nodes, args=[nav2_dir, stepit_dir]),
        ]
    )
    return LaunchDescription(
        actions
    )


if __name__ == "__main__":
    direct_overrides, launch_argv = _parse_direct_launch_arguments(sys.argv[1:])
    service = LaunchService(argv=launch_argv)
    service.include_launch_description(generate_launch_description(direct_overrides))
    raise SystemExit(service.run())
