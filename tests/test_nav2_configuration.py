#!/usr/bin/python3

import ast
import hashlib
import importlib.util
import json
import math
import re
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


REPO_DIR = Path(__file__).resolve().parents[1]
NAV2_DIR = REPO_DIR / "nav2"
PARAMS_FILE = NAV2_DIR / "nav2_params.yaml"
LAUNCH_FILE = NAV2_DIR / "launch" / "sim_global_planning.launch.py"
RUN_SCRIPT = REPO_DIR / "scripts" / "run_nav2_global_planning.sh"
BRIDGE_FILE = REPO_DIR / "scripts" / "nav2_global_goal_to_waypoints.py"
LATTICE_FILE = NAV2_DIR / "lattice" / "g1_diff_5cm_1m.json"
WORLD_FILE = NAV2_DIR / "worlds" / "simple_columns.json"
MAP_YAML_FILE = NAV2_DIR / "maps" / "simple_columns.yaml"
MAP_PGM_FILE = NAV2_DIR / "maps" / "simple_columns.pgm"
GENERATOR_FILE = NAV2_DIR / "generate_static_world.py"

EXPECTED_PROFILES = {
    "navfn_xy_legacy": "GridBased",
    "smac_hybrid_xy_forward": "SmacHybrid",
    "smac_terminal_yaw": "SmacHybrid",
    "smac_lattice_full_se2": "SmacLattice",
}
EXPECTED_DIRECT_LAUNCH_ARGUMENTS = {
    "planner_profile",
    "publish_map_to_world",
    "publish_terrain_markers",
    "terrain_markers_topic",
    "terrain_markers_frame",
}


def read_params() -> dict:
    with PARAMS_FILE.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def load_launch_profiles() -> dict:
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "PLANNER_PROFILES":
                    return ast.literal_eval(node.value)
    raise AssertionError("PLANNER_PROFILES assignment not found")


def load_direct_launch_arguments() -> set[str]:
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "DIRECT_LAUNCH_ARGUMENTS":
                    return ast.literal_eval(node.value)
    raise AssertionError("DIRECT_LAUNCH_ARGUMENTS assignment not found")


def load_generator_module():
    spec = importlib.util.spec_from_file_location("generate_static_world", GENERATOR_FILE)
    if spec is None or spec.loader is None:
        raise AssertionError("unable to load generate_static_world.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_pgm(path: Path) -> tuple[int, int, bytes]:
    with path.open("rb") as stream:
        magic = stream.readline().strip()
        size = stream.readline().strip()
        max_value = stream.readline().strip()
        pixels = stream.read()
    if magic != b"P5" or max_value != b"255":
        raise AssertionError(f"unexpected PGM header in {path}")
    width, height = (int(value) for value in size.split())
    return width, height, pixels


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Nav2ConfigurationTests(unittest.TestCase):
    def test_launch_registers_exactly_four_startup_profiles_with_planner_ids(self) -> None:
        profiles = load_launch_profiles()

        self.assertEqual(profiles, EXPECTED_PROFILES)

    def test_launch_passes_only_the_startup_profile_to_the_bridge(self) -> None:
        text = LAUNCH_FILE.read_text(encoding="utf-8")

        self.assertIn('f"planner_profile:={planner_profile}"', text)
        self.assertNotIn('f"planner_id:=', text)
        self.assertNotIn('f"use_goal_heading:=', text)

    def test_direct_launch_entrypoint_seeds_named_launch_arguments(self) -> None:
        text = LAUNCH_FILE.read_text(encoding="utf-8")

        self.assertEqual(load_direct_launch_arguments(), EXPECTED_DIRECT_LAUNCH_ARGUMENTS)
        self.assertIn('_parse_direct_launch_arguments(sys.argv[1:])', text)
        self.assertIn('generate_launch_description(direct_overrides)', text)
        self.assertIn('choices=sorted(PLANNER_PROFILES)', text)

    def test_run_script_defaults_and_validates_the_same_startup_profiles(self) -> None:
        text = RUN_SCRIPT.read_text(encoding="utf-8")

        self.assertRegex(text, r'planner_profile="navfn_xy_legacy"')
        case_match = re.search(r'case "\$\{planner_profile\}" in(?P<body>.*?)esac', text, re.S)
        self.assertIsNotNone(case_match)
        valid_patterns = re.findall(r"^\s{2}([a-z0-9_|]+)\)\s*$", case_match.group("body"), re.M)
        self.assertIn("|".join(EXPECTED_PROFILES), valid_patterns)
        self.assertIn("Invalid --profile", case_match.group("body"))
        self.assertIn("planner_profile:=${planner_profile}", text)

    def test_yaml_registers_humble_planner_plugin_strings(self) -> None:
        planner_params = read_params()["planner_server"]["ros__parameters"]

        self.assertEqual(planner_params["planner_plugins"], ["GridBased", "SmacHybrid", "SmacLattice"])
        self.assertEqual(planner_params["GridBased"]["plugin"], "nav2_navfn_planner/NavfnPlanner")
        self.assertEqual(planner_params["SmacHybrid"]["plugin"], "nav2_smac_planner/SmacPlannerHybrid")
        self.assertEqual(planner_params["SmacLattice"]["plugin"], "nav2_smac_planner/SmacPlannerLattice")

    def test_hybrid_planner_uses_dubin_one_meter_radius_and_no_reverse_model(self) -> None:
        hybrid = read_params()["planner_server"]["ros__parameters"]["SmacHybrid"]

        self.assertEqual(hybrid["motion_model_for_search"], "DUBIN")
        self.assertEqual(hybrid["minimum_turning_radius"], 1.0)
        self.assertNotIn("REEDS", hybrid["motion_model_for_search"])
        self.assertNotIn("allow_reverse_expansion", hybrid)

    def test_hybrid_analytic_expansion_max_length_is_at_least_four_times_turning_radius(self) -> None:
        hybrid = read_params()["planner_server"]["ros__parameters"]["SmacHybrid"]

        self.assertGreaterEqual(
            hybrid["analytic_expansion_max_length"],
            4.0 * hybrid["minimum_turning_radius"],
        )

    def test_lattice_planner_config_points_to_expected_profile_and_disables_reverse_expansion(self) -> None:
        lattice = read_params()["planner_server"]["ros__parameters"]["SmacLattice"]

        self.assertEqual(lattice["lattice_filepath"], "nav2/lattice/g1_diff_5cm_1m.json")
        self.assertEqual(lattice["analytic_expansion_max_length"], 5.0)
        self.assertFalse(lattice["allow_reverse_expansion"])
        self.assertEqual(lattice["rotation_penalty"], 1.0)
        self.assertFalse(lattice["smooth_path"])

    def test_lattice_json_metadata_primitives_and_hash_match_expected_contract(self) -> None:
        lattice = json.loads(LATTICE_FILE.read_text(encoding="utf-8"))
        metadata = lattice["lattice_metadata"]
        primitives = lattice["primitives"]
        pure_rotations = [
            primitive
            for primitive in primitives
            if primitive["trajectory_radius"] == 0.0
            and primitive["trajectory_length"] == 0.0
            and primitive["arc_length"] == 0.0
            and primitive["straight_length"] == 0.0
            and len({(round(pose[0], 9), round(pose[1], 9)) for pose in primitive["poses"]}) == 1
            and not math.isclose(primitive["poses"][0][2], primitive["poses"][-1][2])
        ]

        self.assertEqual(metadata["grid_resolution"], 0.05)
        self.assertEqual(metadata["turning_radius"], 1)
        self.assertEqual(metadata["motion_model"], "diff")
        self.assertGreater(len(pure_rotations), 0)
        self.assertEqual(sha256(LATTICE_FILE), "984ecd63a24eb705ad24a102f7b8021c2f5fa46dcced03122449308728625590")

    def test_global_costmap_uses_expected_resolution_radius_and_inflation(self) -> None:
        costmap = read_params()["global_costmap"]["global_costmap"]["ros__parameters"]

        self.assertEqual(costmap["resolution"], 0.05)
        self.assertEqual(costmap["robot_radius"], 0.95)
        self.assertEqual(costmap["inflation_layer"]["inflation_radius"], 1.10)

    def test_bridge_yaml_exposes_timed_trajectory_and_settle_contract(self) -> None:
        bridge = read_params()["nav2_global_goal_to_waypoints"]["ros__parameters"]

        self.assertNotIn("planner_id", bridge)
        self.assertNotIn("use_goal_heading", bridge)
        self.assertNotIn("sampling_spacings", bridge)
        self.assertNotIn("remain_time", bridge)
        self.assertEqual(bridge["planner_profile"], "navfn_xy_legacy")
        self.assertEqual(bridge["publish_rate"], 50.0)
        self.assertEqual(bridge["num_waypoints"], 5)
        self.assertEqual(bridge["waypoint_interval"], 0.5)
        self.assertEqual(bridge["cruise_speed"], 0.30)
        self.assertEqual(bridge["max_acceleration"], 0.80)
        self.assertEqual(bridge["max_lateral_acceleration"], 0.25)
        self.assertEqual(bridge["terminal_deceleration"], 1.00)
        self.assertEqual(bridge["max_tracking_urgency"], 0.0)
        self.assertEqual(bridge["smac_hybrid_cruise_speed"], 0.80)
        self.assertEqual(bridge["smac_hybrid_max_acceleration"], 1.20)
        self.assertEqual(bridge["smac_hybrid_terminal_deceleration"], 0.50)
        self.assertEqual(bridge["smac_hybrid_max_lateral_acceleration"], 0.30)
        self.assertEqual(bridge["smac_hybrid_max_tracking_urgency"], 0.0)
        self.assertLessEqual(bridge["smac_hybrid_cruise_speed"], 1.2)
        self.assertLessEqual(bridge["smac_hybrid_max_acceleration"], 4.0)
        self.assertGreater(bridge["max_deceleration"], 0.0)
        self.assertEqual(bridge["replan_commit_time"], 0.5)
        self.assertEqual(bridge["replan_max_waypoint_shift"], 0.30)
        self.assertEqual(bridge["replan_max_heading_shift"], 0.35)
        self.assertGreater(bridge["transient_failure_timeout"], 0.0)
        self.assertIn("costmap_timeout", bridge)
        self.assertGreater(bridge["costmap_timeout"], 0.0)
        self.assertEqual(bridge["goal_tolerance"], 0.15)
        self.assertEqual(bridge["goal_linear_speed_tolerance"], 0.10)
        self.assertEqual(bridge["goal_yaw_tolerance"], 0.10)
        self.assertEqual(bridge["goal_angular_speed_tolerance"], 0.15)
        self.assertEqual(bridge["goal_hold_time"], 0.50)
        self.assertEqual(bridge["goal_capture_release_tolerance"], 0.30)
        self.assertEqual(bridge["handover_hold_time"], 0.20)
        self.assertEqual(bridge["odom_speed_filter_window"], 2)
        self.assertEqual(bridge["terminal_response_time"], 0.20)
        self.assertEqual(bridge["terminal_braking_margin"], 0.10)
        self.assertEqual(bridge["emergency_speed_upper_bound"], 1.20)
        self.assertEqual(bridge["tracking_pacing_max_delay"], 0.75)

        available_braking_time = (
            bridge["num_waypoints"] * bridge["waypoint_interval"]
            - bridge["replan_commit_time"]
        )
        self.assertLessEqual(
            bridge["cruise_speed"] / bridge["max_deceleration"],
            available_braking_time,
        )
        self.assertLessEqual(
            bridge["smac_hybrid_cruise_speed"] / bridge["max_deceleration"],
            available_braking_time,
        )
        self.assertLessEqual(
            bridge["emergency_speed_upper_bound"] / bridge["max_deceleration"],
            available_braking_time,
        )
        self.assertLessEqual(
            bridge["smac_hybrid_cruise_speed"]
            / bridge["smac_hybrid_terminal_deceleration"],
            available_braking_time
            - bridge["terminal_response_time"]
            - bridge["terminal_braking_margin"]
            / bridge["smac_hybrid_cruise_speed"],
        )

    def test_invalid_goal_branch_clears_execution_and_publishes_zero(self) -> None:
        source = BRIDGE_FILE.read_text(encoding="utf-8")
        tree = ast.parse(source)
        functions = {
            node.name: ast.get_source_segment(source, node)
            for node in ast.walk(tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }

        on_goal = functions["_on_goal"]
        invalid_branch = on_goal.split("if not all(math.isfinite(value) for value in values):", 1)[1]
        invalid_branch = invalid_branch.split("if not message.header.frame_id:", 1)[0]
        self.assertIn("self._reset_goal_execution(clear_goal=True)", invalid_branch)
        self.assertIn("self._publish_zero_waypoints()", invalid_branch)

        reset = functions["_reset_goal_execution"]
        for required in (
            "self.goal = None",
            "self._clear_path()",
            "self._reset_terminal_planning_batch()",
            "self.plan_sequence += 1",
            "self._cancel_active_plan()",
        ):
            self.assertIn(required, reset)

    def test_world_generator_recreates_map_outputs_reproducibly_in_temp_directory(self) -> None:
        generator = load_generator_module()
        world = generator.load_world(WORLD_FILE)

        with tempfile.TemporaryDirectory() as tmp:
            tmp_dir = Path(tmp)
            width, height, rows = generator.rasterize(world)
            generated_pgm = tmp_dir / "simple_columns.pgm"
            generated_yaml = tmp_dir / "simple_columns.yaml"
            generator.write_pgm(generated_pgm, width, height, rows)
            generator.write_map_yaml(generated_yaml, generated_pgm.name, world)

            self.assertEqual(generated_yaml.read_text(encoding="utf-8"), MAP_YAML_FILE.read_text(encoding="utf-8"))
            self.assertEqual(generated_pgm.read_bytes(), MAP_PGM_FILE.read_bytes())

    def test_world_generator_rasterized_occupancy_matches_committed_pgm_pixels(self) -> None:
        generator = load_generator_module()
        world = generator.load_world(WORLD_FILE)
        width, height, rows = generator.rasterize(world)
        pgm_width, pgm_height, pixels = parse_pgm(MAP_PGM_FILE)
        expected_pixels = bytes(
            0 if occupied else 254
            for row in reversed(rows)
            for occupied in row
        )

        self.assertEqual((width, height), (pgm_width, pgm_height))
        self.assertEqual(pixels, expected_pixels)

    def test_traversable_stairs_do_not_change_nav2_occupancy(self) -> None:
        generator = load_generator_module()
        world = generator.load_world(WORLD_FILE)
        world_without_stairs = dict(world)
        world_without_stairs.pop("traversable_staircases")

        self.assertEqual(
            generator.rasterize(world),
            generator.rasterize(world_without_stairs),
        )

    def test_traversable_stairs_clear_existing_inflated_obstacles(self) -> None:
        generator = load_generator_module()
        world = generator.load_world(WORLD_FILE)
        staircase = world["traversable_staircases"][0]
        self.assertEqual(staircase["yaw"], 0.0)
        boxes = generator.staircase_boxes(staircase)
        min_x = min(float(box["center_x"]) - float(box["half_x"]) for box in boxes)
        max_x = max(float(box["center_x"]) + float(box["half_x"]) for box in boxes)
        min_y = min(float(box["center_y"]) - float(box["half_y"]) for box in boxes)
        max_y = max(float(box["center_y"]) + float(box["half_y"]) for box in boxes)
        inflation = read_params()["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]["inflation_layer"]["inflation_radius"]
        map_spec = world["map"]
        boundary = map_spec["boundary_thickness"]
        free_min_x = map_spec["origin"][0] + boundary
        free_min_y = map_spec["origin"][1] + boundary
        free_max_x = map_spec["origin"][0] + map_spec["width"] - boundary
        free_max_y = map_spec["origin"][1] + map_spec["height"] - boundary

        self.assertGreaterEqual(min_x - free_min_x, inflation)
        self.assertGreaterEqual(free_max_x - max_x, inflation)
        self.assertGreaterEqual(min_y - free_min_y, inflation)
        self.assertGreaterEqual(free_max_y - max_y, inflation)
        for cylinder in world["cylinders"]:
            dx = max(min_x - cylinder["x"], 0.0, cylinder["x"] - max_x)
            dy = max(min_y - cylinder["y"], 0.0, cylinder["y"] - max_y)
            self.assertGreaterEqual(
                math.hypot(dx, dy) - cylinder["radius"],
                inflation,
            )

    def test_world_generator_writes_mujoco_cylinders_matching_world_json(self) -> None:
        generator = load_generator_module()
        world = generator.load_world(WORLD_FILE)

        with tempfile.TemporaryDirectory() as tmp:
            scene = Path(tmp) / "scene.xml"
            generator.write_mujoco_scene(scene, world)
            root = ET.fromstring(scene.read_text(encoding="utf-8"))
            geoms = {geom.attrib["name"]: geom.attrib for geom in root.findall(".//geom") if "name" in geom.attrib}

            for cylinder in world["cylinders"]:
                geom = geoms[cylinder["name"]]
                half_height = 0.5 * cylinder["height"]
                self.assertEqual(geom["type"], "cylinder")
                self.assertEqual(geom["pos"], f"{cylinder['x']:.3f} {cylinder['y']:.3f} {half_height:.3f}")
                self.assertEqual(geom["size"], f"{cylinder['radius']:.3f} {half_height:.3f}")

    def test_world_generator_writes_traversable_stair_geometry(self) -> None:
        generator = load_generator_module()
        world = generator.load_world(WORLD_FILE)
        staircase = world["traversable_staircases"][0]
        boxes = generator.staircase_boxes(staircase)

        self.assertEqual(len(boxes), 20)
        self.assertAlmostEqual(float(boxes[0]["center_x"]), -3.30)
        self.assertAlmostEqual(float(boxes[0]["half_z"]), 0.075)
        self.assertEqual(boxes[10]["name"], "stairs_platform")
        self.assertAlmostEqual(float(boxes[10]["center_x"]), 0.15)
        self.assertAlmostEqual(float(boxes[10]["half_z"]), 0.75)
        self.assertAlmostEqual(float(boxes[-1]["center_x"]), 3.30)
        self.assertAlmostEqual(float(boxes[-1]["half_z"]), 0.075)

        with tempfile.TemporaryDirectory() as tmp:
            scene = Path(tmp) / "scene.xml"
            generator.write_mujoco_scene(scene, world)
            root = ET.fromstring(scene.read_text(encoding="utf-8"))
            geoms = {
                geom.attrib["name"]: geom.attrib
                for geom in root.findall(".//geom")
                if "name" in geom.attrib
            }

        for box in boxes:
            geom = geoms[str(box["name"])]
            self.assertEqual(geom["type"], "box")
            self.assertEqual(
                geom["size"],
                f'{float(box["half_x"]):.3f} {float(box["half_y"]):.3f} '
                f'{float(box["half_z"]):.3f}',
            )


if __name__ == "__main__":
    unittest.main()
