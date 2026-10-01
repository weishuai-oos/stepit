#!/usr/bin/python3
"""Generate a Nav2 PGM/YAML map and a matching MuJoCo scene.

The JSON world description is the single source of truth for both views.
Obstacles are projected into the Nav2 occupancy map and the MuJoCo scene.
Traversable terrain, such as stairs, is emitted only into the 3-D scene so the
locomotion policy can climb it without Nav2 treating every tread as an obstacle.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from typing import Any


SAFE_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def finite_number(value: Any, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def positive_integer(value: Any, name: str) -> int:
    number = int(value)
    if number <= 0 or float(value) != number:
        raise ValueError(f"{name} must be a positive integer")
    return number


def staircase_boxes(staircase: dict[str, Any]) -> list[dict[str, float | str]]:
    """Expand a staircase specification into MuJoCo box geometry."""
    center_x = float(staircase["center_x"])
    center_y = float(staircase["center_y"])
    yaw = float(staircase["yaw"])
    width = float(staircase["width"])
    step_depth = float(staircase["step_depth"])
    step_height = float(staircase["step_height"])
    up_steps = int(staircase["up_steps"])
    platform_length = float(staircase["platform_length"])
    down_steps = int(staircase["down_steps"])
    total_length = (up_steps + down_steps) * step_depth + platform_length
    local_start = -0.5 * total_length
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    def box(
        suffix: str,
        local_x: float,
        length: float,
        top_height: float,
    ) -> dict[str, float | str]:
        return {
            "name": f'{staircase["name"]}_{suffix}',
            "center_x": center_x + cos_yaw * local_x,
            "center_y": center_y + sin_yaw * local_x,
            "half_x": 0.5 * length,
            "half_y": 0.5 * width,
            "half_z": 0.5 * top_height,
            "yaw": yaw,
        }

    boxes: list[dict[str, float | str]] = []
    for index in range(up_steps):
        boxes.append(
            box(
                f"up_{index:02d}",
                local_start + (index + 0.5) * step_depth,
                step_depth,
                (index + 1) * step_height,
            )
        )

    platform_start = local_start + up_steps * step_depth
    boxes.append(
        box(
            "platform",
            platform_start + 0.5 * platform_length,
            platform_length,
            up_steps * step_height,
        )
    )

    down_start = platform_start + platform_length
    for index in range(down_steps):
        boxes.append(
            box(
                f"down_{index:02d}",
                down_start + (index + 0.5) * step_depth,
                step_depth,
                (down_steps - index) * step_height,
            )
        )
    return boxes


def load_world(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        world = json.load(stream)

    map_spec = world["map"]
    width = finite_number(map_spec["width"], "map.width")
    height = finite_number(map_spec["height"], "map.height")
    resolution = finite_number(map_spec["resolution"], "map.resolution")
    boundary = finite_number(map_spec["boundary_thickness"], "map.boundary_thickness")
    origin = map_spec["origin"]
    if width <= 0.0 or height <= 0.0 or resolution <= 0.0 or boundary <= 0.0:
        raise ValueError("map dimensions, resolution, and boundary thickness must be positive")
    if len(origin) != 2:
        raise ValueError("map.origin must contain [x, y]")
    finite_number(origin[0], "map.origin[0]")
    finite_number(origin[1], "map.origin[1]")

    for index, cylinder in enumerate(world.get("cylinders", [])):
        name = str(cylinder.get("name", f"column_{index}"))
        if not SAFE_NAME.fullmatch(name):
            raise ValueError(f"invalid MuJoCo geom name: {name!r}")
        cylinder["name"] = name
        for field in ("x", "y", "radius", "height"):
            cylinder[field] = finite_number(cylinder[field], f"cylinders[{index}].{field}")
        if cylinder["radius"] <= 0.0 or cylinder["height"] <= 0.0:
            raise ValueError("cylinder radius and height must be positive")

    for index, staircase in enumerate(world.get("traversable_staircases", [])):
        name = str(staircase.get("name", f"staircase_{index}"))
        if not SAFE_NAME.fullmatch(name):
            raise ValueError(f"invalid MuJoCo geom prefix: {name!r}")
        staircase["name"] = name
        for field in (
            "center_x",
            "center_y",
            "yaw",
            "width",
            "step_depth",
            "step_height",
            "platform_length",
        ):
            staircase[field] = finite_number(
                staircase[field], f"traversable_staircases[{index}].{field}"
            )
        staircase["up_steps"] = positive_integer(
            staircase["up_steps"], f"traversable_staircases[{index}].up_steps"
        )
        staircase["down_steps"] = positive_integer(
            staircase["down_steps"],
            f"traversable_staircases[{index}].down_steps",
        )
        for field in ("width", "step_depth", "step_height", "platform_length"):
            if staircase[field] <= 0.0:
                raise ValueError(
                    f"traversable_staircases[{index}].{field} must be positive"
                )
    return world


def rasterize(world: dict[str, Any]) -> tuple[int, int, list[bytearray]]:
    map_spec = world["map"]
    width_m = float(map_spec["width"])
    height_m = float(map_spec["height"])
    resolution = float(map_spec["resolution"])
    origin_x, origin_y = (float(value) for value in map_spec["origin"])
    boundary = float(map_spec["boundary_thickness"])

    width_cells = round(width_m / resolution)
    height_cells = round(height_m / resolution)
    if not math.isclose(width_cells * resolution, width_m, abs_tol=1e-9):
        raise ValueError("map.width must be an integer multiple of map.resolution")
    if not math.isclose(height_cells * resolution, height_m, abs_tol=1e-9):
        raise ValueError("map.height must be an integer multiple of map.resolution")

    # OccupancyGrid convention: row 0 is the bottom row.  Values here are 1 for
    # occupied and 0 for free; write_pgm() reverses the rows for image order.
    rows = [bytearray(width_cells) for _ in range(height_cells)]
    max_x = origin_x + width_m
    max_y = origin_y + height_m
    for my in range(height_cells):
        y = origin_y + (my + 0.5) * resolution
        for mx in range(width_cells):
            x = origin_x + (mx + 0.5) * resolution
            occupied = (
                x < origin_x + boundary
                or x > max_x - boundary
                or y < origin_y + boundary
                or y > max_y - boundary
            )
            if not occupied:
                for cylinder in world.get("cylinders", []):
                    dx = x - float(cylinder["x"])
                    dy = y - float(cylinder["y"])
                    if dx * dx + dy * dy <= float(cylinder["radius"]) ** 2:
                        occupied = True
                        break
            rows[my][mx] = 1 if occupied else 0
    return width_cells, height_cells, rows


def write_pgm(path: Path, width: int, height: int, rows: list[bytearray]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        # PGM starts at the image's top-left while OccupancyGrid starts at the
        # map's bottom-left, so Y must be flipped here.
        for row in reversed(rows):
            stream.write(bytes(0 if occupied else 254 for occupied in row))


def write_map_yaml(path: Path, pgm_name: str, world: dict[str, Any]) -> None:
    map_spec = world["map"]
    origin_x, origin_y = map_spec["origin"]
    text = (
        f"image: {pgm_name}\n"
        "mode: trinary\n"
        f"resolution: {float(map_spec['resolution']):.6f}\n"
        f"origin: [{float(origin_x):.6f}, {float(origin_y):.6f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )
    path.write_text(text, encoding="utf-8")


def write_mujoco_scene(path: Path, world: dict[str, Any]) -> None:
    map_spec = world["map"]
    width = float(map_spec["width"])
    height = float(map_spec["height"])
    origin_x, origin_y = (float(value) for value in map_spec["origin"])
    boundary = float(map_spec["boundary_thickness"])
    wall_height = float(map_spec["boundary_height"])
    center_x = origin_x + 0.5 * width
    center_y = origin_y + 0.5 * height
    wall_z = 0.5 * wall_height

    lines = [
        '<mujoco model="g1_29dof nav2 scene">',
        '  <include file="g1_29dof.xml"/>',
        '',
        f'  <statistic center="{center_x:.3f} {center_y:.3f} 0.5" extent="{0.6 * max(width, height):.3f}"/>',
        '',
        '  <visual>',
        '    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>',
        '    <rgba haze="0.15 0.25 0.35 1"/>',
        '    <global azimuth="-130" elevation="-20"/>',
        '  </visual>',
        '',
        '  <asset>',
        '    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>',
        '    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"',
        '      markrgb="0.8 0.8 0.8" width="300" height="300"/>',
        '    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="8 8" reflectance="0.2"/>',
        '  </asset>',
        '',
        '  <worldbody>',
        '    <light pos="0 0 4" dir="0 0 -1" directional="true"/>',
        '    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>',
        '',
        '    <!-- Generated boundary: matches occupied border cells in the Nav2 map. -->',
        f'    <geom name="nav2_wall_west" pos="{origin_x + boundary / 2:.3f} {center_y:.3f} {wall_z:.3f}" type="box" size="{boundary / 2:.3f} {height / 2:.3f} {wall_z:.3f}" rgba="0.25 0.25 0.30 1"/>',
        f'    <geom name="nav2_wall_east" pos="{origin_x + width - boundary / 2:.3f} {center_y:.3f} {wall_z:.3f}" type="box" size="{boundary / 2:.3f} {height / 2:.3f} {wall_z:.3f}" rgba="0.25 0.25 0.30 1"/>',
        f'    <geom name="nav2_wall_south" pos="{center_x:.3f} {origin_y + boundary / 2:.3f} {wall_z:.3f}" type="box" size="{width / 2:.3f} {boundary / 2:.3f} {wall_z:.3f}" rgba="0.25 0.25 0.30 1"/>',
        f'    <geom name="nav2_wall_north" pos="{center_x:.3f} {origin_y + height - boundary / 2:.3f} {wall_z:.3f}" type="box" size="{width / 2:.3f} {boundary / 2:.3f} {wall_z:.3f}" rgba="0.25 0.25 0.30 1"/>',
        '',
        '    <!-- Generated cylinders: their x/y/radius exactly match the PGM map. -->',
    ]
    colors = ("0.78 0.20 0.18 1", "0.90 0.55 0.12 1", "0.20 0.55 0.80 1", "0.45 0.25 0.70 1")
    for index, cylinder in enumerate(world.get("cylinders", [])):
        half_height = 0.5 * float(cylinder["height"])
        lines.append(
            f'    <geom name="{cylinder["name"]}" pos="{float(cylinder["x"]):.3f} '
            f'{float(cylinder["y"]):.3f} {half_height:.3f}" type="cylinder" '
            f'size="{float(cylinder["radius"]):.3f} {half_height:.3f}" '
            f'rgba="{colors[index % len(colors)]}"/>'
        )

    stairs = world.get("traversable_staircases", [])
    if stairs:
        lines.extend(
            [
                "",
                "    <!-- Traversable terrain: intentionally absent from the Nav2 occupancy map. -->",
            ]
        )
    for staircase in stairs:
        for box in staircase_boxes(staircase):
            half_yaw = 0.5 * float(box["yaw"])
            lines.append(
                f'    <geom name="{box["name"]}" '
                f'pos="{float(box["center_x"]):.3f} {float(box["center_y"]):.3f} '
                f'{float(box["half_z"]):.3f}" type="box" '
                f'size="{float(box["half_x"]):.3f} {float(box["half_y"]):.3f} '
                f'{float(box["half_z"]):.3f}" '
                f'quat="{math.cos(half_yaw):.9f} 0.0 0.0 {math.sin(half_yaw):.9f}" '
                'rgba="0.55 0.58 0.62 1"/>'
            )
    lines.extend(["  </worldbody>", "</mujoco>", ""])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    here = Path(__file__).resolve().parent
    parser.add_argument("--world", type=Path, default=here / "worlds" / "simple_columns.json")
    parser.add_argument("--map-dir", type=Path, default=here / "maps")
    parser.add_argument("--map-name", default="simple_columns")
    parser.add_argument("--mujoco-scene", type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    world = load_world(args.world.resolve())
    width, height, rows = rasterize(world)
    pgm_path = args.map_dir.resolve() / f"{args.map_name}.pgm"
    yaml_path = args.map_dir.resolve() / f"{args.map_name}.yaml"
    write_pgm(pgm_path, width, height, rows)
    write_map_yaml(yaml_path, pgm_path.name, world)
    print(f"generated Nav2 map: {yaml_path} ({width}x{height} cells)")
    if args.mujoco_scene is not None:
        scene_path = args.mujoco_scene.resolve()
        write_mujoco_scene(scene_path, world)
        print(f"generated MuJoCo scene: {scene_path}")


if __name__ == "__main__":
    main()
