import argparse
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Wall:
    name: str
    pos_x: float
    pos_y: float
    size_x: float
    size_y: float
    floor_z: float
    height: float


@dataclass(frozen=True)
class Door:
    side: str
    center: float
    width: float


def wall_xml(wall):
    return (
        f'    <geom name="{wall.name}" type="box" '
        f'pos="{wall.pos_x:.3f} {wall.pos_y:.3f} {wall.floor_z + wall.height * 0.5:.3f}" '
        f'size="{wall.size_x:.3f} {wall.size_y:.3f} {wall.height * 0.5:.3f}" '
        'material="room_wall"/>'
    )


def floor_xml(name, center_x, center_y, floor_z, size_x, size_y):
    return (
        f'    <geom name="{name}" type="box" '
        f'pos="{center_x:.3f} {center_y:.3f} {floor_z - 0.018:.3f}" '
        f'size="{size_x:.3f} {size_y:.3f} 0.025" material="room_floor"/>'
    )


def floor_tiles_with_holes(name, center_x, center_y, floor_z, size_x, size_y, holes):
    min_x = center_x - size_x
    max_x = center_x + size_x
    min_y = center_y - size_y
    max_y = center_y + size_y
    x_edges = [min_x, max_x]
    y_edges = [min_y, max_y]
    clipped_holes = []
    for hole_center_x, hole_center_y, hole_size_x, hole_size_y in holes:
        hole_min_x = max(min_x, hole_center_x - hole_size_x)
        hole_max_x = min(max_x, hole_center_x + hole_size_x)
        hole_min_y = max(min_y, hole_center_y - hole_size_y)
        hole_max_y = min(max_y, hole_center_y + hole_size_y)
        if hole_min_x >= hole_max_x or hole_min_y >= hole_max_y:
            continue
        clipped_holes.append((hole_min_x, hole_max_x, hole_min_y, hole_max_y))
        x_edges.extend((hole_min_x, hole_max_x))
        y_edges.extend((hole_min_y, hole_max_y))

    x_edges = sorted(set(round(edge, 6) for edge in x_edges))
    y_edges = sorted(set(round(edge, 6) for edge in y_edges))
    tiles = []
    tile_index = 0
    for x_index in range(len(x_edges) - 1):
        tile_min_x = x_edges[x_index]
        tile_max_x = x_edges[x_index + 1]
        if tile_max_x - tile_min_x < 0.08:
            continue
        tile_center_x = (tile_min_x + tile_max_x) * 0.5
        for y_index in range(len(y_edges) - 1):
            tile_min_y = y_edges[y_index]
            tile_max_y = y_edges[y_index + 1]
            if tile_max_y - tile_min_y < 0.08:
                continue
            tile_center_y = (tile_min_y + tile_max_y) * 0.5
            if any(
                hole_min_x < tile_center_x < hole_max_x and hole_min_y < tile_center_y < hole_max_y
                for hole_min_x, hole_max_x, hole_min_y, hole_max_y in clipped_holes
            ):
                continue
            tiles.append(
                floor_xml(
                    f"{name}_tile_{tile_index:02d}",
                    tile_center_x,
                    tile_center_y,
                    floor_z,
                    (tile_max_x - tile_min_x) * 0.5,
                    (tile_max_y - tile_min_y) * 0.5,
                )
            )
            tile_index += 1
    return tiles


def box_xml(name, x_pos, y_pos, z_pos, size_x, size_y, size_z, material="room_obstacle"):
    return (
        f'    <geom name="{name}" type="box" pos="{x_pos:.3f} {y_pos:.3f} {z_pos:.3f}" '
        f'size="{size_x:.3f} {size_y:.3f} {size_z:.3f}" material="{material}"/>'
    )


def feature_box_xml(name, x_pos, y_pos, z_pos, size_x, size_y, size_z, material):
    return (
        f'    <geom name="{name}" type="box" pos="{x_pos:.3f} {y_pos:.3f} {z_pos:.3f}" '
        f'size="{size_x:.3f} {size_y:.3f} {size_z:.3f}" material="{material}" '
        'contype="0" conaffinity="0"/>'
    )


def axis_positions(center, half_length, spacing=0.62, margin=0.22):
    start = center - half_length + margin
    end = center + half_length - margin
    if end <= start:
        return [center]
    positions = []
    value = start
    while value <= end + 1e-6:
        positions.append(value)
        value += spacing
    return positions


def split_segments(start, end, openings):
    segments = [(start, end)]
    for center, width in openings:
        gap_min = max(start, center - width * 0.5)
        gap_max = min(end, center + width * 0.5)
        next_segments = []
        for seg_min, seg_max in segments:
            if gap_max <= seg_min or gap_min >= seg_max:
                next_segments.append((seg_min, seg_max))
                continue
            if gap_min - seg_min > 0.05:
                next_segments.append((seg_min, gap_min))
            if seg_max - gap_max > 0.05:
                next_segments.append((gap_max, seg_max))
        segments = next_segments
    return segments


def make_wall_features(walls):
    features = []
    offset = 0.006
    for wall in walls:
        horizontal = wall.size_x >= wall.size_y
        side_half_thickness = wall.size_y if horizontal else wall.size_x
        long_center = wall.pos_x if horizontal else wall.pos_y
        long_half = wall.size_x if horizontal else wall.size_y
        long_positions = axis_positions(long_center, long_half, spacing=0.92, margin=0.32)
        z_rows = (
            wall.floor_z + 0.48,
            wall.floor_z + 0.86,
            wall.floor_z + 1.24,
            wall.floor_z + 1.66,
            wall.floor_z + 2.04,
        )
        for side_index, side in enumerate((-1.0, 1.0)):
            surface = (wall.pos_y if horizontal else wall.pos_x) + side * (side_half_thickness + offset)
            for axis_index, long_pos in enumerate(long_positions):
                for z_index, z_pos in enumerate(z_rows):
                    material = "feature_dark" if (axis_index + z_index + side_index) % 2 == 0 else "feature_light"
                    patch_long = 0.050 + 0.020 * ((axis_index + z_index) % 4)
                    patch_z = 0.035 + 0.015 * ((axis_index + 2 * z_index) % 3)
                    name = f"feature_{wall.name}_{side_index}_{axis_index}_{z_index}"
                    if horizontal:
                        features.append(feature_box_xml(name, long_pos, surface, z_pos, patch_long, 0.004, patch_z, material))
                    else:
                        features.append(feature_box_xml(name, surface, long_pos, z_pos, 0.004, patch_long, patch_z, material))
    return features


def make_floor_features(prefix, center_x, center_y, width, depth, floor_z):
    half_width = width * 0.5
    half_depth = depth * 0.5
    features = []
    x_positions = axis_positions(center_x, half_width, spacing=0.95, margin=0.55)
    y_positions = axis_positions(center_y, half_depth, spacing=0.95, margin=0.55)
    for x_index, x_pos in enumerate(x_positions):
        for y_index, y_pos in enumerate(y_positions):
            material = "feature_floor_dark" if (x_index + y_index) % 2 == 0 else "feature_floor_light"
            size = 0.024 + 0.010 * ((x_index + 2 * y_index) % 3)
            features.append(
                feature_box_xml(
                    f"feature_{prefix}_floor_dot_{x_index}_{y_index}",
                    x_pos,
                    y_pos,
                    floor_z + 0.008,
                    size,
                    size,
                    0.001,
                    material,
                )
            )
    for line_index, y_pos in enumerate(axis_positions(center_y, half_depth, spacing=1.55, margin=0.75)):
        features.append(
            feature_box_xml(
                f"feature_{prefix}_floor_line_x_{line_index}",
                center_x,
                y_pos,
                floor_z + 0.008,
                half_width * 0.78,
                0.010,
                0.001,
                "feature_floor_dark",
            )
        )
    for line_index, x_pos in enumerate(axis_positions(center_x, half_width, spacing=1.55, margin=0.75)):
        features.append(
            feature_box_xml(
                f"feature_{prefix}_floor_line_y_{line_index}",
                x_pos,
                center_y,
                floor_z + 0.008,
                0.010,
                half_depth * 0.78,
                0.001,
                "feature_floor_light",
            )
        )
    return features


def make_level_walls(
    prefix,
    rows,
    cols,
    room_size,
    center_x,
    center_y,
    floor_z,
    wall_height,
    wall_thickness,
    door_width,
    outer_doors,
    skip_outer_sides=None,
    extra_horizontal_doors=None,
):
    width = cols * room_size
    depth = rows * room_size
    half_width = width * 0.5
    half_depth = depth * 0.5
    min_x = center_x - half_width
    max_x = center_x + half_width
    min_y = center_y - half_depth
    max_y = center_y + half_depth
    half_thickness = wall_thickness * 0.5
    doors_by_side = {side: [] for side in ("north", "south", "east", "west")}
    for door in outer_doors:
        doors_by_side[door.side].append((door.center, door.width))
    skip_outer_sides = set(skip_outer_sides or ())
    extra_horizontal_doors = extra_horizontal_doors or {}

    walls = []
    if "north" not in skip_outer_sides:
        for index, (seg_min, seg_max) in enumerate(split_segments(min_x, max_x, doors_by_side["north"])):
            walls.append(Wall(f"{prefix}_wall_north_{index}", (seg_min + seg_max) * 0.5, max_y, (seg_max - seg_min) * 0.5, half_thickness, floor_z, wall_height))
    if "south" not in skip_outer_sides:
        for index, (seg_min, seg_max) in enumerate(split_segments(min_x, max_x, doors_by_side["south"])):
            walls.append(Wall(f"{prefix}_wall_south_{index}", (seg_min + seg_max) * 0.5, min_y, (seg_max - seg_min) * 0.5, half_thickness, floor_z, wall_height))
    if "east" not in skip_outer_sides:
        for index, (seg_min, seg_max) in enumerate(split_segments(min_y, max_y, doors_by_side["east"])):
            walls.append(Wall(f"{prefix}_wall_east_{index}", max_x, (seg_min + seg_max) * 0.5, half_thickness, (seg_max - seg_min) * 0.5, floor_z, wall_height))
    if "west" not in skip_outer_sides:
        for index, (seg_min, seg_max) in enumerate(split_segments(min_y, max_y, doors_by_side["west"])):
            walls.append(Wall(f"{prefix}_wall_west_{index}", min_x, (seg_min + seg_max) * 0.5, half_thickness, (seg_max - seg_min) * 0.5, floor_z, wall_height))

    for col_index in range(1, cols):
        wall_x = min_x + col_index * room_size
        for row_index in range(rows):
            room_y_min = min_y + row_index * room_size
            room_y_max = room_y_min + room_size
            door_center = (room_y_min + room_y_max) * 0.5
            for seg_index, (seg_min, seg_max) in enumerate(split_segments(room_y_min, room_y_max, [(door_center, door_width)])):
                walls.append(
                    Wall(
                        f"{prefix}_wall_v_{col_index}_{row_index}_{seg_index}",
                        wall_x,
                        (seg_min + seg_max) * 0.5,
                        half_thickness,
                        (seg_max - seg_min) * 0.5,
                        floor_z,
                        wall_height,
                    )
                )
    for row_index in range(1, rows):
        wall_y = min_y + row_index * room_size
        for col_index in range(cols):
            room_x_min = min_x + col_index * room_size
            room_x_max = room_x_min + room_size
            door_center = (room_x_min + room_x_max) * 0.5
            openings = [(door_center, door_width)]
            openings.extend(extra_horizontal_doors.get((row_index, col_index), []))
            for seg_index, (seg_min, seg_max) in enumerate(split_segments(room_x_min, room_x_max, openings)):
                walls.append(
                    Wall(
                        f"{prefix}_wall_h_{row_index}_{col_index}_{seg_index}",
                        (seg_min + seg_max) * 0.5,
                        wall_y,
                        (seg_max - seg_min) * 0.5,
                        half_thickness,
                        floor_z,
                        wall_height,
                    )
                )
    return walls


def make_stairs(prefix, lower_floor_z, upper_floor_z, start_x, end_x, center_y, stair_width, step_count, side_walls=False):
    steps = []
    step_run = (end_x - start_x) / step_count
    step_height = (upper_floor_z - lower_floor_z) / step_count
    for step_index in range(step_count):
        top_height = lower_floor_z + (step_index + 1) * step_height
        center_x = start_x + (step_index + 0.5) * step_run
        steps.append(
            box_xml(
                f"{prefix}_step_{step_index:02d}",
                center_x,
                center_y,
                (lower_floor_z + top_height) * 0.5,
                abs(step_run) * 0.5,
                stair_width * 0.5,
                (top_height - lower_floor_z) * 0.5,
                "stair_mat",
            )
        )
        material = "feature_floor_dark" if step_index % 2 == 0 else "feature_floor_light"
        steps.append(
            feature_box_xml(
                f"feature_{prefix}_tread_{step_index:02d}",
                center_x,
                center_y,
                top_height + 0.010,
                abs(step_run) * 0.34,
                stair_width * 0.36,
                0.003,
                material,
            )
        )
    if side_walls:
        rail_height = 1.15
        rail_size_z = rail_height * 0.5
        rail_size_x = abs(step_run) * 0.5
        left_y = center_y + stair_width * 0.5 + 0.07
        right_y = center_y - stair_width * 0.5 - 0.07
        for step_index in range(step_count):
            top_height = lower_floor_z + (step_index + 1) * step_height
            center_x = start_x + (step_index + 0.5) * step_run
            rail_z = top_height + rail_size_z
            steps.append(box_xml(f"{prefix}_left_side_wall_{step_index:02d}", center_x, left_y, rail_z, rail_size_x, 0.045, rail_size_z, "room_wall"))
            steps.append(box_xml(f"{prefix}_right_side_wall_{step_index:02d}", center_x, right_y, rail_z, rail_size_x, 0.045, rail_size_z, "room_wall"))
    return steps


def make_stair_flight(
    prefix,
    base_z,
    top_z,
    center_x,
    start_y,
    end_y,
    stair_width,
    step_count,
    side_walls=True,
):
    geoms = []
    step_run = (end_y - start_y) / step_count
    step_height = (top_z - base_z) / step_count
    for step_index in range(step_count):
        previous_height = base_z + step_index * step_height
        tread_z = base_z + (step_index + 1) * step_height
        center_y = start_y + (step_index + 0.5) * step_run
        geoms.append(
            box_xml(
                f"{prefix}_step_{step_index:02d}",
                center_x,
                center_y,
                (previous_height + tread_z) * 0.5,
                stair_width * 0.5,
                abs(step_run) * 0.5,
                step_height * 0.5,
                "stair_mat",
            )
        )
        material = "feature_floor_dark" if step_index % 2 == 0 else "feature_floor_light"
        geoms.append(
            feature_box_xml(
                f"feature_{prefix}_tread_{step_index:02d}",
                center_x,
                center_y,
                tread_z + 0.004,
                stair_width * 0.32,
                abs(step_run) * 0.34,
                0.001,
                material,
            )
        )
    if side_walls:
        rail_height = 1.15
        rail_size_z = rail_height * 0.5
        rail_size_y = abs(step_run) * 0.5
        left_x = center_x + stair_width * 0.5 + 0.07
        right_x = center_x - stair_width * 0.5 - 0.07
        for step_index in range(step_count):
            tread_z = base_z + (step_index + 1) * step_height
            center_y = start_y + (step_index + 0.5) * step_run
            rail_z = tread_z + rail_size_z
            geoms.append(box_xml(f"{prefix}_left_side_wall_{step_index:02d}", left_x, center_y, rail_z, 0.045, rail_size_y, rail_size_z, "room_wall"))
            geoms.append(box_xml(f"{prefix}_right_side_wall_{step_index:02d}", right_x, center_y, rail_z, 0.045, rail_size_y, rail_size_z, "room_wall"))
    return geoms


def make_switchback_stairs(
    prefix,
    lower_floor_z,
    upper_floor_z,
    east_wall_x,
    center_y,
    stair_width,
    step_count_per_flight,
    stair_run_half=2.10,
):
    mid_z = (lower_floor_z + upper_floor_z) * 0.5
    y_low = center_y - stair_run_half
    y_high = center_y + stair_run_half
    x_first = east_wall_x + 3.10
    x_second = east_wall_x + 7.80
    landing_x = (x_first + x_second) * 0.5
    landing_half_x = (x_second - x_first) * 0.5 + stair_width * 0.5 + 0.10
    landing_depth = 1.0
    landing_center_y = y_high + landing_depth * 0.5
    geoms = [
        floor_xml(f"{prefix}_mid_landing", landing_x, landing_center_y, mid_z, landing_half_x, landing_depth * 0.5),
        box_xml(f"{prefix}_mid_landing_north_guard", landing_x, y_high + 1.02, mid_z + 0.85, landing_half_x + 0.10, 0.055, 0.85, "room_wall"),
        box_xml(f"{prefix}_mid_landing_west_guard", landing_x - landing_half_x, landing_center_y, mid_z + 0.85, 0.055, landing_depth * 0.5 - 0.02, 0.85, "room_wall"),
        box_xml(f"{prefix}_mid_landing_east_guard", landing_x + landing_half_x, landing_center_y, mid_z + 0.85, 0.055, landing_depth * 0.5 - 0.02, 0.85, "room_wall"),
    ]
    geoms.extend(
        make_stair_flight(
            f"{prefix}_flight_a",
            lower_floor_z,
            mid_z,
            x_first,
            y_low,
            y_high,
            stair_width,
            step_count_per_flight,
            side_walls=True,
        )
    )
    geoms.extend(
        make_stair_flight(
            f"{prefix}_flight_b",
            mid_z,
            upper_floor_z,
            x_second,
            y_high,
            y_low,
            stair_width,
            step_count_per_flight,
            side_walls=True,
        )
    )
    geoms.append(
        make_stair_rail_connector_board(
            f"{prefix}_mid_landing_inner_rail_board",
            mid_z,
            x_first,
            x_second,
            y_high,
            stair_width,
        )
    )
    return geoms


def make_stair_hall_rails(prefix, center_x, center_y, floor_z, size_x, size_y, doorway_width):
    rail_height = 1.65
    rail_z = floor_z + rail_height * 0.5
    min_x = center_x - size_x
    max_x = center_x + size_x
    min_y = center_y - size_y
    max_y = center_y + size_y
    geoms = [
        box_xml(f"{prefix}_stair_hall_east_guard", max_x, center_y, rail_z, 0.060, size_y, rail_height * 0.5, "room_wall"),
        box_xml(f"{prefix}_stair_hall_north_guard", center_x, max_y, rail_z, size_x, 0.060, rail_height * 0.5, "room_wall"),
        box_xml(f"{prefix}_stair_hall_south_guard", center_x, min_y, rail_z, size_x, 0.060, rail_height * 0.5, "room_wall"),
    ]
    return geoms


def make_stair_rail_connector_board(name, floor_z, x_first, x_second, y_pos, stair_width):
    rail_overlap = 0.04
    board_height = 1.16
    left_inner_rail_x = x_first + stair_width * 0.5 + 0.07
    right_inner_rail_x = x_second - stair_width * 0.5 - 0.07
    center_x = (left_inner_rail_x + right_inner_rail_x) * 0.5
    half_x = max(0.05, (right_inner_rail_x - left_inner_rail_x) * 0.5 + rail_overlap)
    return box_xml(
        name,
        center_x,
        y_pos,
        floor_z + board_height,
        half_x,
        0.075,
        0.035,
        "wood_plank",
    )


def level_prefix(index, floors):
    if floors == 1:
        return "level_00"
    if index == 0:
        return "lower"
    if index == floors - 1:
        return "top"
    return f"middle_{index:02d}"


def make_level_obstacles(prefix, center_x, floor_z):
    return [
        box_xml(f"{prefix}_block_sw_table", center_x - 2.35, -2.15, floor_z + 0.36, 0.55, 0.85, 0.36),
        box_xml(f"{prefix}_block_nw_column", center_x - 2.25, 2.10, floor_z + 0.55, 0.38, 0.38, 0.55),
        box_xml(f"{prefix}_block_se_sofa", center_x + 2.25, -2.20, floor_z + 0.42, 0.95, 0.42, 0.42),
        box_xml(f"{prefix}_block_ne_box", center_x + 2.05, 2.10, floor_z + 0.32, 0.44, 0.70, 0.32),
    ]


def make_scene(
    rows=2,
    cols=2,
    room_size=5.0,
    wall_height=3.5,
    wall_thickness=0.12,
    door_width=4.8,
    include_file="elf3.xml",
    floor_height=4.2,
    floors=4,
    inter_level_gap=0.0,
    stair_width=2.2,
    stair_step_count=14,
):
    level_width = cols * room_size
    level_depth = rows * room_size
    first_center_x = room_size * 0.5
    center_y = room_size * 0.5

    floor_geoms = []
    walls = []
    stair_geoms = []
    obstacles = []
    floor_features = []

    level_centers = [first_center_x + index * inter_level_gap for index in range(floors)]
    level_floor_z = [index * floor_height for index in range(floors)]
    stair_run_half = 2.10
    stair_hole_margin = 0.12
    stair_hole_y_margin = 0.0
    for index, (level_center_x, floor_z) in enumerate(zip(level_centers, level_floor_z)):
        prefix = level_prefix(index, floors)
        east_x = level_center_x + level_width * 0.5
        outer_doors = [Door("east", center_y, door_width)]
        extra_horizontal_doors = {(1, cols - 1): [(east_x - 0.80, 2.6)]}
        level_walls = make_level_walls(
            prefix,
            rows,
            cols,
            room_size,
            level_center_x,
            center_y,
            floor_z,
            wall_height,
            wall_thickness,
            door_width,
            outer_doors,
            extra_horizontal_doors=extra_horizontal_doors,
        )
        walls.extend(level_walls)
        floor_geoms.append(floor_xml(f"{prefix}_floor", level_center_x, center_y, floor_z, level_width * 0.5 + 0.35, level_depth * 0.5 + 0.35))
        stair_hall_center_x = east_x + 4.65
        stair_hall_size_x = 5.90
        stair_hall_size_y = 5.35
        stair_holes = []
        if index > 0:
            incoming_stair_x = east_x + 7.80
            incoming_hole_min_y = center_y - stair_run_half - stair_hole_y_margin
            incoming_hole_max_y = center_y + stair_run_half + stair_hole_y_margin
            stair_holes.append(
                (
                    incoming_stair_x,
                    (incoming_hole_min_y + incoming_hole_max_y) * 0.5,
                    stair_width * 0.5 + stair_hole_margin,
                    (incoming_hole_max_y - incoming_hole_min_y) * 0.5,
                )
            )
        floor_geoms.extend(
            floor_tiles_with_holes(
                f"{prefix}_stair_hall",
                stair_hall_center_x,
                center_y,
                floor_z,
                stair_hall_size_x,
                stair_hall_size_y,
                stair_holes,
            )
        )
        floor_geoms.append(feature_box_xml(f"{prefix}_stair_doorway_marker", east_x + 1.05, center_y, floor_z + 0.020, 1.80, door_width * 0.46, 0.004, "feature_floor_light"))
        stair_geoms.extend(make_stair_hall_rails(prefix, stair_hall_center_x, center_y, floor_z, stair_hall_size_x, stair_hall_size_y, door_width))
        if 0 < index < floors - 1:
            stair_geoms.append(
                make_stair_rail_connector_board(
                    f"{prefix}_floor_inner_rail_board",
                    floor_z,
                    east_x + 3.10,
                    east_x + 7.80,
                    center_y - stair_run_half,
                    stair_width,
                )
            )
        obstacles.extend(make_level_obstacles(prefix, level_center_x, floor_z))
        floor_features.extend(make_floor_features(prefix, level_center_x, center_y, level_width, level_depth, floor_z))

    for index in range(floors - 1):
        lower_floor_z = level_floor_z[index]
        upper_floor_z = level_floor_z[index + 1]
        lower_east_x = level_centers[index] + level_width * 0.5
        prefix = f"stair_{index:02d}_{index + 1:02d}"
        stair_geoms.extend(
            make_switchback_stairs(
                prefix=prefix,
                lower_floor_z=lower_floor_z,
                upper_floor_z=upper_floor_z,
                east_wall_x=lower_east_x,
                center_y=center_y,
                stair_width=stair_width,
                step_count_per_flight=stair_step_count,
                stair_run_half=stair_run_half,
            )
        )

    lower_floor_z = 0.0
    spawn_marker = [
        feature_box_xml("spawn_marker_line_x", 0.8, -1.0, lower_floor_z + 0.020, 0.45, 0.025, 0.004, "spawn_marker"),
        feature_box_xml("spawn_marker_line_y", 0.8, -1.0, lower_floor_z + 0.024, 0.025, 0.45, 0.004, "spawn_marker"),
        feature_box_xml("spawn_marker_pole", 0.38, -1.42, lower_floor_z + 0.55, 0.035, 0.035, 0.55, "spawn_marker"),
    ]
    wall_features = make_wall_features(walls)
    all_geoms = (
        floor_geoms
        + [wall_xml(wall) for wall in walls]
        + stair_geoms
        + obstacles
        + spawn_marker
        + wall_features
        + floor_features
    )
    body = "\n".join(all_geoms)
    return f'''<mujoco model="elf3">
  <include file="{include_file}"/>
  <asset>
    <texture name="wall_feature_checker" type="2d" builtin="checker" rgb1="0.50 0.50 0.50" rgb2="0.88 0.88 0.84" width="256" height="256"/>
    <texture name="floor_feature_checker" type="2d" builtin="checker" rgb1="0.46 0.46 0.46" rgb2="0.74 0.74 0.70" width="256" height="256"/>
    <texture name="obstacle_feature_checker" type="2d" builtin="checker" rgb1="0.42 0.42 0.42" rgb2="0.82 0.80 0.74" width="192" height="192"/>
    <material name="room_floor" texture="floor_feature_checker" texuniform="true" texrepeat="18 18" rgba="0.62 0.62 0.60 1"/>
    <material name="room_wall" texture="wall_feature_checker" texuniform="true" texrepeat="18 10" rgba="0.72 0.72 0.70 1"/>
    <material name="room_obstacle" texture="obstacle_feature_checker" texuniform="true" texrepeat="6 6" rgba="0.66 0.66 0.62 1"/>
    <material name="stair_mat" rgba="0.58 0.58 0.56 1"/>
    <material name="wood_plank" rgba="0.56 0.34 0.16 1"/>
    <material name="spawn_marker" rgba="1.00 0.08 0.02 1"/>
    <material name="feature_dark" rgba="0.28 0.28 0.28 1"/>
    <material name="feature_light" rgba="0.92 0.90 0.82 1"/>
    <material name="feature_floor_dark" rgba="0.34 0.34 0.34 1"/>
    <material name="feature_floor_light" rgba="0.82 0.80 0.70 1"/>
  </asset>
  <worldbody>
{body}
  </worldbody>
</mujoco>
'''


def parse_args():
    parser = argparse.ArgumentParser(description="Generate an ELF3 multi-floor multi-room MJCF navigation scene.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rows", type=int, default=2)
    parser.add_argument("--cols", type=int, default=2)
    parser.add_argument("--floors", type=int, default=4)
    parser.add_argument("--room-size", type=float, default=5.0)
    parser.add_argument("--wall-height", type=float, default=3.5)
    parser.add_argument("--wall-thickness", type=float, default=0.12)
    parser.add_argument("--door-width", type=float, default=4.8)
    parser.add_argument("--floor-height", type=float, default=4.2)
    parser.add_argument("--inter-level-gap", type=float, default=0.0)
    parser.add_argument("--stair-width", type=float, default=2.2)
    parser.add_argument("--stair-step-count", type=int, default=14)
    parser.add_argument("--include-file", default="elf3.xml")
    return parser.parse_args()


def main():
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        make_scene(
            rows=args.rows,
            cols=args.cols,
            floors=args.floors,
            room_size=args.room_size,
            wall_height=args.wall_height,
            wall_thickness=args.wall_thickness,
            door_width=args.door_width,
            include_file=args.include_file,
            floor_height=args.floor_height,
            inter_level_gap=args.inter_level_gap,
            stair_width=args.stair_width,
            stair_step_count=args.stair_step_count,
        ),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
