#!/usr/bin/env python3
"""Benchmark PyCuVSLAM against MuJoCo ground truth without using ROS odometry."""

import argparse
import json
import math
from pathlib import Path
import time

import cuvslam
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp
import yaml


CAMERA_NAMES = ("head_depth_camera", "body_depth_camera")
CV_FROM_MUJOCO_CAMERA = np.diag((1.0, -1.0, -1.0))
COLOR_VERTICAL_FOV_DEG = 43.173360


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--path", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument(
        "--camera",
        choices=("head", "body", "both"),
        default="head",
        help="RGB-D tracker(s) to benchmark.",
    )
    parser.add_argument(
        "--frame-distance",
        type=float,
        default=0.02,
        help="Distance in meters between rendered poses.",
    )
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--angular-frame-deg", type=float, default=1.0)
    parser.add_argument(
        "--orientation-source",
        choices=("original", "path_tangent"),
        default="original",
        help=(
            "Use orientations stored in the trajectory, or derive a smooth planar "
            "heading from the path positions."
        ),
    )
    parser.add_argument("--async-sba", action="store_true")
    parser.add_argument("--enable-slam", action="store_true")
    return parser.parse_args()


def load_last_ros_path(path):
    documents = yaml.safe_load_all(path.read_text(encoding="utf-8"))
    poses = None
    for document in documents:
        if isinstance(document, dict) and document.get("poses"):
            poses = document["poses"]
    if not poses:
        raise RuntimeError(f"No nav_msgs/Path document found in {path}")
    points = np.asarray(
        [
            [pose["pose"]["position"][axis] for axis in ("x", "y", "z")]
            for pose in poses
        ],
        dtype=np.float64,
    )
    quaternions = []
    for pose in poses:
        orientation = pose["pose"].get("orientation")
        if orientation is None or not all(
            axis in orientation for axis in ("x", "y", "z", "w")
        ):
            return points, None
        quaternions.append(
            [orientation[axis] for axis in ("x", "y", "z", "w")]
        )
    return points, Rotation.from_quat(np.asarray(quaternions, dtype=np.float64))


def load_rtabmap_poses(path):
    values = np.loadtxt(path)
    points = values[:, 1:4]
    rotations = Rotation.from_quat(values[:, 4:8])
    jumps = np.flatnonzero(np.linalg.norm(np.diff(points, axis=0), axis=1) > 1.0)
    boundaries = np.concatenate(([0], jumps + 1, [len(points)]))
    start, end = max(
        zip(boundaries[:-1], boundaries[1:]), key=lambda limits: limits[1] - limits[0]
    )
    return points[start:end], rotations[start:end]


def load_trajectory(path):
    first_line = path.read_text(encoding="utf-8", errors="replace").splitlines()[0]
    if first_line.startswith("#timestamp x y z qx qy qz qw"):
        return load_rtabmap_poses(path)
    return load_last_ros_path(path)


def resample_path(points, rotations, spacing, angular_spacing, max_frames):
    if rotations is None:
        tangent = np.gradient(points[:, :2], axis=0)
        yaw = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
        if len(yaw) >= 9:
            kernel = np.ones(9, dtype=np.float64) / 9.0
            yaw = np.convolve(
                np.pad(yaw, (4, 4), mode="edge"), kernel, mode="valid"
            )
        rotations = Rotation.from_euler("z", yaw[:, None])

    output_points = [points[0]]
    output_quaternions = [rotations[0].as_quat()]
    for index in range(len(points) - 1):
        translation = np.linalg.norm(points[index + 1] - points[index])
        relative_rotation = rotations[index].inv() * rotations[index + 1]
        angle = relative_rotation.magnitude()
        steps = max(
            1,
            int(math.ceil(translation / spacing)),
            int(math.ceil(angle / angular_spacing)),
        )
        fractions = np.linspace(0.0, 1.0, steps + 1)[1:]
        interpolated_rotations = Slerp(
            [0.0, 1.0], rotations[index : index + 2]
        )(fractions)
        for fraction, quaternion in zip(
            fractions, interpolated_rotations.as_quat()
        ):
            output_points.append(
                points[index] * (1.0 - fraction) + points[index + 1] * fraction
            )
            output_quaternions.append(quaternion)
            if max_frames > 0 and len(output_points) >= max_frames:
                return np.asarray(output_points), Rotation.from_quat(
                    output_quaternions
                )
    return np.asarray(output_points), Rotation.from_quat(output_quaternions)


def set_robot_pose(model, data, position, rotation):
    quaternion = rotation.as_quat()
    data.qpos[:] = model.qpos0
    data.qpos[:7] = (
        position[0],
        position[1],
        position[2],
        quaternion[3],
        quaternion[0],
        quaternion[1],
        quaternion[2],
    )
    for joint_name in ("head_z_joint", "head_y_joint"):
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
        )
        if joint_id >= 0:
            data.qpos[model.jnt_qposadr[joint_id]] = 0.0
    mujoco.mj_forward(model, data)


def hide_torso_like_sim_publisher(model, renderer):
    torso_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
    if torso_id < 0:
        return
    ids = np.flatnonzero(model.geom_bodyid == torso_id)
    model.geom_group[ids] = 5
    scene_option = getattr(renderer, "_scene_option", None)
    if scene_option is not None:
        scene_option.geomgroup[5] = 0


def create_camera(
    model, data, camera_id, width, height, base_position, world_from_base
):
    focal = height * 0.5 / math.tan(
        math.radians(COLOR_VERTICAL_FOV_DEG) * 0.5
    )
    camera = cuvslam.Camera(
        size=(width, height),
        focal=(focal, focal),
        principal=(width * 0.5, height * 0.5),
    )
    world_from_camera = (
        data.cam_xmat[camera_id].reshape(3, 3) @ CV_FROM_MUJOCO_CAMERA
    )
    base_from_camera = world_from_base.T @ world_from_camera
    camera.rig_from_camera = cuvslam.Pose(
        rotation=Rotation.from_matrix(base_from_camera).as_quat(),
        translation=world_from_base.T @ (data.cam_xpos[camera_id] - base_position),
    )
    return camera


def create_tracker(cameras, enable_slam, async_sba):
    settings = cuvslam.Tracker.OdometryRGBDSettings(
        depth_scale_factor=1000.0,
        depth_camera_id=0,
        enable_depth_stereo_tracking=False,
    )
    config = cuvslam.Tracker.OdometryConfig(
        async_sba=async_sba,
        enable_final_landmarks_export=True,
        odometry_mode=cuvslam.Tracker.OdometryMode.RGBD,
        rgbd_settings=settings,
    )
    slam_config = (
        cuvslam.Tracker.SlamConfig(sync_mode=True) if enable_slam else None
    )
    return cuvslam.Tracker(cuvslam.Rig(cameras), config, slam_config)


def set_camera_fov(model, camera_name, vertical_fov_deg):
    camera_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name
    )
    model.cam_fovy[camera_id] = vertical_fov_deg


def render_rgb(renderer, model, data, camera_name):
    set_camera_fov(model, camera_name, COLOR_VERTICAL_FOV_DEG)
    renderer.update_scene(data, camera=camera_name)
    return renderer.render().copy()


def render_aligned_depth_mm(renderer, model, data, camera_name):
    set_camera_fov(model, camera_name, COLOR_VERTICAL_FOV_DEG)
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera=camera_name)
    depth_m = renderer.render().copy()
    renderer.disable_depth_rendering()
    return np.clip(depth_m * 1000.0, 0.0, 65535.0).astype(np.uint16)


def rigid_alignment(estimate, truth):
    estimate_center = estimate.mean(axis=0)
    truth_center = truth.mean(axis=0)
    u, _, vt = np.linalg.svd(
        (estimate - estimate_center).T @ (truth - truth_center)
    )
    row_rotation = u @ vt
    if np.linalg.det(row_rotation) < 0.0:
        u[:, -1] *= -1.0
        row_rotation = u @ vt
    aligned = (estimate - estimate_center) @ row_rotation + truth_center
    return aligned, row_rotation.T


def initial_pose_alignment(estimate_positions, estimate_rotations, truth_positions, truth_rotations):
    world_alignment = truth_rotations[0] @ estimate_rotations[0].T
    aligned_positions = (
        (world_alignment @ (estimate_positions - estimate_positions[0]).T).T
        + truth_positions[0]
    )
    aligned_rotations = world_alignment[None, :, :] @ estimate_rotations
    return aligned_positions, aligned_rotations


def relative_pose_metrics(
    estimate_positions,
    estimate_rotations,
    truth_positions,
    truth_rotations,
    distance_m,
):
    cumulative = np.concatenate(
        (
            [0.0],
            np.cumsum(np.linalg.norm(np.diff(truth_positions, axis=0), axis=1)),
        )
    )
    translation_errors = []
    rotation_errors = []
    for start in range(len(cumulative)):
        end = int(np.searchsorted(cumulative, cumulative[start] + distance_m))
        if end >= len(cumulative):
            break
        estimate_delta = estimate_rotations[start].T @ (
            estimate_positions[end] - estimate_positions[start]
        )
        truth_delta = truth_rotations[start].T @ (
            truth_positions[end] - truth_positions[start]
        )
        translation_errors.append(np.linalg.norm(estimate_delta - truth_delta))
        estimate_relative_rotation = (
            estimate_rotations[start].T @ estimate_rotations[end]
        )
        truth_relative_rotation = truth_rotations[start].T @ truth_rotations[end]
        rotation_errors.append(
            Rotation.from_matrix(
                truth_relative_rotation.T @ estimate_relative_rotation
            ).magnitude()
        )
    if not translation_errors:
        return None
    translation_errors = np.asarray(translation_errors)
    rotation_errors = np.asarray(rotation_errors)
    return {
        "samples": int(len(translation_errors)),
        "translation_rmse_m": float(
            np.sqrt(np.mean(translation_errors**2))
        ),
        "translation_mean_m": float(np.mean(translation_errors)),
        "rotation_rmse_deg": float(
            np.degrees(np.sqrt(np.mean(rotation_errors**2)))
        ),
        "rotation_mean_deg": float(np.degrees(np.mean(rotation_errors))),
    }


def calculate_metrics(
    records, truth_positions, truth_rotations, frame_count, times_ms, observation_counts
):
    if len(records) < 2:
        return {"tracked_frames": len(records), "total_frames": frame_count}
    indices = np.asarray([record[0] for record in records], dtype=np.int64)
    estimate_positions = np.asarray([record[1] for record in records])
    estimate_rotations = Rotation.from_quat(
        np.asarray([record[2] for record in records])
    ).as_matrix()
    truth_positions = truth_positions[indices]
    truth_rotations = truth_rotations[indices]
    aligned, world_alignment = rigid_alignment(estimate_positions, truth_positions)
    errors = np.linalg.norm(aligned - truth_positions, axis=1)
    aligned_rotations = world_alignment[None, :, :] @ estimate_rotations
    rotation_errors = Rotation.from_matrix(
        np.swapaxes(truth_rotations, 1, 2) @ aligned_rotations
    ).magnitude()
    initial_aligned, initial_aligned_rotations = initial_pose_alignment(
        estimate_positions, estimate_rotations, truth_positions, truth_rotations
    )
    initial_errors = np.linalg.norm(initial_aligned - truth_positions, axis=1)
    initial_rotation_errors = Rotation.from_matrix(
        np.swapaxes(truth_rotations, 1, 2) @ initial_aligned_rotations
    ).magnitude()
    estimate_steps = np.linalg.norm(np.diff(estimate_positions, axis=0), axis=1)
    truth_steps = np.linalg.norm(np.diff(truth_positions, axis=0), axis=1)
    step_excess = estimate_steps - truth_steps
    max_step_index = int(np.argmax(estimate_steps))

    first_error_frames = {}
    for threshold in (0.1, 0.2, 0.5, 1.0):
        exceeded = np.flatnonzero(initial_errors > threshold)
        first_error_frames[f"{threshold:g}m"] = (
            int(indices[exceeded[0]]) if len(exceeded) else None
        )
    estimated_displacement = np.linalg.norm(
        estimate_positions[-1] - estimate_positions[0]
    )
    truth_displacement = np.linalg.norm(truth_positions[-1] - truth_positions[0])
    metrics = {
        "tracked_frames": int(len(records)),
        "total_frames": int(frame_count),
        "tracking_rate": float(len(records) / frame_count),
        "ate_rmse_m": float(np.sqrt(np.mean(errors**2))),
        "ate_mean_m": float(np.mean(errors)),
        "ate_max_m": float(np.max(errors)),
        "initial_aligned_rmse_m": float(np.sqrt(np.mean(initial_errors**2))),
        "initial_aligned_mean_m": float(np.mean(initial_errors)),
        "initial_aligned_max_m": float(np.max(initial_errors)),
        "initial_aligned_endpoint_error_m": float(initial_errors[-1]),
        "initial_aligned_rotation_rmse_deg": float(
            np.degrees(np.sqrt(np.mean(initial_rotation_errors**2)))
        ),
        "rotation_rmse_deg": float(
            np.degrees(np.sqrt(np.mean(rotation_errors**2)))
        ),
        "rotation_mean_deg": float(np.degrees(np.mean(rotation_errors))),
        "endpoint_scale": (
            float(estimated_displacement / truth_displacement)
            if truth_displacement > 1e-9
            else None
        ),
        "estimated_displacement_m": float(estimated_displacement),
        "truth_displacement_m": float(truth_displacement),
        "tracking_median_ms": float(np.median(times_ms)),
        "tracking_p95_ms": float(np.percentile(times_ms, 95)),
        "observations_median": float(np.median(observation_counts)),
        "observations_p05": float(np.percentile(observation_counts, 5)),
        "observations_min": int(np.min(observation_counts)),
        "max_estimated_frame_step_m": float(estimate_steps[max_step_index]),
        "truth_step_at_max_estimate_m": float(truth_steps[max_step_index]),
        "max_estimated_step_frame": int(indices[max_step_index + 1]),
        "max_frame_step_excess_m": float(np.max(step_excess)),
        "first_initial_aligned_error_frame": first_error_frames,
    }
    metrics["relative_pose_error"] = {
        f"{distance:g}m": relative_pose_metrics(
            estimate_positions,
            estimate_rotations,
            truth_positions,
            truth_rotations,
            distance,
        )
        for distance in (1.0, 5.0, 10.0)
    }
    return metrics


def main():
    args = parse_args()
    source_points, source_rotations = load_trajectory(args.path)
    if args.orientation_source == "path_tangent":
        source_rotations = None
    positions, rotations = resample_path(
        source_points,
        source_rotations,
        args.frame_distance,
        math.radians(args.angular_frame_deg),
        args.max_frames,
    )
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), args.width)
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), args.height)
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    hide_torso_like_sim_publisher(model, renderer)
    set_robot_pose(model, data, positions[0], rotations[0])
    camera_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        for name in CAMERA_NAMES
    ]
    cameras = [
        create_camera(
            model,
            data,
            camera_id,
            args.width,
            args.height,
            positions[0],
            rotations[0].as_matrix(),
        )
        for camera_id in camera_ids
    ]
    all_trackers = {
        "head_rgbd": (
            create_tracker([cameras[0]], args.enable_slam, args.async_sba),
            0,
        ),
        "body_rgbd": (
            create_tracker([cameras[1]], args.enable_slam, args.async_sba),
            1,
        ),
    }
    selected_names = {
        "head": ("head_rgbd",),
        "body": ("body_rgbd",),
        "both": tuple(all_trackers),
    }[args.camera]
    trackers = {name: all_trackers[name] for name in selected_names}
    records = {name: [] for name in trackers}
    times_ms = {name: [] for name in trackers}
    observation_counts = {name: [] for name in trackers}
    truth_rotations = []
    period_ns = int(round(1e9 / args.fps))

    try:
        for frame_index, (position, rotation) in enumerate(zip(positions, rotations)):
            set_robot_pose(model, data, position, rotation)
            camera_indices = sorted({value[1] for value in trackers.values()})
            images = {
                index: render_rgb(renderer, model, data, CAMERA_NAMES[index])
                for index in camera_indices
            }
            depths = {
                index: render_aligned_depth_mm(
                    renderer, model, data, CAMERA_NAMES[index]
                )
                for index in camera_indices
            }
            truth_rotations.append(rotation.as_matrix())
            timestamp_ns = (frame_index + 1) * period_ns
            for name, (tracker, camera_index) in trackers.items():
                started = time.perf_counter()
                odometry_pose, slam_pose = tracker.track(
                    timestamp_ns,
                    images=[images[camera_index]],
                    depths=[depths[camera_index]],
                )
                times_ms[name].append((time.perf_counter() - started) * 1000.0)
                observation_counts[name].append(
                    len(tracker.get_last_observations(0))
                )
                estimate = slam_pose
                if estimate is None and odometry_pose.world_from_rig is not None:
                    estimate = odometry_pose.world_from_rig.pose
                if estimate is not None:
                    records[name].append(
                        (
                            frame_index,
                            np.asarray(estimate.translation, dtype=np.float64),
                            np.asarray(estimate.rotation, dtype=np.float64),
                        )
                    )
            if (frame_index + 1) % 100 == 0:
                print(f"Processed {frame_index + 1}/{len(positions)} frames", flush=True)
    finally:
        renderer.close()

    truth_rotations = np.asarray(truth_rotations)
    report = {
        "cuvslam_version": cuvslam.get_version()[0],
        "model": str(args.model.resolve()),
        "path": str(args.path.resolve()),
        "source_waypoints": int(len(source_points)),
        "rendered_frames": int(len(positions)),
        "resolution": [args.width, args.height],
        "fps": args.fps,
        "camera_selection": args.camera,
        "frame_distance_m": args.frame_distance,
        "orientation_source": args.orientation_source,
        "async_sba": args.async_sba,
        "slam_enabled": args.enable_slam,
        "path_length_m": float(
            np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()
        ),
        "vertical_range_m": float(np.ptp(positions[:, 2])),
        "results": {
            name: calculate_metrics(
                records[name],
                positions,
                truth_rotations,
                len(positions),
                times_ms[name],
                observation_counts[name],
            )
            for name in trackers
        },
    }
    encoded = json.dumps(report, indent=2, ensure_ascii=True)
    print(encoded)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
