#!/usr/bin/env python3
"""Drive the ELF3 MuJoCo scene with a PowerA joystick and collect data."""

from __future__ import annotations

import argparse
import importlib.util
import math
import os
import select
import struct
import sys
import time
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-cache")

import numpy as np

RL_ROOT = Path(__file__).resolve().parents[2] / "RL"
if str(RL_ROOT) not in sys.path:
    sys.path.insert(0, str(RL_ROOT))

from track_rl.elf3_runner import FrozenElf3MujocoRunnerPolicy
from track_rl.runner_policy import HighLevelCommand, RunnerState

CATALOG_PATH = RL_ROOT / "track_rl" / "track_catalog.py"
catalog_spec = importlib.util.spec_from_file_location("powera_track_catalog", CATALOG_PATH)
if catalog_spec is None or catalog_spec.loader is None:
    raise RuntimeError(f"cannot load track catalog: {CATALOG_PATH}")
catalog_module = importlib.util.module_from_spec(catalog_spec)
sys.modules[catalog_spec.name] = catalog_module
catalog_spec.loader.exec_module(catalog_module)
DEFAULT_CATALOG = catalog_module.DEFAULT_CATALOG
get_track = catalog_module.get_track


EVENT_FORMAT = "@IhBB"
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
AXIS_MAX = 32767.0
FIXED_FORWARD_SPEED_MPS = 2.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", type=Path, default=Path("/dev/input/js0"))
    parser.add_argument("--track", default="straight_50m")
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--output", type=Path, default=Path("data/powera_mujoco/session.npz"))
    parser.add_argument("--elf3-root", type=Path, default=Path("/home/hwc/code/bxi_rl_controller_ros2_example/src/bxi_example_py_elf3"))
    parser.add_argument("--runner-model", type=Path, default=None)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--speed-step", type=float, default=0.0)
    parser.add_argument("--max-speed", type=float, default=5.0)
    parser.add_argument("--max-heading-angle", type=float, default=0.6)
    parser.add_argument("--deadzone", type=float, default=0.03)
    parser.add_argument("--left-y-axis", type=int, default=1)
    parser.add_argument("--left-x-axis", type=int, default=0)
    parser.add_argument("--yaw-axis", type=int, default=2)
    parser.add_argument("--a-button", type=int, default=0)
    parser.add_argument("--b-button", type=int, default=1)
    parser.add_argument("--x-button", type=int, default=3)
    parser.add_argument("--y-button", type=int, default=4)
    parser.add_argument("--camera-width", type=int, default=160)
    parser.add_argument("--camera-height", type=int, default=96)
    parser.add_argument("--world-width", type=int, default=640)
    parser.add_argument("--world-height", type=int, default=360)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--forward-sign", type=float, default=1.0, choices=(-1.0, 1.0))
    parser.add_argument("--initial-yaw-offset", type=float, default=math.pi)
    parser.add_argument("--invert-vx", action="store_true", dest="invert_vx", default=False)
    parser.add_argument("--no-invert-vx", action="store_false", dest="invert_vx")
    parser.add_argument("--no-invert-vy", action="store_false", dest="invert_vy", default=True)
    parser.add_argument("--invert-yaw", action="store_true", dest="invert_yaw", default=True)
    parser.add_argument("--no-invert-yaw", action="store_false", dest="invert_yaw")
    return parser.parse_args()


def normalize(value: int, deadzone: float) -> float:
    result = max(-1.0, min(1.0, value / AXIS_MAX))
    if abs(result) <= deadzone:
        return 0.0
    sign = -1.0 if result < 0.0 else 1.0
    return sign * (abs(result) - deadzone) / max(1e-6, 1.0 - deadzone)


def save_dataset(path: Path, data: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **data)


def main() -> int:
    args = parse_args()
    if not args.device.exists():
        print(f"joystick device not found: {args.device}", file=sys.stderr)
        return 1
    track = get_track(args.track, args.catalog)
    os.environ["MUJOCO_GL"] = "glfw" if not args.no_viewer else "egl"
    runner = FrozenElf3MujocoRunnerPolicy(
        elf3_root=args.elf3_root,
        model_path=args.runner_model,
        xml_path=track.model_path,
        max_speed_mps=args.max_speed,
        max_heading_angle_rad=args.max_heading_angle,
        forward_sign=args.forward_sign,
    )
    mujoco = runner._mujoco
    start_x = track.start.x_m
    start_y = track.start.y_m
    start_yaw = track.start.yaw_rad
    state = RunnerState(
        x=start_x,
        y=start_y,
        z=track.start.z_m,
        yaw=start_yaw + args.initial_yaw_offset,
        speed=track.start.speed_mps,
    )
    runner.reset(state)
    world_renderer = mujoco.Renderer(runner.model, height=args.world_height, width=args.world_width)
    head_renderer = mujoco.Renderer(runner.model, height=args.camera_height, width=args.camera_width)
    world_camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(runner.model, world_camera)
    world_camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    world_camera.distance = 6.0
    world_camera.azimuth = 135.0
    world_camera.elevation = -25.0
    viewer = None
    if not args.no_viewer:
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(runner.model, runner.data)
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = runner.model.body("torso_link").id
        viewer.cam.distance = 4.0
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -25.0
        viewer.cam.lookat[:] = [state.x, state.y, 0.7]

    axes: dict[int, int] = {}
    buttons: dict[int, int] = {}
    previous_buttons: dict[int, int] = {}
    world_frames: list[np.ndarray] = []
    head_frames: list[np.ndarray] = []
    raw_axes: list[list[int]] = []
    normalized_actions: list[list[float]] = []
    commands: list[list[float]] = []
    states: list[list[float]] = []
    timestamps: list[float] = []
    recording = False
    paused = False
    speed = FIXED_FORWARD_SPEED_MPS
    poller = select.poll()

    def rising(button: int) -> bool:
        return bool(buttons.get(button, 0)) and not bool(previous_buttons.get(button, 0))

    def save() -> None:
        save_dataset(
            args.output,
            {
                "world_frames": np.asarray(world_frames, dtype=np.uint8),
                "head_frames": np.asarray(head_frames, dtype=np.uint8),
                "raw_axes": np.asarray(raw_axes, dtype=np.int16),
                "normalized_actions": np.asarray(normalized_actions, dtype=np.float32),
                "commands": np.asarray(commands, dtype=np.float32),
                "states": np.asarray(states, dtype=np.float32),
                "timestamps": np.asarray(timestamps, dtype=np.float64),
                "track_id": np.asarray(track.track_id),
                "track_name": np.asarray(track.name),
                "track_model": np.asarray(str(track.model_path)),
                "track_length_m": np.asarray(track.length_m, dtype=np.float32),
                "lane_width_m": np.asarray(track.lane_width_m, dtype=np.float32),
                "speed_step_mps": np.asarray(args.speed_step, dtype=np.float32),
            },
        )

    print(f"track: {track.track_id} ({track.name})")
    print(f"model: {track.model_path}")
    print("A=start  B=save+exit  X=pause/resume  Y=+0.5 m/s")
    print("fixed forward speed, left Y recorded as x/vx, left X=y/vy, right X=yaw")
    print("waiting for A...")

    try:
        with args.device.open("rb", buffering=0) as joystick:
            poller.register(joystick, select.POLLIN)
            while True:
                if poller.poll(0):
                    data = joystick.read(EVENT_SIZE * 32)
                    for offset in range(0, len(data) - EVENT_SIZE + 1, EVENT_SIZE):
                        _, value, event_type, number = struct.unpack(EVENT_FORMAT, data[offset : offset + EVENT_SIZE])
                        event_type &= ~JS_EVENT_INIT
                        if event_type == JS_EVENT_AXIS:
                            axes[number] = value
                        elif event_type == JS_EVENT_BUTTON:
                            buttons[number] = value

                if rising(args.a_button):
                    recording = True
                    paused = False
                    print("\nrecording started")
                if rising(args.b_button):
                    save()
                    print(f"\nsaved {len(timestamps)} samples to {args.output}")
                    return 0
                if rising(args.x_button) and recording:
                    paused = not paused
                    print(f"\n{'paused' if paused else 'resumed'}")
                if rising(args.y_button):
                    print("\nY ignored: forward speed is fixed at 2.00 m/s")

                yaw = normalize(axes.get(args.yaw_axis, 0), args.deadzone)
                if args.invert_yaw:
                    yaw = -yaw
                target_speed = FIXED_FORWARD_SPEED_MPS
                target_heading = yaw * args.max_heading_angle
                command = HighLevelCommand(target_speed=target_speed, target_heading_angle=target_heading)

                if recording and not paused:
                    world_camera.lookat[:] = [state.x + 8.0, state.y, 0.0]
                    world_renderer.update_scene(runner.data, camera=world_camera)
                    world_rgb = world_renderer.render().copy()
                    head_renderer.update_scene(runner.data, camera="front_line_camera")
                    head_rgb = head_renderer.render().copy()
                    world_frames.append(world_rgb)
                    head_frames.append(head_rgb)
                    raw_axes.append([axes.get(args.left_y_axis, 0), axes.get(args.left_x_axis, 0), axes.get(args.yaw_axis, 0)])
                    normalized_actions.append([0.0, 0.0, yaw])
                    commands.append([target_speed, 0.0, target_heading])
                    states.append([state.x, state.y, state.z, state.yaw, state.speed])
                    timestamps.append(time.time())

                if not paused:
                    state = runner.step(state, command, args.dt)
                if viewer is not None:
                    viewer.sync()
                    if not viewer.is_running():
                        save()
                        return 0
                print(
                    f"{'REC' if recording and not paused else 'PAUSE' if paused else 'IDLE'} "
                    f"samples={len(timestamps)} speed={speed:.2f} "
                    f"vx={target_speed:+.2f} vy=+0.00 yaw={target_heading:+.2f}",
                    end="\r",
                    flush=True,
                )
                previous_buttons = buttons.copy()
                time.sleep(args.dt)
    except KeyboardInterrupt:
        save()
        print(f"\nsaved {len(timestamps)} samples to {args.output}")
        return 0
    finally:
        if viewer is not None:
            viewer.close()
        world_renderer.close()
        head_renderer.close()
        runner.close()


if __name__ == "__main__":
    raise SystemExit(main())
