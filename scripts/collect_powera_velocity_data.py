#!/usr/bin/env python3
"""Collect PowerA joystick velocity commands without ROS.

Mapping:
  left stick Y (axis 1) -> x/vx
  left stick X (axis 0) -> y/vy
  right stick X (axis 2) -> yaw

Buttons:
  A (0): start recording
  B (1): save and exit
  X (3): pause/resume recording
  Y (4): increase translational speed by 0.5 m/s
"""

from __future__ import annotations

import argparse
import select
import struct
import importlib.util
import sys
import time
from pathlib import Path

RL_ROOT = Path(__file__).resolve().parents[2] / "RL"
CATALOG_MODULE_PATH = RL_ROOT / "track_rl" / "track_catalog.py"
_catalog_spec = importlib.util.spec_from_file_location("powera_track_catalog", CATALOG_MODULE_PATH)
if _catalog_spec is None or _catalog_spec.loader is None:
    raise RuntimeError(f"cannot load track catalog module: {CATALOG_MODULE_PATH}")
_catalog_module = importlib.util.module_from_spec(_catalog_spec)
sys.modules[_catalog_spec.name] = _catalog_module
_catalog_spec.loader.exec_module(_catalog_module)
DEFAULT_CATALOG = _catalog_module.DEFAULT_CATALOG
get_track = _catalog_module.get_track


EVENT_FORMAT = "@IhBB"
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
AXIS_MAX = 32767.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Collect PowerA velocity data without ROS")
    parser.add_argument("--device", type=Path, default=Path("/dev/input/js0"))
    parser.add_argument("--output", type=Path, default=Path("data/powera_velocity/session.npz"))
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--track", type=str, default="straight_50m")
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--speed-step", type=float, default=0.5)
    parser.add_argument("--yaw-rate", type=float, default=1.0)
    parser.add_argument("--deadzone", type=float, default=0.03)
    parser.add_argument("--print-rate", type=float, default=10.0)
    parser.add_argument("--max-speed", type=float, default=5.0)
    parser.add_argument("--left-y-axis", type=int, default=1)
    parser.add_argument("--left-x-axis", type=int, default=0)
    parser.add_argument("--yaw-axis", type=int, default=2)
    parser.add_argument("--a-button", type=int, default=0)
    parser.add_argument("--b-button", type=int, default=1)
    parser.add_argument("--x-button", type=int, default=3)
    parser.add_argument("--y-button", type=int, default=4)
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


def device_name(device: Path) -> str:
    try:
        return (Path("/sys/class/input") / device.name / "device/name").read_text().strip()
    except OSError:
        return "unknown"


def main() -> int:
    args = parse_args()
    if not args.device.exists():
        print(f"joystick device not found: {args.device}", file=sys.stderr)
        return 1
    if args.speed < 0.0 or args.speed_step < 0.0 or args.max_speed <= 0.0:
        print("speed values must be non-negative and max-speed must be positive", file=sys.stderr)
        return 2
    try:
        track = get_track(args.track, args.catalog)
    except (OSError, ValueError, KeyError) as exc:
        print(f"invalid track selection: {exc}", file=sys.stderr)
        return 2

    axes: dict[int, int] = {}
    buttons: dict[int, int] = {}
    previous_buttons: dict[int, int] = {}
    frames: list[list[int]] = []
    normalized_actions: list[list[float]] = []
    velocity_actions: list[list[float]] = []
    speeds: list[float] = []
    timestamps: list[float] = []
    recording = False
    paused = False
    speed = min(args.speed, args.max_speed)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    print(f"device: {args.device} ({device_name(args.device)})")
    print(f"track: {track.track_id} ({track.name})")
    print(f"model: {track.model_path}")
    print("A=start  B=save+exit  X=pause/resume  Y=+0.5 m/s")
    print(
        f"mapping: axis {args.left_y_axis}=vx, axis {args.left_x_axis}=vy, "
        f"axis {args.yaw_axis}=yaw"
    )
    print("Waiting for A...")

    def rising(button: int) -> bool:
        return bool(buttons.get(button, 0)) and not bool(previous_buttons.get(button, 0))

    def save() -> None:
        np = __import__("numpy")
        np.savez_compressed(
            args.output,
            raw_axes=np.asarray(frames, dtype=np.int16),
            normalized_actions=np.asarray(normalized_actions, dtype=np.float32),
            velocity_actions=np.asarray(velocity_actions, dtype=np.float32),
            speeds_mps=np.asarray(speeds, dtype=np.float32),
            timestamps=np.asarray(timestamps, dtype=np.float64),
            axis_x=np.asarray(args.left_y_axis, dtype=np.int32),
            axis_y=np.asarray(args.left_x_axis, dtype=np.int32),
            axis_yaw=np.asarray(args.yaw_axis, dtype=np.int32),
            track_id=np.asarray(track.track_id),
            track_name=np.asarray(track.name),
            track_geometry=np.asarray(track.geometry),
            track_model=np.asarray(str(track.model_path)),
            track_length_m=np.asarray(track.length_m, dtype=np.float32),
            lane_width_m=np.asarray(track.lane_width_m, dtype=np.float32),
        )

    try:
        with args.device.open("rb", buffering=0) as joystick:
            poller = select.poll()
            poller.register(joystick, select.POLLIN)
            next_print = 0.0
            while True:
                if poller.poll(100):
                    data = joystick.read(EVENT_SIZE * 32)
                    for offset in range(0, len(data) - EVENT_SIZE + 1, EVENT_SIZE):
                        _, value, event_type, number = struct.unpack(
                            EVENT_FORMAT, data[offset : offset + EVENT_SIZE]
                        )
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
                    speed = min(args.max_speed, speed + args.speed_step)
                    print(f"\nspeed={speed:.2f} m/s")

                vx_norm = normalize(axes.get(args.left_y_axis, 0), args.deadzone)
                vy_norm = normalize(axes.get(args.left_x_axis, 0), args.deadzone)
                yaw_norm = normalize(axes.get(args.yaw_axis, 0), args.deadzone)
                if args.invert_vx:
                    vx_norm = -vx_norm
                if args.invert_vy:
                    vy_norm = -vy_norm
                if args.invert_yaw:
                    yaw_norm = -yaw_norm
                velocity = [vx_norm * speed, vy_norm * speed, yaw_norm * args.yaw_rate]

                if recording and not paused:
                    now = time.time()
                    frames.append([axes.get(index, 0) for index in (args.left_y_axis, args.left_x_axis, args.yaw_axis)])
                    normalized_actions.append([vx_norm, vy_norm, yaw_norm])
                    velocity_actions.append(velocity)
                    speeds.append(speed)
                    timestamps.append(now)

                now = time.monotonic()
                if now >= next_print:
                    next_print = now + 1.0 / max(args.print_rate, 1.0)
                    status = "REC" if recording and not paused else "PAUSE" if paused else "IDLE"
                    print(
                        f"{status} samples={len(timestamps)} speed={speed:.2f} "
                        f"vx={velocity[0]:+.2f} vy={velocity[1]:+.2f} yaw={velocity[2]:+.2f}",
                        end="\r",
                        flush=True,
                    )
                previous_buttons = buttons.copy()
    except KeyboardInterrupt:
        save()
        print(f"\nsaved {len(timestamps)} samples to {args.output}")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
