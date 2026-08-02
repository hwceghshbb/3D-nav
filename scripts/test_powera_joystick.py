#!/usr/bin/env python3
"""Read a Linux joystick device and print mapped motion commands.

This is intentionally independent of ROS. It uses /dev/input/js* and the
Linux joystick API directly, so it is useful for validating a controller
before connecting it to a robot or a ROS node.
"""

from __future__ import annotations

import argparse
import select
import struct
import sys
import time
from pathlib import Path


# Linux joystick API: __u32 time, __s16 value, __u8 type, __u8 number.
EVENT_FORMAT = "@IhBB"
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
AXIS_MAX = 32767.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test a PowerA/Linux joystick without ROS. Press Ctrl-C to exit."
    )
    parser.add_argument("--device", type=Path, default=Path("/dev/input/js0"))
    parser.add_argument("--left-y-axis", type=int, default=1)
    parser.add_argument("--left-x-axis", type=int, default=0)
    parser.add_argument("--yaw-axis", type=int, default=2)
    parser.add_argument("--invert-vx", action="store_true", default=True)
    parser.add_argument("--no-invert-vx", action="store_false", dest="invert_vx")
    parser.add_argument("--invert-vy", action="store_true", default=True)
    parser.add_argument("--no-invert-vy", action="store_false", dest="invert_vy")
    parser.add_argument("--invert-yaw", action="store_true", default=True)
    parser.add_argument("--no-invert-yaw", action="store_false", dest="invert_yaw")
    parser.add_argument("--deadzone", type=float, default=0.03)
    parser.add_argument("--print-rate", type=float, default=10.0)
    parser.add_argument("--raw", action="store_true", help="also print raw events")
    parser.add_argument(
        "--all-axes",
        action="store_true",
        help="show every axis value in the status line",
    )
    parser.add_argument(
        "--no-cr",
        action="store_true",
        help="print each status update on a new line",
    )
    return parser.parse_args()


def normalize(value: int, deadzone: float) -> float:
    result = max(-1.0, min(1.0, value / AXIS_MAX))
    if abs(result) <= deadzone:
        return 0.0
    sign = -1.0 if result < 0.0 else 1.0
    return sign * (abs(result) - deadzone) / max(1e-6, 1.0 - deadzone)


def joystick_name(device: Path) -> str:
    name_path = Path("/sys/class/input") / device.name / "device/name"
    try:
        return name_path.read_text().strip()
    except OSError:
        return "unknown"


def main() -> int:
    args = parse_args()
    if not 0.0 <= args.deadzone < 1.0:
        print("--deadzone must be in [0, 1)", file=sys.stderr)
        return 2
    if not args.device.exists():
        print(f"joystick device not found: {args.device}", file=sys.stderr)
        print("Connect the PowerA controller and run: ls -l /dev/input/js*", file=sys.stderr)
        return 1

    axes: dict[int, int] = {}
    buttons: dict[int, int] = {}
    print(f"device: {args.device} ({joystick_name(args.device)})")
    print(
        "mapping: "
        f"axis {args.left_y_axis}=vx, axis {args.left_x_axis}=vy, "
        f"axis {args.yaw_axis}=yaw (right stick X); deadzone={args.deadzone:.2f}"
    )
    print("Move the sticks and press buttons; Ctrl-C exits.")

    with args.device.open("rb", buffering=0) as joystick:
        poller = select.poll()
        poller.register(joystick, select.POLLIN)
        next_print = 0.0
        try:
            while True:
                events = poller.poll(100)
                if events:
                    data = joystick.read(EVENT_SIZE * 32)
                    for offset in range(0, len(data) - EVENT_SIZE + 1, EVENT_SIZE):
                        _, value, event_type, number = struct.unpack(
                            EVENT_FORMAT, data[offset : offset + EVENT_SIZE]
                        )
                        event_type &= ~JS_EVENT_INIT
                        if event_type == JS_EVENT_AXIS:
                            axes[number] = value
                            if args.raw:
                                print(f"axis event: axis={number} raw={value}")
                        elif event_type == JS_EVENT_BUTTON:
                            buttons[number] = value
                            if args.raw:
                                print(f"button event: button={number} value={value}")

                now = time.monotonic()
                if now < next_print:
                    continue
                next_print = now + 1.0 / max(args.print_rate, 1.0)

                vx = normalize(axes.get(args.left_y_axis, 0), args.deadzone)
                vy = normalize(axes.get(args.left_x_axis, 0), args.deadzone)
                yaw = normalize(axes.get(args.yaw_axis, 0), args.deadzone)
                if args.invert_vx:
                    vx = -vx
                if args.invert_vy:
                    vy = -vy
                if args.invert_yaw:
                    yaw = -yaw
                pressed = [str(index) for index, value in sorted(buttons.items()) if value]
                fields = [
                    f"vx={vx:+.3f}",
                    f"vy={vy:+.3f}",
                    f"yaw={yaw:+.3f}",
                    f"buttons={','.join(pressed) if pressed else '-'}",
                ]
                if args.all_axes:
                    fields.append(
                        "axes=" + ",".join(
                            f"{index}:{normalize(value, 0.0):+.2f}"
                            for index, value in sorted(axes.items())
                        )
                    )
                print(" ".join(fields), end="\n" if args.no_cr else "\r", flush=True)
        except KeyboardInterrupt:
            print("\nexited")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
