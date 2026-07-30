"""Launch the packaged GEAR-SONIC PICO manager in its isolated vendor Python.

Only the headless runtime dependency set required by ELF3 is vendored.
"""

from __future__ import annotations

import os
import sys

from bootstrap_python import ensure_clean_start


ensure_clean_start()

import runpy
from pathlib import Path

from runtime_config import PICO_PORT
from python_runtime import reexec_if_needed


MANAGER_IMPORTS = (
    "msgpack",
    "numpy",
    "scipy",
    "zmq",
    "pinocchio",
    "xrobotoolkit_sdk",
)
CONFIG_ERROR_EXIT_CODE = getattr(os, "EX_CONFIG", 78)
XRT_REQUIRED_CALLS = (
    "init",
    "close",
    "is_body_data_available",
    "get_body_joints_pose",
    "get_time_stamp_ns",
    "get_left_trigger",
    "get_right_trigger",
    "get_left_grip",
    "get_right_grip",
    "get_left_axis",
    "get_right_axis",
    "get_left_menu_button",
    "get_A_button",
    "get_B_button",
    "get_X_button",
    "get_Y_button",
)


def _vendor_root() -> Path:
    root = Path(__file__).resolve().parent
    manager = root / "gear_sonic" / "scripts" / "pico_manager_thread_server.py"
    if not manager.is_file():
        raise FileNotFoundError(f"Packaged PICO manager is missing: {manager}")
    return root


def _validate_manager_runtime() -> None:
    import xrobotoolkit_sdk as xrt

    missing = tuple(
        name for name in XRT_REQUIRED_CALLS if not callable(getattr(xrt, name, None))
    )
    if missing:
        raise RuntimeError(
            "xrobotoolkit_sdk is incompatible; missing callable API: "
            + ", ".join(missing)
        )
    service_root = Path(
        os.environ.get("SONIC_XRT_SERVICE_DIR", "/opt/apps/roboticsservice")
    ).expanduser()
    service_executable = service_root / "RoboticsServiceProcess"
    if not service_executable.is_file() or not os.access(service_executable, os.X_OK):
        raise RuntimeError(
            "RoboticsServiceProcess is not executable: "
            f"{service_executable}; set SONIC_XRT_SERVICE_DIR to its directory"
        )


def main() -> int:
    try:
        reexec_if_needed("pico_manager", MANAGER_IMPORTS)
        vendor_root = _vendor_root()
        _validate_manager_runtime()
    except (RuntimeError, FileNotFoundError) as exc:
        print(f"[sonic-pico] configuration error: {exc}", file=sys.stderr, flush=True)
        return CONFIG_ERROR_EXIT_CODE
    manager_script = (
        vendor_root / "gear_sonic" / "scripts" / "pico_manager_thread_server.py"
    )
    sys.path.insert(0, str(vendor_root))
    sys.argv[0] = str(manager_script)
    if not any(arg == "--port" or arg.startswith("--port=") for arg in sys.argv[1:]):
        sys.argv.extend(("--port", str(PICO_PORT)))
    runpy.run_path(str(manager_script), run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
