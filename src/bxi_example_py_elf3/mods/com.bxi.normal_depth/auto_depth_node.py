from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time

from rclpy.logging import get_logger

from bxi_example_py_elf3.mod_api import NodeBuildContext

from .orbbec_depth_node import ORBBEC_DEFAULTS, OrbbecDepthPublisher
from .realsense_depth_node import (
    _DEFAULTS as REALSENSE_DEFAULTS,
    RealSenseDepthPublisher,
)


INTEL_USB_VENDOR = "8086"
ORBBEC_USB_VENDOR = "2bc5"
GEMINI_335_USB_PRODUCTS = frozenset({"0800"})

AUTO_DEFAULTS: dict[str, object] = {
    "camera_backend": "auto",
    "camera_preference": "orbbec",
    "camera_detect_timeout_sec": 5.0,
    "camera_detect_interval_sec": 0.2,
}


@dataclass(frozen=True)
class UsbDevice:
    vendor: str
    product: str
    name: str = ""


def scan_usb_devices(
    sysfs_root: Path = Path("/sys/bus/usb/devices"),
) -> tuple[UsbDevice, ...]:
    devices: list[UsbDevice] = []
    if not sysfs_root.is_dir():
        return ()
    for vendor_file in sorted(sysfs_root.glob("*/idVendor")):
        device_root = vendor_file.parent
        product_file = device_root / "idProduct"
        if not product_file.is_file():
            continue
        try:
            vendor = vendor_file.read_text(encoding="ascii").strip().lower()
            product = product_file.read_text(encoding="ascii").strip().lower()
            name_file = device_root / "product"
            name = (
                name_file.read_text(encoding="utf-8", errors="replace").strip()
                if name_file.is_file()
                else ""
            )
        except OSError:
            continue
        devices.append(UsbDevice(vendor, product, name))
    return tuple(devices)


def choose_backend(
    requested: str,
    preference: str,
    devices: tuple[UsbDevice, ...],
) -> str | None:
    if requested in ("realsense", "orbbec"):
        return requested
    if requested != "auto":
        raise ValueError("camera_backend must be 'auto', 'realsense' or 'orbbec'")
    if preference not in ("realsense", "orbbec"):
        raise ValueError("camera_preference must be 'realsense' or 'orbbec'")

    detected: set[str] = set()
    for device in devices:
        if device.vendor == INTEL_USB_VENDOR:
            detected.add("realsense")
        elif (
            device.vendor == ORBBEC_USB_VENDOR
            and device.product in GEMINI_335_USB_PRODUCTS
        ):
            detected.add("orbbec")
    if preference in detected:
        return preference
    other = "realsense" if preference == "orbbec" else "orbbec"
    return other if other in detected else None


def _select_backend(params: dict[str, object]) -> tuple[str, tuple[UsbDevice, ...]]:
    requested = params["camera_backend"]
    preference = params["camera_preference"]
    timeout = params["camera_detect_timeout_sec"]
    interval = params["camera_detect_interval_sec"]
    if not isinstance(requested, str):
        raise ValueError("camera_backend must be a string")
    if not isinstance(preference, str):
        raise ValueError("camera_preference must be a string")
    if (
        isinstance(timeout, bool)
        or not isinstance(timeout, (int, float))
        or timeout < 0
    ):
        raise ValueError("camera_detect_timeout_sec must be non-negative")
    if (
        isinstance(interval, bool)
        or not isinstance(interval, (int, float))
        or interval <= 0
    ):
        raise ValueError("camera_detect_interval_sec must be greater than zero")

    if requested != "auto":
        selected = choose_backend(requested, preference, ())
        assert selected is not None
        return selected, ()

    deadline = time.monotonic() + float(timeout)
    devices: tuple[UsbDevice, ...] = ()
    while True:
        devices = scan_usb_devices()
        selected = choose_backend(requested, preference, devices)
        if selected is not None:
            return selected, devices
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        time.sleep(min(float(interval), remaining))

    visible = ", ".join(
        f"{device.vendor}:{device.product}"
        + (f" ({device.name})" if device.name else "")
        for device in devices
    )
    raise RuntimeError(
        "no supported depth camera detected; expected Intel RealSense USB vendor "
        "8086 or Orbbec Gemini 335 2bc5:0800"
        + (f"; visible USB devices: {visible}" if visible else "")
    )


def create_node(context: NodeBuildContext):
    params = dict(context.params)
    allowed = set(AUTO_DEFAULTS) | set(REALSENSE_DEFAULTS) | set(ORBBEC_DEFAULTS)
    unknown = set(params) - allowed
    if unknown:
        raise ValueError(f"unknown automatic depth camera params: {sorted(unknown)}")
    auto_params = {
        name: params.get(name, default) for name, default in AUTO_DEFAULTS.items()
    }
    backend, devices = _select_backend(auto_params)
    detected = (
        ", ".join(f"{device.vendor}:{device.product}" for device in devices) or "forced"
    )
    get_logger(context.node_name).info(
        f"selected depth camera backend: backend={backend}, detected={detected}"
    )

    if backend == "orbbec":
        backend_params = {
            name: params.get(name, default) for name, default in ORBBEC_DEFAULTS.items()
        }
        return OrbbecDepthPublisher(
            context.node_name,
            context.mod_root,
            backend_params,
        )

    backend_params = {
        name: params.get(name, default) for name, default in REALSENSE_DEFAULTS.items()
    }
    return RealSenseDepthPublisher(context.node_name, backend_params)


__all__ = [
    "AUTO_DEFAULTS",
    "GEMINI_335_USB_PRODUCTS",
    "INTEL_USB_VENDOR",
    "ORBBEC_USB_VENDOR",
    "UsbDevice",
    "choose_backend",
    "create_node",
    "scan_usb_devices",
]
