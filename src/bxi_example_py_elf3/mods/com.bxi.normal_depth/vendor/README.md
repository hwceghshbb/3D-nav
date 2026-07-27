# RealSense vendor runtime

This directory contains the prebuilt, Mod-local RealSense runtime used by
`com.bxi.normal_depth`.

- Target: Linux x86_64, CPython 3.10 (`cp310`)
- `python/linux-x86_64-cpython-310/pyrealsense2`: PyPI
  `pyrealsense2==2.58.1.10581`
- `lib/linux-x86_64/librealsense2.so.2.57.7`: ROS Humble package
  `ros-humble-librealsense2==2.57.7-1jammy.20260324.115117`

SHA-256:

- `pyrealsense2.cpython-310-x86_64-linux-gnu.so`:
  `1e075c5740bb685b39396289cb636b6679c2ce4b0c7d95c5077dc545c79882a0`
- `librealsense2.so.2.57.7`:
  `c93d52c5ab2d91d79e2b591775b739d94bc548f7d365c4fdc496f6a4db5f46f6`

The Python extension includes the RealSense SDK implementation and dynamically
links the target system's glibc, libstdc++, libusb and libudev. Those base OS
libraries, ROS 2 itself and `sensor_msgs` are intentionally not bundled here.

The runtime only adds the directory matching its OS, CPU architecture and
Python ABI. For example, Linux aarch64 with CPython 3.10 looks for
`python/linux-aarch64-cpython-310`; because this bundle does not contain that
directory, it ignores the x86_64 artifacts and falls back to the host
`pyrealsense2`. Cross-platform pure Python dependencies belong in
`python/common`.

`licenses/` and the Python distribution metadata retain the upstream license
notices. Replace these artifacts as a unit when changing the target Python ABI,
CPU architecture or RealSense SDK version.
