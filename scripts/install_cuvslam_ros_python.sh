#!/usr/bin/env bash
set -euo pipefail

CUVSLAM_SOURCE="${1:-/home/hwc/code/cuVSLAM}"
PYTHON_BIN="${ROS_PYTHON_BIN:-/usr/bin/python3}"

if [[ ! -f "${CUVSLAM_SOURCE}/build/bin/libcuvslam.so" ]]; then
  printf 'Missing %s/build/bin/libcuvslam.so\n' "${CUVSLAM_SOURCE}" >&2
  exit 1
fi

"${PYTHON_BIN}" -m pip install --user \
  'scikit-build-core>=0.10' 'nanobind>=2.10.2' pyyaml

CUVSLAM_BUILD_DIR="${CUVSLAM_SOURCE}/build" \
  "${PYTHON_BIN}" -m pip install --user --no-build-isolation \
  "${CUVSLAM_SOURCE}/python"

"${PYTHON_BIN}" -c \
  'import cuvslam; print(f"cuVSLAM {cuvslam.__version__} ready for ROS Python")'
