#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV3D_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="${NAV3D_WORKSPACE:-$(cd "$NAV3D_ROOT/../.." && pwd)}"
MAP_ROOT="${MAP_ROOT:-$WORKSPACE_DIR/maps}"
MAP_NAME="${MAP_NAME:-elf3_full_test}"
VOXEL_SIZE="${VOXEL_SIZE:-0.10}"
DECIMATION="${DECIMATION:-4}"
MAX_RANGE="${MAX_RANGE:-4.5}"

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [map_name|database.db]" >&2
  exit 2
fi

if [[ $# -eq 1 ]]; then
  if [[ "$1" == *.db ]]; then
    RTABMAP_DATABASE="$(realpath -m "$1")"
    MAP_NAME="$(basename "${RTABMAP_DATABASE%.db}")"
    MAP_ROOT="$(dirname "$RTABMAP_DATABASE")"
  else
    MAP_NAME="$1"
  fi
fi

RTABMAP_DATABASE="${RTABMAP_DATABASE:-$MAP_ROOT/$MAP_NAME.db}"
OUTPUT_NAME="${OUTPUT_NAME:-${MAP_NAME}_octo}"
OUTPUT_PLY="$MAP_ROOT/${OUTPUT_NAME}_cloud.ply"

if [[ ! -f "$RTABMAP_DATABASE" ]]; then
  echo "RTAB-Map database not found: $RTABMAP_DATABASE" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash

echo "Exporting OctoPlanner point cloud"
echo "  database:   $RTABMAP_DATABASE"
echo "  output:     $OUTPUT_PLY"
echo "  voxel:      $VOXEL_SIZE m"
echo "  decimation: $DECIMATION"
echo "  max range:  $MAX_RANGE m"

rtabmap-export --cloud \
  --output "$OUTPUT_NAME" \
  --output_dir "$MAP_ROOT" \
  --voxel "$VOXEL_SIZE" \
  --decimation "$DECIMATION" \
  --max_range "$MAX_RANGE" \
  "$RTABMAP_DATABASE"

if [[ ! -s "$OUTPUT_PLY" ]]; then
  echo "Export finished without producing a non-empty PLY: $OUTPUT_PLY" >&2
  exit 1
fi

echo "OctoPlanner point cloud ready:"
ls -lh "$OUTPUT_PLY"
