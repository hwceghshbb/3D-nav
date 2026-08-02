#!/usr/bin/env bash
set +u

cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export DISPLAY="${DISPLAY:-:1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_ideal_line_sweep}"
mkdir -p "$ROS_LOG_DIR"

duration="${IDEAL_LINE_DURATION:-18s}"
results="${IDEAL_LINE_RESULTS:-/tmp/ideal_line_pid_sweep.tsv}"
: > "$results"
printf 'score\tkp\tkd\trms_offset\tmax_offset\tsaturated_ratio\tlog\n' | tee "$results"

for pair in "0.05 0.00" "0.08 0.02" "0.10 0.03" "0.10 0.05" "0.12 0.05" "0.16 0.05"; do
  read -r kp kd <<< "$pair"
  log="/tmp/ideal_line_kp${kp//./p}_kd${kd//./p}.log"
  echo "testing kp=$kp kd=$kd"
  timeout "$duration" ros2 launch bxi_example_py_elf3 example_launch_ideal_line_pid.py \
    line_pid_kp:="$kp" line_pid_ki:=0.0 line_pid_kd:="$kd" \
    error_sign:=-1.0 line_max_yawdot:=0.10 \
    line_max_amp_input_yawdot:=0.10 line_max_yawdot_rate:=0.50 \
    > "$log" 2>&1 || true

  metrics=$(python3 - "$log" <<'PY'
import re
import sys

values = []
for line in open(sys.argv[1], errors="ignore"):
    match = re.search(r"offset=([-+0-9.eE]+)", line)
    if match:
        values.append(float(match.group(1)))
if not values:
    print("999\t999\t999\t999")
else:
    abs_values = [abs(value) for value in values]
    rms = (sum(value * value for value in values) / len(values)) ** 0.5
    maximum = max(abs_values)
    saturated = sum(value >= 0.98 for value in abs_values) / len(values)
    score = rms + 0.5 * maximum + 0.4 * saturated
    print(f"{score:.6f}\t{rms:.6f}\t{maximum:.6f}\t{saturated:.6f}")
PY
)
  score=$(cut -f1 <<< "$metrics")
  rms=$(cut -f2 <<< "$metrics")
  maximum=$(cut -f3 <<< "$metrics")
  saturated=$(cut -f4 <<< "$metrics")
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$score" "$kp" "$kd" "$rms" "$maximum" "$saturated" "$log" | tee -a "$results"
done

echo
echo "best candidate:"
sort -t $'\t' -k1,1n "$results" | sed -n '2p'
