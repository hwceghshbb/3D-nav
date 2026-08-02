#!/usr/bin/env python3
import argparse
import itertools
import math


def reference_center(x_position, profile):
    if profile == "straight":
        return 0.0
    if profile == "curve":
        return 0.24 * math.sin(2.0 * math.pi * x_position / 14.0)
    return (
        0.28 * math.sin(2.0 * math.pi * x_position / 18.0)
        + 0.08 * math.sin(2.0 * math.pi * x_position / 7.0)
    )


def run_trial(kp, kd, yaw_limit, yaw_rate_limit, profile, noise_level):
    dt = 0.02
    speed = 1.5
    duration = 30.0
    lateral_position = 0.04
    heading = 0.0
    previous_error = 0.04
    yaw_command = 0.0
    measured_error = 0.0
    derivative_error = 0.0
    errors = []
    saturated_steps = 0
    direction_changes = 0
    previous_yaw = 0.0

    for step in range(int(duration / dt)):
        x_position = speed * step * dt
        center = reference_center(x_position, profile)
        error = center - lateral_position
        sensor_error = error + noise_level * math.sin(37.0 * x_position + 0.7)
        measured_error = 0.82 * measured_error + 0.18 * sensor_error
        derivative = (measured_error - previous_error) / dt
        derivative_error = 0.82 * derivative_error + 0.18 * derivative
        target_yaw = kp * measured_error + kd * derivative_error
        target_yaw = max(-yaw_limit, min(yaw_limit, target_yaw))
        max_step = yaw_rate_limit * dt
        yaw_delta = max(-max_step, min(max_step, target_yaw - yaw_command))
        yaw_command += yaw_delta
        lateral_position += speed * heading * dt
        heading += yaw_command * dt
        previous_error = measured_error
        errors.append(error)
        saturated_steps += int(abs(target_yaw) >= yaw_limit * 0.995)
        direction_changes += int(yaw_command * previous_yaw < 0.0)
        previous_yaw = yaw_command

    rms_error = math.sqrt(sum(value * value for value in errors) / len(errors))
    max_error = max(abs(value) for value in errors)
    saturation_ratio = saturated_steps / len(errors)
    oscillation_ratio = direction_changes / len(errors)
    score = rms_error + 0.45 * max_error + 0.20 * saturation_ratio + 0.08 * oscillation_ratio
    return score, rms_error, max_error, saturation_ratio, oscillation_ratio


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaw-limit", type=float, default=0.10)
    parser.add_argument("--noise", type=float, default=0.008)
    args = parser.parse_args()
    candidates = {}
    for kp, kd, yaw_limit, yaw_rate_limit in itertools.product(
        (0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.10),
        (0.00, 0.02, 0.05, 0.08, 0.10, 0.14, 0.18, 0.22),
        (args.yaw_limit,),
        (0.25, 0.35, 0.50, 0.70),
    ):
        for profile in ("straight", "mixed", "curve"):
            result = run_trial(kp, kd, yaw_limit, yaw_rate_limit, profile, args.noise)
            key = (kp, kd, yaw_limit, yaw_rate_limit)
            candidates.setdefault(key, []).append(result)

    ranked = []
    for (kp, kd, yaw_limit, yaw_rate_limit), results in candidates.items():
        score = sum(result[0] for result in results) / len(results)
        rms_error = sum(result[1] for result in results) / len(results)
        max_error = max(result[2] for result in results)
        saturation_ratio = sum(result[3] for result in results) / len(results)
        oscillation_ratio = sum(result[4] for result in results) / len(results)
        ranked.append((score, kp, kd, yaw_limit, yaw_rate_limit, rms_error, max_error, saturation_ratio, oscillation_ratio))

    ranked.sort()
    print("score kp kd yaw_limit yaw_rate rms_error max_error saturation oscillation")
    for result in ranked[:10]:
        print("%.5f %.2f %.2f %.2f %.2f %.5f %.5f %.3f %.3f" % result)


if __name__ == "__main__":
    main()
