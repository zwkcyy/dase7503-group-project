#!/usr/bin/env bash
# Explicit verification command; never invoked by normal build or startup.
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:-offline}
output=${2:-"$repo/output/nav-validation-current"}
export PYTHONDONTWRITEBYTECODE=1
mkdir -p "$output"
cd "$repo"
case "$mode" in
  offline)
    python3 -m unittest discover -s ros2_ws/src/common/tests -v 2>&1 | tee "$output/offline.log"
    ;;
  robustness)
    python3 scripts/vision_robustness.py "$output/robustness"
    ;;
  ec_adapter)
    python3 ros2_ws/src/common/tests/ros_ec_adapter.py "$output"
    ;;
  launch_parameters)
    python3 ros2_ws/src/common/tests/ros_launch_parameters.py "$output"
    ;;
  motion_safety)
    python3 ros2_ws/src/common/tests/ros_motion_safety.py "$output"
    ;;
  collision_stress)
    python3 ros2_ws/src/common/tests/ros_collision_scenarios.py "$output"
    ;;
  collision_faults)
    python3 ros2_ws/src/common/tests/ros_collision_faults.py "$output" "${3:-}"
    ;;
  viewer)
    xvfb-run -a -s '-screen 0 1600x1000x24' python3 ros2_ws/src/common/tests/ros_viewer_check.py "$output"
    ;;
  rehearsal|navigation|vision|cancellation|envelope|integrated|integrated_faults|course|preflight)
    python3 ros2_ws/src/common/tests/ros_integration.py "$mode" "$output"
    ;;
  all)
    for item in offline robustness launch_parameters motion_safety rehearsal navigation vision cancellation envelope integrated integrated_faults course collision_stress collision_faults preflight ec_adapter; do
      bash scripts/check_nav.sh "$item" "$output"
    done
    ;;
  *) echo 'Choose offline, robustness, launch_parameters, motion_safety, rehearsal, navigation, vision, cancellation, envelope, integrated, integrated_faults, course, collision_stress, collision_faults, viewer, preflight, ec_adapter or all.' >&2; exit 2 ;;
esac
