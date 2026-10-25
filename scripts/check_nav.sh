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
  rehearsal|navigation|vision|cancellation|envelope|integrated|integrated_faults|preflight)
    python3 ros2_ws/src/common/tests/ros_integration.py "$mode" "$output"
    ;;
  all)
    for item in offline robustness rehearsal navigation vision cancellation envelope integrated integrated_faults preflight; do
      bash scripts/check_nav.sh "$item" "$output"
    done
    ;;
  *) echo 'Choose offline, robustness, rehearsal, navigation, vision, cancellation, envelope, integrated, integrated_faults, preflight or all.' >&2; exit 2 ;;
esac
