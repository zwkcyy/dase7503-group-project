#!/usr/bin/env bash
# One entry point for the NAV/VI workflows. Run with bash; source is unnecessary.
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
workspace=${NAV_WORKSPACE:-"$HOME/dase7503-nav-work"}
command=${1:-help}
if (( $# )); then shift; fi
packages=(logistics_robot_description robot_geometry robot_description robot_navigation robot_mission robot_vision robot_bringup robot_simulation robot_gazebo)

load_ros() {
  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo 'ROS 2 Jazzy is missing. Install it on Ubuntu 24.04 first.' >&2; exit 2
  fi
  set +u
  source /opt/ros/jazzy/setup.bash
  if [[ -f "$workspace/install/setup.bash" ]]; then source "$workspace/install/setup.bash"; fi
  set -u
  export PYTHONDONTWRITEBYTECODE=1
}
simulation_env() {
  export ROS_DOMAIN_ID=${NAV_DEMO_DOMAIN:-75}
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
}
case "$command" in
  install)
    source /etc/os-release
    if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
      echo 'This dependency script targets Ubuntu 24.04 + ROS 2 Jazzy.' >&2; exit 2
    fi
    load_ros
    elevate=(); if (( EUID != 0 )); then elevate=(sudo); fi
    "${elevate[@]}" apt-get update
    "${elevate[@]}" apt-get install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
      ros-jazzy-robot-localization ros-jazzy-laser-filters ros-jazzy-xacro \
      ros-jazzy-cv-bridge python3-colcon-common-extensions python3-opencv \
      python3-numpy python3-yaml python3-tk python3-pil python3-pil.imagetk python3-matplotlib fonts-noto-cjk \
      python3-serial ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui
    ;;
  build)
    load_ros
    mkdir -p "$workspace/src"
    for group in nav vi common; do
      source_path="$repo/ros2_ws/src/$group"
      target="$workspace/src/$group"
      if [[ -e "$target" || -L "$target" ]]; then
        if [[ $(readlink -f "$target") != $(readlink -f "$source_path") ]]; then
          echo "Workspace source mismatch: $target; choose a different NAV_WORKSPACE." >&2; exit 2
        fi
      else
        ln -s "$source_path" "$target"
      fi
    done
    cd "$workspace"
    colcon build --symlink-install --packages-select "${packages[@]}" "$@"
    ;;
  gazebo)
    load_ros; simulation_env
    exec ros2 launch robot_gazebo course.launch.py "$@"
    ;;
  gazebo-install)
    load_ros
    elevate=(); if (( EUID != 0 )); then elevate=(sudo); fi
    "${elevate[@]}" apt-get update
    "${elevate[@]}" apt-get install -y ros-jazzy-ros-gz ros-jazzy-gz-sim-vendor
    bash "$repo/ros2_ws/src/vi/robot_vision/scripts/download_qr_models.sh"
    ;;
  gazebo-check)
    load_ros
    exec ros2 run robot_gazebo verify_gazebo "$@"
    ;;
  gazebo-faults)
    load_ros
    exec ros2 run robot_gazebo verify_gazebo_faults "$@"
    ;;
  gazebo-lidar-check)
    load_ros
    exec python3 -m robot_gazebo.lidar_checks "$@"
    ;;
  gazebo-visibility)
    load_ros
    exec python3 -m robot_gazebo.visibility "$@"
    ;;
  demo|regression-demo|rehearsal|navigation|vision)
    load_ros; simulation_env
    case "$command" in
      demo) launch=course_demo.launch.py ;;
      regression-demo) launch=integrated_demo.launch.py ;;
      rehearsal) launch=rehearsal.launch.py ;;
      navigation) launch=navigation_demo.launch.py ;;
      vision) launch=vision_replay.launch.py ;;
    esac
    exec ros2 launch robot_simulation "$launch" "$@"
    ;;
  doctor)
    load_ros; simulation_env
    exec ros2 run robot_bringup preflight "$@"
    ;;
  robot-doctor)
    load_ros
    exec ros2 run robot_bringup preflight "$@"
    ;;
  robot)
    load_ros
    exec ros2 launch robot_bringup system.launch.py "$@"
    ;;
  cad)
    load_ros; simulation_env
    exec ros2 launch robot_bringup cad_view.launch.py "$@"
    ;;
  hardware)
    load_ros
    exec ros2 launch robot_bringup cad_ec.launch.py "$@"
    ;;
  ec)
    load_ros
    exec ros2 launch robot_bringup ec_bridge.launch.py "$@"
    ;;
  check)
    load_ros
    exec bash "$repo/scripts/check_nav.sh" "$@"
    ;;
  report)
    exec python3 "$repo/scripts/export_nav_evidence.py" "$@"
    ;;
  help|--help|-h)
    echo 'Usage: bash scripts/nav.sh install|build|gazebo-install|gazebo|gazebo-check|gazebo-faults|gazebo-lidar-check|gazebo-visibility|demo|regression-demo|rehearsal|navigation|vision|doctor|robot-doctor|robot|cad|hardware|ec|check|report [arguments]'
    echo 'gazebo: Gazebo Harmonic physics + rendered camera + real NAV/VI; gazebo-check --output PATH runs verification.'
    echo 'gazebo-faults --output PATH: isolated simulated watchdog, intentional contact and bridge-failure checks.'
    echo 'gazebo-lidar-check --output PATH: visible obstacle braking and blind reverse refusal; gazebo-visibility --output PATH: CAD shadow analysis.'
    echo 'demo: course field + actual Nav2/QR; dynamic rack collisions, docking, four placements and synthetic lift.'
    echo 'regression-demo: original central-obstacle fixture with RViz; not the course field layout.'
    echo 'doctor: read-only readiness check on localhost domain 75; robot-doctor keeps your ROS environment.'
    echo 'cad: supplied CAD display. hardware: CAD + EC adapter; unloaded commissioning, no delivery mission.'
    echo 'ec: isolate an existing EC Python bridge and adapt its ROS topics; no automatic arm or lift completion.'
    echo 'NAV_WORKSPACE selects the build workspace; NAV_DEMO_DOMAIN selects the isolated simulation domain.'
    ;;
  *) echo "Unknown command: $command" >&2; exit 2 ;;
esac
