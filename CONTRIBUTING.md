# Contributing

DASE7503 Group Project — Smart Wheel-driven Logistics Robot

## 1. Repository layout and ownership

| Path | Content | Owner |
|---|---|---|
| `docs/course/` | Course documents | All |
| `docs/interfaces/` | Interface tables between groups | NAV |
| `cad/` | SolidWorks models, STL (3D printing), DXF (laser cutting) | CAD |
| `ec/firmware/` | ESP32 micro-ROS firmware | EC |
| `ec/wiring/` | Wiring diagrams, power supply, pin assignment | EC |
| `ros2_ws/src/common/` | `robot_interfaces` (custom msgs/actions), `robot_bringup` (launch files) | NAV |
| `ros2_ws/src/nav/` | `robot_description`, `robot_navigation`, `robot_mission` | NAV |
| `ros2_ws/src/vi/` | `robot_vision` (camera, QR code detection) | VI |
| `scripts/` | udev rules, environment setup | All |

Work only inside your own group's directories. If you need to change another group's files, tag that group's lead in the pull request.

## 2. Branches

- Never push directly to `main`. All changes go through a pull request.
- Branch name format: `<group>/<short-description>`, lowercase, words joined by `-`.
- Group prefixes: `cad/`, `ec/`, `nav/`, `vi/`, and `repo/` for repository-wide changes.

Examples:

```text
cad/chassis-v1
ec/motor-pid
nav/map-generator
vi/qr-detector
repo/contributing-guide
```

## 3. Workflow

```bash
# Before starting work: sync main
git checkout main
git pull

# Create your branch
git checkout -b nav/map-generator

# Commit your changes
git add .
git commit -m "[New]: Add map generator from field drawing"

# Push and open a pull request on GitHub
git push -u origin nav/map-generator
```

Before opening a pull request, run `git pull origin main` on your branch and resolve any conflicts.

## 4. Commit messages

Use this format:

```text
[Type]: concise English description
```

| Type | Use for |
|---|---|
| `New` | Add a new feature or file |
| `Modify` | Change existing behavior or implementation |
| `Fix` | Fix a bug or incorrect behavior |
| `Refactor` | Restructure code without changing behavior |
| `Docs` | Update documentation |
| `Test` | Add or update tests |
| `Chore` | Maintenance work (directory changes, configs, dependencies) |

Rules:

- Write in English, start with a capital letter, no period at the end.
- Use the imperative mood: "Add", "Fix", "Update" (not "Added", "Fixes").
- Keep the line within 72 characters.
- One commit, one logical change. Do not mix unrelated changes.

Examples:

```text
[New]: Add 3D-printed lift platform
[Modify]: Increase chassis wheelbase to 180 mm
[New]: Publish wheel odometry on /odom/wheel
[Fix]: Reverse M3 motor direction
[New]: Add Nav2 params with MPPI omni model
[Modify]: Tune AMCL particle count
[New]: Add QR code detection node
[Fix]: Correct camera frame orientation
[Docs]: Add EC interface table
[Chore]: Restructure repo into CAD/EC/NAV/VI
```

## 5. Pull requests

- Title: same format as commit messages.
- Description: what changed, how it was tested (simulation / real robot / docs only), and whether it affects other groups.
- At least one approval from your group lead before merging.
- Changes to `ros2_ws/src/common/robot_interfaces/` or `docs/interfaces/` change the contract between groups. They require approval from every group involved and must be announced in the group chat.

## 6. Shared conventions

- Units: documents and hand-filled tables use mm. ROS messages, TF and parameter files use SI units (m, rad, m/s), following ROS REP-103. Convert only at the interface.
- Frames: `map → odom → base_link → laser / camera_link / imu_link`. `base_link`: x forward, y left, z up.
- The `odom → base_link` transform is published only by the EKF on the Raspberry Pi. The ESP32 must not publish this TF.

## 7. Do not commit

- Build outputs: `build/`, `install/`, `log/`, `.pio/`
- SolidWorks temporary files: `~$*`
- Files larger than 100 MB, videos, zip archives (including the final Pack-and-Go). Store them in the shared drive and put the link in the relevant README.
- Passwords, tokens, Wi-Fi credentials
