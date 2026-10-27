# Offline ROS workflows

These launch files use an isolated ROS domain (75) and localhost discovery.
Use the same environment in every terminal. Run one workflow at a time.
The course robot uses the supplied CAD planar envelope. Rack dimensions/poses
include explicit assumptions; DEMO QR suffixes are not the team's physical labels.
The course scenario adopts the published field dimensions and start/end zones.

```bash
source /opt/ros/jazzy/setup.bash
source ~/dase7503-nav-work/install/setup.bash
export ROS_DOMAIN_ID=75
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

## 1. Course presentation and task rehearsal

For the complete automatic workflow with actual image decoding, use:

```bash
ros2 launch robot_simulation course_demo.launch.py
# Equivalent repository entry:
bash scripts/nav.sh demo
```

It opens a clean portrait field viewer, waits for a read-only readiness check, then starts
the mission. Set `show_field:=false` for headless runs; add `rviz:=true` for debugging. It saves run
events and the exact confirming frames in a fresh `~/.ros/nav_vi_runs/` directory.
The camera is a task-driven image fixture, not a geometric optical sensor model;
lift and motion physics remain simulated. `image_scenario:=wrong_qr` or
`image_scenario:=missing_qr` checks task failure using actual image decoding.
Use a fresh path for each explicit `output_dir` to keep records separate.

`config/course_layout.yaml` is the single source for the viewer, simulated world,
mission coordinates and generated wall map. Its dimensions/zones come from page 2
of the original course PDF. Rack centres are estimates from the labelled page 3
schematic; the dimensioned drawing does not identify A/B/C/D unambiguously.
The view places destination above start; ROS coordinates remain x along 3020 mm,
y along 2000 mm. Vehicle outlines use the same geometry snapshot as navigation.
Four-legged racks are contact bodies. Lifting preserves the actual relative
transform; lowering restores legs at the actual placement, with no snapping.
The vehicle body checks every floor leg; carried decks check other decks and
walls. Four distinct destination slots fill the back row first, with straight
insertion and exit corridors. Fresh physical alignment is required before the
simulated lift acknowledges completion. The camera fixture also checks position.

The 5 mm wall map and ideal-sensor AMCL settings are simulation-specific. The
body uses the CAD envelope (282.502 x 286.568 mm); rack short side 140 mm is a
drawing-based baseline, long side 400 mm and 15 mm legs are explicit assumptions.
The loaded footprint includes attachment translation/yaw tolerances. Vertical
under-deck clearance, lift forces, wheel slip and optical visibility are not
certified; the CAD height conflict (223.04 mm versus 180 mm course limit) remains.

Every run saves `collision_summary.json` and `collision_trace.jsonl`. The explicit
`check course` command replays the trajectory and writes `collision_audit.json`.
Passing requires COMPLETE, four separate placements, zero actual collisions,
zero simulator-blocked collision commands and >10 mm swept clearance throughout.
The viewer renders actual rack positions, legs, slots and measured minimum gap.

```bash
bash scripts/nav.sh check course
bash scripts/nav.sh check collision_stress   # offsets, thicker legs, repeat
bash scripts/nav.sh check collision_faults   # live sensor failures/operator stop
bash scripts/nav.sh check motion_safety      # real ROS safety callbacks
```

All navigation launches include `motion_guard` between the velocity smoother
and `/cmd_vel`. It stops on stale controller commands, laser, wheel odometry,
IMU, fused odometry or TF, and holds still during scan/lift/terminal phases.
`/nav/emergency_stop` (Trigger) latches zero motion until restart. The EC hardware
watchdog remains necessary if ROS or the guard process itself fails.

The original central-obstacle scene remains an explicitly named regression fixture:

```bash
bash scripts/nav.sh regression-demo
# Direct equivalent; headless arguments belong to this launch:
ros2 launch robot_simulation integrated_demo.launch.py rviz:=false show_results:=false
```

It uses RViz plus the existing QR window. Do not present this obstacle fixture
as the course layout. The coloured costmap bands are navigation debug layers.

The lower-level rehearsal below intentionally retains all mock peers.

```bash
ros2 launch robot_simulation rehearsal.launch.py
```

The production mission uses real ROS services, topics and a mock Nav2 action.
Expect four `Delivered rack` messages followed by `Mission state: COMPLETE`.
QR and lift feedback are mocked. This does not verify navigation or hardware.
Observe `/mission/status` and `/simulation/events` to inspect the sequence.

Fault injection: `scenario:=wrong_qr`, `missing_qr`, `lift_timeout`,
`navigation_abort`, `navigation_reject`, `navigation_timeout`, or `envelope_failure`.
`competition_time_s:=2.0` exercises the competition deadline.
Ctrl+C exercises mission cancellation and scanner stop.

## 2. Navigation simulation

```bash
ros2 launch robot_simulation navigation_demo.launch.py
# Full four-rack mission with real navigation and mock QR/lift:
ros2 launch robot_simulation navigation_demo.launch.py mission:=true
```

RViz shows the map, robot, scan and paths. Set a Nav2 goal inside the field.
The planar simulator accepts `/cmd_vel`, ray casts walls and a central obstacle,
publishes wheel velocity, IMU yaw rate and laser scans, and blocks collisions.
The real EKF alone publishes `odom -> base_link`; real AMCL publishes `map -> odom`.
Smac and MPPI perform planning and control. This is an ideal 2D kinematic model,
not Gazebo wheel/contact physics, real lift movement or pickup validation.
It does not model rack removal/placement or camera visibility. QR/lift remain mocks.
Use `rviz:=false` for headless runs. The demo uses wall time consistently.

## 3. Real vision from images or video

```bash
ros2 launch robot_simulation vision_replay.launch.py payload:=RACKA_DEMO
# In a second sourced terminal, start one scan:
ros2 service call /qr/start_scan std_srvs/srv/Trigger '{}'
# Change a generated marker, then request another scan:
ros2 param set /image_replay payload END
ros2 service call /qr/start_scan std_srvs/srv/Trigger '{}'
# Or replay a local image/video:
ros2 launch robot_simulation vision_replay.launch.py input_file:=/absolute/path/clip.mp4
```

Images receive fresh replay timestamps and are published repeatedly, enabling
the production two-frame confirmation logic. This checks actual decoding and
ROS transport, not optical performance. The detector remains on-demand.

## Geometry and integration boundaries

Edit `robot_description/config/mechanical.yaml` only with confirmed CAD values.
It supplies the URDF, both Nav2 footprints, inflation clearance and self-filter.
`loaded_bounds_mm` must contain `empty_bounds_mm`, including protruding parts.
Production bringup rejects unconfirmed or incomplete geometry. Demo overrides
are confined to the explicit simulation launch and its separate config file.
The mission requests `/nav/set_loaded` only after fresh lift completion feedback,
then waits for acknowledgement from both costmaps and the self-filter.
An unsuccessful update stops mission progression; do not resume after a partial
update without checking the configuration and restarting.

The EC return-form adapter is implemented in `robot_bringup/ec_adapter.py`.
It isolates the existing bridge topics, checks SI telemetry and permits lift
completion only after an explicit operator confirmation. The private bridge
source and hardware have not been exercised here; see the CAD/EC integration note.
No internal EC PID, encoder or gearing parameters are requested.
