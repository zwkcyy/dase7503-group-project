# Offline ROS workflows

These launch files use an isolated ROS domain (75) and localhost discovery.
Use the same environment in every terminal. Run one workflow at a time.
All demo geometry, rack poses and QR suffixes are examples, not measured values.

```bash
source /opt/ros/jazzy/setup.bash
source ~/dase7503-nav-work/install/setup.bash
export ROS_DOMAIN_ID=75
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

## 1. Task rehearsal

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

EC transport is deliberately not guessed. The submodule README describes ASCII
serial firmware without native micro-ROS. Agree on transport, exact commands,
feedback units/frames and completion semantics before implementing an adapter.
No internal EC PID, encoder or gearing parameters are requested.
