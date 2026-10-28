# Gazebo Harmonic course simulation

ROS 2 Jazzy / Ubuntu 24.04. This package runs a three-dimensional DART physics
world with the supplied CAD visuals, wheel joints, gravity, contact lift,
rendered camera, GPU lidar, IMU, real Nav2 and the existing QR / mission nodes.
The supplied CAD and EC source files are read without modification.

## Run from the repository root in Ubuntu

```bash
# Once on a new ROS 2 Jazzy machine:
bash scripts/nav.sh install
bash scripts/nav.sh gazebo-install
bash scripts/nav.sh build

# Actual Gazebo GUI; starts the delivery mission after readiness checks:
bash scripts/nav.sh gazebo

# Optional camera / result window:
bash scripts/nav.sh gazebo show_results:=true

# Physics and rendered sensors without a GUI:
bash scripts/nav.sh gazebo gui:=false
```

Stop the complete launch with Ctrl+C in its terminal. Closing only the Gazebo
GUI does not stop the server or mission. Restart the launch for a fresh task;
do not use the GUI reset button mid-mission (ROS clocks / mission state are not
rewound together). A contact emergency remains latched until a fresh launch.
For inspection without motion: `mission:=false`; to omit Nav2 too, also use
`navigation:=false`.

Each launch uses localhost ROS domain 75 and a unique Gazebo transport partition.
`domain_id:=N` selects another isolated domain; do not share it with hardware.
Run only one demonstration per ROS domain. Normal build / startup never runs
verification. The older `demo` command remains the planar regression simulator.

## Explicit verification

```bash
bash scripts/nav.sh gazebo-check --output output/gazebo-my-run
bash scripts/nav.sh gazebo-check --output output/gazebo-my-run-gui --gui
bash scripts/nav.sh gazebo-faults --output output/gazebo-my-fault-check
```

Output paths must be new. `gazebo-check` runs the full mission, collects actual
camera frames, stops the launch and audits physical observations. It exits
nonzero for incomplete missions, wrong QR sequence, unexpected contacts,
placement errors, trace gaps, cargo sag / tilt or an exceeded task deadline.
`--scene-file`, `--physics-file`, `--domain` and `--timeout` select variants.
The default wall timeout is 600 s; the competition deadline remains 180 **simulated**
seconds. Wall time depends on the computer and rendering load.

`gazebo-faults` uses a separate domain (180), disables the mission / navigation,
adds a physical obstacle and intentionally drives into it. It checks command
timeout, contact detection, zero output despite further commands, the emergency
service call, and shutdown after the bridge process is terminated. Nonzero
contacts in this fault run are expected; it is not a nominal delivery pass.

Runs save under the chosen output path, or `~/.ros/nav_vi_runs/gazebo-*` for
interactive startup:

| Evidence | Meaning |
|---|---|
| `course.sdf`, `assets/`, YAML files | Exact generated world, fixed QR textures, physics and navigation settings |
| `model_assumptions.json` | CAD bounds, masses, under-deck clearance and modelling choices |
| `source_manifest.json` | Git revision plus source SHA256 hashes when launched from a checkout |
| `gazebo_trace.jsonl` | Buffered physical pose samples with their original Gazebo timestamps, lift joint position, encoder velocity and mission state |
| `gazebo_events.jsonl`, `gazebo_summary.json` | Lift / placement events and contact classifications |
| `qr/`, `qr_scan.csv` | Confirming camera frames from the real decoder |
| `observer.json`, `camera-*.png` | Independent verification observations and phase snapshots |
| `gazebo_audit.json` | Fourteen full-mission checks and measured simulation results |
| `fault_checks.json` | Results from the separate intentional fault run |

The recorder buffers transport samples before writing, independently of ROS
`/clock` delivery and executor scheduling. It never interpolates gaps; queue
overflow or a trace gap of 0.2 s or more fails the audit.

Preserve failed runs as evidence. Generated files, images and logs stay outside
Git. A directory generated on one machine contains absolute asset paths; regenerate
it on another machine instead of treating the SDF as a portable binary release.

## Physical and software model

- Course dimensions: 3020 × 2000 mm; wall height 500 mm. Rack height 232.5 mm
  and depth 140 mm follow the course PDF. Rack centres, width 400 mm, leg width
  15 mm, deck thickness 6 mm and mass 0.5 kg are explicit simulation assumptions.
- Original CAD meshes retain their dimensions. Fixed component boxes conservatively
  replace concave mesh collisions. Wheel collisions are cylinders; anisotropic
  friction models ideal free mecanum rollers. CAD masses are retained with box
  approximations for inertia; actual roller contacts / motor dynamics are not modelled.
- The saved CAD fixed lift becomes an equivalent force-controlled prismatic joint:
  25 mm stroke, 15 mm/s command ramp, 80 N limit, PID 10000 / 2000 / 100. Rack motion
  comes from gravity, tray contact and friction, without attachments or pose commands.
  The controller's integral term removes gravity-induced static error. These are
  simulation actuator settings, not measured servo performance.
- All four racks persist, including delivered racks. Contact sensors cover the body,
  tray, wheels and racks. Only wheel-floor, rack-leg-floor and the currently handled
  rack-deck / tray contacts are allowed. Other contacts latch zero velocity and request
  the existing mission emergency stop. This is contact detection, not predictive
  collision avoidance; Nav2 and the existing motion guard provide avoidance.
- `/cmd_vel` expires after 0.4 wall seconds. The actuator timer uses a steady clock.
  Adapter / bridge exit shuts the world down. Velocity limits are 0.4 m/s forward,
  0.3 m/s lateral, and 1.5 rad/s yaw.
- Lift completion requires the physical joint within 1 mm and actual rack elevation
  above 10 mm (up), or ground height within 2 mm (down). Before lifting, true alignment
  must be within 25 mm / 0.05 rad. Truth is used here as a simulated lift sensor and
  for audits; it is never fed to wheel odometry, EKF or AMCL.
- Wheel odometry comes from Gazebo wheel-joint integration. EKF alone publishes
  `odom → base_link`; AMCL supplies `map → odom`. Gazebo's wheel TF is not bridged.
- Real rendered camera: 640 × 480, 10 Hz, horizontal FOV 1.4 rad. Static START / END
  fixtures and rack-mounted QR boards always exist in the scene, with perspective
  and occlusion. Scan services only enable the actual decoder. Expected-marker
  preference chooses among decoded pixels and never supplies a fake result.
- The Gazebo profile proposes a level laser at [145, 0, 70] mm and camera upward
  pitch 0.35 rad; the original CAD laser frame is upside down. These proposed mounts,
  the IMU frame and camera intrinsics require physical implementation / calibration.
- Simulation-specific AMCL and goal tolerances are generated in each run. Hardware
  defaults remain unchanged. Task / docking deadlines use ROS time in simulation;
  hardware keeps monotonic deadlines and communication guards use wall time.

## Physical limits still requiring resolution

The saved CAD reaches **223.04 mm**, above the coursework **180 mm** robot-height
limit. This simulation retains that geometry; it does not demonstrate height
compliance. With the assumed 6 mm deck, static tray clearance is only **3.46 mm**.
Measure the actual rack underside and the lowest lift position before hardware
pickup. Friction, payload, motor response, sensor mounts, marker locations and
official QR payloads must also be checked on the real field.

Passed simulations establish behaviour only for the saved model and tested
conditions. They do not certify collision-free hardware operation.

References: [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/),
[official mecanum example](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/mecanum_drive.sdf),
[joint position controller](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/joint_position_controller/JointPositionController.cc).
