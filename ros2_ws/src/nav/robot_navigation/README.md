# Robot navigation

This package contains the game-field map, EKF and laser-filter configuration,
and the Nav2 settings for map-based localisation and navigation.

## Build the field map

From the package directory, run the map generator from the NAV Python tooling
environment:

```bash
python3 scripts/generate_map.py --preview
```

The generator writes `maps/field_map.pgm`, `maps/field_map.yaml`, and a visual
preview. Rack and zone positions in `config/field_config.yaml` remain unset
until the physical field drawing is measured and confirmed.
Dimensions and resolution must be finite and positive, and margin must be
nonnegative. Invalid configurations fail before writing map files. Map names
accept letters, digits, underscores and hyphens, so output stays in `--out-dir`.

## Start Nav2

After sourcing ROS 2 Jazzy and building the workspace, launch the full NAV stack
(robot description, EKF, scan filter, map server, AMCL, and Nav2):

```bash
ros2 launch robot_navigation bringup.launch.py geometry_file:=/absolute/path/mechanical.yaml initial_x:=0.3 initial_y:=0.3 initial_loaded:=false
```

The initial pose values above are examples. Set them from the robot's measured
starting pose in the `map` frame. The EC firmware and lidar must be running so
these inputs are available:

- `/scan` (`sensor_msgs/msg/LaserScan`), filtered to `/scan_filtered`
- `/odom/wheel` and `/imu/data_raw` for the Pi EKF, which publishes
  `/odometry/filtered`
- `map -> odom` from AMCL and `odom -> base_link` from the Pi EKF
- `base_link -> laser` and other sensor transforms from the robot description

The velocity limits and footprint in `config/nav2_params.yaml` must be checked
against the measured robot and confirmed motor interface before floor testing.
The static map contains the field walls; temporary racks are detected by the
lidar costmap.

## Mechanical profile and load state

`robot_description/config/mechanical.yaml` is the single CAD input for body,
wheel visualisation, sensor transforms, empty/loaded bounds and filter margin.
The checked-in template is unconfirmed and its loaded dimensions are deliberately
unset. Full bringup refuses incomplete/unconfirmed geometry. Copy it, fill measured
values, and set `confirmed: true` only after checking with CAD. Dimensions are mm;
angles are radians. `initial_loaded` must match the actual starting condition.

Bringup generates a temporary Nav2 parameter file from that profile, including
both footprints, padding and inflation radius sufficient for the loaded bounds.
The `load_envelope` node replaces the static laser filter in full bringup. The
mission calls `/nav/set_loaded` after fresh lift completion; it resumes only after
both costmap parameter services acknowledge and the self-filter switches. A
partial/failed update is latched as a failure until the node is restarted.

The standalone `navigation.launch.py` and `localization.launch.py` remain low-level
components. Use full `bringup.launch.py` for a consistent mechanical profile.
Standalone localisation retains the legacy fixed filter for compatibility.

For a computer without the robot, use the dedicated examples:

```bash
ros2 launch robot_simulation navigation_demo.launch.py
```

See `robot_simulation/README.md` for domains, task rehearsal and vision replay.

## Offline checks

The [NAV/VI regression suite](../../common/tests/README.md) checks map geometry,
PGM orientation, map origin, invalid settings, and reproduction of the committed
map. ROS integration is exercised by `common/tests/ros_integration.py`. Loaded
geometry, sensor noise, movement limits and pickup accuracy still require physical measurements.
