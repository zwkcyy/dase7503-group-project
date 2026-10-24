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
ros2 launch robot_navigation bringup.launch.py initial_x:=0.3 initial_y:=0.3
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

## Offline checks

The [NAV/VI regression suite](../../common/tests/README.md) checks map geometry,
PGM orientation, map origin, invalid settings, and reproduction of the committed
map. The Nav2 launch files still require a ROS 2 Jazzy startup check. Footprints
and self-filter bounds for a carried rack remain dependent on measured geometry.
