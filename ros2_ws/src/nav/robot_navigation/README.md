# Robot navigation

This package contains the generated game-field map, the initial Nav2 settings,
and a launch file for map-based localisation and navigation.

## Build the field map

From the package directory, run the map generator from the NAV Python tooling
environment:

```bash
python3 scripts/generate_map.py --preview
```

The generator writes `maps/field_map.pgm`, `maps/field_map.yaml`, and a visual
preview. Rack and zone positions in `config/field_config.yaml` remain unset
until the physical field drawing is measured and confirmed.

## Start Nav2

After sourcing ROS 2 Jazzy and building the workspace:

```bash
ros2 launch robot_navigation navigation.launch.py
```

Before launching, bring up the robot base and make sure these inputs are active:

- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/odom` (`nav_msgs/msg/Odometry`)
- `map -> odom` from AMCL and `odom -> base_link` from the Pi EKF
- `base_link -> laser` from the robot description / static transform

The limits and footprint in `config/nav2_params.yaml` are conservative initial
values. Verify them against the measured robot size and the confirmed motor
interface before testing on the floor. The current map includes field walls;
temporary racks are expected to be detected by the lidar costmap.
