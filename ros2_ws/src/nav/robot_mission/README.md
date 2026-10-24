# Robot mission

This package sequences the known rack poses through Nav2 and waits for visual
and lift feedback at each step:

1. Wait for the `START` QR marker.
2. Navigate to the next configured rack approach pose.
3. Call `/qr/start_scan` and require the matching `RACKA` through `RACKD` marker
   from `/qr/data` (stop the scan if the mission times out).
4. Raise the rack and wait for `/lift_state == 1`.
5. Navigate to the destination, scan its `END` marker, lower the rack, and wait
   for `/lift_state == 0`.
6. Continue until the configured rack list is delivered.

## Configure before a run

Edit `config/mission_params.yaml` with measured approach and destination poses
in the `map` frame, using metres and radians. Replace all placeholder zeroes,
then set `coordinates_confirmed` to `true`. Each rack pose should leave enough
clearance for the measured robot footprint and camera field of view.

The node calls `/qr/start_scan` and `/qr/stop_scan`
(`std_srvs/srv/Trigger`) and reads `/qr/data` (`std_msgs/msg/String`) from the
on-demand vision package. It also expects `/lift_cmd` and `/lift_state`
(`std_msgs/msg/Int8`) and uses the Nav2 `navigate_to_pose` action. Ensure the
mission list matches the physical game and the team's QR labels before starting.

```bash
ros2 launch robot_mission delivery_mission.launch.py
```

The mission node does not initialize AMCL from the START marker. Set the initial
map pose through the localisation workflow before expecting Nav2 goals to run.
