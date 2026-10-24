# Robot mission

This package sequences the known rack poses through Nav2 and waits for visual
and lift feedback at each step:

1. Wait for the `START` QR marker.
2. Navigate to the next configured rack approach pose.
3. Call `/qr/start_scan` and require the matching full `RACKA_...` through `RACKD_...` marker
   from `/qr/data` (stop the scan if the mission times out).
4. Raise the rack and wait for `/lift_state == 1`.
5. Navigate to the destination, scan its `END` marker, lower the rack, and wait
   for `/lift_state == 0`.
6. Continue until the configured rack list is delivered.

The 180-second competition timer starts when `START` is detected. If time runs
out, the mission cancels its active Nav2 goal, asks the vision node to stop
scanning, and leaves the lift in its current state so a carried rack is not
lowered automatically. If the stop service is temporarily unavailable or its
request fails, the mission retries while in the terminal error state.
Unexpected valid markers are logged and cause a fresh scan request, within the
QR timeout for that mission step. QR suffixes are case-sensitive.

Before `START`, `start_timeout_s: 0.0` allows an indefinite wait for the operator.
Rack and destination scans use `qr_timeout_s`. Deadlines take precedence over
late results, including a continuous stream of incorrect markers. Missing QR
service responses are bounded by `request_timeout_s`.

Lift steps wait for an EC subscriber, publish one command using reliable QoS,
then require a feedback message received after that command. Cached feedback
cannot skip the command. Commands are not periodically repeated, so an EC
delay-based motion timer is not restarted. The existing Int8 interface has no
request ID or timestamp: the EC implementation still needs to report completed
motion correctly, and delayed in-flight feedback cannot be correlated to a
specific command. No EC internal motor or encoder parameters are needed here.

SIGINT/Ctrl+C and SIGTERM request navigation cancellation and scanner stop before
destroying the ROS context. Shutdown allows up to two seconds for cleanup and
logs a warning if it remains unconfirmed. Delayed navigation and scanner start
responses are handled during cleanup. Cancellation and stop requests are
retried on failures while the mission node is running; they do not establish a
physical emergency-stop guarantee when peers or transport have shut down.

## Configure before a run

Edit `config/mission_params.yaml` with measured approach and destination poses
in the `map` frame, using metres and radians. Replace all placeholder zeroes,
enter the exact team-specific text for each rack in `rack_qr_codes`, then set
`coordinates_confirmed` to `true`. The node requires one unique full code such
as `RACKA_4X6M` per rack; it will not start with only a rack-letter prefix.
Each rack pose should leave enough clearance for the measured robot footprint
and camera field of view.

The node calls `/qr/start_scan` and `/qr/stop_scan`
(`std_srvs/srv/Trigger`) and reads `/qr/data` (`std_msgs/msg/String`) from the
on-demand vision package. It also expects `/lift_cmd` and `/lift_state`
(`std_msgs/msg/Int8`) and uses the Nav2 `navigate_to_pose` action. Ensure the
mission list matches the physical game and the team's QR labels before starting.

```bash
ros2 launch robot_mission delivery_mission.launch.py
# Use a measured configuration without editing the installed package:
ros2 launch robot_mission delivery_mission.launch.py params_file:=/path/to/mission.yaml
```

The mission node does not initialize AMCL from the START marker. Set the initial
map pose through the localisation workflow before expecting Nav2 goals to run.
`use_sim_time:=true` is available for ROS timestamps; mission deadlines still use
a monotonic wall clock, including when simulation is paused.

## Offline regression checks

See [the NAV/VI regression suite](../../common/tests/README.md) for a complete
four-rack sequence, timeout, delayed-response and failure-path checks without ROS.
Actual ROS discovery, QoS, action cancellation and lift timing must also be
checked on ROS 2 Jazzy before moving the robot.
