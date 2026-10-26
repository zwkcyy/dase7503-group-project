# NAV/VI offline regression checks

Run from the repository root with Python 3.11 or newer. The suite imports the
production mission, scanner and map generator. Only ROS boundaries (nodes,
messages, services, actions and cv_bridge) are replaced by small test fakes;
OpenCV decoding, NumPy and YAML processing run normally.

Use an isolated host environment. On Windows, use `.venv-nav-review/Scripts/python`
instead of `.venv-nav-review/bin/python` in the commands below.

```bash
python3 -m venv .venv-nav-review
.venv-nav-review/bin/python -m pip install -r ros2_ws/src/common/tests/requirements.txt
.venv-nav-review/bin/python -m unittest discover -s ros2_ws/src/common/tests -v
```

Do not commit the virtual environment. Do not install the host-side OpenCV wheel
into the Pi's ROS Python environment; its cv_bridge uses system libraries.

## Coverage

- Mission: full four-rack sequence; exact QR matching; pre-start wait; step and
  competition deadlines; fresh lift feedback after a single command; discovery
  wait; delayed goal acceptance/results; service failures; cancellation retries.
- Scanner: actual OpenCV decoding of generated START/END/rack codes; rejection of
  unrelated codes; two-frame confirmation; stale/duplicate frames; confirmation
  expiry; idempotent services; file and debug failures; optional model fallback.
- Maps: default geometry and identical committed PGM; zero margin; PGM row order;
  metre-based origin; invalid dimensions; CLI output and filename validation.
- Course scene: published dimensions/zones, schematic label arrangement, common
  world/mission/map source, loaded wall clearance, invalid data rejection, and
  separation of accepted empty-only CAD data from full physical acceptance.

## Limits and checks on the robot

The unittest suite above checks offline logic. The separate opt-in ROS integration
runner below checks live DDS, services/actions and a planar simulation. Neither
establishes physical acceptance.
Before a run, verify on ROS 2 Jazzy:

1. Build with colcon and start NAV, vision and mission with the measured settings.
   Check lifecycle activation, TF, image frame/timestamps and topic QoS.
2. Check camera decoding at pickup/delivery distance, including blur, reflections,
   lighting, simultaneous markers and Pi processing latency. Exercise WeChat if used.
3. Verify one lift command produces the expected completion feedback. The Int8
   interface has no sequence ID; these tests cannot prove feedback belongs to the
   current physical command or that a rack was actually picked up.
4. Exercise missing QR, unavailable services, lost feedback, failed navigation,
   deadline expiry and Ctrl+C. Confirm Nav2 stops the robot and the EC command
   watchdog handles transport loss. A cancellation acknowledgement alone is not
   a physical stop measurement.
5. Measure empty/loaded footprints, sensor poses and carrying clearance; configure
   the costmap/self-filter accordingly. Then run and time the four-rack mission.

No new requests for EC internal motor, PID, encoder or gear parameters are added
by these regression checks.

## ROS integration runner

Build `robot_geometry`, `robot_description`, `robot_navigation`, `robot_mission`,
`robot_vision` and `robot_simulation`, then source the workspace. From the repo root:

```bash
python3 ros2_ws/src/common/tests/ros_integration.py rehearsal /tmp/nav-checks
python3 ros2_ws/src/common/tests/ros_integration.py navigation /tmp/nav-checks
python3 ros2_ws/src/common/tests/ros_integration.py vision /tmp/nav-checks
python3 ros2_ws/src/common/tests/ros_integration.py cancellation /tmp/nav-checks
python3 ros2_ws/src/common/tests/ros_integration.py envelope /tmp/nav-checks
```

The runner starts only its own processes in localhost ROS domains 100-117 and
stops them at completion. The envelope check uses domain 118. It writes full logs and JSON summaries. Reserve those
domains during a run. Rehearsal covers success, wrong/missing QR, lift timeout,
navigation abort/reject/timeout, envelope failure and the competition deadline.
Navigation uses actual Nav2/EKF/AMCL with synthetic sensors and mock QR/lift.
Vision decodes generated frames using the production node. Cancellation leaves
the mock peers alive while interrupting the mission and checks both stop and cancel.

## Complete image-to-mission integration

Build the additional `robot_bringup` package (the repository's `scripts/nav.sh build`
does this automatically). New modes:

```bash
bash scripts/nav.sh check integrated
bash scripts/nav.sh check integrated_faults
bash scripts/nav.sh check course
bash scripts/nav.sh check preflight
bash scripts/nav.sh check ec_adapter
bash scripts/nav.sh check launch_parameters
bash scripts/nav.sh check robustness
```

The integrated modes use domains 120-121, actual Nav2 and actual QR decoding.
They require the expected full QR sequence, eight load changes and one QR result
publisher named qr_detector. The mock peer provides lift feedback only. Nine exact
confirming images and six distinct QR categories must be saved for a successful run.
Wrong and blank images exercise the production scanner and task QR deadline.
Readiness checks use domains 125-130 for healthy, stale, empty, missing-service,
duplicate-result-publisher and missing-stream cases. Run suites sequentially.
The course mode repeats the successful integration in the course scene on domain
123 and saves its generated configuration/map snapshots. The EC callback suite
uses real ROS message classes on domain 178, replaces outputs with capture objects,
and never sends hardware commands. Its 20 checks cover data age including transport
delay, frames, quaternion validity, IMU rotation, velocity limits/timeout, and lift
target-versus-arrival semantics. It does not validate the external serial bridge.
Three launch-parameter checks exercise the actual template expansion and Nav2 YAML
rewriting order, including Chinese paths with spaces and AMCL initial-pose overrides.

The robustness command measures 71 deterministic image cases, three attempts each,
using the production OpenCV fallback with ROS boundaries replaced. Required clean
and 90-degree samples must decode, and all cases must avoid false positives.
Other distortions are measured limitations, not assumed camera acceptance targets.
JSON/CSV and a visual HTML index are saved in `output/nav-validation-current/robustness`.

GitHub Actions runs only the offline logic and robustness checks on Ubuntu 24.04.
ROS checks remain an explicit local workflow and are not claimed as CI coverage.
