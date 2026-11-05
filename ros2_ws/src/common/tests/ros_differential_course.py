"""Explicit full four-rack differential mission; existing safety limits remain intact."""
import argparse
import json
from pathlib import Path
from rcl_interfaces.srv import GetParameters
from ros_integration import run_case, mission_check, call
from robot_simulation.collision_audit import audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--domain', type=int, default=206)
    args = parser.parse_args()
    if not 1 <= args.domain <= 232:
        parser.error('Use an isolated domain 1..232')
    output = Path(args.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    run = output/'course'

    def check(node):
        # Subscribe before querying parameters: do not miss START/early phases.
        # Mission checker creates these subscriptions at its entry, so obtain
        # runtime parameters after the mission while processes remain alive.
        result = mission_check('COMPLETE', 220, True, True, evidence_dir=run)(node)
        expected = {
            '/planar_base': {'drive_mode': 'differential'},
            '/docking_controller': {'drive_mode': 'differential'},
            '/controller_server': {
                'FollowPath.motion_model': 'DiffDrive', 'FollowPath.vy_max': 0.0},
            '/amcl': {'robot_model_type': 'nav2_amcl::DifferentialMotionModel'},
            '/velocity_smoother': {'max_velocity': [.15, 0., .4], 'min_velocity': [-.08, 0., -.4]},
            '/motion_guard': {'allow_lateral_motion': False},
        }
        observed = {}
        for name, values in expected.items():
            client = node.create_client(GetParameters, name+'/get_parameters')
            response = call(node, client, GetParameters.Request(names=list(values)))
            actual = {}
            for key, value in zip(values, response.values):
                actual[key] = {1: value.bool_value, 2: value.integer_value,
                               3: value.double_value, 4: value.string_value,
                               8: list(value.double_array_value)}.get(value.type)
            assert actual == values, (name, actual, values)
            observed[name] = actual
        result.update(runtime_parameters=observed)
        return result

    result = run_case('differential_course', 'course_demo.launch.py',
        ['drive_mode:=differential', 'rviz:=false', 'show_field:=false', f'output_dir:={run}'],
        args.domain, output, check, 'Mission complete: delivered 4 racks')
    # Audit after process shutdown so JSONL and terminal snapshots are complete.
    # Preserve partial progress and actuator metrics when the mission rejects
    # an unsafe manoeuvre or times out; a partial run is never a passing mission.
    for filename, key in (('collision_summary.json', 'collision_summary'),
                          ('summary.json', 'mission_summary')):
        path = run/filename
        if path.is_file():
            result[key] = json.loads(path.read_text())
    if result['passed']:
        try:
            collision = audit(run)
            assert collision['passed'], collision
            metrics = json.loads((run/'collision_summary.json').read_text())['metrics']
            assert metrics['drive_mode'] == 'differential', metrics
            assert metrics['maximum_applied_abs_vy'] == 0.0, metrics
            assert metrics['rejected_lateral_commands'] == 0, metrics
            rows = [json.loads(line) for line in (run/'collision_trace.jsonl').read_text().splitlines()]
            assert rows and all(row['velocity'][1] == 0.0 for row in rows)
            assert any(abs(row['velocity'][0])+abs(row['velocity'][2]) > .005 for row in rows)
            confirmations = [json.loads(p.read_text()) for p in (run/'qr').glob('*.json')]
            assert len(confirmations) == 9
            assert {r['payload'] for r in confirmations} == {'START', 'END', *(f'RACK{x}_DEMO' for x in 'ABCD')}
            result.update(collision_audit=collision, actuator_metrics=metrics, saved_confirmations=len(confirmations))
        except (AssertionError, OSError, ValueError, KeyError) as exc:
            result.update(passed=False, error=f'Persisted differential audit failed: {exc}')
    (output/'differential_course.json').write_text(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
