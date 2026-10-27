"""Run requested ROS checks in isolated domains; retain full logs and JSON results.

Usage: source ROS/workspace, then python3 ros_integration.py rehearsal|navigation|vision OUTDIR
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy


def spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        process = getattr(node, '_checked_launch_process', None)
        if process is not None and process.poll() is not None:
            raise AssertionError(f'Launch exited before the expected outcome (code {process.returncode})')
        rclpy.spin_once(node, timeout_sec=.1)
    if not predicate():
        raise AssertionError(f'Condition timed out after {timeout}s')


def call(node, client, request):
    spin_until(node, client.service_is_ready, 20)
    future = client.call_async(request)
    spin_until(node, future.done, 5)
    return future.result()


def run_case(name, launch, arguments, domain, outdir, check, expected_log=None):
    environment = {**os.environ, 'ROS_DOMAIN_ID': str(domain),
                   'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST', 'PYTHONDONTWRITEBYTECODE': '1'}
    path = outdir / f'{name}.log'
    process = node = None
    result = {'case': name, 'passed': False, 'log': str(path)}
    started = time.monotonic()
    with path.open('w') as log:
        try:
            rclpy.init(domain_id=domain)
            node = Node('integration_observer')
            process = subprocess.Popen(['ros2', 'launch', 'robot_simulation', launch,
                                        f'domain_id:={domain}', *arguments],
                                       env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            node._checked_launch_process = process
            result.update(check(node))
            result['passed'] = True
        except Exception as exc:
            result['error'] = str(exc)
        finally:
            if process:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            if node:
                node.destroy_node()
            rclpy.try_shutdown()
    result['seconds'] = round(time.monotonic()-started, 2)
    if expected_log and expected_log not in path.read_text():
        result['passed'] = False
        result['error'] = result.get('error', '') + f'; missing expected outcome: {expected_log}'
    print(json.dumps(result), flush=True)
    return result


def mission_check(expected, timeout=35, navigation=False, real_vision=False, frame_dir=None):
    def check(node):
        states, events, loads, codes = [], [], [], []
        counts = {'scan': 0, 'odometry': 0}
        node.create_subscription(String, '/mission/status', lambda msg: states.append(json.loads(msg.data)), 10)
        node.create_subscription(String, '/simulation/events', lambda msg: events.append(json.loads(msg.data)),
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(Bool, '/nav/loaded', lambda msg: loads.append(msg.data), 10)
        node.create_subscription(String, '/qr/data', lambda msg: codes.append(msg.data), 10)
        if frame_dir:
            from cv_bridge import CvBridge
            import cv2
            frame_counter = [0]
            def record_frame(msg):
                frame_counter[0] += 1
                if frame_counter[0] % 10 == 0:
                    # The launch owns creation of its fresh evidence directory.
                    # Wait for actual images before adding observer snapshots.
                    frame_dir.mkdir(parents=True, exist_ok=True)
                    state = states[-1]['state'] if states else 'STARTUP'
                    cv2.imwrite(str(frame_dir/(state+'.png')), CvBridge().imgmsg_to_cv2(msg, 'bgr8'))
            node.create_subscription(Image, '/camera/image_raw', record_frame, qos_profile_sensor_data)
        def count(key):
            def callback(msg):
                counts[key] += 1
            return callback
        node.create_subscription(LaserScan, '/scan_filtered', count('scan'), qos_profile_sensor_data)
        node.create_subscription(Odometry, '/odometry/filtered', count('odometry'), 10)
        spin_until(node, lambda: bool(states) and states[-1]['state'] in ('COMPLETE', 'ERROR'), timeout)
        assert states[-1]['state'] == expected, states[-1]
        if expected == 'COMPLETE':
            assert states[-1]['rack_index'] == 4, states[-1]
            assert [e['command'] for e in events if e['event'] == 'lift_command'] == [1, 0]*4, events
        if navigation:
            assert counts['scan'] > 20 and counts['odometry'] > 20, counts
            assert loads[-8:] == [True, False]*4, loads
        if real_vision:
            publishers = node.get_publishers_info_by_topic('/qr/data')
            assert len(publishers) == 1 and publishers[0].node_name == 'qr_detector', publishers
            assert not any(e['event'] == 'mock_qr' for e in events), events
            if expected == 'COMPLETE':
                sequence = ['START']
                for label in 'ABCD':
                    sequence.extend([f'RACK{label}_DEMO', 'END'])
                assert codes == sequence, codes
        return {'final_status': states[-1], 'sensor_messages': counts, 'load_changes': loads,
                'decoded_payloads': codes, 'real_vision': real_vision}
    return check


def vision_check(node):
    results, images = [], []
    node.create_subscription(String, '/qr/data', lambda msg: results.append(msg.data), 10)
    node.create_subscription(Image, '/camera/image_raw', lambda msg: images.append(msg.header.stamp.sec), qos_profile_sensor_data)
    start = node.create_client(Trigger, '/qr/start_scan')
    params = node.create_client(SetParameters, '/image_replay/set_parameters')
    spin_until(node, lambda: len(images) >= 3, 20)
    for index, payload in enumerate(('START', 'RACKA_DEMO', 'RACKB_DEMO', 'RACKC_DEMO', 'RACKD_DEMO', 'END')):
        parameter = Parameter('payload', value=payload).to_parameter_msg()
        response = call(node, params, SetParameters.Request(parameters=[parameter]))
        assert response.results[0].successful
        # Drain one replay interval before starting the next scan.
        until = time.monotonic() + .4
        spin_until(node, lambda: time.monotonic() >= until, 2)
        assert call(node, start, Trigger.Request()).success
        spin_until(node, lambda: len(results) > index, 10)
        assert results[index] == payload, results
    return {'decoded_payloads': results, 'frames_received': len(images)}


def preflight_checks(outdir):
    """Exercise the actual readiness executable with healthy and broken ROS streams."""
    results = []
    for index, scenario in enumerate(('healthy', 'stale', 'empty', 'missing_service', 'duplicate_results', 'missing_stream')):
        domain = 125 + index
        environment = {**os.environ, 'ROS_DOMAIN_ID': str(domain), 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
        rclpy.init(domain_id=domain)
        node = Node('preflight_fixture')
        process = None
        result = {'case': 'preflight_' + scenario, 'passed': False}
        logfile, report = outdir/f'preflight-{scenario}.log', outdir/f'preflight-{scenario}.json'
        with logfile.open('w') as log:
            try:
                publisher = node.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
                node.create_publisher(String, '/qr/data', 10)
                if scenario == 'duplicate_results':
                    node.create_publisher(String, '/qr/data', 10)
                def trigger(request, response):
                    response.success = True
                    return response
                if scenario != 'missing_service':
                    node.create_service(Trigger, '/qr/start_scan', trigger)
                    node.create_service(Trigger, '/qr/stop_scan', trigger)
                def publish():
                    if scenario == 'missing_stream':
                        return
                    msg = Image()
                    msg.header.frame_id = 'camera_optical_frame'
                    stamp = node.get_clock().now().to_msg()
                    if scenario == 'stale':
                        stamp.sec -= 20
                    msg.header.stamp = stamp
                    if scenario != 'empty':
                        msg.height = msg.width = 8
                        msg.encoding, msg.step = 'bgr8', 24
                        msg.data = bytes([255]*192)
                    publisher.publish(msg)
                node.create_timer(.1, publish)
                process = subprocess.Popen(['ros2', 'run', 'robot_bringup', 'preflight', '--mode', 'vision',
                    '--timeout', '4', '--stable-for', '.4', '--output', str(report)],
                    env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                spin_until(node, lambda: process.poll() is not None, 12)
                data = json.loads(report.read_text())
                assert (process.returncode == 0) == (scenario == 'healthy'), data
                reasons = {'stale': 'stale message timestamp', 'empty': 'empty image',
                           'missing_service': 'service unavailable', 'duplicate_results': 'exactly one',
                           'missing_stream': 'no messages received'}
                if scenario in reasons:
                    assert reasons[scenario] in json.dumps(data), data
                result.update(passed=True, report=data)
            except Exception as exc:
                result['error'] = str(exc)
            finally:
                if process and process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                node.destroy_node()
                rclpy.try_shutdown()
        print(json.dumps(result), flush=True)
        results.append(result)
    return results


def cancellation_check(outdir):
    """Interrupt only the mission so peers stay alive to acknowledge cleanup."""
    path = outdir / 'cancellation.log'
    result = {'case': 'cancellation', 'passed': False, 'log': str(path)}
    env = {**os.environ, 'ROS_DOMAIN_ID': '117', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    processes = []
    node = None
    with path.open('w') as log:
        try:
            rclpy.init(domain_id=117)
            node = Node('cancel_observer')
            events = []
            node.create_subscription(String, '/simulation/events', lambda msg: events.append(json.loads(msg.data)),
                                     QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            commands = [
                ['ros2', 'run', 'robot_simulation', 'mock_peers', '--ros-args', '-p', 'scenario:=navigation_timeout'],
                ['ros2', 'run', 'robot_mission', 'delivery_mission', '--ros-args', '--params-file',
                 str(Path(get_package_share_directory('robot_simulation')) / 'config/demo_mission.yaml')]]
            for command in commands:
                processes.append(subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                                   start_new_session=True))
            spin_until(node, lambda: any(e['event'] == 'navigation_goal' for e in events), 20)
            os.killpg(processes[1].pid, signal.SIGINT)
            spin_until(node, lambda: {'navigation_cancelled', 'scan_stopped'} <= {e['event'] for e in events}, 5)
            processes[1].wait(timeout=5)
            result['passed'] = True
            result['events'] = events
        except Exception as exc:
            result['error'] = str(exc)
        finally:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=6)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
            if node:
                node.destroy_node()
            rclpy.try_shutdown()
    print(json.dumps(result), flush=True)
    return result


def envelope_check():
    from rcl_interfaces.msg import SetParametersResult
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import SetBool
    from robot_geometry.load_envelope import LoadEnvelope
    from robot_geometry.profile import load_profile, polygon
    profile_path = str(Path(get_package_share_directory('robot_simulation')) / 'config/demo_geometry.yaml')
    rclpy.init(domain_id=118)
    executor = SingleThreadedExecutor()
    nodes = []
    result = {'case': 'envelope', 'passed': False}
    try:
        profile = load_profile(profile_path, True)
        for name in ('local_costmap', 'global_costmap'):
            node = Node(name, namespace='/' + name)
            node.declare_parameter('footprint', json.dumps(polygon(profile, False)))
            nodes.append(node)
        manager = LoadEnvelope(parameter_overrides=[
            Parameter('geometry_file', value=profile_path),
            Parameter('allow_unconfirmed_geometry', value=True),
            Parameter('initial_loaded', value=False)])
        nodes.append(manager)
        observer = Node('envelope_observer')
        nodes.append(observer)
        client = observer.create_client(SetBool, '/nav/set_loaded')
        for node in nodes:
            executor.add_node(node)
        def wait(predicate, timeout=5):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
            assert predicate(), 'Envelope condition timed out'
        wait(lambda: client.service_is_ready() and all(c.service_is_ready() for c in manager.costmap_clients))
        future = client.call_async(SetBool.Request(data=True))
        wait(future.done)
        assert future.result().success and manager.loaded
        for node in nodes[:2]:
            assert json.loads(node.get_parameter('footprint').value) == polygon(profile, True)
        nodes[1].add_on_set_parameters_callback(lambda params: SetParametersResult(successful=False, reason='injected'))
        future = client.call_async(SetBool.Request(data=False))
        wait(future.done)
        assert not future.result().success and manager.failed and manager.loaded
        # After a partial update, repeated requests must not resume operation.
        future = client.call_async(SetBool.Request(data=True))
        wait(future.done)
        assert not future.result().success
        result['passed'] = True
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps(result), flush=True)
    return result


def main():
    mode = sys.argv[1]
    outdir = Path(sys.argv[2]).resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    results = []
    if mode == 'rehearsal':
        scenarios = ['success', 'wrong_qr', 'missing_qr', 'lift_timeout', 'navigation_abort',
                     'navigation_reject', 'navigation_timeout', 'envelope_failure', 'competition_timeout']
        if len(sys.argv) > 3:
            scenarios = [sys.argv[3]]
        messages = {'success': 'Mission complete: delivered 4 racks',
                    'wrong_qr': 'Timed out waiting for QR marker RACKA_DEMO',
                    'missing_qr': 'Timed out waiting for QR marker START',
                    'lift_timeout': 'Timed out waiting for lift state 1',
                    'navigation_abort': 'Navigation ended with action status 6',
                    'navigation_reject': 'Nav2 rejected the mission goal',
                    'navigation_timeout': 'Timed out while navigating to the rack',
                    'envelope_failure': 'Load envelope update failed: Injected envelope rejection',
                    'competition_timeout': 'Competition time limit reached'}
        for index, scenario in enumerate(scenarios):
            args = [f'scenario:={scenario}'] if scenario != 'competition_timeout' else ['competition_time_s:=2.0']
            results.append(run_case(scenario, 'rehearsal.launch.py', args, 100+index, outdir,
                                    mission_check('COMPLETE' if scenario == 'success' else 'ERROR'), messages[scenario]))
    elif mode == 'navigation':
        results.append(run_case('navigation', 'navigation_demo.launch.py',
                                ['rviz:=false', 'mission:=true'], 115, outdir,
                                mission_check('COMPLETE', 160, True)))
    elif mode == 'vision':
        results.append(run_case('vision', 'vision_replay.launch.py', ['show_results:=false'],
                                116, outdir, vision_check))
    elif mode in ('integrated', 'integrated_faults', 'course'):
        scenarios = ['success'] if mode in ('integrated', 'course') else ['wrong_qr', 'missing_qr']
        for index, scenario in enumerate(scenarios):
            name = ('course_' if mode == 'course' else 'integrated_') + scenario
            run_dir = outdir/(name + '-' + uuid.uuid4().hex[:8])
            launch_file = 'course_demo.launch.py' if mode == 'course' else 'integrated_demo.launch.py'
            display_arg = 'show_field:=false' if mode == 'course' else 'show_results:=false'
            results.append(run_case(name, launch_file,
                ['rviz:=false', display_arg, f'image_scenario:={scenario}', f'output_dir:={run_dir}'],
                123 if mode == 'course' else 120+index, outdir, mission_check('COMPLETE' if scenario == 'success' else 'ERROR',
                    180, scenario == 'success', True, run_dir/'state_frames'),
                'Mission complete: delivered 4 racks' if scenario == 'success' else 'Timed out waiting for QR marker RACKA_DEMO'))
            results[-1]['artifact_dir'] = str(run_dir)
            if scenario == 'success' and results[-1]['passed']:
                confirmations = [json.loads(p.read_text()) for p in (run_dir/'qr').glob('*.json')]
                expected = {'START', 'END', *(f'RACK{x}_DEMO' for x in 'ABCD')}
                if {r['payload'] for r in confirmations} != expected or len(confirmations) != 9:
                    results[-1].update(passed=False, error='Missing or duplicate confirmation artifacts')
                results[-1]['saved_confirmations'] = len(confirmations)
                if mode == 'course':
                    from robot_simulation.collision_audit import audit
                    try:
                        results[-1]['collision_audit'] = audit(run_dir)
                    except (AssertionError, OSError, ValueError) as exc:
                        results[-1].update(passed=False, error=f'Collision audit failed: {exc}')
    elif mode == 'preflight':
        results.extend(preflight_checks(outdir))
    elif mode == 'cancellation':
        results.append(cancellation_check(outdir))
    elif mode == 'envelope':
        results.append(envelope_check())
    else:
        raise ValueError('Choose rehearsal, navigation, vision, cancellation, envelope, integrated, integrated_faults, course or preflight')
    (outdir / f'{mode}.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    if not all(result['passed'] for result in results):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
