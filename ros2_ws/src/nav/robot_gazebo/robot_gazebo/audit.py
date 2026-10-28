"""Audit persisted Gazebo observations, independently of mission success text."""
import argparse
import json
import math
from pathlib import Path
import yaml
from robot_simulation.world import corners, overlap


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def tilt(pose):
    return math.acos(max(-1., min(1., 1-2*(pose['qx']**2+pose['qy']**2))))


def audit(path):
    path = Path(path)
    summary = json.loads((path/'gazebo_summary.json').read_text())
    scene = yaml.safe_load((path/'scene.yaml').read_text())
    geometry = yaml.safe_load((path/'geometry.yaml').read_text())
    rows = [json.loads(line) for line in (path/'gazebo_trace.jsonl').read_text().splitlines()]
    observer = json.loads((path/'observer.json').read_text())
    checks = {'mission_complete': summary.get('mission_state') == 'COMPLETE' and summary.get('rack_index') == 4,
              'contact_monitor_active': summary.get('contact_samples_received', 0) > 100,
              'no_unexpected_contacts': summary.get('unexpected_contact_samples') == 0,
              'no_emergency': not summary.get('emergency_latched', True),
              'eight_lift_transitions': [(x['label'], x['up']) for x in summary.get('moves', [])] ==
                                        [(label, up) for label in 'ABCD' for up in (True, False)],
              'nine_decoded_markers': observer.get('codes') == ['START', 'RACKA_DEMO', 'END', 'RACKB_DEMO',
                                                               'END', 'RACKC_DEMO', 'END', 'RACKD_DEMO', 'END'],
              'four_empty_exits': sum(p['state'] == 'EXIT_DESTINATION' for p in observer['phases']) == 4}
    active = [r for r in rows if r['mission_state'] not in ('WAIT_START',)]
    complete = next((r for r in rows if r['mission_state'] == 'COMPLETE'), None)
    duration = complete['sim_seconds']-active[0]['sim_seconds'] if complete and active else None
    duration = observer.get('state', {}).get('task_elapsed_s', duration)
    checks['within_180_simulation_seconds'] = complete is not None and duration is not None and 0 < duration < 180
    gaps = [b['sim_seconds']-a['sim_seconds'] for a, b in zip(rows, rows[1:])]
    checks['continuous_trace'] = (bool(gaps) and min(gaps) > 0 and max(gaps) < .2
                                  and summary.get('dropped_truth_rows', 0) == 0)
    depth, width = [scene['rack_geometry_mm'][k]/1000 for k in ('depth', 'width')]
    zone = [v/1000 for v in scene['destination_zone_mm']]
    placements, polygons = {}, []
    for label, goal in zip('ABCD', scene['placement_poses_mm_rad']):
        p = summary['poses']['rack_'+label]
        polygon = corners([p['x'], p['y'], p['yaw']], [-depth/2, -width/2, depth/2, width/2])
        position_error = math.hypot(p['x']-goal[0]/1000, p['y']-goal[1]/1000)
        yaw_error = abs(wrap(p['yaw']-goal[2]))
        inside = all(zone[0] < x < zone[2] and zone[1] < y < zone[3] for x, y in polygon)
        placements[label] = {'position_error_m': position_error, 'yaw_error_rad': yaw_error,
            'ground_height_m': p['z'], 'tilt_rad': tilt(p), 'entire_rack_in_zone': inside}
        polygons.append(polygon)
    checks['all_placements_within_tolerance'] = all(
        p['position_error_m'] < .04 and p['yaw_error_rad'] < .08 and abs(p['ground_height_m']) < .002
        and p['tilt_rad'] < .05 and p['entire_rack_in_zone'] for p in placements.values())
    checks['placed_racks_do_not_overlap'] = all(not overlap(a, b) for i, a in enumerate(polygons) for b in polygons[i+1:])
    loaded_rows = [r for r in rows if r['carried'] and r['mission_state'] not in ('LOWER_RACK', 'ERROR')]
    minimum_lift = min((r['poses']['rack_'+r['carried']]['z'] for r in loaded_rows), default=None)
    max_tilt = max((tilt(p) for r in rows for p in r['poses'].values()), default=None)
    checks['cargo_stays_lifted_in_transit'] = minimum_lift is not None and minimum_lift > .005
    checks['robot_and_racks_remain_upright'] = max_tilt is not None and max_tilt < .05
    loaded_limits = [v/1000 for v in geometry['loaded_bounds_mm']]
    max_excess = 0.
    for row in loaded_rows:
        robot, rack = row['poses']['robot'], row['poses']['rack_'+row['carried']]
        c, s = math.cos(robot['yaw']), math.sin(robot['yaw'])
        dx, dy = rack['x']-robot['x'], rack['y']-robot['y']
        polygon = corners([c*dx+s*dy, -s*dx+c*dy, wrap(rack['yaw']-robot['yaw'])],
                          [-depth/2, -width/2, depth/2, width/2])
        for x, y in polygon:
            max_excess = max(max_excess, loaded_limits[0]-x, loaded_limits[1]-y,
                             x-loaded_limits[2], y-loaded_limits[3])
    checks['cargo_within_planned_loaded_footprint'] = bool(loaded_rows) and max_excess <= 1e-5
    distance = sum(math.hypot(b['poses']['robot']['x']-a['poses']['robot']['x'],
                             b['poses']['robot']['y']-a['poses']['robot']['y']) for a, b in zip(rows, rows[1:]))
    report = {'passed': all(checks.values()), 'checks': checks, 'engine': 'Gazebo Harmonic 8 / DART',
              'task_simulation_seconds': duration, 'process_wall_seconds': observer['wall_seconds'],
              'travel_distance_m': distance, 'minimum_cargo_ground_clearance_m': minimum_lift,
              'maximum_tilt_rad': max_tilt, 'maximum_trace_gap_s': max(gaps, default=None),
              'maximum_loaded_footprint_excess_m': max_excess, 'placements': placements,
              'unexpected_contact_samples': summary['unexpected_contact_samples'],
              'contact_samples_received': summary['contact_samples_received'], 'qr_codes': observer['codes'],
              'limitations': ['Unmeasured friction, masses, deck thickness, sensor and QR poses.',
                              'Component box collisions and ideal roller friction; equivalent prismatic lift.',
                              'Simulation validation does not certify real hardware or course height compliance.']}
    (path/'gazebo_audit.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    return report


def main():
    p = argparse.ArgumentParser(); p.add_argument('directory'); args = p.parse_args()
    report = audit(args.directory)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['passed'] else 1)
