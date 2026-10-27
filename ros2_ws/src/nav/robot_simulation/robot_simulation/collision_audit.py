"""Replay logged poses against contact geometry; never infer safety from COMPLETE."""
import json
import math
from pathlib import Path
import yaml
from .rack_world import RackWorld, corners, separation


def route_metrics(directory):
    """Measure actual travelled distance; never substitute planned path length."""
    distances, reverse, entries = {}, {}, []
    previous = None
    with (Path(directory)/'collision_trace.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            state = row['state']
            if not entries or entries[-1] != state:
                entries.append(state)
            if previous is not None:
                a, b = previous['pose'], row['pose']
                dx, dy = b[0]-a[0], b[1]-a[1]
                angle = a[2]+math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))/2
                distances[state] = distances.get(state, 0.)+math.hypot(dx, dy)
                reverse[state] = reverse.get(state, 0.)+max(0., -dx*math.cos(angle)-dy*math.sin(angle))
            previous = row
    return {'total_translation_m': sum(distances.values()),
            'reverse_translation_m': sum(reverse.values()),
            'phase_distance_m': distances, 'phase_reverse_m': reverse,
            'phase_entries': entries}


def mission_duration_seconds(trace_seconds, observer_seconds):
    """A delayed state subscription must not shorten START-to-COMPLETE timing."""
    values = (trace_seconds, observer_seconds)
    assert all(isinstance(v, (int, float)) and not isinstance(v, bool)
               and math.isfinite(v) and v > 0 for v in values), 'Missing or invalid mission timing'
    duration = max(values)
    assert duration < 180., 'Competition deadline exceeded'
    return duration


def audit(directory, margin=.01):
    directory = Path(directory)
    world = yaml.safe_load((directory/'world.yaml').read_text())
    profile = yaml.safe_load((directory/'geometry.yaml').read_text())
    final = json.loads((directory/'collision_summary.json').read_text())
    assert final['mission_state'] == 'COMPLETE', final['mission_state']
    metrics = final['metrics']
    assert metrics['actual_collisions'] == metrics['blocked_collision_commands'] == 0, metrics
    assert not metrics['lift_rejections'], metrics
    moves = final['moves']
    assert [(m['event'], m['label']) for m in moves] == [(kind, label) for label in 'ABCD' for kind in ('attach', 'place')]
    model = RackWorld(world, [v/1000 for v in profile['empty_bounds_mm']])
    index, samples, previous, minimum = 0, 0, None, float('inf')
    phases = {}
    started, completed = None, None
    last_t = None
    with (directory/'collision_trace.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            if started is None and row['state'] == 'NAV_TO_RACK':
                started = row['t']
            if completed is None and row['state'] == 'COMPLETE':
                completed = row['t']
            assert last_t is None or row['t'] > last_t, 'Non-monotonic trace'
            last_t = row['t']
            # Lift transitions happen after that tick's trace was sampled.
            changed = False
            while index < len(moves) and moves[index]['t'] < row['t']:
                move = moves[index]
                (model.lift if move['event'] == 'attach' else model.lower)(move['robot_pose'], move['label'])
                index += 1
                changed = True
            assert row['carried'] == model.carried, 'Missing attachment transition'
            assert not row['blocked'], 'Collision command was blocked by simulator'
            value, _ = model.clearance(row['pose'])
            # Include an additional swept check between independently read poses.
            if previous is not None and not changed:
                value = min(value, model.swept_clearance(previous, row['pose'])[0])
            assert value > margin, f"Clearance {value:.6f} <= required {margin}, state={row['state']}"
            minimum = min(minimum, value)
            phases[row['state']] = min(phases.get(row['state'], float('inf')), value)
            previous = row['pose']
            samples += 1
    assert index == 8 and samples > 1000, 'Incomplete trace'
    assert all(r['state'] == 'delivered' for r in model.racks)
    for i, rack in enumerate(model.racks):
        for other in model.racks[i+1:]:
            assert separation(corners(rack['pose'], model.outer), corners(other['pose'], model.outer)) > margin
    assert 'EXIT_DESTINATION' in phases
    assert started is not None and completed is not None, 'Missing mission transitions'
    observer = json.loads((directory/'summary.json').read_text())
    duration = mission_duration_seconds(completed-started, observer.get('elapsed_after_start_s'))
    route = route_metrics(directory)
    mission = yaml.safe_load((directory/'mission.yaml').read_text())['delivery_mission']['ros__parameters']
    if not mission.get('retreat_after_pickup', True):
        assert 'EXIT_RACK' not in route['phase_entries'], 'Unexpected fixed retreat after pickup'
    assert route['phase_entries'].count('EXIT_DESTINATION') == 4, 'Missing empty-rack exit'
    result = {'passed': True, 'samples_replayed': samples, 'minimum_swept_clearance_m': minimum,
              'required_clearance_m': margin, 'phase_minimum_m': phases,
              'mission_seconds_from_trace': completed-started,
              'mission_seconds_for_deadline': duration,
              'timing_basis': 'maximum of trace phase interval and QR START-to-COMPLETE observer interval',
              'all_four_placed': True, 'actual_collisions': 0, 'blocked_collision_commands': 0,
              'route': route,
              'scope': 'planar geometric replay; no vertical, slip, force or optical certification'}
    (directory/'collision_audit.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    return result
