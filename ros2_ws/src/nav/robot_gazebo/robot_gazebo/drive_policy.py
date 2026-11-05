"""ROS-independent actuator admission and command evidence for Gazebo."""
import math


DRIVE_MODES = ('omni', 'differential')


def validate_drive_mode(mode):
    if mode not in DRIVE_MODES:
        raise ValueError('drive_mode must be omni or differential')
    return mode


def admitted_velocity(values, mode):
    """Reject the whole unsupported command; never silently crop lateral motion."""
    validate_drive_mode(mode)
    if not all(math.isfinite(v) for v in values):
        return (0., 0., 0.), 'nonfinite_velocity'
    if mode == 'differential' and values[1] != 0.:
        return (0., 0., 0.), 'lateral_velocity_not_supported'
    return tuple(max(-limit, min(limit, value))
                 for value, limit in zip(values, (.4, .3, 1.5))), ''


def velocity_metrics():
    return {'samples': 0, 'nonzero_samples': 0, 'lateral_samples': 0,
            'nonfinite_samples': 0, 'max_abs_vx_m_s': 0., 'max_abs_vy_m_s': 0.,
            'max_abs_wz_rad_s': 0.}


def observe_velocity(metrics, values):
    metrics['samples'] += 1
    if not all(math.isfinite(v) for v in values):
        metrics['nonfinite_samples'] += 1
        return
    metrics['nonzero_samples'] += int(any(v != 0. for v in values))
    metrics['lateral_samples'] += int(values[1] != 0.)
    for key, value in zip(('max_abs_vx_m_s', 'max_abs_vy_m_s', 'max_abs_wz_rad_s'), values):
        metrics[key] = max(metrics[key], abs(value))


def command_audit(summary, requested_mode):
    """Keep legacy Omni audits compatible; require observed evidence for DiffDrive."""
    validate_drive_mode(requested_mode)
    if requested_mode == 'omni':
        return ({'drive_mode_matches_recorded': summary['drive_mode'] == requested_mode}
                if 'drive_mode' in summary else {})
    metrics = summary.get('command_metrics', {})
    inputs = metrics.get('input', {})
    outputs = metrics.get('output', {})
    transport = metrics.get('gazebo_transport', {})
    checks = {'differential_mode_applied': summary.get('drive_mode') == requested_mode,
              'differential_input_observed': inputs.get('nonzero_samples', 0) > 0,
              'differential_input_has_no_lateral_commands': inputs.get('lateral_samples') == 0,
              'differential_no_rejected_commands': summary.get('rejected_velocity_commands') == 0}
    for name, stream in (('output', outputs), ('gazebo_transport', transport)):
        checks['differential_'+name+'_observed'] = stream.get('nonzero_samples', 0) > 0
        checks['differential_'+name+'_finite'] = stream.get('nonfinite_samples') == 0
        checks['differential_'+name+'_vy_zero'] = (stream.get('lateral_samples') == 0
                                                and stream.get('max_abs_vy_m_s') == 0.)
    return checks
