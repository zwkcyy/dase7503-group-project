"""Read-only checks before starting a production mission."""
import math
import re
import yaml


def mission_parameters(path):
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    params = data['delivery_mission']['ros__parameters']
    if params.get('coordinates_confirmed') is not True:
        raise ValueError('Mission coordinates are unconfirmed; fill the measured mission file first')
    labels = params.get('rack_labels', [])
    codes = params.get('rack_qr_codes', [])
    if (not isinstance(labels, list) or not labels
            or any(not isinstance(x, str) or x not in {'A', 'B', 'C', 'D'} for x in labels)
            or len(set(labels)) != len(labels)):
        raise ValueError('Rack labels must be unique A/B/C/D entries')
    if not isinstance(codes, list) or len(codes) != len(labels) or any(not isinstance(code, str) or not re.fullmatch(
            f'RACK{label}_[A-Za-z0-9]+', code) for label, code in zip(labels, codes)):
        raise ValueError('Each rack needs its full, matching team QR code')
    values = []
    for key in ('rack_x_m', 'rack_y_m', 'rack_yaw_rad'):
        array = params.get(key, [])
        if not isinstance(array, list) or len(array) != len(labels):
            raise ValueError(f'{key} must have one value per rack')
        values.extend(array)
    values.extend(params.get('destination_' + name) for name in ('x_m', 'y_m', 'yaw_rad'))
    if any(isinstance(x, bool) or not isinstance(x, (float, int)) or not math.isfinite(x) for x in values):
        raise ValueError('All mission positions and angles must be finite numbers')
    for key in ('qr_timeout_s', 'request_timeout_s', 'lift_timeout_s', 'navigation_timeout_s', 'competition_time_s'):
        value = params.get(key)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
    start_timeout = params.get('start_timeout_s', 0.)
    if (isinstance(start_timeout, bool) or not isinstance(start_timeout, (float, int))
            or not math.isfinite(start_timeout) or start_timeout < 0):
        raise ValueError('start_timeout_s must be finite and nonnegative')
    return params
