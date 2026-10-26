"""Validate one CAD input and derive robot model, footprint and filter bounds."""

import copy
import json
import math

import yaml


def numbers(value, count, name):
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ValueError(f'{name} needs {count} numbers')
    if any(isinstance(x, bool) or not isinstance(x, (int, float))
           or not math.isfinite(x) for x in value):
        raise ValueError(f'{name} must contain finite numbers')
    return list(map(float, value))


def bounds(value, name):
    a, b, c, d = numbers(value, 4, name)
    if not (a < c and b < d and a <= 0 <= c and b <= 0 <= d):
        raise ValueError(f'{name} must have positive area and contain base_link')
    return [a, b, c, d]


def load_profile(path, allow_unconfirmed=False, empty_only=False):
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get('confirmed'), bool):
        raise ValueError('Geometry requires a boolean confirmed field')
    accepted_empty = empty_only and data.get('development_baseline_accepted') is True
    if not data['confirmed'] and not allow_unconfirmed and not accepted_empty:
        raise ValueError('Geometry requires physical verification for a full mission, or an accepted development baseline with empty_only.')
    for key in ('body_length', 'body_width', 'body_height', 'body_clearance',
                'wheel_radius', 'wheel_width', 'wheel_base', 'wheel_track',
                'footprint_padding', 'self_filter_margin'):
        value = numbers([data.get(key + '_mm')], 1, key)[0]
        if value < 0 or (key not in ('body_clearance', 'footprint_padding',
                                    'self_filter_margin') and value == 0):
            raise ValueError(f'{key} has an invalid dimension')
    for sensor in ('laser', 'camera', 'imu'):
        numbers(data.get(sensor + '_xyz_mm'), 3, sensor + '_xyz_mm')
        numbers(data.get(sensor + '_rpy_rad'), 3, sensor + '_rpy_rad')
    empty = bounds(data.get('empty_bounds_mm'), 'empty_bounds_mm')
    # This effective envelope only supports unloaded commissioning. It is never
    # written to the source file, and the service refuses a loaded transition.
    if empty_only and data.get('loaded_bounds_mm') is None:
        data['loaded_bounds_mm'] = list(empty)
    data['empty_only'] = bool(empty_only)
    loaded = bounds(data.get('loaded_bounds_mm'), 'loaded_bounds_mm')
    if not (loaded[0] <= empty[0] and loaded[1] <= empty[1]
            and loaded[2] >= empty[2] and loaded[3] >= empty[3]):
        raise ValueError('Loaded envelope must contain the complete empty envelope')
    if not (empty[0] <= -data['body_length_mm'] / 2
            and empty[2] >= data['body_length_mm'] / 2
            and empty[1] <= -data['body_width_mm'] / 2
            and empty[3] >= data['body_width_mm'] / 2):
        raise ValueError('Empty envelope must contain the chassis')
    low, high = numbers(data.get('self_filter_z_mm'), 2, 'self_filter_z_mm')
    if low >= high:
        raise ValueError('self_filter_z_mm needs increasing heights')
    return data


def polygon(data, loaded):
    x0, y0, x1, y1 = [v / 1000 for v in data[
        'loaded_bounds_mm' if loaded else 'empty_bounds_mm']]
    return [[x1, y1], [x1, y0], [x0, y0], [x0, y1]]


def nav_parameters(template, data, loaded=False):
    """Copy a Nav2 config; never rewrite the source template or CAD input."""
    result = copy.deepcopy(template)
    radius = max(math.hypot(x, y) for x, y in polygon(data, True))
    padding = data['footprint_padding_mm'] / 1000
    for name in ('local_costmap', 'global_costmap'):
        params = result[name][name]['ros__parameters']
        params['footprint'] = json.dumps(polygon(data, loaded))
        params['footprint_padding'] = padding
        params['inflation_layer']['inflation_radius'] = max(
            params['inflation_layer']['inflation_radius'], radius + 2 * padding + .03)
    return result


def inside_filter(data, loaded, x, y, z):
    x0, y0, x1, y1 = [v / 1000 for v in data[
        'loaded_bounds_mm' if loaded else 'empty_bounds_mm']]
    margin = data['self_filter_margin_mm'] / 1000
    z0, z1 = [v / 1000 for v in data['self_filter_z_mm']]
    return x0 - margin <= x <= x1 + margin and y0 - margin <= y <= y1 + margin and z0 <= z <= z1


def transform_point(transform, x, y, z=0.0):
    """Apply a normalised quaternion and translation to a scan endpoint."""
    q, t = transform.rotation, transform.translation
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid scan transform quaternion')
    qx, qy, qz, qw = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    ux, uy, uz = 2*(qy*z-qz*y), 2*(qz*x-qx*z), 2*(qx*y-qy*x)
    return (x + qw*ux + qy*uz-qz*uy + t.x,
            y + qw*uy + qz*ux-qx*uz + t.y,
            z + qw*uz + qx*uy-qy*ux + t.z)
