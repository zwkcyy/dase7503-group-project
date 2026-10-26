"""Validate one course scene and derive all simulated mission/world positions."""
import re
import yaml
from robot_geometry.profile import numbers


def load_scene(path):
    with open(path, encoding='utf-8') as stream:
        scene = yaml.safe_load(stream)
    if not isinstance(scene, dict) or scene.get('layout_kind') != 'course_schematic_simulation':
        raise ValueError('Course scene must declare its simulation-only layout_kind')
    size = numbers(scene.get('size_mm'), 2, 'size_mm')
    if min(size) <= 0:
        raise ValueError('Field dimensions must be positive')
    for key in ('start_zone_mm', 'destination_zone_mm'):
        x0, y0, x1, y1 = numbers(scene.get(key), 4, key)
        if not (0 <= x0 < x1 <= size[0] and 0 <= y0 < y1 <= size[1]):
            raise ValueError(f'{key} must lie inside the field')
    for key, zone_key in (('initial_pose_mm_rad', 'start_zone_mm'),
                          ('destination_pose_mm_rad', 'destination_zone_mm')):
        x, y, _ = numbers(scene.get(key), 3, key)
        x0, y0, x1, y1 = scene[zone_key]
        if not (x0 < x < x1 and y0 < y < y1):
            raise ValueError(f'{key} must lie inside {zone_key}')
    racks = scene.get('racks')
    if not isinstance(racks, list) or len(racks) != 4 or not all(isinstance(r, dict) for r in racks):
        raise ValueError('Course scene requires four labelled rack points')
    if {rack.get('label') for rack in racks} != set('ABCD'):
        raise ValueError('Course scene requires unique labels A, B, C, D')
    for rack in racks:
        x, y, _ = numbers([rack.get(k) for k in ('x_mm', 'y_mm', 'yaw_rad')], 3, 'rack pose')
        if not (0 < x < size[0] and 0 < y < size[1]):
            raise ValueError('Rack point must lie inside the field')
        if not isinstance(rack.get('qr'), str) or not re.fullmatch(f"RACK{rack['label']}_DEMO", rack['qr']):
            raise ValueError('Course simulation must retain explicit DEMO QR payloads')
    if scene.get('position_basis') != 'approximate_centres_from_page_3_not_surveyed':
        raise ValueError('Rack position provenance must remain explicit')
    obstacles = scene.get('obstacles_mm')
    if not isinstance(obstacles, list):
        raise ValueError('obstacles_mm must be a list')
    for box in obstacles:
        x0, y0, x1, y1 = numbers(box, 4, 'obstacle')
        if not (0 <= x0 < x1 <= size[0] and 0 <= y0 < y1 <= size[1]):
            raise ValueError('Obstacle must lie inside the field')
    return scene


def world_config(scene):
    x, y, yaw = scene['initial_pose_mm_rad']
    return {'size': [v/1000. for v in scene['size_mm']],
            'initial_pose': [x/1000., y/1000., yaw],
            'obstacles': [[v/1000. for v in box] for box in scene['obstacles_mm']]}


def map_config(scene):
    """Use the existing wall-map generator with the same scene dimensions."""
    return {'field': {'length_mm': scene['size_mm'][0], 'width_mm': scene['size_mm'][1],
                      'wall_thickness_mm': 20.},
            'map': {'name': 'course_map', 'resolution_mm': 20., 'margin_mm': 200.},
            'zones': {'start': list(scene['start_zone_mm']), 'destination': list(scene['destination_zone_mm'])}}


def mission_config(scene):
    racks = scene['racks']
    x, y, yaw = scene['destination_pose_mm_rad']
    return {'delivery_mission': {'ros__parameters': {
        # This flag enables the simulated task only. The source scene remains
        # explicitly non-surveyed and is never installed as a hardware default.
        'coordinates_confirmed': True, 'manage_load_envelope': True,
        'rack_labels': [r['label'] for r in racks], 'rack_qr_codes': [r['qr'] for r in racks],
        'rack_x_m': [r['x_mm']/1000. for r in racks], 'rack_y_m': [r['y_mm']/1000. for r in racks],
        'rack_yaw_rad': [float(r['yaw_rad']) for r in racks],
        'destination_x_m': x/1000., 'destination_y_m': y/1000., 'destination_yaw_rad': float(yaw),
        'qr_timeout_s': 4., 'start_timeout_s': 20., 'request_timeout_s': 2.,
        'lift_timeout_s': 4., 'navigation_timeout_s': 30., 'competition_time_s': 180.}}}


def screen_point(x_m, y_m, size_m, rectangle):
    """Portrait presentation: destination above start; never change ROS coordinates."""
    left, top, right, bottom = rectangle
    return (left + y_m/size_m[1]*(right-left), top + x_m/size_m[0]*(bottom-top))
