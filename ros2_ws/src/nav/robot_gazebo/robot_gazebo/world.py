"""Build a self-contained Gazebo world from the course scene and untouched CAD.

Wheel contacts use the anisotropic friction approach in Gazebo's official
mecanum_drive.sdf example. CAD visuals retain their original dimensions;
conservative component AABBs replace concave STL collision meshes. The lift is
a prismatic equivalent of the saved fixed linkage, driven through real contact.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
from xml.etree import ElementTree as ET

import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from robot_geometry.profile import load_profile
from robot_simulation.course_scene import load_scene, course_profile, mission_config, map_config, world_config
from robot_simulation.qr_images import qr_image


def e(parent, tag, value=None, **attrs):
    child = ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    if value is not None:
        child.text = (' '.join(format(float(v), '.12g') for v in value)
                      if isinstance(value, (list, tuple, np.ndarray)) else str(value))
    return child


def plugin(parent, filename, name, **values):
    result = e(parent, 'plugin', filename='gz-sim-'+filename+'-system', name='gz::sim::systems::'+name)
    for key, value in values.items():
        e(result, key, value)
    return result


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def vertices(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84+50*count:
        raise ValueError(f'Expected binary STL: {path}')
    triangles = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), offset=84)
    return triangles['v'].reshape(-1, 3).astype(float)


def material(visual, colour):
    m = e(visual, 'material')
    e(m, 'ambient', colour)
    e(m, 'diffuse', colour)
    return m


def box(link, name, size, pose, colour=None, collide=True, friction=.8):
    if colour is not None:
        v = e(link, 'visual', name=name)
        e(v, 'pose', pose)
        e(e(e(v, 'geometry'), 'box'), 'size', size)
        material(v, colour)
    if collide:
        c = e(link, 'collision', name=name)
        e(c, 'pose', pose)
        e(e(e(c, 'geometry'), 'box'), 'size', size)
        ode = e(e(e(c, 'surface'), 'friction'), 'ode')
        e(ode, 'mu', friction)
        e(ode, 'mu2', friction)
        return c


def inertia(link, mass, size, centre=(0, 0, 0)):
    i = e(link, 'inertial')
    e(i, 'pose', [*centre, 0, 0, 0])
    e(i, 'mass', mass)
    tensor = e(i, 'inertia')
    x, y, z = size
    for key, value in {'ixx': mass*(y*y+z*z)/12, 'iyy': mass*(x*x+z*z)/12,
                       'izz': mass*(x*x+y*y)/12, 'ixy': 0, 'ixz': 0, 'iyz': 0}.items():
        e(tensor, key, value)


def contact_sensor(link, topic):
    sensor = e(link, 'sensor', name='contacts', type='contact')
    e(sensor, 'always_on', 'true')
    e(sensor, 'update_rate', 100)
    e(sensor, 'topic', topic)
    contact = e(sensor, 'contact')
    e(contact, 'topic', topic)
    for collision in link.findall('collision'):
        e(contact, 'collision', collision.get('name'))


def marker(link, name, payload, pose, size, assets):
    """A textured static surface, always in the world; never mission-gated."""
    texture = assets / ('qr_'+payload+'.png')
    cv2.imwrite(str(texture), qr_image(payload, 512))
    visual = e(link, 'visual', name=name)
    e(visual, 'pose', [*pose[:3], 0, math.pi/2, pose[5]])
    m = e(e(visual, 'geometry'), 'plane')
    e(m, 'normal', [0, 0, 1])
    e(m, 'size', [size, size])
    mat = material(visual, [1, 1, 1, 1])
    pbr = e(e(mat, 'pbr'), 'metal')
    e(pbr, 'albedo_map', str(texture))
    e(pbr, 'metalness', 0)
    e(pbr, 'roughness', 1)
    e(visual, 'cast_shadows', 'false')


def floor_label(link, label, centre, assets, colour=(.45, .48, .5)):
    canvas = np.full((160, 640, 3), [int(v*255) for v in colour[::-1]], dtype=np.uint8)
    scale = min(2., 530./max(1, cv2.getTextSize(label, cv2.FONT_HERSHEY_DUPLEX, 1., 2)[0][0]))
    width = cv2.getTextSize(label, cv2.FONT_HERSHEY_DUPLEX, scale, 2)[0][0]
    cv2.putText(canvas, label, ((640-width)//2, 103), cv2.FONT_HERSHEY_DUPLEX, scale, (245, 245, 245), 2, cv2.LINE_AA)
    path = assets/('label_'+label.replace(' ', '_')+'.png'); cv2.imwrite(str(path), canvas)
    visual = e(link, 'visual', name=label.replace(' ', '_')+'_label')
    e(visual, 'pose', [*centre, .001, 0, 0, math.pi/2])
    plane = e(e(visual, 'geometry'), 'plane'); e(plane, 'normal', [0, 0, 1]); e(plane, 'size', [.42, .105])
    pbr = e(e(material(visual, [1, 1, 1, 1]), 'pbr'), 'metal')
    e(pbr, 'albedo_map', str(path)); e(pbr, 'metalness', 0); e(pbr, 'roughness', 1)


def cad_robot(world, scene, profile, settings, assets):
    cad = Path(get_package_share_directory('logistics_robot_description'))
    root = ET.parse(cad/'urdf/logistics_robot.urdf').getroot()
    links = {x.get('name'): x for x in root.findall('link')}
    joints = {x.find('child').get('link'): x for x in root.findall('joint')}
    offset = .111519852810214
    model = e(world, 'model', name='robot')
    x, y, yaw = scene['initial_pose_mm_rad']
    e(model, 'pose', [x/1000, y/1000, .001, 0, 0, yaw])
    e(model, 'self_collide', 'false')
    body = e(model, 'link', name='base_link')
    tray = e(model, 'link', name='lift_tray')
    bounds, body_mass, tray_bounds, tray_mass = [], 0., [], 0.
    wheels = {}
    for name, source in links.items():
        if name == 'base_link':
            continue
        joint = joints[name]
        if joint.find('parent').get('link') != 'base_link':
            raise ValueError('CAD tree changed: explicitly compose nested transforms before importing')
        origin = joint.find('origin')
        xyz = np.array([float(v) for v in origin.get('xyz').split()]) + [0, 0, offset]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        if name == 'lidar_link':
            # Move the supplied housing together with the proposed optical mount.
            # Its CAD origin is at the top face; allow 1 mm optical clearance.
            # This is an explicit bracket proposal, not a measured sensor origin.
            xyz = np.array(profile['laser_xyz_mm'])/1000-[0, 0, .001]
        mass = float(source.find('inertial/mass').get('value'))
        is_wheel = 'wheel_link' in name
        is_tray = name == 'lift_tray_1'
        target = tray if is_tray else body
        if is_wheel:
            target = e(model, 'link', name=name)
            # The CAD saved wheel spin is visual only; axle +Y is common to all wheels.
            centre = [xyz[0], xyz[1], profile['wheel_radius_mm']/1000]
            e(target, 'pose', [*centre, 0, 0, 0])
            wheels[name] = centre
            inertia(target, mass, [.06, .0315, .06])
        elif is_tray:
            tray_mass += mass
        else:
            body_mass += mass
        for index, visual in enumerate(source.findall('visual')):
            filename = visual.find('geometry/mesh').get('filename').replace('package://logistics_robot_description/', '')
            path = cad / filename
            points = vertices(path) @ rotation(rpy).T + xyz
            lower, upper = points.min(axis=0), points.max(axis=0)
            if is_tray:
                tray_bounds.extend([lower, upper])
            elif not is_wheel:
                bounds.extend([lower, upper])
            v = e(target, 'visual', name=name+'_'+str(index))
            local = xyz-np.array(wheels[name]) if is_wheel else xyz
            e(v, 'pose', [*local, *rpy])
            e(e(e(v, 'geometry'), 'mesh'), 'uri', str(path))
            colour = visual.find('material/color')
            material(v, [float(a) for a in colour.get('rgba').split()] if colour is not None else [.7, .7, .7, 1])
            if not is_wheel:
                box(target, 'cad_'+name, np.maximum(upper-lower, .001), [*((upper+lower)/2), 0, 0, 0],
                    friction=settings['lift_contact_friction'])
        if is_wheel:
            collision = e(target, 'collision', name='tire')
            e(collision, 'pose', [0, 0, 0, math.pi/2, 0, 0])
            cylinder = e(e(collision, 'geometry'), 'cylinder')
            e(cylinder, 'radius', profile['wheel_radius_mm']/1000)
            e(cylinder, 'length', profile['wheel_width_mm']/1000)
            friction = e(e(e(collision, 'surface'), 'friction'), 'ode')
            e(friction, 'mu', settings['wheel_friction'])
            e(friction, 'mu2', settings['wheel_roller_friction'])
            sign = -1 if name.startswith('front_left') or name.startswith('rear_right') else 1
            e(friction, 'fdir1', [1, sign, 0], **{'gz:expressed_in': 'base_link'})
            j = e(model, 'joint', name=name+'_joint', type='revolute')
            e(j, 'parent', 'base_link'); e(j, 'child', name)
            axis = e(j, 'axis'); e(axis, 'xyz', [0, 1, 0])
            limit = e(axis, 'limit'); e(limit, 'lower', -1e16); e(limit, 'upper', 1e16); e(limit, 'effort', 2)
            contact_sensor(target, '/gazebo/contacts/'+name)
    lower, upper = np.min(bounds, axis=0), np.max(bounds, axis=0)
    inertia(body, body_mass, upper-lower, (upper+lower)/2)
    low, high = np.min(tray_bounds, axis=0), np.max(tray_bounds, axis=0)
    inertia(tray, tray_mass, np.maximum(high-low, .001), (high+low)/2)
    j = e(model, 'joint', name='lift_joint', type='prismatic')
    e(j, 'parent', 'base_link'); e(j, 'child', 'lift_tray')
    axis = e(j, 'axis'); e(axis, 'xyz', [0, 0, 1])
    limit = e(axis, 'limit'); e(limit, 'lower', 0); e(limit, 'upper', settings['lift_stroke_mm']/1000)
    e(limit, 'effort', settings['lift_force_limit_n']); e(limit, 'velocity', settings['lift_speed_mm_s']/1000)
    plugin(model, 'joint-position-controller', 'JointPositionController', joint_name='lift_joint',
           topic='/gazebo/lift_target', p_gain=settings['lift_kp'], i_gain=settings['lift_ki'],
           d_gain=settings['lift_kd'], i_min=-settings['lift_force_limit_n'], i_max=settings['lift_force_limit_n'],
           cmd_min=-settings['lift_force_limit_n'], cmd_max=settings['lift_force_limit_n'])
    drive = plugin(model, 'mecanum-drive', 'MecanumDrive', wheel_separation=profile['wheel_track_mm']/1000,
                   wheelbase=profile['wheel_base_mm']/1000, wheel_radius=profile['wheel_radius_mm']/1000,
                   topic='/gazebo/cmd_vel', odom_topic='/gazebo/wheel_odom', tf_topic='/gazebo/unused_wheel_tf',
                   frame_id='odom', child_frame_id='base_link', odom_publish_frequency=50,
                   min_linear_acceleration=-.8, max_linear_acceleration=.8,
                   min_angular_acceleration=-1.5, max_angular_acceleration=1.5)
    for prefix, wheel in [('front_left', 'front_left'), ('front_right', 'front_right'),
                           ('back_left', 'rear_left'), ('back_right', 'rear_right')]:
        e(drive, prefix+'_joint', wheel+'_wheel_link_joint')
    plugin(model, 'joint-state-publisher', 'JointStatePublisher', topic='/gazebo/joint_states', update_rate=50)
    contact_sensor(body, '/gazebo/contacts/body')
    contact_sensor(tray, '/gazebo/contacts/tray')
    imu = e(body, 'sensor', name='imu', type='imu')
    e(imu, 'pose', [*[v/1000 for v in profile['imu_xyz_mm']], *profile['imu_rpy_rad']])
    e(imu, 'topic', '/gazebo/imu'); e(imu, 'gz_frame_id', 'imu_link'); e(imu, 'update_rate', 50)
    e(imu, 'always_on', 'true'); e(imu, 'imu')
    scan = e(body, 'sensor', name='laser', type='gpu_lidar')
    e(scan, 'pose', [*[v/1000 for v in profile['laser_xyz_mm']], *profile['laser_rpy_rad']])
    e(scan, 'topic', '/gazebo/scan'); e(scan, 'gz_frame_id', 'laser'); e(scan, 'update_rate', 15)
    e(scan, 'always_on', 'true')
    lidar = e(scan, 'lidar'); horizontal = e(e(lidar, 'scan'), 'horizontal')
    for k, v in dict(samples=720, resolution=1, min_angle=-math.pi, max_angle=math.pi).items(): e(horizontal, k, v)
    limits = e(lidar, 'range')
    for k, v in dict(min=.025, max=5., resolution=.001).items(): e(limits, k, v)
    camera = e(body, 'sensor', name='camera', type='camera')
    e(camera, 'pose', [*[v/1000 for v in profile['camera_xyz_mm']], *profile['camera_rpy_rad']])
    e(camera, 'topic', '/gazebo/camera'); e(camera, 'gz_frame_id', 'camera_optical_frame')
    e(camera, 'update_rate', settings['camera_hz']); e(camera, 'always_on', 'true')
    cam = e(camera, 'camera'); e(cam, 'horizontal_fov', settings['camera_hfov_rad'])
    img = e(cam, 'image'); e(img, 'width', settings['camera_width']); e(img, 'height', settings['camera_height'])
    e(img, 'format', 'R8G8B8'); clip = e(cam, 'clip'); e(clip, 'near', .01); e(clip, 'far', 8)
    return {'body_mass_kg': body_mass, 'tray_mass_kg': tray_mass,
            'body_bounds_m': [lower.tolist(), upper.tolist()], 'tray_bounds_m': [low.tolist(), high.tolist()],
            'wheel_centres_m': wheels, 'rack_underdeck_m': (settings['rack_height_mm']-settings['rack_deck_thickness_mm'])/1000,
            'saved_cad_to_underdeck_clearance_m': (settings['rack_height_mm']-settings['rack_deck_thickness_mm'])/1000-high[2]}


def generate(directory, scene_file=None, physics_file=None):
    directory = Path(directory).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=False)
    assets = directory/'assets'; assets.mkdir()
    sim = Path(get_package_share_directory('robot_simulation'))
    own = Path(get_package_share_directory('robot_gazebo'))
    scene = load_scene(scene_file or sim/'config/course_layout.yaml')
    settings = yaml.safe_load(Path(physics_file or own/'config/physics.yaml').read_text())
    if settings['schema'] != 'gazebo_course_physics_v1': raise ValueError('Unsupported physics schema')
    if any(not math.isfinite(float(v)) or float(v) < 0 for k, v in settings.items() if k != 'schema'):
        raise ValueError('Physics parameters must be finite and nonnegative')
    for key in ('physics_step_s', 'real_time_factor', 'rack_mass_kg', 'rack_height_mm', 'rack_deck_thickness_mm',
                'lift_stroke_mm', 'lift_speed_mm_s', 'lift_force_limit_n', 'lift_kp', 'camera_hz',
                'camera_width', 'camera_height', 'camera_hfov_rad', 'qr_size_mm', 'end_qr_size_mm'):
        if settings[key] <= 0:
            raise ValueError(f'{key} must be positive')
    if not .0001 <= settings['physics_step_s'] <= .002:
        raise ValueError('Physics step must be between 0.1 and 2 ms')
    if not .1 <= settings['camera_hfov_rad'] < math.pi:
        raise ValueError('Camera HFOV must be in [0.1, pi)')
    profile = course_profile(scene, load_profile(sim/'config/course_geometry.yaml', True))
    # Proposed unobstructed level laser reference. The saved CAD sensor is upside down.
    profile['laser_xyz_mm'] = [settings['laser_x_mm'], 0., settings['laser_z_mm']]
    profile['camera_rpy_rad'] = [0., -settings['camera_upward_pitch_rad'], 0.]
    profile['self_filter_z_mm'] = [0., 300.]
    sdf = ET.Element('sdf', version='1.9', attrib={'xmlns:gz': 'http://gazebosim.org/schema'})
    world = e(sdf, 'world', name='course')
    p = e(world, 'physics', name='physics', type='ignored')
    e(p, 'max_step_size', settings['physics_step_s']); e(p, 'real_time_factor', settings['real_time_factor'])
    plugin(world, 'physics', 'Physics'); plugin(world, 'user-commands', 'UserCommands')
    plugin(world, 'scene-broadcaster', 'SceneBroadcaster'); plugin(world, 'imu', 'Imu')
    plugin(world, 'contact', 'Contact'); plugin(world, 'sensors', 'Sensors', render_engine='ogre2')
    e(world, 'gravity', [0, 0, -9.81])
    light = e(world, 'light', name='sun', type='directional')
    e(light, 'pose', [1, 1, 5, 0, 0, 0]); e(light, 'direction', [-.3, .2, -1])
    e(light, 'diffuse', [.85, .85, .85, 1]); e(light, 'cast_shadows', 'true')
    render = e(world, 'scene'); e(render, 'ambient', [.65, .65, .65, 1]); e(render, 'background', [.2, .24, .3, 1])
    field = e(world, 'model', name='field'); e(field, 'static', 'true'); floor = e(field, 'link', name='ground')
    length, width = [v/1000 for v in scene['size_mm']]
    box(floor, 'floor', [length+.04, width+.04, .04], [length/2, width/2, -.02, 0, 0, 0], [.45, .48, .5, 1])
    h = settings['wall_height_mm']/1000
    for name, size, pose in [('west', [.02, width+.04, h], [-.01, width/2, h/2, 0, 0, 0]),
                              ('east', [.02, width+.04, h], [length+.01, width/2, h/2, 0, 0, 0]),
                              ('south', [length, .02, h], [length/2, -.01, h/2, 0, 0, 0]),
                              ('north', [length, .02, h], [length/2, width+.01, h/2, 0, 0, 0])]:
        box(floor, name, size, pose, [.8, .84, .87, 1])
    for key, colour in [('start_zone_mm', [.17, .46, .7, 1]), ('destination_zone_mm', [.34, .58, .22, 1])]:
        x0, y0, x1, y1 = [v/1000 for v in scene[key]]
        box(floor, key, [x1-x0, y1-y0, .0005], [(x0+x1)/2, (y0+y1)/2, .0003, 0, 0, 0], colour, False)
    for i, bounds in enumerate(scene['obstacles_mm']):
        x0, y0, x1, y1 = [v/1000 for v in bounds]
        box(floor, f'obstacle_{i}', [x1-x0, y1-y0, h], [(x0+x1)/2, (y0+y1)/2, h/2, 0, 0, 0], [.6, .4, .2, 1])
    qsize, qheight = settings['qr_size_mm']/1000, settings['qr_centre_height_mm']/1000
    sx, sy = settings['start_qr_x_mm']/1000, settings['start_qr_y_mm']/1000
    marker(floor, 'start_qr', 'START', [sx+.002, sy, qheight, 0, 0, 0], qsize, assets)
    box(floor, 'start_qr_panel', [.003, qsize, qsize], [sx, sy, qheight, 0, 0, 0], [.9, .9, .9, 1])
    box(floor, 'start_qr_post', [.02, .02, qheight-qsize/2], [sx, sy, (qheight-qsize/2)/2, 0, 0, 0], [.2, .5, .2, 1])
    floor_label(floor, 'START', [2.85, 1.], assets, (.17, .46, .7))
    floor_label(floor, 'DESTINATION', [.4, 1.], assets, (.34, .58, .22))
    for i, y in enumerate((.61, 1.39)):
        marker(floor, f'end_qr_{i}', 'END', [.001, y, settings['end_qr_centre_height_mm']/1000, 0, 0, 0],
               settings['end_qr_size_mm']/1000, assets)
    depth, width, leg = [scene['rack_geometry_mm'][k]/1000 for k in ('depth', 'width', 'leg')]
    height, thickness = settings['rack_height_mm']/1000, settings['rack_deck_thickness_mm']/1000
    physical_poses = {r['label']: r['pose'] for r in world_config(scene)['racks']}
    for rack in scene['racks']:
        m = e(world, 'model', name='rack_'+rack['label'])
        px, py, pa = physical_poses[rack['label']]
        e(m, 'pose', [px, py, 0, 0, 0, pa])
        floor_label(floor, 'RACK '+rack['label'], [px+.16, py], assets)
        link = e(m, 'link', name='rack')
        inertia(link, settings['rack_mass_kg'], [depth, width, height], [0, 0, height*.65])
        box(link, 'deck', [depth, width, thickness], [0, 0, height-thickness/2, 0, 0, 0], [.88, .64, .18, 1])
        for i, (x, y) in enumerate(( (x,y) for x in (-1,1) for y in (-1,1))):
            box(link, f'leg_{i}', [leg, leg, height-thickness],
                [x*(depth-leg)/2, y*(width-leg)/2, (height-thickness)/2, 0, 0, 0], [.7, .5, .12, 1],
                friction=settings['rack_floor_friction'])
        # Course photo shows the QR above the rack. Keep the under-deck corridor clear.
        rack_qr_height = height+qsize/2
        box(link, 'qr_panel', [.001, qsize, qsize], [-depth/2-.001, 0, rack_qr_height, 0, 0, 0], [1, 1, 1, 1])
        marker(link, 'qr', rack['qr'], [-depth/2-.0016, 0, rack_qr_height, 0, 0, math.pi], qsize, assets)
        contact_sensor(link, '/gazebo/contacts/rack_'+rack['label'])
    metadata = cad_robot(world, scene, profile, settings, assets)
    # A front-mounted housing protrudes beyond the saved chassis. Include it in
    # BOTH footprints, self filtering and the clearance needed before turning.
    low_body, high_body = metadata['body_bounds_m']
    bounds = profile['empty_bounds_mm']
    profile['empty_bounds_mm'] = [min(bounds[0], low_body[0]*1000), min(bounds[1], low_body[1]*1000),
                                  max(bounds[2], high_body[0]*1000), max(bounds[3], high_body[1]*1000)]
    profile = course_profile(scene, profile)
    metadata['proposed_lidar_mount'] = {'optical_origin_mm': profile['laser_xyz_mm'],
        'housing_origin_mm': [profile['laser_xyz_mm'][0], profile['laser_xyz_mm'][1], profile['laser_xyz_mm'][2]-1.],
        'basis': 'Translate supplied housing; optical plane assumed 1 mm above CAD top face; bracket unmeasured'}
    if metadata['saved_cad_to_underdeck_clearance_m'] <= 0:
        raise ValueError('Saved CAD tray does not fit below this rack deck; fix geometry instead of disabling collisions')
    metadata.update(engine='Gazebo Harmonic DART', camera_source='Gazebo Ogre2 rendered scene',
                    lift='prismatic equivalent with friction contact; no attachment or teleport',
                    collision_geometry='CAD component AABBs; cylindrical anisotropic wheel contacts',
                    unmeasured=['friction', 'rack mass', 'deck thickness', 'lift stroke and speed',
                                'sensor placement and intrinsics', 'rack centres and widths'])
    ET.indent(sdf)
    ET.ElementTree(sdf).write(directory/'course.sdf', encoding='utf-8', xml_declaration=True)
    mission = mission_config(scene, profile)
    mission['delivery_mission']['ros__parameters']['use_sim_time'] = True
    mission['delivery_mission']['ros__parameters']['lift_timeout_s'] = 8.
    mission['delivery_mission']['ros__parameters']['prefer_expected_qr'] = True
    nav = Path(get_package_share_directory('robot_navigation'))
    params = yaml.safe_load((nav/'config/nav2_params.yaml').read_text())
    # These zero-noise rendered sensors require narrower likelihoods than hardware.
    amcl = params['amcl']['ros__parameters']
    amcl.update(sigma_hit=.025, z_hit=.95, z_rand=.05, max_beams=360,
                update_min_d=.01, update_min_a=.02)
    for key in ('alpha1', 'alpha2', 'alpha3', 'alpha4', 'alpha5'):
        amcl[key] = .005
    # These goals are safe staging points, not final lift/drop poses. Allow
    # 40 mm here to avoid shim re-entry at the 25 mm boundary after AMCL updates.
    # Docking still finishes at 8 mm / 0.018 rad; cargo audit limits are unchanged.
    params['controller_server']['ros__parameters']['general_goal_checker'].update(xy_goal_tolerance=.04, yaw_goal_tolerance=.04)
    for name, data in [('scene.yaml', scene), ('physics.yaml', settings), ('geometry.yaml', profile),
                       ('mission.yaml', mission), ('field.yaml', map_config(scene)), ('nav2.yaml', params)]:
        (directory/name).write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    (directory/'model_assumptions.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    package_root = Path(__file__).resolve().parents[1]
    repo = next((p for p in package_root.parents if (p/'.git').exists()), None)
    if repo:
        files = [p for group in ('nav', 'vi', 'common') for p in (repo/'ros2_ws/src'/group).rglob('*')
                 if p.is_file() and p.suffix in ('.py', '.yaml', '.xml', '.config', '.xacro', '.urdf')
                 and '__pycache__' not in p.parts]
        manifest = {'revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip(),
                    'sha256': {str(p.relative_to(repo)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}}
        (directory/'source_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    subprocess.run([sys.executable, str(nav/'scripts/generate_map.py'), '--config', str(directory/'field.yaml'),
                    '--out-dir', str(directory)], check=True)
    return directory


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', required=True); p.add_argument('--scene'); p.add_argument('--physics')
    args = p.parse_args()
    print(generate(args.output, args.scene, args.physics))
