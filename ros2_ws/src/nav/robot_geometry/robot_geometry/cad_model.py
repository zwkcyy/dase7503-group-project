"""Compose supplied fixed CAD visuals with the NAV frame convention."""

import copy
from pathlib import Path
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from .profile import numbers


def vector(values):
    return ' '.join(format(value, '.12g') for value in values)


def fixed(root, name, parent, child, xyz_mm, rpy):
    joint = ET.SubElement(root, 'joint', name=name, type='fixed')
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)
    ET.SubElement(joint, 'origin', xyz=vector([x / 1000 for x in numbers(xyz_mm, 3, name)]),
                  rpy=vector(numbers(rpy, 3, name)))


def description(profile):
    """Do not modify the imported URDF or publish a second base_link tree."""
    source = Path(get_package_share_directory('logistics_robot_description'))
    source = ET.parse(source / 'urdf/model_snapshot_fixed.urdf').getroot()
    root = ET.Element('robot', name='dase7503_cad_navigation')
    ET.SubElement(root, 'link', name='base_link')
    for item in source:
        item = copy.deepcopy(item)
        if item.tag == 'link':
            item.set('name', 'cad_' + item.get('name'))
        elif item.tag == 'joint':
            if item.get('type') != 'fixed':
                raise ValueError('NAV CAD description requires the fixed snapshot')
            item.set('name', 'cad_' + item.get('name'))
            for element in ('parent', 'child'):
                endpoint = item.find(element)
                endpoint.set('link', 'cad_' + endpoint.get('link'))
        root.append(item)
    fixed(root, 'cad_mount_joint', 'base_link', 'cad_base_link',
          profile['cad_origin_xyz_mm'], profile['cad_origin_rpy_rad'])
    for name, key in (('laser', 'laser'), ('camera_link', 'camera'), ('imu_link', 'imu')):
        ET.SubElement(root, 'link', name=name)
        fixed(root, name + '_reference_joint', 'base_link', name,
              profile[key + '_xyz_mm'], profile[key + '_rpy_rad'])
    ET.SubElement(root, 'link', name='camera_optical_frame')
    fixed(root, 'camera_optical_joint', 'camera_link', 'camera_optical_frame',
          [0, 0, 0], [-1.5707963267948966, 0, -1.5707963267948966])
    return ET.tostring(root, encoding='unicode')
