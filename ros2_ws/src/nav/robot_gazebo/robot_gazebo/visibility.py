"""Measure horizontal CAD/rack shadows; never hide visuals from Gazebo lidar."""
import argparse
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from ament_index_python.packages import get_package_share_directory
from .world import vertices, rotation


def cad_triangles():
    cad = Path(get_package_share_directory('logistics_robot_description'))
    root = ET.parse(cad/'urdf/logistics_robot.urdf').getroot()
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    result = {}
    for link in root.findall('link'):
        name = link.get('name')
        if name == 'base_link':
            continue
        joint = joints[name]
        if joint.find('parent').get('link') != 'base_link':
            raise ValueError('CAD changed; compose nested transforms before analysis')
        origin = joint.find('origin')
        xyz = np.array(list(map(float, origin.get('xyz').split())))+[0, 0, .111519852810214]
        rpy = list(map(float, origin.get('rpy', '0 0 0').split()))
        pieces = []
        for visual in link.findall('visual'):
            local = visual.find('origin')
            if local is not None and any(float(v) != 0 for k in ('xyz', 'rpy') for v in local.get(k, '0 0 0').split()):
                raise ValueError('CAD visual origin changed; mirror world importer before analysis')
            path = cad/visual.find('geometry/mesh').get('filename').replace('package://logistics_robot_description/', '')
            pieces.append((vertices(path) @ rotation(rpy).T+xyz).reshape(-1, 3, 3))
        if pieces:
            result[name] = np.concatenate(pieces)
    return result


def slice_triangles(triangles, z):
    triangles = triangles[(triangles[:, :, 2].min(axis=1) < z) & (triangles[:, :, 2].max(axis=1) > z)]
    if not len(triangles):
        return np.empty((0, 2, 2))
    endpoints = []
    for a, b in ((0, 1), (1, 2), (2, 0)):
        p, q = triangles[:, a, :], triangles[:, b, :]
        valid = (p[:, 2] < z) != (q[:, 2] < z)
        with np.errstate(divide='ignore', invalid='ignore'):
            fraction = (z-p[:, 2])/(q[:, 2]-p[:, 2])
            hit = p[:, :2]+fraction[:, None]*(q[:, :2]-p[:, :2])
        hit[~valid] = np.nan
        endpoints.append(hit)
    points = np.stack(endpoints, axis=1)
    return points[np.isfinite(points[:, :, 0])].reshape(-1, 2, 2)


def sections(meshes, z, loaded, rack_offset=(0., 0., 0.), laser_x=.145):
    result = []
    for name, triangles in meshes.items():
        if name == 'lidar_link':
            # Match the generated world: translate the CAD top-face datum to
            # 1 mm below the proposed optical centre, retaining housing shape.
            triangles = triangles+np.array([laser_x-.108750002671, 0, z-.001-.046883306518314])
        height = z-(.025 if loaded and name == 'lift_tray_1' else 0.)
        result.extend((name, segment) for segment in slice_triangles(triangles, height))
    # The declared course fixture: four 15 mm legs, 140 x 400 mm deck footprint.
    # This is a horizontal optical slice, not an analysis of overhang detection.
    if loaded and .021 < z < .247:
        dx, dy, angle = rack_offset
        matrix = rotation([0, 0, angle])[:2, :2]
        for x in (-.0625, .0625):
            for y in (-.1925, .1925):
                corners = np.array([[x-.0075, y-.0075], [x+.0075, y-.0075],
                                    [x+.0075, y+.0075], [x-.0075, y+.0075]]) @ matrix.T+[dx, dy]
                result.extend(('rack_leg', np.array([corners[i], corners[(i+1) % 4]])) for i in range(4))
    return result


def ray_hits(segments, origin, angles):
    if not len(segments):
        return np.full(len(angles), np.inf), np.zeros(len(angles), dtype=int)
    segments = np.asarray(segments)
    a, side = segments[:, 0, :]-origin, segments[:, 1, :]-segments[:, 0, :]
    directions = np.stack((np.cos(angles), np.sin(angles)), axis=1)
    cross = lambda u, v: u[..., 0]*v[..., 1]-u[..., 1]*v[..., 0]
    determinant = cross(directions[:, None, :], side[None, :, :])
    with np.errstate(divide='ignore', invalid='ignore'):
        distance = cross(a, side)[None, :]/determinant
        along = cross(a[None, :, :], directions[:, None, :])/determinant
    distance[(distance < .001) | (along < 0) | (along > 1) | ~np.isfinite(distance)] = np.inf
    return distance.min(axis=1), distance.argmin(axis=1)


def measure(meshes, x=.145, z=.09, loaded=False, rack_offset=(0., 0., 0.)):
    pairs = sections(meshes, z, loaded, rack_offset, x)
    # Match Gazebo's inclusive 720-ray angular grid.
    angles = np.linspace(-math.pi, math.pi, 720)
    distances, owners = ray_hits([p for _, p in pairs], [x, 0], angles)
    blocked = np.isfinite(distances)
    intervals, start = [], None
    for i, value in enumerate([*blocked, False]):
        if value and start is None:
            start = i
        if not value and start is not None:
            intervals.append([round(math.degrees(angles[start]), 3), round(math.degrees(angles[i-1]), 3)])
            start = None
    names = np.array([name for name, _ in pairs])
    counts = {name: int(sum(blocked & (names[owners] == name))) for name in sorted(set(names[owners[blocked]]))} if len(names) else {}
    central = np.abs(angles) >= math.radians(168)
    report = {'optical_origin_mm': [x*1000, 0, z*1000], 'loaded': loaded,
              'rack_offset_m_rad': list(rack_offset), 'samples': len(angles),
              'blocked_fraction': float(np.mean(blocked)),
              'equivalent_blocked_angle_deg': float(np.mean(blocked)*360),
              'blocked_intervals_deg': intervals, 'occluder_ray_counts': counts,
              'central_rear_blocked_rays': int(sum(blocked & central))}
    return report, pairs, angles, distances


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    directory = Path(args.output).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    meshes = cad_triangles()
    reports = []
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(10, 9))
    for row, z in enumerate((.07, .09)):
        for col, loaded in enumerate((False, True)):
            report, pairs, angles, distances = measure(meshes, z=z, loaded=loaded)
            reports.append(report)
            ax = axes[row, col]
            for _, segment in pairs:
                ax.plot(segment[:, 0]*1000, segment[:, 1]*1000, color='black', lw=.5)
            for a, d in zip(angles, distances):
                if math.isfinite(d):
                    ax.plot([145+1000*d*math.cos(a), 145+600*math.cos(a)],
                            [1000*d*math.sin(a), 600*math.sin(a)], color='#df6b4a', alpha=.12)
            ax.plot(145, 0, 'bo')
            ax.annotate('front +X', xy=(260, 0), xytext=(150, -55), arrowprops={'arrowstyle': '->'})
            ax.set(xlim=(-400, 300), ylim=(-350, 350), aspect='equal', xlabel='X (mm)', ylabel='Y (mm)',
                   title=f'{"Loaded" if loaded else "Empty"}, z={z*1000:.0f} mm; shadow {report["equivalent_blocked_angle_deg"]:.1f} deg')
            ax.grid(alpha=.2)
    fig.suptitle('CAD horizontal lidar shadows (red = occluded, not free space)')
    fig.tight_layout(); fig.savefig(directory/'cad-lidar-shadows.png', dpi=160); plt.close(fig)
    offsets = [measure(meshes, loaded=True, rack_offset=(x, y, a))[0]
               for x in (-.025, .025) for y in (-.025, .025) for a in (-.05, .05)]
    result = {'nominal': reports, 'loaded_offset_corners': offsets,
              'limitations': ['Ideal horizontal slice of supplied CAD, not measured optical calibration.',
                              '0.5 degree sampling; shadows smaller than angular resolution may be missed.',
                              'Rack legs/deck use declared course simulation dimensions.',
                              'Only z=70 and 90 mm are compared; 2D lidar cannot cover obstacles outside its scan plane.',
                              'Supplied housing is translated with the optical mount; proposed bracket and optical datum are unmeasured.']}
    (directory/'visibility.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
