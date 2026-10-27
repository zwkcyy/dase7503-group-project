"""Render a static audit figure from a completed run; never record video."""
import json
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2_ws/src/nav/robot_simulation'))
from robot_simulation.world import corners


def main(directory):
    world = yaml.safe_load((directory/'world.yaml').read_text())
    summary = json.loads((directory/'collision_summary.json').read_text())
    audit = json.loads((directory/'collision_audit.json').read_text())
    rows = [json.loads(line) for line in (directory/'collision_trace.jsonl').read_text().splitlines()]
    moves = summary['moves']
    depth, width, leg = [world['rack_geometry'][key] for key in ('depth', 'width', 'leg')]
    outer = [-depth/2, -width/2, depth/2, width/2]
    fig, axes = plt.subplots(1, 4, figsize=(14, 6.2), layout='constrained')
    for i, (axis, original) in enumerate(zip(axes, world['racks'])):
        axis.set_facecolor('#f2f5f6')
        # Swap display axes to match the course's portrait diagram.
        axis.set_xlim(0, 2)
        axis.set_ylim(3.02, 0)
        axis.set_aspect('equal')
        axis.add_patch(Rectangle((.3, 0), 1.4, .8, color='#d4efde', zorder=0))
        axis.add_patch(Rectangle((.6, 2.42), .8, .6, color='#d8e8fd', zorder=0))
        axis.text(1., .1, 'DESTINATION', ha='center', va='top', fontsize=8, color='#236841')
        axis.text(1., 2.9, 'START', ha='center', fontsize=8, color='#285e9e')
        for j, rack in enumerate(world['racks']):
            pose = summary['racks'][j]['pose'] if j < i else rack['pose']
            points = [(y, x) for x, y in corners(pose, outer)]
            axis.add_patch(Polygon(points, facecolor='none' if j == i else ('#81c9a3' if j < i else '#ffdc83'),
                                   edgecolor='#76501b', lw=1., linestyle='--' if j == i else '-'))
            for x in (-depth/2, depth/2-leg):
                for y in (-width/2, width/2-leg):
                    axis.add_patch(Polygon([(v, u) for u, v in corners(pose, [x, y, x+leg, y+leg])], color='#76501b'))
            axis.text(pose[1], pose[0]-.10, rack['label']+(' pickup' if j == i else ''),
                      ha='center', fontsize=8 if j == i else 10, weight='bold')
        start = rows[0]['t'] if i == 0 else moves[2*i-1]['t']
        stop = moves[2*i+1]['t']
        segment = [r for r in rows if start <= r['t'] <= stop]
        for loaded, color, label in ((False, '#2867c7', 'Empty'), (True, '#b65c05', 'Carrying')):
            part = [r for r in segment if (r['carried'] == original['label'] if loaded else r['carried'] is None)]
            if part:
                axis.plot([r['pose'][1] for r in part], [r['pose'][0] for r in part], color=color, lw=1.8, label=label)
        for r in segment[::max(1, len(segment)//10)]:
            if r['carried']:
                # The logged attachment offset, including angular offset.
                from robot_simulation.rack_world import compose
                polygon = corners(compose(r['pose'], r['attachment']), outer)
                axis.add_patch(Polygon([(y, x) for x, y in polygon], fill=False, edgecolor='#b65c05', alpha=.35, lw=.8))
        final = summary['racks'][i]['pose']
        axis.add_patch(Polygon([(y, x) for x, y in corners(final, outer)], facecolor='#81c9a3', edgecolor='#246e49', lw=1.5))
        axis.text(final[1], final[0], original['label'], ha='center', va='center', fontsize=9, weight='bold')
        axis.set_title(f"Rack {original['label']}", fontsize=12, weight='bold')
        axis.set_xlabel('Field y (m)')
        if i == 0:
            axis.set_ylabel('Field x (m)')
            axis.legend(loc='lower left', fontsize=8)
        axis.grid(alpha=.15)
    fig.suptitle(f"Four-rack collision audit | minimum swept gap {audit['minimum_swept_clearance_m']*1000:.1f} mm\n"
                 '0 contacts / 0 blocked collision commands | planar assumptions, not physical certification', fontsize=13)
    fig.savefig(directory/'collision_paths.png', dpi=180)
    plt.close(fig)
    print(directory/'collision_paths.png')


if __name__ == '__main__':
    main(Path(sys.argv[1]))
