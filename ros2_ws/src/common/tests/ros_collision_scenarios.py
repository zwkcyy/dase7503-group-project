"""Full Nav2/QR missions with explicitly perturbed physical-world assumptions."""
import copy
import json
from pathlib import Path
import sys
import uuid
import yaml
from ros_integration import run_case, mission_check
from robot_simulation.collision_audit import audit


def main(output):
    output.mkdir(parents=True, exist_ok=True)
    source = Path(__file__).resolve().parents[2]/'nav/robot_simulation/config/course_layout.yaml'
    scene = yaml.safe_load(source.read_text())
    variants = {
        'localization_and_rack_offsets': {
            'localization_offset_mm_rad': [30., -30., .035],
            'rack_pose_error_mm_rad': {'A': [8., -8., .008], 'B': [-8., 8., -.008],
                                       'C': [8., -8., .008], 'D': [-8., 8., -.008]}},
        'narrower_racks_thicker_legs': {'rack_geometry_mm': {'depth': 140., 'width': 380., 'leg': 20.}},
        'repeat_nominal': {}}
    results = []
    for i, (name, changes) in enumerate(variants.items()):
        variant = copy.deepcopy(scene)
        variant.update(changes)
        scene_file = output/(name+'.yaml')
        scene_file.write_text(yaml.safe_dump(variant))
        directory = output/(name+'-'+uuid.uuid4().hex[:8])
        result = run_case(name, 'course_demo.launch.py',
            ['rviz:=false', 'show_field:=false', f'scene_file:={scene_file}', f'output_dir:={directory}'],
            124+i, output, mission_check('COMPLETE', 210, True, True, evidence_dir=directory), 'Mission complete: delivered 4 racks')
        if result['passed']:
            try:
                result['collision_audit'] = audit(directory)
            except (AssertionError, OSError, ValueError) as exc:
                result.update(passed=False, error=str(exc))
        result['artifact_dir'] = str(directory)
        results.append(result)
        (output/'collision_stress.json').write_text(json.dumps(results, indent=2))
    if not all(r['passed'] for r in results):
        raise SystemExit(1)


if __name__ == '__main__':
    main(Path(sys.argv[1]))
