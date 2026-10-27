"""Complete a real course run and save ONE final UI screenshot (no video)."""
import json
import os
from pathlib import Path
import sys
import time
import uuid
from PIL import ImageGrab
from ros_integration import run_case, mission_check, spin_until
from robot_simulation.collision_audit import audit


def main(output):
    output.mkdir(parents=True, exist_ok=True)
    directory = output/('viewer-'+uuid.uuid4().hex[:8])

    def check(node):
        result = mission_check('COMPLETE', 210, True, True)(node)
        until = time.monotonic()+.8
        spin_until(node, lambda: time.monotonic() >= until, 2.)
        image = ImageGrab.grab(xdisplay=os.environ.get('DISPLAY'))
        image.save(directory/'field_viewer_final.png')
        result['screenshot'] = str(directory/'field_viewer_final.png')
        return result

    result = run_case('field_viewer', 'course_demo.launch.py',
        ['rviz:=false', 'show_field:=true', 'fullscreen:=true', f'output_dir:={directory}'],
        137, output, check, 'Mission complete: delivered 4 racks')
    if result['passed']:
        try:
            result['collision_audit'] = audit(directory)
        except (AssertionError, OSError, ValueError) as exc:
            result.update(passed=False, error=str(exc))
    result['artifact_dir'] = str(directory)
    (output/'viewer.json').write_text(json.dumps(result, indent=2))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main(Path(sys.argv[1]))
