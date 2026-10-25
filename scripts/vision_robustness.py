"""Measure the production decoder on deterministic synthetic images without ROS.

Only clean/90-degree samples and absence of false positives are pass/fail gates.
Other distortions measure limitations, without inventing camera acceptance targets.
"""
import argparse
import csv
import html
import json
from pathlib import Path
import statistics
import sys
import tempfile
import time
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2_ws/src/common/tests'))
sys.path.insert(0, str(ROOT/'ros2_ws/src/nav/robot_simulation'))
from ros_fakes import Node, load_module
from robot_simulation.qr_images import qr_image, variants


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    module = load_module('robustness_detector', 'vi/robot_vision/robot_vision/qr_detector_node.py', True)
    rows = []
    with tempfile.TemporaryDirectory() as tmp, patch.object(Node, 'overrides', {
            'model_dir': str(Path(tmp)/'no_optional_models'), 'log_file': str(Path(tmp)/'log.csv')}):
        node = module.QRDetectorNode()
        cases = []
        for payload in ('START', 'RACKA_DEMO', 'RACKB_DEMO', 'RACKC_DEMO', 'RACKD_DEMO', 'END'):
            cases.extend((payload, name, frame, mandatory) for name, frame, mandatory in variants(payload))
        for payload in ('start', 'RACKE_1234', 'RACKA', 'https://example.org'):
            cases.append((None, 'invalid_'+str(len(cases)), qr_image(payload), True))
        cases.append((None, 'blank', np.full((480, 480, 3), 255, np.uint8), True))
        for index, (expected, name, frame, required) in enumerate(cases):
            times, decoded = [], []
            for _ in range(3):
                begin = time.perf_counter()
                result = node.detect_qr(frame)
                times.append((time.perf_counter()-begin)*1000)
                decoded.append(result[0] if result else None)
            correct = sum(value == expected for value in decoded)
            false_positive = any(value is not None and value != expected for value in decoded)
            filename = f'{index:03d}-{name}.png'
            if not cv2.imwrite(str(output/filename), frame):
                raise OSError(filename)
            rows.append({'payload': expected, 'variant': name, 'decoded': decoded, 'correct': correct,
                         'attempts': 3, 'required': required, 'false_positive': false_positive,
                         'median_ms': round(statistics.median(times), 2), 'image': filename})
    passed = all(not row['false_positive'] and (not row['required'] or row['correct'] == 3) for row in rows)
    report = {'passed': passed, 'source': 'synthetic_images', 'opencv_version': cv2.__version__,
              'backend': 'production QRCodeDetector fallback; optional WeChat models absent',
              'cases': len(rows), 'attempts': len(rows)*3,
              'correct_attempts': sum(row['correct'] for row in rows),
              'false_positive_cases': sum(row['false_positive'] for row in rows),
              'physical_acceptance': False, 'results': rows}
    (output/'results.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    with (output/'results.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    cards = ''.join(f'<article><h3>{html.escape(str(r["payload"]))} / {r["variant"]}</h3>'
                    f'<img src="{r["image"]}" alt="Input fixture"><p>{r["correct"]}/3 correct; '
                    f'{r["median_ms"]} ms median; baseline gate: {r["required"]}</p></article>' for r in rows)
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>QR image robustness</title>'
        '<style>body{font:16px sans-serif;max-width:1200px;margin:32px auto}main{display:grid;grid-template-columns:repeat(3,1fr);gap:20px}'
        'article{border:1px solid #bbb;padding:16px}img{max-width:100%;height:180px;object-fit:contain}</style>'
        '<h1>QR image robustness</h1><p>Synthetic inputs; actual production decoder. This is not camera acceptance.</p>'
        f'<p>Baseline/false-positive gates passed: {passed}. OpenCV {cv2.__version__}.</p><main>{cards}</main>', encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k != 'results'}))
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
