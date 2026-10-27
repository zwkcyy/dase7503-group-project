"""Create a readable run index from actual saved confirmations, without fabricating screenshots."""
import argparse
import csv
import html
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir')
    args = parser.parse_args()
    path = Path(args.run_dir).expanduser().resolve()
    if not path.is_dir():
        parser.error('run_dir must already exist')
    records = []
    for record in sorted((path/'qr').glob('*.json')):
        data = json.loads(record.read_text(encoding='utf-8'))
        image = record.parent/data['image']
        if image.parent.resolve() != record.parent.resolve() or not image.is_file():
            raise ValueError(f'Missing or invalid image path in {record}')
        records.append(data)
    if not records:
        parser.error('No saved confirmations found under run_dir/qr')
    categories = {'START', 'RACKA', 'RACKB', 'RACKC', 'RACKD', 'END'}
    present = {record['payload'].split('_')[0] for record in records}
    manifest = {'confirmations': len(records), 'categories_present': sorted(present),
                'missing_categories': sorted(categories-present),
                'sources': sorted({r['source'] for r in records}),
                'screenshots': 'Capture actual result window separately; input PNGs are not UI screenshots',
                'physical_acceptance': False}
    collision_html = ''
    if (path/'collision_summary.json').is_file():
        collision = json.loads((path/'collision_summary.json').read_text(encoding='utf-8'))
        audit_path = path/'collision_audit.json'
        audit = json.loads(audit_path.read_text(encoding='utf-8')) if audit_path.is_file() else None
        manifest['collision_metrics'] = collision['metrics']
        manifest['collision_audit'] = audit
        metrics = collision['metrics']
        gap = metrics.get('minimum_clearance_m')
        gap_text = '无记录' if gap is None else f'{gap*1000:.1f} mm'
        collision_html = ('<h2>平面货架碰撞记录</h2>'
            f'<p>实际接触：{int(metrics["actual_collisions"])}；被拦截碰撞指令：{int(metrics["blocked_collision_commands"])}；'
            f'最小间距：{gap_text}。</p><p>轨迹重算：'
            + ('通过' if audit and audit.get('passed') else '尚未通过或尚未执行')
            + '。仅对应记录中的平面模型，不代表实机净空、轮滑或接触力已验收。</p>'
            '<p><a href="collision_summary.json">碰撞与放置详情</a> · '
            '<a href="collision_trace.jsonl">逐步位姿</a></p>')
        if (path/'collision_paths.png').is_file():
            collision_html += '<img style="max-width:100%" src="collision_paths.png" alt="Four-rack trajectories">'
    (path/'evidence_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    with (path/'confirmations.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader(); writer.writerows(records)
    cards = ''.join(f'<article><h2>{html.escape(r["payload"])}</h2><img src="qr/{html.escape(r["image"], quote=True)}">'
        f'<p>{html.escape(r["received_at"])}</p><p>Source: {html.escape(r["source"])}</p>'
        f'<p>Decoder: {html.escape(r["detector"])}</p></article>' for r in records)
    (path/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>NAV / VI run evidence</title>'
        '<style>body{font:16px sans-serif;max-width:1100px;margin:32px auto}main{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}'
        'article{border:1px solid #aaa;padding:16px}img{width:100%;max-width:300px}</style>'
        '<h1>NAV / VI 扫码记录</h1><p>这些图片是实际确认时的输入帧，不是扫码窗口截图。'
        '来源为 synthetic/image_replay 的记录不能作为实机摄像头验收证明。</p>'
        f'<p>Missing categories: {html.escape(str(manifest["missing_categories"]))}</p>'
        f'{collision_html}<main>{cards}</main>', encoding='utf-8')
    print(path/'index.html')


if __name__ == '__main__':
    main()
