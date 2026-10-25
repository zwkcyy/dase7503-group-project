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
        f'<p>Missing categories: {html.escape(str(manifest["missing_categories"]))}</p><main>{cards}</main>', encoding='utf-8')
    print(path/'index.html')


if __name__ == '__main__':
    main()
