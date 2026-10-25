#!/usr/bin/env python3
"""Step 7-6: build a run-scoped inspection report from inspection_results.jsonl.

Examples:
  python3 inspection_report.py mark
  python3 inspection_report.py build
  python3 inspection_report.py build --expected B,A

Only lines after the `mark` are used. For a new run, mark BEFORE sending
an instruction, then build AFTER both VLM results arrive.
"""
import argparse
import base64
import csv
import html
import json
import mimetypes
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path.home() / 'ros2_ws' / 'data'
LOG = ROOT / 'inspection_results.jsonl'
MARKER = ROOT / '.inspection_report_baseline.json'
OUTPUT = ROOT / 'reports'
STATUS = {'NORMAL', 'ABNORMAL', 'WARNING', 'UNKNOWN', 'ERROR'}


def line_count(path):
    if not path.exists():
        return 0
    with path.open('r', encoding='utf-8') as f:
        return sum(1 for _ in f)


def mark(log, marker):
    marker.parent.mkdir(parents=True, exist_ok=True)
    info = {'line_count': line_count(log), 'created_at': datetime.now().astimezone().isoformat(), 'log_path': str(log.resolve())}
    marker.write_text(json.dumps(info, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'Run baseline recorded: {info["line_count"]} existing lines. Start your new ROS2 mission now.')
    print(f'Marker: {marker}')


def rows_since(log, baseline):
    rows, errors = [], []
    if not log.exists():
        return rows, ['result log does not exist']
    with log.open('r', encoding='utf-8') as f:
        for i, line in enumerate(f, start=1):
            if i <= baseline or not line.strip():
                continue
            try:
                r = json.loads(line)
                if not isinstance(r, dict) or r.get('equipment') not in ('A', 'B'):
                    raise ValueError('missing equipment A/B')
                r = dict(r)
                r['_line'] = i
                rows.append(r)
            except (ValueError, TypeError) as e:
                errors.append(f'line {i}: {e}')
    return rows, errors


def data_url(path):
    if not path or not Path(path).is_file():
        return None
    p = Path(path)
    if p.suffix.lower() not in ('.png', '.jpg', '.jpeg') or p.stat().st_size > 10_000_000:
        return None
    mime = mimetypes.guess_type(p.name)[0] or 'image/png'
    return f'data:{mime};base64,{base64.b64encode(p.read_bytes()).decode("ascii")}'


def build(args):
    if not args.marker.exists():
        raise SystemExit('No run marker. Execute: python3 inspection_report.py mark BEFORE starting a new run.')
    info = json.loads(args.marker.read_text(encoding='utf-8'))
    if info.get('log_path') != str(args.log.resolve()):
        raise SystemExit('Marker points at a different log file; run mark again.')
    if args.log.exists() and line_count(args.log) < info['line_count']:
        raise SystemExit('Result log was truncated after mark; run mark again.')
    rows, errors = rows_since(args.log, info['line_count'])
    expected = [x.strip().upper() for x in args.expected.split(',') if x.strip()]
    if not expected or any(x not in ('A', 'B') for x in expected) or len(set(expected)) != len(expected):
        raise SystemExit('Expected must be a unique sequence of A and B, e.g. --expected B,A')
    latest = {name: None for name in expected}
    for r in rows:
        if r['equipment'] in latest:
            latest[r['equipment']] = r
    complete = all(latest.values())
    successes = sum(1 for r in latest.values() if r and r.get('success') is True and r.get('status') in STATUS - {'ERROR'})
    correct_map = {'A': ('green', 'NORMAL'), 'B': ('red', 'ABNORMAL')}
    matched = sum(1 for name, r in latest.items() if r and r.get('success') is True and (r.get('indicator_color'), r.get('status')) == correct_map[name])
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    fields = ['equipment', 'success', 'status', 'indicator_color', 'target_visible', 'reason', 'inference_sec', 'image_path', 'model']
    with (out / 'inspection_summary.csv').open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for name in expected:
            r = latest[name] or {}
            writer.writerow({field: (name if field == 'equipment' else r.get(field, '')) for field in fields})
    report = {
        'run_marked_at': info['created_at'],
        'expected_order': expected,
        'received_total_lines_since_marker': len(rows),
        'received_equipment': [r['equipment'] for r in rows],
        'all_expected_results_present': complete,
        'successful_results': successes,
        'expected_color_status_matches': matched,
        'parse_errors': errors,
        'latest_results': {k: latest[k] for k in expected},
        'note': 'This report summarizes VLM results only; it does not verify robot traversal, capture completion, or ROS timings.',
    }
    (out / 'inspection_summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')

    styles = '''body{font-family:system-ui,'Noto Sans CJK JP',sans-serif;max-width:1050px;margin:40px auto;padding:0 20px;color:#24313b;background:#f8fafc}h1{margin-bottom:6px}.sub{color:#617282}.summary{display:flex;gap:12px;flex-wrap:wrap;margin:22px 0}.metric{background:white;padding:16px 20px;border-radius:12px;box-shadow:0 1px 4px #d9e1e7;min-width:140px}.metric strong{display:block;font-size:24px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(310px,1fr));gap:16px}.card{background:white;border-radius:12px;padding:18px;box-shadow:0 1px 4px #d9e1e7}.card img{width:100%;height:auto;max-height:320px;object-fit:contain;background:#eee;border-radius:8px}.status{font-weight:bold}.ok{color:#087a45}.alert{color:#b91c1c}.warn{color:#91640a}.dim{color:#718096}.bar{height:15px;background:#e2e8f0;border-radius:6px;overflow:hidden}.fill{height:100%;background:#2563eb}.field{word-break:break-word}small{color:#64748b}code{white-space:pre-wrap}'''
    def esc(v): return html.escape(str(v if v is not None else '—'))
    cards = []
    for name in expected:
        r = latest[name]
        if r is None:
            cards.append(f'<article class="card"><h2>設備 {name}</h2><p class="status warn">結果未受信</p></article>')
            continue
        state = r.get('status', 'UNKNOWN')
        cls = 'ok' if state == 'NORMAL' else ('alert' if state in ('ABNORMAL', 'ERROR') else 'warn')
        src = data_url(r.get('image_path'))
        image_html = f'<img src="{src}" alt="設備 {name} の撮影画像">' if src else '<p class="warn">画像を表示できません</p>'
        inference = r.get('inference_sec')
        bar = ''
        if isinstance(inference, (float, int)) and inference >= 0:
            w = min(100, inference / 30 * 100)
            bar = f'<div class="bar"><div class="fill" style="width:{w:.1f}%"></div></div><small>推論 {inference:.2f}秒（横棒の尺度：30秒）</small>'
        cards.append(f'<article class="card"><h2>設備 {name} <span class="status {cls}">{esc(state)}</span></h2>{image_html}<p>表示灯: <strong>{esc(r.get("indicator_color"))}</strong> / 対象識別: {esc(r.get("target_visible"))}</p><p>{esc(r.get("reason"))}</p>{bar}<p class="field"><small>画像: {esc(r.get("image_path"))}</small></p></article>')
    warnings = ('<p class="warn">未受信の設備結果があります。VLMノードの処理完了を待ってから再生成してください。</p>' if not complete else '')
    warnings += ''.join(f'<p class="warn">JSONL読み込み: {esc(e)}</p>' for e in errors)
    page = f'''<!doctype html><html lang="ja"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>設備点検レポート</title><style>{styles}</style></head><body>
<h1>設備点検レポート</h1><p class="sub">Step 7-6 / 実行基準: {esc(info['created_at'])} / 指定順序: {esc(' → '.join(expected))}</p>
<div class="summary"><div class="metric">受信設備数<strong>{sum(bool(r) for r in latest.values())} / {len(expected)}</strong></div><div class="metric">推論成功<strong>{successes} / {len(expected)}</strong></div><div class="metric">模擬正解一致<strong>{matched} / {len(expected)}</strong></div></div>
{warnings}<section class="grid">{''.join(cards)}</section><p class="sub">※ 模擬正解: A=緑/NORMAL, B=赤/ABNORMAL。現時点の設備状態について、VLMの出力と模擬正解を照合したものです。実機の異常検知精度ではありません。</p>
<p class="sub">※ このレポートはJSONLの記録から生成。ロボット巡回時間・停止位置・撮影時刻の証明には別途Controllerログが必要です。</p></body></html>'''
    (out / 'inspection_dashboard.html').write_text(page, encoding='utf-8')
    print(f'Report output: {out}')
    print(f'Received {sum(bool(r) for r in latest.values())}/{len(expected)} equipment results; success={successes}, expected_label_matches={matched}')
    for name in expected:
        r = latest[name]
        print(f'  {name}: {r.get("status", "UNKNOWN") if r else "NOT_RECEIVED"} / {r.get("indicator_color", "-") if r else "-"}')
    if not complete:
        print('INCOMPLETE: Await VLM result(s), then run build again. Do not report this run as complete.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['mark', 'build'])
    parser.add_argument('--log', type=Path, default=LOG)
    parser.add_argument('--marker', type=Path, default=MARKER)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    parser.add_argument('--expected', default='B,A')
    args = parser.parse_args()
    if args.action == 'mark': mark(args.log, args.marker)
    else: build(args)

if __name__ == '__main__':
    main()
