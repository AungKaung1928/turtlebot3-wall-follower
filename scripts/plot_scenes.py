#!/usr/bin/env python3
"""plot_scenes.py <scenes.csv> <out.svg> [title]: /cmd_vel over time from the launch test (SCENE_CSV):
linear speed and angular rate, with the synthetic scan scene and the node's state as bands.
Standard library only."""
import csv
import sys

src, out = sys.argv[1], sys.argv[2]
title = sys.argv[3] if len(sys.argv) > 3 else 'wall follower, launch test scenes'
rows = [(float(r['t']), r['scene'], r['state'], float(r['v']), float(r['w'])) for r in csv.DictReader(open(src))]
T = rows[-1][0]
W, L, R = 640, 56, 12
PH, TOP, GAP = 120, 48, 36
H = TOP + 2 * PH + GAP + 64


def X(t):
    return L + t / T * (W - L - R)


def panel(y_top, key, lo, hi, label):
    def Y(v):
        return y_top + (hi - v) / (hi - lo) * PH
    pts = ' '.join(f'{X(r[0]):.1f},{Y(r[key]):.1f}' for r in rows)
    el = [f'<rect x="{L}" y="{y_top}" width="{W - L - R}" height="{PH}" fill="none" stroke="#c9d1d9"/>',
          f'<line x1="{L}" y1="{Y(0):.1f}" x2="{W - R}" y2="{Y(0):.1f}" stroke="#c9d1d9" stroke-dasharray="3 3"/>',
          f'<text x="{L - 6}" y="{Y(hi) + 10:.1f}" text-anchor="end" fill="#6b7785">{hi:g}</text>',
          f'<text x="{L - 6}" y="{Y(lo):.1f}" text-anchor="end" fill="#6b7785">{lo:g}</text>',
          f'<text x="{L - 6}" y="{Y(0) + 4:.1f}" text-anchor="end" fill="#6b7785">0</text>',
          f'<text x="{L + 6}" y="{y_top + 14}" fill="#10161e">{label}</text>',
          f'<polyline points="{pts}" fill="none" stroke="#005f73" stroke-width="2"/>']
    return el


bands, start = [], 0
for i in range(1, len(rows) + 1):
    if i == len(rows) or rows[i][1] != rows[start][1]:
        bands.append((rows[start][0], rows[i - 1][0], rows[start][1], rows[i - 1][2]))
        start = i
el = [f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
      f'<text x="{L}" y="18" fill="#10161e" font-weight="700">{title}</text>']
for k, (a, b, scene, state) in enumerate(bands):
    fill = '#f3f6f8' if k % 2 == 0 else '#ffffff'
    el.append(f'<rect x="{X(a):.1f}" y="{TOP}" width="{X(b) - X(a):.1f}" height="{2 * PH + GAP}" fill="{fill}"/>')
    el.append(f'<text x="{(X(a) + X(b)) / 2:.1f}" y="{TOP - 16}" text-anchor="middle" fill="#3f4d58">{scene}</text>')
    el.append(f'<text x="{(X(a) + X(b)) / 2:.1f}" y="{TOP - 4}" text-anchor="middle" fill="#005f73" '
              f'font-family="ui-monospace, monospace" font-size="11">{state}</text>')
vmax = max(0.05, max(abs(r[3]) for r in rows))
wmax = max(0.05, max(abs(r[4]) for r in rows))
el += panel(TOP, 3, -vmax, vmax, 'linear.x (m/s)')
el += panel(TOP + PH + GAP, 4, -wmax, wmax, 'angular.z (rad/s)')
for t in range(0, int(T) + 1):
    el.append(f'<text x="{X(t):.1f}" y="{TOP + 2 * PH + GAP + 16}" text-anchor="middle" fill="#6b7785">{t}</text>')
el.append(f'<text x="{(L + W - R) / 2:.0f}" y="{TOP + 2 * PH + GAP + 32}" text-anchor="middle" fill="#6b7785">'
          f'seconds; {len(rows)} /cmd_vel messages from the CI launch test, synthetic scans, no dynamics</text>')
svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" '
       f'font-family="Helvetica, Arial, sans-serif" font-size="12">\n' + '\n'.join(el) + '\n</svg>\n')
open(out, 'w').write(svg)
print(f'{out}: {len(rows)} rows, {len(bands)} scenes')
