"""Build tests/fixtures/ezc3d_ref.json from the public ezc3d test files (https://github.com/pyomeca/ezc3d-testFiles).

Reference values come from ezc3d (pip install ezc3d), an independent C++ C3D implementation.
Usage:  python3 tests/tools/make_ezc3d_ref.py <dir with ezc3d-testFiles> > tests/fixtures/ezc3d_ref.json
Only sampled frames are stored so the fixture stays small; the node test re-reads the C3D files.
"""
import json, os, sys, math
import numpy as np
import ezc3d

SCALE = {'mm': 1e-3, 'cm': 1e-2, 'm': 1.0}


def unit(p, grp, key, default='mm'):
    try:
        v = p[grp][key]['value']
        return (v[0] if len(v) else default).strip().lower()
    except Exception:
        return default


def num(x):
    x = float(x)
    return None if not math.isfinite(x) else round(x, 6)


out = {'generator': 'ezc3d ' + getattr(ezc3d, '__version__', '?'), 'files': {}}
d = sys.argv[1]
for fn in sorted(os.listdir(d)):
    if not fn.lower().endswith('.c3d'):
        continue
    path = os.path.join(d, fn)
    try:
        c = ezc3d.c3d(path, extract_forceplat_data=True)
    except Exception:
        try:
            c = ezc3d.c3d(path)
        except Exception as e:
            out['files'][fn] = {'error': str(e)}
            continue
    p = c['parameters']
    pts = c['data']['points']  # 4 x N x F, file units
    n_pt, n_fr = pts.shape[1], pts.shape[2]
    if n_pt == 0:   # analog only (e.g. force plates): frame count from the header
        h = c['header']['points']; n_fr = int(h['last_frame'] - h['first_frame'] + 1)
    us = SCALE.get(unit(p, 'POINT', 'UNITS'), 1e-3) if n_pt else 1e-3
    frames = sorted(set(int(round(i)) for i in np.linspace(0, n_fr - 1, min(25, n_fr))))
    mk = list(range(min(n_pt, 30)))
    res = c['data']['meta_points']['residuals'] if 'meta_points' in c['data'] and n_pt else None
    P = []
    for f in frames:
        row = []
        for m in mk:
            ok = all(math.isfinite(float(pts[a, m, f])) for a in range(3))
            if res is not None and res[0, m, f] < 0:
                ok = False
            row.append([num(pts[a, m, f] * us) for a in range(3)] if ok else None)
        P.append(row)
    rec = {
        'nFrames': int(n_fr), 'nPoints': int(n_pt), 'rate': num(p['POINT']['RATE']['value'][0]),
        'labels': [s.strip() for s in p['POINT']['LABELS']['value']][:n_pt] if n_pt else [],
        'frames': frames, 'markers': mk, 'points': P, 'plates': []
    }
    an = c['data']['analogs']  # 1 x C x S
    spf = an.shape[2] // n_fr if n_fr else 0
    rec['analogPerFrame'] = int(spf)
    if spf and an.shape[1]:
        rec['analog'] = [[num(an[0, ch, f * spf]) for ch in range(min(an.shape[1], 16))] for f in frames]
    for k, pl in enumerate(c['data'].get('platform', [])):
        fu = SCALE.get(str(pl.get('unit_position', 'mm')).strip().lower(), 1e-3)
        mu = SCALE.get(str(pl.get('unit_moment', 'Nmm')).strip().lower().replace('n', '', 1), 1e-3)
        if not (p['POINT'].get('UNITS', {}).get('value') or []):   # empty POINT:UNITS: ezc3d labels 'm' but the plate geometry is in mm
            fu, mu = 1e-3, 1e-3
        F, CoP, Tz = pl['force'], pl['center_of_pressure'], pl['Tz']
        idx = [f * spf for f in frames]
        rec['plates'].append({
            'type': int(p['FORCE_PLATFORM']['TYPE']['value'][k]),
            'force': [[num(F[a, i]) for a in range(3)] for i in idx],
            'cop': [[num(CoP[a, i] * fu) for a in range(3)] for i in idx],
            'tz': [[num(Tz[a, i] * mu) for a in range(3)] for i in idx],
        })
    out['files'][fn] = rec
json.dump(out, sys.stdout, separators=(',', ':'))
