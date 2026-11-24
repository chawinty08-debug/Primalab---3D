"""Build msk_data.js: bone meshes, landmarks and muscle paths for the musculoskeletal overlay.

Sources (OpenSim models repository, https://github.com/opensim-org/opensim-models), both CC BY 3.0:
  - Gait 2392 (Delp et al. 1990; Anderson & Pandy 1999; Thelen 2003): pelvis, legs, torso (HAT), 92 muscle paths
  - Arm 26 (Holzbaur et al. 2005, adapted by the OpenSim team): humerus, ulna, radius, hand, scapula, clavicle
Meshes are decimated (fast-simplification) and stored in the model's neutral pose (all coordinates 0) in metres.

Usage: python3 tools/make_msk.py <opensim-models checkout> > msk_data.js
Needs: numpy, fast-simplification (pip install fast-simplification)
"""
import sys, os, json, base64, math
import xml.etree.ElementTree as ET
import numpy as np
import fast_simplification

ROOT = sys.argv[1]
GEO = os.path.join(ROOT, 'Geometry')


def vec(s):
    return np.array([float(x) for x in s.split()])


def read_vtp(path):
    r = ET.parse(path).getroot()
    pts = None; conn = None; offs = None
    for pc in r.iter('Piece'):
        p = pc.find('Points/DataArray')
        pts = np.array(p.text.split(), float).reshape(-1, 3)
        po = pc.find('Polys')
        for da in po.findall('DataArray'):
            if da.get('Name') == 'connectivity': conn = np.array(da.text.split(), int)
            if da.get('Name') == 'offsets': offs = np.array(da.text.split(), int)
        break
    tris = []; start = 0
    for o in offs:
        poly = conn[start:o]; start = o
        for k in range(1, len(poly) - 1): tris.append((poly[0], poly[k], poly[k + 1]))
    return pts, np.array(tris, int)


def weld(v, t):
    key, inv = np.unique(np.round(v, 6), axis=0, return_inverse=True)
    t = inv.reshape(-1)[t]
    t = t[(t[:, 0] != t[:, 1]) & (t[:, 1] != t[:, 2]) & (t[:, 0] != t[:, 2])]
    return key, t


def decimate(v, t, target):
    v, t = weld(v, t)
    if len(t) <= target: return v, t
    v2, t2 = fast_simplification.simplify(v.astype(np.float32), t.astype(np.int32), target_reduction=1 - target / len(t))
    return np.asarray(v2, float), np.asarray(t2, int)


def b64(a): return base64.b64encode(a.tobytes()).decode()


def natural_spline(xs, ys):
    xs = np.asarray(xs, float); ys = np.asarray(ys, float); n = len(xs)
    if n < 3: return lambda x: float(np.interp(x, xs, ys))
    h = np.diff(xs); A = np.zeros((n, n)); r = np.zeros(n); A[0, 0] = A[-1, -1] = 1
    for i in range(1, n - 1):
        A[i, i - 1] = h[i - 1]; A[i, i] = 2 * (h[i - 1] + h[i]); A[i, i + 1] = h[i]
        r[i] = 3 * ((ys[i + 1] - ys[i]) / h[i] - (ys[i] - ys[i - 1]) / h[i - 1])
    c = np.linalg.solve(A, r)
    def f(x):
        i = int(np.clip(np.searchsorted(xs, x) - 1, 0, n - 2)); dx = x - xs[i]
        b = (ys[i + 1] - ys[i]) / h[i] - h[i] * (2 * c[i] + c[i + 1]) / 3; d = (c[i + 1] - c[i]) / (3 * h[i])
        return ys[i] + b * dx + c[i] * dx * dx + d * dx ** 3
    return f


def func_value(fn_el, x=0.0):
    """value of an OpenSim function element at x (Constant, LinearFunction, SimmSpline/NaturalCubicSpline)"""
    for e in fn_el.iter():
        if e.tag == 'Constant': return float(e.findtext('value'))
        if e.tag == 'LinearFunction': a, b = vec(e.findtext('coefficients')); return a * x + b
        if e.tag in ('SimmSpline', 'NaturalCubicSpline'): return natural_spline(vec(e.findtext('x')), vec(e.findtext('y')))(x)
    return 0.0


def spline_of(el):
    for e in el.iter():
        if e.tag in ('SimmSpline', 'NaturalCubicSpline'): return {'x': [round(v, 7) for v in vec(e.findtext('x'))], 'y': [round(v, 7) for v in vec(e.findtext('y'))]}
        if e.tag == 'Constant': return {'c': float(e.findtext('value'))}
        if e.tag == 'LinearFunction': a, b = vec(e.findtext('coefficients')); return {'a': a, 'b': b}
    return {'c': 0.0}


def body_world(model):
    """neutral-pose world translation of each body (models used here have zero offset-frame orientations)"""
    m = model.find('Model'); W = {'ground': np.zeros(3)}
    joints = list(m.find('JointSet/objects'))
    for _ in range(len(joints) + 2):
        for j in joints:
            fr = {f.get('name'): f for f in j.iter('PhysicalOffsetFrame')}
            par = j.findtext('socket_parent_frame'); chi = j.findtext('socket_child_frame')
            pf = fr.get(par); cf = fr.get(chi)
            pbody = pf.findtext('socket_parent').split('/')[-1]; cbody = cf.findtext('socket_parent').split('/')[-1]
            for f in (pf, cf): assert np.allclose(vec(f.findtext('orientation')), 0), 'non-zero offset orientation'
            if pbody not in W or cbody in W: continue
            tr = np.zeros(3)
            st = j.find('SpatialTransform')
            if st is not None:
                for ax in st.findall('TransformAxis'):
                    if ax.get('name').startswith('translation'):
                        tr += vec(ax.findtext('axis')) * func_value(ax, 0.0)
            W[cbody] = W[pbody] + vec(pf.findtext('translation')) + tr - vec(cf.findtext('translation'))
    return W


gait = ET.parse(os.path.join(ROOT, 'Models/Gait2392_Simbody/gait2392_thelen2003muscle.osim')).getroot()
arm = ET.parse(os.path.join(ROOT, 'Models/Arm26/arm26.osim')).getroot()
GW = body_world(gait); AW = body_world(arm)
# model markers (gait2392 Scale marker set, arm26 markers)
mk = {}
for m in ET.parse(os.path.join(ROOT, 'Pipelines/Gait2392_Simbody/gait2392_Scale_MarkerSet.xml')).getroot().iter('Marker'):
    b = (m.findtext('socket_parent_frame') or m.findtext('body')).strip().split('/')[-1]
    mk[m.get('name')] = GW[b] + vec(m.findtext('location'))
for m in arm.iter('Marker'):
    b = m.findtext('socket_parent_frame').split('/')[-1]
    mk['arm.' + m.get('name')] = AW[b] + vec(m.findtext('location'))

SEG_OF = {'pelvis': 'PV', 'femur_r': 'RTH', 'tibia_r': 'RSK', 'talus_r': 'RFT', 'calcn_r': 'RFT', 'toes_r': 'RFT',
          'femur_l': 'LTH', 'tibia_l': 'LSK', 'talus_l': 'LFT', 'calcn_l': 'LFT', 'toes_l': 'LFT', 'torso': 'TX'}
TARGET = {'pelvis.vtp': 240, 'l_pelvis.vtp': 240, 'sacrum.vtp': 160, 'femur_r.vtp': 600, 'femur_l.vtp': 600, 'tibia_r.vtp': 286, 'tibia_l.vtp': 286,
          'fibula.vtp': 200, 'l_fibula.vtp': 200, 'talus.vtp': 220, 'l_talus.vtp': 220, 'foot.vtp': 600, 'l_foot.vtp': 600, 'bofoot.vtp': 280, 'l_bofoot.vtp': 280,
          'hat_spine.vtp': 900, 'hat_ribs.vtp': 1000, 'hat_skull.vtp': 600, 'hat_jaw.vtp': 200,
          'arm_r_humerus.vtp': 500, 'arm_r_ulna.vtp': 320, 'arm_r_radius.vtp': 320, 'ground_r_scapula.vtp': 350, 'ground_r_clavicle.vtp': 120}
meshes = []


def add_mesh(name, v, t, seg, place=None, mirror=False):
    v = v.copy()
    if mirror:
        v[:, 2] *= -1; t = t[:, ::-1]
    s = max(np.abs(v).max(), 1e-6) / 32000.0
    q = np.round(v / s).astype(np.int16)
    meshes.append({'name': name, 'seg': seg, 'place': place or seg, 's': s, 'nV': len(v), 'nT': len(t), 'v': b64(q), 't': b64(t.astype(np.uint16))})


# --- gait2392 bodies
for b in gait.find('Model/BodySet/objects'):
    bn = b.get('name')
    for g in b.iter('Mesh'):
        f = g.findtext('mesh_file')
        v, t = read_vtp(os.path.join(GEO, f)); v, t = decimate(v, t, TARGET.get(f, 400))
        seg = SEG_OF[bn]
        if f in ('hat_skull.vtp', 'hat_jaw.vtp'): seg = 'HD'
        add_mesh(f[:-4], v + GW[bn], t, seg, place='TX' if seg == 'HD' else None)
# thorax: sternal notch / xiphoid / C7 / T10 from the HAT geometry (midline extremes)
rv, _ = read_vtp(os.path.join(GEO, 'hat_ribs.vtp')); rv = rv + GW['torso']
sv, _ = read_vtp(os.path.join(GEO, 'hat_spine.vtp')); sv = sv + GW['torso']
acr_y = mk['R.Acromium'][1]
mlr = rv[np.abs(rv[:, 2]) < 0.012]; ant = mlr[mlr[:, 0] > mlr[:, 0].max() - 0.035]
IJ = ant[ant[:, 1].argmax()]; PX = ant[ant[:, 1].argmin()]
mls = sv[np.abs(sv[:, 2]) < 0.01]
band = mls[np.abs(mls[:, 1] - (acr_y - 0.03)) < 0.03]; C7 = band[band[:, 0].argmin()]
band = mls[np.abs(mls[:, 1] - PX[1]) < 0.03]; T10 = band[band[:, 0].argmin()]

# arm26 shoulder girdle on the gait2392 thorax: medial end of the clavicle at the sternal notch (the two models differ;
# matching the acromion marker instead would float the clavicle 10 cm above the rib cage)
cv0, _ = read_vtp(os.path.join(GEO, 'ground_r_clavicle.vtp')); cv0 = cv0 + AW['base']
cmed = cv0[cv0[:, 2].argmin()]
acr_shift = np.array([IJ[0] - cmed[0], IJ[1] - cmed[1], 0.0])
ACR = mk['arm.r_acromion'] + acr_shift
LM_TX = {'CLAV': IJ, 'STRN': PX, 'C7': C7, 'T10': T10, 'RSHO': ACR, 'LSHO': ACR * np.array([1, 1, -1])}
# --- arm26: humerus, forearm (ulna/radius), hand; scapula + clavicle on the thorax (see above)
arm_bodies = {b.get('name'): b for b in arm.find('Model/BodySet/objects')}
HAND = lambda f: not (f.endswith('ulna.vtp') or f.endswith('radius.vtp'))
for side, mir in (('R', False), ('L', True)):
    for bn, seg in (('r_humerus', side + 'UA'), ('r_ulna_radius_hand', side + 'FA'), ('base', 'TX')):
        for g in arm_bodies[bn].iter('Mesh'):
            f = g.findtext('mesh_file')
            if bn == 'base' and f not in ('ground_r_scapula.vtp', 'ground_r_clavicle.vtp'): continue
            v, t = read_vtp(os.path.join(GEO, f)); v, t = decimate(v, t, TARGET.get(f, 40))
            w = v + AW[bn]
            sg = seg; place = None
            if bn == 'base': w = w + acr_shift; place = 'TX'
            elif bn == 'r_ulna_radius_hand' and HAND(f): sg = side + 'HA'; place = side + 'FA'
            add_mesh(side.lower() + '_' + f[:-4].replace('arm_r_', '').replace('ground_r_', ''), w, t, sg, place, mir)


# --- landmarks in the neutral pose (model side), matched to the app's static landmarks
def mid(a, b): return (np.asarray(a) + np.asarray(b)) / 2


LM = {}
hip = {s: GW['femur_' + s.lower()] for s in 'RL'}
LM['TX'] = LM_TX
LM['PV'] = {'RASI': mk['R.ASIS'], 'LASI': mk['L.ASIS'], 'SACR': mk['V.Sacral'], 'RHJC': hip['R'], 'LHJC': hip['L']}
for s in 'RL':
    S = s + '.'; kjc = mid(mk[S + 'Knee.Lat'], mk[S + 'Knee.Med']); ajc = mid(mk[S + 'Ankle.Lat'], mk[S + 'Ankle.Med'])
    toe = GW['calcn_' + s.lower()] + np.array([0.19, 0.004, -0.01 if s == 'R' else 0.01])   # 2nd metatarsal head, dorsal
    LM[s + 'TH'] = {s + 'HJC': hip[s], s + 'KJC': kjc, s + 'KNE': mk[S + 'Knee.Lat'], s + 'KNM': mk[S + 'Knee.Med']}
    LM[s + 'SK'] = {s + 'KJC': kjc, s + 'AJC': ajc, s + 'ANK': mk[S + 'Ankle.Lat'], s + 'ANM': mk[S + 'Ankle.Med']}
    LM[s + 'FT'] = {s + 'AJC': ajc, s + 'HEE': mk[S + 'Heel'], s + 'TOE': toe, s + 'ANK': mk[S + 'Ankle.Lat']}
# arms (arm26 neutral pose, forearm supinated): GH, elbow centre, epicondyles, styloids
uv, _ = read_vtp(os.path.join(GEO, 'arm_r_ulna.vtp')); uv = uv + AW['r_ulna_radius_hand']
ejc = AW['r_ulna_radius_hand']; rs = mk['arm.r_radius_styloid']
axis = (rs - ejc) / np.linalg.norm(rs - ejc)
dist = uv[(uv - ejc) @ axis > ((uv - ejc) @ axis).max() - 0.012]; us_ = dist[dist[:, 2].argmin()]   # medial-distal ulna = ulnar styloid
lat = mk['arm.r_humerus_epicondyle']; med = 2 * ejc - lat; med[0] = lat[0]
for s, mir in (('R', 1), ('L', -1)):
    f = lambda p: np.array([p[0], p[1], p[2] * mir])
    LM[s + 'UA'] = {s + 'SJC': f(AW['r_humerus']), s + 'EJC': f(ejc), s + 'ELB': f(lat), s + 'ELBM': f(med)}
    LM[s + 'FA'] = {s + 'EJC': f(ejc), s + 'WRA': f(rs), s + 'WRB': f(us_)}

# --- muscles (gait2392)
COORD = {'knee_angle_r': 'RKnee', 'knee_angle_l': 'LKnee', 'hip_flexion_r': 'RHip', 'hip_flexion_l': 'LHip'}
muscles = []
for x in gait.find('Model/ForceSet/objects'):
    name = x.get('name'); pts = []
    for pp in x.find('GeometryPath/PathPointSet/objects'):
        body = pp.findtext('socket_parent_frame').split('/')[-1]
        e = {'seg': SEG_OF[body], 'o': [round(v, 6) for v in GW[body]]}
        if pp.tag == 'MovingPathPoint':
            cz = pp.findtext('socket_x_coordinate').split('/')[-1]
            e['mov'] = {'q': COORD[cz], 'x': spline_of(pp.find('x_location')), 'y': spline_of(pp.find('y_location')), 'z': spline_of(pp.find('z_location'))}
        else:
            e['p'] = [round(v, 6) for v in vec(pp.findtext('location'))]
            if pp.tag == 'ConditionalPathPoint':
                rng = vec(pp.findtext('range')); e['cond'] = {'q': COORD[pp.findtext('socket_coordinate').split('/')[-1]], 'lo': rng[0], 'hi': rng[1]}
        pts.append(e)
    col = vec(x.findtext('GeometryPath/Appearance/color') or '0.8 0.1 0.1')
    muscles.append({'name': name, 'F': float(x.findtext('max_isometric_force')), 'lopt': float(x.findtext('optimal_fiber_length')),
                    'lts': float(x.findtext('tendon_slack_length')), 'pts': pts})

out = {
    'credits': 'Bone geometry and muscle paths: OpenSim Gait 2392 (Delp et al. 1990; Anderson & Pandy 1999; Thelen 2003) and Arm 26 '
               '(Holzbaur et al. 2005; OpenSim team), opensim-org/opensim-models, CC BY 3.0. Meshes decimated; arm meshes mirrored for the left side.',
    'meshes': meshes,
    'landmarks': {k: {n: [round(float(c), 6) for c in p] for n, p in v.items()} for k, v in LM.items()},
    'muscles': muscles,
    'coordSign': {'RKnee': -1, 'LKnee': -1, 'RHip': 1, 'LHip': 1},
}
sys.stdout.write('/* generated by tools/make_msk.py — do not edit */\nconst MSK_DATA = ' + json.dumps(out, separators=(',', ':')) + ';\n')
sys.stderr.write('meshes %d, triangles %d, muscles %d\n' % (len(meshes), sum(m['nT'] for m in meshes), len(muscles)))
