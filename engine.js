/* =====================================================================
   Mocap Viewer Engine  v0.1
   - C3D reader (Intel/PC, float or int16 points, analog, force platform type 2/4, events)
   - Dataset contract (frame source independent: C3D / NatNet / Rokoko can fill the same shape)
   - Orbit camera + Canvas2D renderer (markers, links, trails, grid, plates, GRF, gizmo)
   - ChartView (time series with synced cursor)
   Lab convention: Z-up, metres. No external dependencies.
   ===================================================================== */
(function (root) {
'use strict';
const D2R = Math.PI / 180;
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

/* ---------------------------------------------------------------- Dataset contract
   {
     name, synthetic, rate, nFrames, firstFrame,
     labels[], base[], side[] ('L'|'R'|'C'|'U'), prefix, nM,
     pts: Float32Array(nFrames*nM*3)   metres, lab frame
     vis: Uint8Array(nFrames*nM)       1 = valid
     bounds: {min:[x,y,z], max:[x,y,z]}
     plates: [{ index, type, supported, note, corners:[[x,y,z]x4] (m), F:Float32Array(n*3) N,
                cop:Float32Array(n*3) m, T:Float32Array(n*3) free moment N·m (optional), on:Uint8Array(n), fzMax, flipped }]
     events: [{frame, label, context}]
     info: { unitsLabel, analogRate, analogChannels }
   }
------------------------------------------------------------------ */

/* ---------------------------------------------------------------- naming helpers */
function splitLabel(label) {
  const k = label.lastIndexOf(':');
  return k >= 0 ? { prefix: label.slice(0, k + 1), base: label.slice(k + 1) } : { prefix: '', base: label };
}
function isUnlabeled(base) { return /^(\*|unlabeled|unlabelled)/i.test(base); }
function sideOf(base) {
  if (isUnlabeled(base)) return 'U';
  const b = base.toUpperCase();
  if (/^L[A-Z]{2,}/.test(b) && !/^(LOW|LAT)/.test(b)) return 'L';
  if (/^R[A-Z]{2,}/.test(b) && !/^(ROOT|REF)/.test(b)) return 'R';
  if (/^(LEFT|L_|L\.)/i.test(base)) return 'L';
  if (/^(RIGHT|R_|R\.)/i.test(base)) return 'R';
  return 'C';
}
function finishDataset(ds, aliases) {
  const nM = ds.labels.length;
  ds.nM = nM;
  ds.base = []; ds.side = []; ds.baseFile = [];
  let prefix = null;
  for (const l of ds.labels) {
    const s = splitLabel(l);
    ds.baseFile.push(s.base);
    ds.base.push(s.base);
    if (s.prefix && !isUnlabeled(s.base)) prefix = prefix === null ? s.prefix : (prefix === s.prefix ? prefix : '(หลาย prefix)');
  }
  // marker-name mapping {file name: model name}; a model name already present in the file is never overwritten
  ds.mapped = {};
  if (aliases === undefined) aliases = ds.aliasMap;
  if (aliases && typeof aliases === 'object') {
    const A = {}; for (const k in aliases) if (aliases[k]) A[k.toUpperCase()] = String(aliases[k]).toUpperCase();
    const have = new Set(ds.base.map(b => b.toUpperCase()));
    ds.base.forEach((b, i) => {
      const c = A[b.toUpperCase()];
      if (c && c !== b.toUpperCase() && !have.has(c)) { ds.base[i] = c; have.add(c); ds.mapped[c] = ds.baseFile[i]; }
    });
    ds.aliasMap = aliases;
  }
  for (const b of ds.base) ds.side.push(sideOf(b));
  ds.prefix = prefix || '';
  // bounds + first-frame centroid
  const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  const step = Math.max(1, Math.floor(ds.nFrames / 400));
  for (let f = 0; f < ds.nFrames; f += step)
    for (let i = 0; i < nM; i++) {
      if (!ds.vis[f * nM + i] || ds.side[i] === 'U') continue;
      for (let a = 0; a < 3; a++) {
        const v = ds.pts[(f * nM + i) * 3 + a];
        if (v < mn[a]) mn[a] = v; if (v > mx[a]) mx[a] = v;
      }
    }
  if (!isFinite(mn[0])) { mn[0] = mn[1] = -1; mn[2] = 0; mx[0] = mx[1] = 1; mx[2] = 2; }
  ds.bounds = { min: mn, max: mx };
  ds.plates = ds.plates || []; ds.events = ds.events || [];
  return ds;
}
function validRatio(ds, i) {
  let c = 0; for (let f = 0; f < ds.nFrames; f++) c += ds.vis[f * ds.nM + i];
  return c / Math.max(1, ds.nFrames);
}

/* Default links for common marker names (Conventional lower / Plug-in Gait style).
   Only pairs where both markers exist are used. */
function defaultLinks(base) {
  const idx = {}; base.forEach((b, i) => { idx[b.toUpperCase()] = i; });
  const pairs = [['LASI', 'RASI'], ['RASI', 'RPSI'], ['RPSI', 'LPSI'], ['LPSI', 'LASI']];
  for (const S of ['L', 'R']) {
    pairs.push([S + 'ASI', S + 'THI'], [S + 'THI', S + 'KNE'], [S + 'KNE', S + 'TIB'], [S + 'TIB', S + 'ANK'],
      [S + 'ANK', S + 'HEE'], [S + 'ANK', S + 'TOE'], [S + 'HEE', S + 'TOE'], [S + 'KNE', S + 'KNEM'], [S + 'ANK', S + 'ANKM'], [S + 'KNEM', S + 'ANKM'],
      ...(('RTIB' in idx || 'LTIB' in idx) ? [] : [[S + 'KNE', S + 'ANK']]),
      [S + 'SHO', S + 'ELB'], [S + 'ELB', S + 'WRA'], [S + 'WRA', S + 'WRB'], [S + 'WRA', S + 'FIN'], [S + 'WRB', S + 'FIN'],
      ['C7', S + 'SHO']);
  }
  pairs.push(['LSHO', 'RSHO'], ['C7', 'T10'], ['CLAV', 'STRN'], ['C7', 'CLAV'], ['LFHD', 'RFHD'], ['RFHD', 'RBHD'], ['RBHD', 'LBHD'], ['LBHD', 'LFHD']);
  const out = [];
  for (const [a, b] of pairs) if (a in idx && b in idx) out.push([idx[a], idx[b]]);
  return out;
}


/* ---------------------------------------------------------------- C3D reader */
function ascii(u8, p, n) { let s = ''; for (let i = 0; i < n; i++) s += String.fromCharCode(u8[p + i]); return s; }

/* ---------------------------------------------------------------- vertical-axis detection */
/* name cues for the vertical axis; labels are cleaned (prefix after ':' or '-' dropped, spaces removed, upper case) */
const UP_CUES_HI = /(FHD|BHD|HEAD|C7|CLAV|STRN|STER|SHO|ACR)/;
const UP_CUES_MID = /(ASI|PSI|SACR)/;
const UP_CUES_LO = /(TOE|(^|[^C])HEE|ANK|MALL|MAL$|MET|MT\d|MTH|CAL$)/;
const upCueName = l => { const b = String(l).toUpperCase(); const t = b.split(/[:\-]/).pop() || b; return t.replace(/[\s_.]/g, ''); };
/* corner sanity: finite, inside ±50 m, edges 5 cm – 5 m (some exporters leave unused plates with garbage) */
function cornersOK(C) {
  if (!C || C.length !== 4) return false;
  const e = j => Math.hypot(...[0, 1, 2].map(i => C[j][i] - C[(j + 1) % 4][i]));
  return !C.some(c => c.some(x => !Number.isFinite(x) || Math.abs(x) > 50)) && [0, 1, 2, 3].every(j => e(j) > 0.05 && e(j) < 5);
}
function upRotation(up) {
  // rotation R (rows = new X, Y, Z in old coordinates) taking the file's `up` axis to +Z, det = +1
  const a = 'XYZ'.indexOf(up[1]), s = up[0] === '-' ? -1 : 1;
  const ez = [0, 0, 0]; ez[a] = s;
  const b = a === 0 ? 1 : 0;           // keep the first other axis as new X
  const ex = [0, 0, 0]; ex[b] = 1;
  const ey = [ez[1] * ex[2] - ez[2] * ex[1], ez[2] * ex[0] - ez[0] * ex[2], ez[0] * ex[1] - ez[1] * ex[0]];
  if (up === '+Z') return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  return [ex, ey, ez];
}
function detectUpAxis(pts, vis, nfr, npts, labels, cornersList, yScreen, zScreen) {
  const name = a => (a >= 0 ? '+' : '-') + 'XYZ'[Math.abs(a) - 1];
  const tag = (ax, sg) => (sg >= 0 ? '+' : '-') + 'XYZ'[ax];
  const step = Math.max(1, Math.floor(nfr / 200));
  const meanAxis = idx => { // mean of visible coordinates of the given markers
    const m = [0, 0, 0]; let c = 0;
    for (let f = 0; f < nfr; f += step) for (const i of idx) { const k = f * npts + i; if (!vis[k]) continue; for (let a = 0; a < 3; a++) m[a] += pts[k * 3 + a]; c++; }
    return c ? m.map(v => v / c) : null;
  };
  const allIdx = []; for (let i = 0; i < npts; i++) allIdx.push(i);
  const ev = [];
  // 1) plate normals
  if (cornersList.length) {
    const n = [0, 0, 0], cen = [0, 0, 0]; let nv = 0;
    for (const C of cornersList) {
      const e1 = [0, 1, 2].map(i => C[0][i] - C[1][i]), e2 = [0, 1, 2].map(i => C[0][i] - C[3][i]);
      const cr = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
      const l = Math.hypot(...cr); if (!(l > 1e-6)) continue; nv++;
      for (let a = 0; a < 3; a++) { n[a] += Math.abs(cr[a] / l); cen[a] += (C[0][a] + C[1][a] + C[2][a] + C[3][a]) / 4; }
    }
    for (let a = 0; a < 3; a++) cen[a] /= Math.max(1, nv);
    const ax = n.indexOf(Math.max(...n));
    if (nv && n[ax] > 0.8 * nv) {
      const m = meanAxis(allIdx);
      const sg = m ? (m[ax] >= cen[ax] ? 1 : -1) : 1;
      ev.push({ up: tag(ax, sg), src: 'force plate' });
    }
  }
  // 2) marker-name cues
  const hi = [], mid = [], lo = [];
  labels.forEach((l, i) => { const b = upCueName(l); if (UP_CUES_HI.test(b)) hi.push(i); else if (UP_CUES_MID.test(b)) mid.push(i); else if (UP_CUES_LO.test(b)) lo.push(i); });
  for (const [A, B, minD] of [[hi, lo, 0.4], [mid, lo, 0.25], [hi, mid, 0.2]]) {   // head/trunk above pelvis above feet
    if (!A.length || !B.length) continue;
    const mh = meanAxis(A), ml = meanAxis(B); if (!mh || !ml) continue;
    const d = [0, 1, 2].map(a => mh[a] - ml[a]), ad = d.map(Math.abs), ax = ad.indexOf(Math.max(...ad));
    const l = Math.hypot(...d);
    if (l > minD && ad[ax] / l > 0.7) { ev.push({ up: tag(ax, d[ax]), src: 'ชื่อ marker' }); break; }
  }
  // 3) POINT:Y_SCREEN (screen up axis written by the capture software)
  const ys = (yScreen || '').toUpperCase().replace(/\s/g, '');
  if (/^[+-]?[XYZ]$/.test(ys)) ev.push({ up: (ys[0] === '-' ? '-' : '+') + ys.slice(-1), src: 'POINT:Y_SCREEN', weak: true });
  // 4) largest per-frame body extent (standing / moving person is taller than wide)
  {
    const ext = [0, 0, 0], med = [[], [], []]; let c = 0;
    for (let f = 0; f < nfr; f += step) {
      const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity]; let nv = 0;
      for (let i = 0; i < npts; i++) { const k = f * npts + i; if (!vis[k]) continue; nv++; for (let a = 0; a < 3; a++) { const v = pts[k * 3 + a]; if (v < mn[a]) mn[a] = v; if (v > mx[a]) mx[a] = v; } }
      if (nv >= 4) { for (let a = 0; a < 3; a++) { ext[a] += mx[a] - mn[a]; med[a].push(mn[a]); } c++; }
    }
    const srt = ext.slice().sort((x, y) => y - x);
    if (c && srt[0] > 1.5 * srt[1]) {
      const ax = ext.indexOf(srt[0]);
      const lowest = med[ax].sort((x, y) => x - y)[med[ax].length >> 1];
      ev.push({ up: tag(ax, lowest >= -0.05 ? 1 : -1), src: 'ช่วงกว้างของ marker', weak: true });
    }
  }
  const strong = ev.filter(e => !e.weak);
  const pick = strong[0] || ev.find(e => e.src === 'POINT:Y_SCREEN') || ev[0] || { up: '+Z', src: 'ไม่มีหลักฐาน ใช้ค่าเริ่มต้น' };
  const other = strong.find(e => e.up !== pick.up);
  return { detected: pick.up, source: pick.src, evidence: ev, conflict: other ? pick.src + ' = ' + pick.up + ' แต่ ' + other.src + ' = ' + other.up : '', name };
}

function readC3D(buf, name, opts) {
  const dv = new DataView(buf), u8 = new Uint8Array(buf), N = u8.length;
  if (N < 1024) throw new Error('ไฟล์เล็กเกินไปสำหรับ C3D');
  const pblock = u8[0];
  if (u8[1] !== 80) throw new Error('ไม่ใช่ไฟล์ C3D (byte ที่ 2 ของ header ต้องเป็น 80)');
  const pbase = (pblock - 1) * 512;
  const ptype = u8[pbase + 3];
  // processor type: 84 Intel (little-endian IEEE), 85 DEC (little-endian ints, VAX-F floats), 86 MIPS/SGI (big-endian IEEE)
  if (ptype !== 84 && ptype !== 85 && ptype !== 86) throw new Error('processor type ' + ptype + ' ไม่รู้จัก (รองรับ 84 Intel, 85 DEC, 86 MIPS)');
  const LE = ptype !== 86;
  const u16 = o => dv.getUint16(o, LE), i16 = o => dv.getInt16(o, LE);
  const tmp4 = new DataView(new ArrayBuffer(4));
  const f32 = ptype === 85
    ? o => { // VAX F-float: swap 16-bit words, read as IEEE, divide by 4 (exponent bias differs by 2)
      tmp4.setUint8(0, u8[o + 2]); tmp4.setUint8(1, u8[o + 3]); tmp4.setUint8(2, u8[o]); tmp4.setUint8(3, u8[o + 1]);
      return tmp4.getFloat32(0, true) / 4;
    }
    : o => dv.getFloat32(o, LE);

  const hdr = {
    first: u16(6), last: u16(8), scale: f32(12),
    dstart: u16(16), spf: u16(18), rate: f32(20)
  };

  /* parameters */
  const groupsById = {}, raw = [];
  let p = pbase + 4;
  while (p + 4 < N) {
    const nchar = dv.getInt8(p), gid = dv.getInt8(p + 1);
    if (nchar === 0) break;
    const nlen = Math.abs(nchar);
    const pname = ascii(u8, p + 2, nlen).trim().toUpperCase();
    const offPos = p + 2 + nlen;
    const off = i16(offPos);
    const q = offPos + 2;
    if (gid < 0) groupsById[-gid] = pname;
    else {
      const t = dv.getInt8(q), nd = u8[q + 1];
      const dims = []; for (let i = 0; i < nd; i++) dims.push(u8[q + 2 + i]);
      const dpos = q + 2 + nd;
      const n = nd ? dims.reduce((a, b) => a * b, 1) : 1;
      let val = null;
      if (t === -1) val = u8.slice(dpos, dpos + n);
      else if (t === 1) { val = new Int8Array(n); for (let i = 0; i < n; i++) val[i] = dv.getInt8(dpos + i); }
      else if (t === 2) { val = new Int16Array(n); for (let i = 0; i < n; i++) val[i] = i16(dpos + 2 * i); }
      else if (t === 4) { val = new Float32Array(n); for (let i = 0; i < n; i++) val[i] = f32(dpos + 4 * i); }
      raw.push({ gid, pname, t, dims, val });
    }
    if (off === 0) break;
    p = offPos + off;
  }
  const P = {};
  for (const r of raw) P[(groupsById[r.gid] || String(r.gid)) + ':' + r.pname] = r;
  const num = (k, d) => { const r = P[k]; return r && r.val && r.val.length ? r.val[0] : d; };
  const arr = (k) => { const r = P[k]; return r && r.val && r.t !== -1 ? r.val : null; };
  const strs = (k) => {
    const r = P[k]; if (!r || r.t !== -1) return [];
    const w = r.dims[0], cnt = r.dims.length > 1 ? r.dims.slice(1).reduce((a, b) => a * b, 1) : 1;
    const out = []; for (let i = 0; i < cnt; i++) out.push(ascii(r.val, i * w, w).replace(/\0/g, '').trim());
    return out;
  };

  const npts = num('POINT:USED', 0);
  if (!npts) throw new Error('ไม่พบ POINT:USED ในไฟล์ (ไม่มีข้อมูล marker)');
  const pscale = num('POINT:SCALE', hdr.scale);
  const rate = num('POINT:RATE', hdr.rate) || hdr.rate;
  const dstart = num('POINT:DATA_START', hdr.dstart) || hdr.dstart;
  const framesParam = num('POINT:FRAMES', 0) & 0xFFFF;
  let nfr = framesParam > 0 && framesParam !== 0xFFFF ? framesParam : Math.max(0, hdr.last - hdr.first + 1);
  const floatData = pscale < 0;
  const esz = floatData ? 4 : 2;
  const na = num('ANALOG:USED', 0) | 0;
  const arate = num('ANALOG:RATE', 0);
  const spf = na && rate ? Math.max(1, Math.round(arate / rate)) : 0;
  const fsize = npts * 4 * esz + na * spf * esz;
  const base = (dstart - 1) * 512;
  const avail = Math.floor((N - base) / fsize);
  const warnings = [];
  if (avail < nfr) { warnings.push('ไฟล์สั้นกว่าจำนวนเฟรมที่ header ระบุ (' + avail + ' จาก ' + nfr + ')'); nfr = avail; }
  if (nfr <= 0) throw new Error('อ่านจำนวนเฟรมไม่ได้');

  let labels = strs('POINT:LABELS');
  for (let k = 2; k <= 9 && labels.length < npts; k++) labels = labels.concat(strs('POINT:LABELS' + k));
  labels = labels.slice(0, npts);
  while (labels.length < npts) labels.push('M' + (labels.length + 1));

  const unitsStr = (strs('POINT:UNITS')[0] || '').toLowerCase();
  let us = unitsStr === 'mm' ? 0.001 : unitsStr === 'cm' ? 0.01 : unitsStr === 'm' ? 1 : null;

  /* point data (file units first, convert after units are known) */
  const pts = new Float32Array(nfr * npts * 3), vis = new Uint8Array(nfr * npts);
  const resid = new Float32Array(nfr * npts);
  for (let f = 0; f < nfr; f++) {
    let o = base + f * fsize;
    for (let i = 0; i < npts; i++) {
      let x, y, z, r;
      if (floatData) { x = f32(o); y = f32(o + 4); z = f32(o + 8); r = f32(o + 12); o += 16; }
      else { x = i16(o) * pscale; y = i16(o + 2) * pscale; z = i16(o + 4) * pscale; r = i16(o + 6); o += 8; }
      const k = f * npts + i;
      const ok = isFinite(x) && isFinite(y) && isFinite(z) && !(x === 0 && y === 0 && z === 0) && r >= 0;
      vis[k] = ok ? 1 : 0; resid[k] = r;
      pts[k * 3] = ok ? x : 0; pts[k * 3 + 1] = ok ? y : 0; pts[k * 3 + 2] = ok ? z : 0;
    }
  }
  if (us === null) { // guess from magnitude
    const m = []; for (let k = 0; k < pts.length && m.length < 3000; k += 97) if (pts[k]) m.push(Math.abs(pts[k]));
    m.sort((a, b) => a - b); const med = m.length ? m[m.length >> 1] : 0;
    us = med > 20 ? 0.001 : 1; warnings.push('ไม่พบ POINT:UNITS เดาหน่วยเป็น ' + (us === 1 ? 'เมตร' : 'มิลลิเมตร'));
  }
  if (us !== 1) for (let k = 0; k < pts.length; k++) pts[k] *= us;

  /* vertical axis: the analysis code works in a Z-up lab, so files recorded Y-up (or with any other
     vertical axis) are rotated rigidly at load. Evidence, strongest first: force-plate normal,
     marker-name cues (head/trunk above feet), POINT:Y_SCREEN, the axis of largest body extent. */
  const fpN = num('FORCE_PLATFORM:USED', 0) | 0, CORN0 = arr('FORCE_PLATFORM:CORNERS');
  const cornersRaw = [];
  if (fpN > 0 && CORN0) for (let k = 0; k < fpN && CORN0.length >= 12 * (k + 1); k++) {
    const c = []; for (let j = 0; j < 4; j++) c.push([0, 1, 2].map(i => CORN0[i + 3 * (j + 4 * k)] * us)); if (cornersOK(c)) cornersRaw.push(c);
  }
  const upInfo = detectUpAxis(pts, vis, nfr, npts, labels, cornersRaw, strs('POINT:Y_SCREEN')[0] || '', strs('POINT:Z_SCREEN')[0] || '');
  const upReq = (opts && opts.up) || 'auto';
  let upUse = upInfo.detected;
  if (upReq !== 'auto' && /^[+-][XYZ]$/.test(upReq)) upUse = upReq;
  const RUP = upRotation(upUse);
  upInfo.applied = upUse; upInfo.requested = upReq; upInfo.R = RUP;
  if (upUse !== '+Z') {
    for (let k = 0; k < nfr * npts; k++) {
      if (!vis[k]) continue;
      const x = pts[k * 3], y = pts[k * 3 + 1], z = pts[k * 3 + 2];
      for (let a = 0; a < 3; a++) pts[k * 3 + a] = RUP[a][0] * x + RUP[a][1] * y + RUP[a][2] * z;
    }
    warnings.push('แกนตั้งของไฟล์เป็น ' + upUse + ' (' + upInfo.source + ') หมุนเป็น Z-up ตอนโหลด');
  }
  if (upInfo.conflict) warnings.push('หลักฐานแกนตั้งขัดกัน: ' + upInfo.conflict);
  const rotUp = v => [0, 1, 2].map(a => RUP[a][0] * v[0] + RUP[a][1] * v[1] + RUP[a][2] * v[2]);

  /* analog */
  let analog = null, analogLabels = [];
  if (na > 0 && spf > 0) {
    analog = new Float32Array(nfr * spf * na);
    const ascale = arr('ANALOG:SCALE'), aoff = arr('ANALOG:OFFSET'), gen = num('ANALOG:GEN_SCALE', 1) || 1;
    const unsigned = (strs('ANALOG:FORMAT')[0] || '').toUpperCase().indexOf('UNSIGNED') === 0;
    analogLabels = strs('ANALOG:LABELS');
    for (let f = 0; f < nfr; f++) {
      const ao = base + f * fsize + npts * 4 * esz;
      for (let s = 0; s < spf; s++) for (let c = 0; c < na; c++) {
        const pos = ao + (s * na + c) * esz;
        let v;
        // ANALOG:SCALE / OFFSET / GEN_SCALE apply to float storage too (as in ezc3d / BTK)
        const rv = floatData ? f32(pos) : unsigned ? u16(pos) : i16(pos);
        v = (rv - (aoff ? aoff[c] : 0)) * (ascale && ascale.length > c ? ascale[c] : 1) * gen;
        analog[(f * spf + s) * na + c] = v;
      }
    }
  }

  /* force platforms */
  const plates = [];
  const fpUsed = num('FORCE_PLATFORM:USED', 0) | 0;
  if (fpUsed > 0) {
    const TYPE = arr('FORCE_PLATFORM:TYPE'), CORNERS = arr('FORCE_PLATFORM:CORNERS'), ORIGIN = arr('FORCE_PLATFORM:ORIGIN');
    const CHAN = P['FORCE_PLATFORM:CHANNEL'], CAL = arr('FORCE_PLATFORM:CAL_MATRIX') || arr('FORCE_PLATFORM:CALMATRIX');
    const ZERO = arr('FORCE_PLATFORM:ZERO'), COPPOLY = arr('FORCE_PLATFORM:FPCOPPOLY');
    for (let k = 0; k < fpUsed; k++) {
      const type = TYPE ? TYPE[k] : 0;
      const pl = { index: k + 1, type, supported: false, note: '', corners: null };
      if (CORNERS) {
        pl.corners = [];
        for (let j = 0; j < 4; j++) pl.corners.push(rotUp([0, 1, 2].map(i => CORNERS[i + 3 * (j + 4 * k)] * us)));
      }
      if (!pl.corners || !ORIGIN || !CHAN || !analog) { pl.note = 'พารามิเตอร์ force plate ไม่ครบ'; plates.push(pl); continue; }
      if (!cornersOK(pl.corners)) { pl.note = 'FORCE_PLATFORM:CORNERS ผิดปกติ (ค่าไม่สมเหตุสมผล) ข้าม plate นี้'; pl.corners = null; plates.push(pl); continue; }
      if (type < 1 || type > 4) { pl.note = 'ยังไม่รองรับ FP_TYPE ' + type + ' (รองรับ 1, 2, 3, 4)'; plates.push(pl); continue; }
      const nch = type === 3 ? 8 : 6;
      const nc = CHAN.dims[0];
      if (nc < nch) { pl.note = 'FP_TYPE ' + type + ' ต้องมี ' + nch + ' channel แต่ CHANNEL มี ' + nc; plates.push(pl); continue; }
      const ch = []; for (let c = 0; c < nch; c++) ch.push(CHAN.val[c + nc * k] - 1);
      if (ch.some(c => c < 0 || c >= na)) { pl.note = 'index ของ analog channel ไม่ถูกต้อง'; plates.push(pl); continue; }
      // ORIGIN: plate-centre → sensor-origin vector in plate axes; sign normalised so z ≤ 0 (as in ezc3d / C3D manual)
      let org = [0, 1, 2].map(i => ORIGIN[i + 3 * k]);
      if (org[2] > 0) org = org.map(x => -x);
      if (type === 1) { org[0] = 0; org[1] = 0; }
      let cal = null;
      if (type === 4) {
        if (!CAL || CAL.length < 36 * (k + 1)) { pl.note = 'FP_TYPE 4 แต่ไม่มี CAL_MATRIX'; plates.push(pl); continue; }
        cal = (r, c) => CAL[r + 6 * (c + 6 * k)];
      }
      const POLY = type === 3 && COPPOLY && COPPOLY.length >= 12 * (k + 1) ? Array.from(COPPOLY.slice(12 * k, 12 * k + 12)) : null;
      // baseline (FP_ZERO)
      const base0 = new Array(nch).fill(0), baseC = new Array(nch).fill(0);
      let zWin = null, zNote = '';
      if (ZERO && ZERO.length >= 2 && ZERO[0] > 0 && ZERO[1] >= ZERO[0]) {
        const s0 = (ZERO[0] - 1) * spf, s1 = Math.min(nfr * spf, ZERO[1] * spf);
        if (s1 > s0) { zWin = [s0, s1]; for (let c = 0; c < nch; c++) { let a = 0, m = 0; for (let s = s0; s < s1; s++) { const x = analog[s * na + ch[c]]; if (x === x) { a += x; m++; } } baseC[c] = m ? a / m : 0; } }
      }
      // plate frame from corners (C3D order: +x+y, -x+y, -x-y, +x-y)
      const C = pl.corners, sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
      const nrm = v => { const l = Math.hypot(v[0], v[1], v[2]) || 1; return [v[0] / l, v[1] / l, v[2] / l]; };
      const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
      const ex = nrm(sub(C[0], C[1])), ez = nrm(cross(ex, sub(C[0], C[3]))), ey = cross(ez, ex);
      const cen = [0, 1, 2].map(i => (C[0][i] + C[1][i] + C[2][i] + C[3][i]) / 4);
      /* one analog sample -> plate-frame force (N), COP relative to the plate centre and free moment Tz (file units).
         Returns false for a non-finite sample. */
      const v = new Array(8).fill(0), w = [0, 0, 0, 0, 0, 0], out = { Fx: 0, Fy: 0, Fz: 0, cx: 0, cy: 0, tz: 0, cop: false };
      const sample = s => {
        for (let c = 0; c < nch; c++) { const x = analog[s * na + ch[c]]; if (!Number.isFinite(x)) { out.Fx = out.Fy = out.Fz = out.tz = 0; out.cop = false; return false; } v[c] = x - base0[c]; }
        let Fx, Fy, Fz, Mx, My, Mz;
        if (type === 1) {            // Fx Fy Fz COPx COPy Tz (COP already in plate axes)
          out.Fx = v[0]; out.Fy = v[1]; out.Fz = v[2]; out.cx = v[3]; out.cy = v[4]; out.tz = v[5]; out.cop = Math.abs(v[2]) > 1e-6; return true;
        }
        if (type === 3) {            // Kistler 8 ch: fx12 fx34 fy14 fy23 fz1 fz2 fz3 fz4 ; ORIGIN = (a, b, az0)
          const a = org[0], b = org[1];
          Fx = v[0] + v[1]; Fy = v[2] + v[3]; Fz = v[4] + v[5] + v[6] + v[7];
          Mx = b * (v[4] + v[5] - v[6] - v[7]); My = a * (v[5] + v[6] - v[4] - v[7]); Mz = b * (v[1] - v[0]) + a * (v[2] - v[3]);
          // transfer to the plate surface: M += F × (0, 0, az0)
          Mx += Fy * org[2]; My -= Fx * org[2];
        } else {
          if (cal) for (let r = 0; r < 6; r++) { let a = 0; for (let c = 0; c < 6; c++) a += cal(r, c) * v[c]; w[r] = a; }
          else for (let c = 0; c < 6; c++) w[c] = v[c];
          Fx = w[0]; Fy = w[1]; Fz = w[2];
          // transfer to the plate centre: M += F × origin
          Mx = w[3] + Fy * org[2] - Fz * org[1]; My = w[4] + Fz * org[0] - Fx * org[2]; Mz = w[5] + Fx * org[1] - Fy * org[0];
        }
        out.Fx = Fx; out.Fy = Fy; out.Fz = Fz; out.cop = Math.abs(Fz) > 1e-6;
        if (out.cop) {
          let cx = -My / Fz, cy = Mx / Fz;
          if (POLY) {             // Kistler COP correction polynomial (FPCOPPOLY), offsets subtracted
            const P = POLY, x2 = cx * cx, y2 = cy * cy;
            const dx = (P[0] * y2 * y2 + P[1] * y2 + P[2]) * cx * x2 + (P[3] * y2 * y2 + P[4] * y2 + P[5]) * cx;
            const dy = (P[6] * x2 * x2 + P[7] * x2 + P[8]) * cy * y2 + (P[9] * x2 * x2 + P[10] * x2 + P[11]) * cy;
            cx -= dx; cy -= dy;
          }
          out.cx = cx; out.cy = cy; out.tz = Mz + Fx * cy - Fy * cx;
        } else { out.cx = out.cy = out.tz = 0; }
        return true;
      };
      /* FP_ZERO baseline: subtract only when the plate is unloaded in that window (|Fz| < 30 N). Many systems
         export already-zeroed signals and keep ZERO as a record; if someone stands on the plate in that window,
         subtracting it would erase the load (ezc3d / BTK do not apply ZERO at all). */
      if (zWin) {
        let a = 0, m = 0; for (let s = zWin[0]; s < zWin[1]; s++) if (sample(s)) { a += out.Fz; m++; }
        const fz0 = m ? Math.abs(a / m) : 0;
        if (fz0 < 30) for (let c = 0; c < nch; c++) base0[c] = baseC[c];
        else zNote = ' · ไม่ลบ baseline FP_ZERO (ช่วงนั้นมีแรง ' + fz0.toFixed(0) + ' N)';
      }
      const F = new Float32Array(nfr * 3), cop = new Float32Array(nfr * 3), on = new Uint8Array(nfr), Tf = new Float32Array(nfr * 3);
      let fzMax = 0, sumZ = 0, cntZ = 0, nBad = 0;   // non-finite analog samples (some exporters pad the last frames with NaN): read as zero load
      for (let f = 0; f < nfr; f++) {
        if (!sample(f * spf)) nBad++;
        const { Fx, Fy, Fz } = out;
        for (let a = 0; a < 3; a++) F[f * 3 + a] = ex[a] * Fx + ey[a] * Fy + ez[a] * Fz;
        const afz = Math.abs(Fz); if (afz > fzMax) fzMax = afz;
        if (Math.abs(F[f * 3 + 2]) > 5) { sumZ += F[f * 3 + 2]; cntZ++; }
        if (out.cop) {
          const ox = out.cx * us, oy = out.cy * us, Tz = out.tz * us;
          for (let a = 0; a < 3; a++) { cop[f * 3 + a] = cen[a] + ex[a] * ox + ey[a] * oy; Tf[f * 3 + a] = ez[a] * Tz; }
        }
      }
      const flipped = cntZ > 0 && sumZ < 0;
      if (flipped) for (let i = 0; i < F.length; i++) { F[i] = -F[i]; Tf[i] = -Tf[i]; }
      // full analog-rate vertical force in the lab frame (used by the force-analysis engine)
      const fzFull = new Float32Array(nfr * spf);
      for (let s = 0; s < nfr * spf; s++) { sample(s); fzFull[s] = (ex[2] * out.Fx + ey[2] * out.Fy + ez[2] * out.Fz) * (flipped ? -1 : 1); }
      const thr = Math.max(10, 0.02 * fzMax);
      for (let f = 0; f < nfr; f++) on[f] = Math.abs(F[f * 3 + 2]) > thr ? 1 : 0;
      Object.assign(pl, { supported: true, F, cop, T: Tf, on, fzMax, flipped, fzFull, note: 'FP_TYPE ' + type + (flipped ? ' · กลับเครื่องหมายแรงให้ชี้ขึ้น' : '') + (nBad ? ' · analog ไม่ใช่ตัวเลข (NaN) ' + nBad + ' frame แทนด้วย 0' : '') + zNote });
      plates.push(pl);
    }
  }

  /* events */
  const events = [];
  const evN = num('EVENT:USED', 0) | 0;
  if (evN > 0) {
    const T = arr('EVENT:TIMES'), L = strs('EVENT:LABELS'), Cx = strs('EVENT:CONTEXTS');
    for (let i = 0; i < evN && T; i++) {
      const t = T[2 * i] * 60 + T[2 * i + 1];
      const fr = Math.round(t * rate - (hdr.first - 1));
      if (fr >= 0 && fr < nfr) events.push({ frame: fr, label: L[i] || 'Event', context: Cx[i] || '' });
    }
  }

  const ds = {
    name: name || 'C3D', synthetic: false, rate, nFrames: nfr, firstFrame: hdr.first, labels,
    pts, vis, plates, events, warnings, up: upInfo,
    analog, analogLabels,
    info: { unitsLabel: unitsStr || '(เดา)', analogRate: arate, analogChannels: na, analogPerFrame: spf, floatData, processor: ptype }
  };
  return finishDataset(ds, opts && opts.aliases);
}

/* ---------------------------------------------------------------- Synthetic sample (CMJ, lower-body 16 markers) */
function mulberry32(a) { return function () { a |= 0; a = a + 0x6D2B79F5 | 0; let t = Math.imul(a ^ a >>> 15, 1 | a); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; }

function makeSyntheticCMJ() {
  const rate = 200, T = 4.5, n = Math.round(T * rate), g = 9.81, mass = 70;
  const names = ['LASI', 'RASI', 'LPSI', 'RPSI', 'LKNE', 'RKNE', 'LTHI', 'RTHI', 'LANK', 'RANK', 'LTIB', 'RTIB', 'LTOE', 'RTOE', 'LHEE', 'RHEE'];
  const labels = names.map(s => 'Skeleton_001:' + s).concat(['Unlabeled 1001', 'Unlabeled 1002']);
  const nM = labels.length, pts = new Float32Array(n * nM * 3), vis = new Uint8Array(n * nM);
  const rnd = mulberry32(7);
  const gauss = () => Math.sqrt(-2 * Math.log(rnd() + 1e-9)) * Math.cos(2 * Math.PI * rnd());
  const Ls = 0.42, Lt = 0.43, toeX = 0.22, toeZ = 0.03;
  const ease = { cos: u => 0.5 - 0.5 * Math.cos(Math.PI * u), in: u => u * u, out: u => 1 - (1 - u) * (1 - u) };
  const kf = (keys, t) => {
    if (t <= keys[0][0]) return keys[0][1];
    for (let i = 0; i < keys.length - 1; i++) {
      const a = keys[i], b = keys[i + 1];
      if (t <= b[0]) return a[1] + (b[1] - a[1]) * ease[b[2] || 'cos']((t - a[0]) / (b[0] - a[0]));
    }
    return keys[keys.length - 1][1];
  };
  const K = [[0, 5], [1.0, 5], [1.6, 82], [1.9, 10, 'in'], [2.13, 34], [2.36, 12], [2.85, 72, 'out'], [3.6, 6], [4.5, 5]];
  const PSI = [[0, 0], [1.75, 0], [1.9, 36, 'in'], [2.13, 40], [2.30, 26], [2.36, 8], [2.5, 2], [2.7, 0], [4.5, 0]];
  const t0 = 1.9, t1 = 2.36;
  const hipX = new Float32Array(n), hipZ = new Float32Array(n), ankX = new Float32Array(n), psiA = new Float32Array(n);
  for (let f = 0; f < n; f++) {
    const t = f / rate;
    const k = kf(K, t) * D2R, psi = kf(PSI, t) * D2R, phi = 0.45 * k;
    const zoff = t > t0 && t < t1 ? 0.5 * g * (t - t0) * (t1 - t) : 0;
    const ths = 0.38 * k, tht = ths - k;
    const ax = toeX + (-0.18 * Math.cos(psi) + 0.05 * Math.sin(psi));
    const az = toeZ + (0.18 * Math.sin(psi) + 0.05 * Math.cos(psi)) + zoff;
    const kx = ax + Ls * Math.sin(ths), kz = az + Ls * Math.cos(ths);
    const hx = kx + Lt * Math.sin(tht), hz = kz + Lt * Math.cos(tht);
    hipX[f] = hx; hipZ[f] = hz; ankX[f] = ax; psiA[f] = psi;
    const rot = (x, z, a) => [x * Math.cos(a) + z * Math.sin(a), -x * Math.sin(a) + z * Math.cos(a)];
    const put = (i, x, y, z) => {
      const o = (f * nM + i) * 3;
      pts[o] = x + gauss() * 0.0008; pts[o + 1] = y + gauss() * 0.0008; pts[o + 2] = z + gauss() * 0.0008;
      vis[f * nM + i] = 1;
    };
    for (let i = 0; i < 16; i++) {
      const nm = names[i], s = nm[0] === 'L' ? 1 : -1, part = nm.slice(1);
      if (part === 'ASI') { const r = rot(0.10, 0.07, phi); put(i, hx + r[0], s * 0.12, hz + r[1]); }
      else if (part === 'PSI') { const r = rot(-0.07, 0.09, phi); put(i, hx + r[0], s * 0.05, hz + r[1]); }
      else if (part === 'KNE') put(i, kx, s * 0.055, kz);
      else if (part === 'THI') put(i, kx + 0.55 * Lt * Math.sin(tht), s * 0.09, kz + 0.55 * Lt * Math.cos(tht));
      else if (part === 'ANK') put(i, ax, s * 0.05, az);
      else if (part === 'TIB') put(i, ax + 0.55 * Ls * Math.sin(ths), s * 0.07, az + 0.55 * Ls * Math.cos(ths));
      else if (part === 'TOE') put(i, toeX, s * 0.04, toeZ + zoff);
      else if (part === 'HEE') { const r = rot(-0.07, -0.05, psi); put(i, ax + r[0], s * 0.03, az + r[1]); }
    }
    if (f >= 100 && f <= 400) { const o = (f * nM + 16) * 3; pts[o] = 0.8 + gauss() * 0.002; pts[o + 1] = 0.5; pts[o + 2] = 0.9 + gauss() * 0.002; vis[f * nM + 16] = 1; }
    if (f >= 500 && f <= 700) { const o = (f * nM + 17) * 3; pts[o] = -0.6; pts[o + 1] = -0.7 + gauss() * 0.002; pts[o + 2] = 1.1; vis[f * nM + 17] = 1; }
  }
  // demo gap on LTHI (index 6) frames 300..345
  for (let f = 300; f <= 345; f++) { vis[f * nM + 6] = 0; const o = (f * nM + 6) * 3; pts[o] = pts[o + 1] = pts[o + 2] = 0; }

  // synthetic GRF from hip height (rough COM), smoothed
  const fz = new Float32Array(n), fx = new Float32Array(n);
  for (let f = 2; f < n - 2; f++) {
    fz[f] = mass * (g + (hipZ[f + 1] - 2 * hipZ[f] + hipZ[f - 1]) * rate * rate);
    fx[f] = mass * (hipX[f + 1] - 2 * hipX[f] + hipX[f - 1]) * rate * rate;
  }
  const sm = (a, w) => { const o = new Float32Array(n); for (let f = 0; f < n; f++) { let s = 0, c = 0; for (let k = -w; k <= w; k++) { const j = f + k; if (j >= 0 && j < n) { s += a[j]; c++; } } o[f] = s / c; } return o; };
  const fzs = sm(sm(fz, 6), 6), fxs = sm(sm(fx, 6), 6);
  /* two plates, one foot each (both feet carry half of the synthetic load; COP under each forefoot) */
  const plates = [];
  for (const [idx, s] of [[1, 1], [2, -1]]) {   // FP1 = left foot (+Y), FP2 = right foot (−Y)
    const F = new Float32Array(n * 3), cop = new Float32Array(n * 3), on = new Uint8Array(n);
    let fzMax = 0;
    for (let f = 0; f < n; f++) {
      const t = f / rate, flight = t > t0 && t < t1;
      const v = flight ? 0 : clamp(fzs[f], 0, 2200) / 2;
      F[f * 3] = flight ? 0 : fxs[f] / 2; F[f * 3 + 1] = 0; F[f * 3 + 2] = v;
      cop[f * 3] = ankX[f] + 0.06 + 0.1 * Math.min(1, psiA[f] / (36 * D2R)); cop[f * 3 + 1] = s * 0.045 + 0.003 * gauss(); cop[f * 3 + 2] = 0;
      on[f] = v > 10 ? 1 : 0; if (v > fzMax) fzMax = v;
    }
    plates.push({ index: idx, type: 0, supported: true, note: 'แรงสังเคราะห์จากความสูงสะโพก (ไม่ใช่ข้อมูลวัดจริง)', flipped: false,
      corners: s > 0 ? [[0.4, 0.3, 0], [-0.2, 0.3, 0], [-0.2, 0, 0], [0.4, 0, 0]] : [[0.4, 0, 0], [-0.2, 0, 0], [-0.2, -0.3, 0], [0.4, -0.3, 0]], F, cop, on, fzMax });
  }
  const ds = { name: 'ตัวอย่างสังเคราะห์ · CMJ lower-body 16 markers', synthetic: true, rate, nFrames: n, firstFrame: 1, labels,
    pts, vis, plates, events: [{ frame: Math.round(t0 * rate), label: 'Take-off', context: '' }, { frame: Math.round(t1 * rate), label: 'Landing', context: '' }],
    warnings: [], info: { unitsLabel: 'm', analogRate: 0, analogChannels: 0, floatData: true } };
  return finishDataset(ds);
}

/* ---------------------------------------------------------------- colours */
const AXIS = { x: '#F2555D', y: '#58C26A', z: '#4C8DFF' };
function shade(hex, a) {
  const n = parseInt(hex.slice(1), 16); let r = n >> 16, g = (n >> 8) & 255, b = n & 255;
  if (a >= 0) { r += (255 - r) * a; g += (255 - g) * a; b += (255 - b) * a; } else { r *= 1 + a; g *= 1 + a; b *= 1 + a; }
  return 'rgb(' + (r | 0) + ',' + (g | 0) + ',' + (b | 0) + ')';
}
const SIDE_COLOR = { L: '#D0C3F1', R: '#EF94CA', C: '#EFEBE2', U: '#8F7FB8' };
const SIDE_SHADE = {}; for (const k in SIDE_COLOR) SIDE_SHADE[k] = { hi: shade(SIDE_COLOR[k], 0.6), mid: SIDE_COLOR[k], lo: shade(SIDE_COLOR[k], -0.45) };
const MONO = '"IBM Plex Mono", ui-monospace, Menlo, Consolas, monospace';

/* ---------------------------------------------------------------- viewer */
class MocapViewer {
  constructor(canvas) {
    this.canvas = canvas; this.ctx = canvas.getContext('2d');
    this.cam = { az: -60, el: 22, dist: 4, target: [0.1, 0, 0.9], fov: 42 };
    this.ds = null; this.f = 0; this.playing = false; this.speed = 1; this.loop = true;
    this.opt = { links: true, grid: true, axes: true, labels: 'selected', plates: true, trail: 0, size: 1, unlabeled: false, segs: true, body: true };
    this.kin = null;
    this.visible = null; this.selected = -1; this.links = []; this.linkMode = false; this._pending = -1;
    this.onframe = null; this.onselect = null; this.onlinks = null;
    this.W = 0; this.H = 0; this.dpr = 1; this._dirty = true; this._scr = null;
    this._ptr = new Map(); this._last = 0;
    this._bind();
    this._ro = new ResizeObserver(() => this._resize());
    this._ro.observe(canvas);
    this._resize();
    this._raf = requestAnimationFrame(t => this._tick(t));
  }
  destroy() { cancelAnimationFrame(this._raf); this._ro.disconnect(); }
  invalidate() { this._dirty = true; }

  setDataset(ds) {
    this.ds = ds; this.f = 0; this.playing = false; this.selected = -1; this._pending = -1;
    this.visible = new Uint8Array(ds.nM); for (let i = 0; i < ds.nM; i++) this.visible[i] = ds.side[i] === 'U' ? 0 : 1;
    this.links = defaultLinks(ds.base);
    this._scr = new Float32Array(ds.nM * 3);
    this.fit(); this._emitFrame(); this._dirty = true;
    if (this.onlinks) this.onlinks(this.links);
  }
  setFrame(f) {
    if (!this.ds) return;
    this.f = clamp(f, 0, this.ds.nFrames - 1); this._dirty = true; this._emitFrame();
  }
  play() { if (!this.ds) return; if (this.f >= this.ds.nFrames - 1.001) this.f = 0; this.playing = true; this._last = 0; this._emitFrame(); }
  pause() { this.playing = false; this._emitFrame(); }
  toggle() { this.playing ? this.pause() : this.play(); }
  _emitFrame() { if (this.onframe) this.onframe(this.f, this.playing); }

  select(i) { this.selected = i; this._dirty = true; if (this.onselect) this.onselect(i); }
  setVisible(i, v) { this.visible[i] = v ? 1 : 0; this._dirty = true; }
  toggleLink(a, b) {
    const k = this.links.findIndex(l => (l[0] === a && l[1] === b) || (l[0] === b && l[1] === a));
    if (k >= 0) this.links.splice(k, 1); else this.links.push([a, b]);
    this._dirty = true; if (this.onlinks) this.onlinks(this.links);
  }

  view(name) {
    const c = this.cam;
    if (name === 'iso') { c.az = -60; c.el = 22; }
    else if (name === 'top') { c.az = -90; c.el = 89; }
    else if (name === 'x') { c.az = -90; c.el = 0; }   // X to the right
    else if (name === 'y') { c.az = 0; c.el = 0; }     // Y to the right
    this._dirty = true;
  }
  fit() {
    if (!this.ds) return;
    const b = this.ds.bounds, c = this.cam;
    c.target = [(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, Math.max(0.3, (b.min[2] + b.max[2]) / 2)];
    const ext = Math.hypot(b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2]);
    c.dist = clamp(ext * 1.2, 1.6, 40);
    this._dirty = true;
  }

  /* ---- layout */
  _resize() {
    const r = this.canvas.getBoundingClientRect();
    this.dpr = Math.min(2, window.devicePixelRatio || 1);
    this.W = Math.max(1, Math.round(r.width)); this.H = Math.max(1, Math.round(r.height));
    this.canvas.width = Math.round(this.W * this.dpr); this.canvas.height = Math.round(this.H * this.dpr);
    this._dirty = true;
  }
  _tick(ts) {
    if (this.playing && this.ds) {
      if (this._last) {
        const dt = Math.min(0.1, (ts - this._last) / 1000);
        this.f += dt * this.ds.rate * this.speed;
        if (this.f >= this.ds.nFrames - 1) {
          if (this.loop) this.f = 0; else { this.f = this.ds.nFrames - 1; this.playing = false; }
        }
        this._emitFrame();
      }
      this._last = ts; this._dirty = true;
    } else this._last = 0;
    if (this._dirty) this.draw();
    this._raf = requestAnimationFrame(t => this._tick(t));
  }

  /* ---- camera maths */
  _prep() {
    const c = this.cam, az = c.az * D2R, el = clamp(c.el, -89.5, 89.5) * D2R, ce = Math.cos(el);
    const dir = [ce * Math.cos(az), ce * Math.sin(az), Math.sin(el)];
    this.eye = [c.target[0] + dir[0] * c.dist, c.target[1] + dir[1] * c.dist, c.target[2] + dir[2] * c.dist];
    const f = [-dir[0], -dir[1], -dir[2]];
    let r = [f[1], -f[0], 0]; const rl = Math.hypot(r[0], r[1]) || 1; r = [r[0] / rl, r[1] / rl, 0];
    const u = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]];
    this.bf = f; this.br = r; this.bu = u;
    this.focal = (this.H / 2) / Math.tan(c.fov * D2R / 2);
  }
  _cam(x, y, z) {
    const e = this.eye, dx = x - e[0], dy = y - e[1], dz = z - e[2], r = this.br, u = this.bu, f = this.bf;
    return [dx * r[0] + dy * r[1] + dz * r[2], dx * u[0] + dy * u[1] + dz * u[2], dx * f[0] + dy * f[1] + dz * f[2]];
  }
  _scrn(c) { const s = this.focal / c[2]; return [this.W / 2 + c[0] * s, this.H / 2 - c[1] * s, s]; }
  proj(x, y, z) { const c = this._cam(x, y, z); return c[2] < 0.05 ? null : this._scrn(c); }
  _seg(ctx, a, b) { // a,b = 3D points; adds to current path with near-plane clipping
    let ca = this._cam(a[0], a[1], a[2]), cb = this._cam(b[0], b[1], b[2]); const n = 0.05;
    if (ca[2] < n && cb[2] < n) return;
    if (ca[2] < n) { const t = (n - ca[2]) / (cb[2] - ca[2]); ca = [ca[0] + t * (cb[0] - ca[0]), ca[1] + t * (cb[1] - ca[1]), n]; }
    else if (cb[2] < n) { const t = (n - cb[2]) / (ca[2] - cb[2]); cb = [cb[0] + t * (ca[0] - cb[0]), cb[1] + t * (ca[1] - cb[1]), n]; }
    const sa = this._scrn(ca), sb = this._scrn(cb);
    ctx.moveTo(sa[0], sa[1]); ctx.lineTo(sb[0], sb[1]);
  }

  /* ---- data access (linear interpolation between frames) */
  _pos(i, out) {
    const ds = this.ds, nM = ds.nM, f0 = Math.floor(this.f), f1 = Math.min(ds.nFrames - 1, f0 + 1), a = this.f - f0;
    const k0 = f0 * nM + i, k1 = f1 * nM + i;
    if (!ds.vis[k0]) return false;
    const p = ds.pts;
    if (a > 0 && ds.vis[k1]) { for (let j = 0; j < 3; j++) out[j] = p[k0 * 3 + j] * (1 - a) + p[k1 * 3 + j] * a; }
    else for (let j = 0; j < 3; j++) out[j] = p[k0 * 3 + j];
    return true;
  }

  /* ---- drawing */
  draw() {
    this._dirty = false;
    const ctx = this.ctx, W = this.W, H = this.H;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    const bg = ctx.createLinearGradient(0, 0, 0, H); bg.addColorStop(0, '#2B2C2F'); bg.addColorStop(1, '#0D0E10');
    ctx.fillStyle = bg; ctx.fillRect(0, 0, W, H);
    this._prep();
    const ds = this.ds;
    if (this.opt.grid) this._grid(ctx);
    if (ds && this.opt.plates) this._plates(ctx);
    if (this.opt.axes) this._labAxes(ctx);
    if (ds) { this._trail(ctx); if ((this.kin || this.ub) && this.opt.body) this._bodies(ctx); this._links(ctx); if (this.kin && this.opt.segs) this._kin(ctx, this.kin); if (this.ub && this.opt.segs) this._kin(ctx, this.ub); this._markers(ctx); if (this.opt.plates) this._grf(ctx); }
    this._gizmo(ctx);
    this._hud(ctx);
  }
  _grid(ctx) {
    const b = this.ds ? this.ds.bounds : { min: [-2, -2, 0], max: [2, 2, 2] };
    let x0 = b.min[0], x1 = b.max[0], y0 = b.min[1], y1 = b.max[1];
    if (this.ds) for (const p of this.ds.plates) if (p.corners) for (const c of p.corners) { x0 = Math.min(x0, c[0]); x1 = Math.max(x1, c[0]); y0 = Math.min(y0, c[1]); y1 = Math.max(y1, c[1]); }
    x0 = Math.min(x0, 0) - 0.6; y0 = Math.min(y0, 0) - 0.6; x1 += 0.6; y1 += 0.6;
    const ext = Math.max(x1 - x0, y1 - y0);
    const steps = [0.1, 0.25, 0.5, 1, 2, 5, 10]; let st = steps[steps.length - 1];
    for (const s of steps) if (ext / s <= 22) { st = s; break; }
    const gx0 = Math.floor(x0 / st) * st, gx1 = Math.ceil(x1 / st) * st, gy0 = Math.floor(y0 / st) * st, gy1 = Math.ceil(y1 / st) * st;
    const major = Math.max(1, st), isMajor = v => Math.abs(v / major - Math.round(v / major)) < 1e-6;
    for (const pass of [0, 1]) {
      ctx.beginPath();
      for (let x = gx0; x <= gx1 + 1e-9; x += st) if (isMajor(x) === !!pass) this._seg(ctx, [x, gy0, 0], [x, gy1, 0]);
      for (let y = gy0; y <= gy1 + 1e-9; y += st) if (isMajor(y) === !!pass) this._seg(ctx, [gx0, y, 0], [gx1, y, 0]);
      ctx.strokeStyle = pass ? 'rgba(208,195,241,0.34)' : 'rgba(208,195,241,0.14)'; ctx.lineWidth = 1; ctx.stroke();
    }
    this._gridStep = st;
  }
  _labAxes(ctx) {
    const L = 0.3, axes = [[L, 0, 0, AXIS.x, 'X'], [0, L, 0, AXIS.y, 'Y'], [0, 0, L, AXIS.z, 'Z']];
    ctx.lineWidth = 2; ctx.font = '600 11px ' + MONO; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    for (const a of axes) {
      ctx.beginPath(); this._seg(ctx, [0, 0, 0], [a[0], a[1], a[2]]); ctx.strokeStyle = a[3]; ctx.stroke();
      const p = this.proj(a[0] * 1.12, a[1] * 1.12, a[2] * 1.12); if (p) { ctx.fillStyle = a[3]; ctx.fillText(a[4], p[0], p[1]); }
    }
  }
  _plates(ctx) {
    const f = Math.min(this.ds.nFrames - 1, Math.round(this.f));
    for (const pl of this.ds.plates) {
      if (!pl.corners) continue;
      const cs = pl.corners.map(c => this._cam(c[0], c[1], c[2]));
      if (cs.some(c => c[2] < 0.05)) continue;
      const sc = cs.map(c => this._scrn(c));
      const active = pl.supported && pl.on && pl.on[f];
      ctx.beginPath(); ctx.moveTo(sc[0][0], sc[0][1]); for (let k = 1; k < 4; k++) ctx.lineTo(sc[k][0], sc[k][1]); ctx.closePath();
      ctx.fillStyle = active ? 'rgba(239,148,202,0.22)' : 'rgba(208,195,241,0.10)'; ctx.fill();
      ctx.strokeStyle = active ? 'rgba(239,148,202,0.9)' : 'rgba(208,195,241,0.5)'; ctx.lineWidth = 1.2; ctx.stroke();
      ctx.font = '500 10px ' + MONO; ctx.fillStyle = 'rgba(208,195,241,0.75)'; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
      ctx.fillText('FP' + pl.index, sc[2][0] + 4, sc[2][1] + 3);
    }
  }
  _trail(ctx) {
    const ds = this.ds, i = this.selected, tr = this.opt.trail;
    if (i < 0 || !tr || !this.visible[i]) return;
    const cur = Math.floor(this.f);
    const a = tr < 0 ? 0 : Math.max(0, cur - Math.round(tr * ds.rate));
    const b = tr < 0 ? ds.nFrames - 1 : cur;
    const stride = Math.max(1, Math.floor((b - a) / 1200));
    const draw = (from, to, style, w) => {
      ctx.beginPath(); let pen = false; let last = null;
      for (let f = from; f <= to; f += stride) {
        const k = f * ds.nM + i;
        if (!ds.vis[k]) { pen = false; continue; }
        const p = [ds.pts[k * 3], ds.pts[k * 3 + 1], ds.pts[k * 3 + 2]];
        if (pen) this._seg(ctx, last, p);
        last = p; pen = true;
      }
      ctx.strokeStyle = style; ctx.lineWidth = w; ctx.stroke();
    };
    if (tr < 0) draw(cur, b, 'rgba(255,255,255,0.18)', 1.2);
    draw(a, cur, 'rgba(255,255,255,0.8)', 1.6);
  }
  _bodies(ctx) { // soft rounded segment shapes (tapered capsules + joint balls), far-to-near
    const k = this.kin || { jc: {}, frames: {} }, f = clamp(Math.round(this.f), 0, this.ds.nFrames - 1), J = {}, o = [0, 0, 0];
    for (const src of [k, this.ub]) if (src) for (const n in src.jc) { const a = src.jc[n], b = f * 3; if (a[b] === a[b]) J[n] = [a[b], a[b + 1], a[b + 2]]; }
    const toe = {};
    for (const s of ['R', 'L']) { const i = this.ds.base.indexOf(s + 'TOE'); if (i >= 0 && this._pos(i, o)) toe[s] = o.slice(); }
    const items = [], COL = { R: '#EF94CA', L: '#D0C3F1', C: '#EFEBE2' };
    const add = (a, b, r1, r2, col) => { if (a && b) items.push({ a, b, r1, r2, col }); };
    for (const s of ['R', 'L']) {
      const H = J[s + 'HJC'], K = J[s + 'KJC'], A = J[s + 'AJC'], c = COL[s];
      add(H, K, 0.030, 0.022, c); add(K, A, 0.022, 0.017, c);
      let T = toe[s];
      if (!T && A && k.frames[s + 'FT']) { const a = k.frames[s + 'FT'], b = f * 12; if (a[b] === a[b]) T = [A[0] + a[b + 3] * 0.17, A[1] + a[b + 4] * 0.17, A[2] - 0.07]; }
      if (T && A) { const A2 = [A[0], A[1], A[2] - 0.03]; add(A2, [T[0], T[1], T[2] + 0.015], 0.018, 0.013, c); }
    }
    if (this.ub) for (const s of ['R', 'L']) {   // upper limbs: shoulder -> elbow -> wrist -> hand
      const c = COL[s]; add(J[s + 'SJC'], J[s + 'EJC'], 0.026, 0.020, c); add(J[s + 'EJC'], J[s + 'WJC'], 0.020, 0.015, c); add(J[s + 'WJC'], J[s + 'FIN'], 0.015, 0.011, c);
    }
    if (this.ub && J.RSJC && J.LSJC) add(J.RSJC, J.LSJC, 0.030, 0.030, COL.C);
    if (J.RHJC && J.LHJC) {
      const a = k.frames.RPV, b = f * 12, up = a && a[b] === a[b] ? [a[b + 6], a[b + 7], a[b + 8]] : [0, 0, 1];
      const sh = p => [p[0] + up[0] * 0.03, p[1] + up[1] * 0.03, p[2] + up[2] * 0.03];
      add(sh(J.RHJC), sh(J.LHJC), 0.026, 0.026, COL.C);
    }
    const sp = (p) => { const c = this._cam(p[0], p[1], p[2]); return c[2] < 0.05 ? null : { s: this._scrn(c), z: c[2] }; };
    const prim = [];
    for (const it of items) { const A = sp(it.a), B = sp(it.b); if (A && B) prim.push({ it, A, B, z: (A.z + B.z) / 2 }); }
    prim.sort((u, v) => v.z - u.z);
    ctx.save(); ctx.globalAlpha = 0.9;
    const ball = (x, y, r, col) => {
      const g = ctx.createRadialGradient(x - r * 0.35, y - r * 0.4, r * 0.1, x, y, r);
      g.addColorStop(0, shade(col, 0.45)); g.addColorStop(0.6, col); g.addColorStop(1, shade(col, -0.35));
      ctx.beginPath(); ctx.arc(x, y, r, 0, 6.2832); ctx.fillStyle = g; ctx.fill();
    };
    for (const { it, A, B } of prim) {
      const r1 = it.r1 * A.s[2], r2 = it.r2 * B.s[2], dx = B.s[0] - A.s[0], dy = B.s[1] - A.s[1], d = Math.hypot(dx, dy);
      if (d > 0.5) {
        const nx = -dy / d, ny = dx / d, g = ctx.createLinearGradient(A.s[0] - nx * r1, A.s[1] - ny * r1, A.s[0] + nx * r1, A.s[1] + ny * r1);
        g.addColorStop(0, shade(it.col, 0.4)); g.addColorStop(0.45, it.col); g.addColorStop(1, shade(it.col, -0.4));
        ctx.beginPath(); ctx.moveTo(A.s[0] + nx * r1, A.s[1] + ny * r1); ctx.lineTo(B.s[0] + nx * r2, B.s[1] + ny * r2);
        ctx.lineTo(B.s[0] - nx * r2, B.s[1] - ny * r2); ctx.lineTo(A.s[0] - nx * r1, A.s[1] - ny * r1); ctx.closePath(); ctx.fillStyle = g; ctx.fill();
      }
      ball(A.s[0], A.s[1], r1, it.col); ball(B.s[0], B.s[1], r2, it.col);
    }
    ctx.restore();
  }
  _kin(ctx, k) {
    const f = clamp(Math.round(this.f), 0, this.ds.nFrames - 1), p = [0, 0, 0], q = [0, 0, 0], L = 0.09;
    ctx.lineCap = 'round'; ctx.lineWidth = 2.4;
    const cols = ['#FF5A4D', '#4ADE80', '#5AA9FF'];
    for (const name of k.segNames) {
      const a = k.frames[name], b = f * 12; if (a[b] !== a[b]) continue;
      p[0] = a[b]; p[1] = a[b + 1]; p[2] = a[b + 2];
      for (let c = 0; c < 3; c++) {
        q[0] = p[0] + a[b + 3 + c * 3] * L; q[1] = p[1] + a[b + 4 + c * 3] * L; q[2] = p[2] + a[b + 5 + c * 3] * L;
        ctx.beginPath(); this._seg(ctx, p, q); ctx.strokeStyle = cols[c]; ctx.stroke();
      }
    }
    // joint centres
    for (const j in k.jc) {
      const a = k.jc[j], b = f * 3; if (a[b] !== a[b]) continue;
      const c = this._cam(a[b], a[b + 1], a[b + 2]); if (c[2] < 0.05) continue;
      const s = this._scrn(c), r = clamp(0.012 * s[2], 2.5, 8);
      ctx.beginPath(); ctx.arc(s[0], s[1], r, 0, 6.2832); ctx.fillStyle = '#EFEBE2'; ctx.fill();
      ctx.strokeStyle = 'rgba(13,14,16,0.9)'; ctx.lineWidth = 1.2; ctx.stroke();
    }
    ctx.lineWidth = 2.2;
  }
  _links(ctx) {
    if (!this.opt.links) return;
    const p = [0, 0, 0], q = [0, 0, 0];
    ctx.lineWidth = 2.2; ctx.lineCap = 'round';
    for (const [a, b] of this.links) {
      if (!this.visible[a] || !this.visible[b] || !this._pos(a, p) || !this._pos(b, q)) continue;
      const sa = this.ds.side[a], sb = this.ds.side[b];
      ctx.beginPath(); this._seg(ctx, p, q);
      ctx.strokeStyle = sa === sb && SIDE_COLOR[sa] ? SIDE_COLOR[sa] + 'B3' : 'rgba(208,195,241,0.5)';
      ctx.stroke();
    }
  }
  _markers(ctx) {
    const ds = this.ds, nM = ds.nM, o = [0, 0, 0], prim = [];
    this._scr.fill(0);
    for (let i = 0; i < nM; i++) {
      if (!this.visible[i] || !this._pos(i, o)) continue;
      const c = this._cam(o[0], o[1], o[2]); if (c[2] < 0.05) continue;
      const s = this._scrn(c);
      const r = clamp(0.0165 * s[2] * this.opt.size, 2.5, 15);
      prim.push([i, s[0], s[1], c[2], r]);
      this._scr[i * 3] = s[0]; this._scr[i * 3 + 1] = s[1]; this._scr[i * 3 + 2] = r;
    }
    prim.sort((a, b) => b[3] - a[3]);
    for (const [i, x, y, , r] of prim) {
      const sh = SIDE_SHADE[ds.side[i]] || SIDE_SHADE.C;
      const g = ctx.createRadialGradient(x - r * 0.35, y - r * 0.35, r * 0.1, x, y, r);
      g.addColorStop(0, sh.hi); g.addColorStop(0.55, sh.mid); g.addColorStop(1, sh.lo);
      ctx.beginPath(); ctx.arc(x, y, r, 0, 6.2832); ctx.fillStyle = g; ctx.fill();
    }
    // selection, pending link, labels
    ctx.font = '500 11px ' + MONO; ctx.textAlign = 'left'; ctx.textBaseline = 'middle'; ctx.lineJoin = 'round';
    for (const [i, x, y, , r] of prim) {
      const isSel = i === this.selected, isPend = i === this._pending;
      if (isSel || isPend) { ctx.beginPath(); ctx.arc(x, y, r + 4, 0, 6.2832); ctx.strokeStyle = isPend ? '#EF94CA' : '#FFFFFF'; ctx.lineWidth = 2; ctx.stroke(); }
      if (this.opt.labels === 'all' || (this.opt.labels === 'selected' && (isSel || isPend))) {
        const t = ds.base[i];
        ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(13,14,16,0.85)'; ctx.strokeText(t, x + r + 6, y);
        ctx.fillStyle = '#EFEBE2'; ctx.fillText(t, x + r + 6, y);
      }
    }
  }
  _grf(ctx) {
    const ds = this.ds, f = Math.floor(this.f);
    let maxF = 0; for (const p of ds.plates) if (p.supported && p.fzMax > maxF) maxF = p.fzMax;
    if (!maxF) return;
    const k = 0.9 / maxF;
    for (const pl of ds.plates) {
      if (!pl.supported || !pl.on[f]) continue;
      const cx = pl.cop[f * 3], cy = pl.cop[f * 3 + 1], cz = pl.cop[f * 3 + 2];
      const tip = [cx + pl.F[f * 3] * k, cy + pl.F[f * 3 + 1] * k, cz + pl.F[f * 3 + 2] * k];
      const a = this.proj(cx, cy, cz), b = this.proj(tip[0], tip[1], tip[2]);
      if (!a || !b) continue;
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.strokeStyle = '#EF94CA'; ctx.lineWidth = 2.5; ctx.lineCap = 'round'; ctx.stroke();
      const ang = Math.atan2(b[1] - a[1], b[0] - a[0]), h = 9;
      ctx.beginPath(); ctx.moveTo(b[0], b[1]); ctx.lineTo(b[0] - h * Math.cos(ang - 0.4), b[1] - h * Math.sin(ang - 0.4)); ctx.lineTo(b[0] - h * Math.cos(ang + 0.4), b[1] - h * Math.sin(ang + 0.4)); ctx.closePath();
      ctx.fillStyle = '#EF94CA'; ctx.fill();
      ctx.beginPath(); ctx.arc(a[0], a[1], 3.5, 0, 6.2832); ctx.fillStyle = '#121315'; ctx.fill(); ctx.strokeStyle = '#EF94CA'; ctx.lineWidth = 1.6; ctx.stroke();
    }
  }
  _gizmo(ctx) {
    const cx = 46, cy = this.H - 46, L = 26, r = this.br, u = this.bu, f = this.bf;
    const ax = [[1, 0, 0, AXIS.x, 'X'], [0, 1, 0, AXIS.y, 'Y'], [0, 0, 1, AXIS.z, 'Z']].map(a => ({
      x: a[0] * r[0] + a[1] * r[1] + a[2] * r[2], y: a[0] * u[0] + a[1] * u[1] + a[2] * u[2], d: a[0] * f[0] + a[1] * f[1] + a[2] * f[2], c: a[3], t: a[4]
    })).sort((a, b) => b.d - a.d);
    ctx.beginPath(); ctx.arc(cx, cy, L + 12, 0, 6.2832); ctx.fillStyle = 'rgba(13,14,16,0.5)'; ctx.fill();
    ctx.font = '600 11px ' + MONO; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.lineWidth = 2.2; ctx.lineCap = 'round';
    for (const a of ax) {
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + a.x * L, cy - a.y * L); ctx.strokeStyle = a.c; ctx.stroke();
      ctx.fillStyle = a.c; ctx.fillText(a.t, cx + a.x * (L + 8), cy - a.y * (L + 8));
    }
  }
  _hud(ctx) {
    const ds = this.ds; if (!ds) return;
    const f = Math.floor(this.f);
    ctx.font = '500 12px ' + MONO; ctx.textAlign = 'left'; ctx.textBaseline = 'top'; ctx.fillStyle = 'rgba(239,235,226,0.85)';
    ctx.fillText('t ' + (this.f / ds.rate).toFixed(3) + ' s   f ' + (f + 1) + '/' + ds.nFrames, 12, 10);
    if (this._gridStep) { ctx.fillStyle = 'rgba(208,195,241,0.6)'; ctx.font = '500 10.5px ' + MONO; ctx.fillText('grid ' + (this._gridStep >= 1 ? this._gridStep + ' m' : Math.round(this._gridStep * 100) + ' cm'), 12, 28); }
  }

  /* ---- interaction */
  pick(x, y) {
    if (!this._scr) return -1;
    let best = -1, bd = 1e9;
    for (let i = 0; i < this.ds.nM; i++) {
      const r = this._scr[i * 3 + 2]; if (!r) continue;
      const d = Math.hypot(this._scr[i * 3] - x, this._scr[i * 3 + 1] - y);
      if (d < r + 9 && d < bd) { bd = d; best = i; }
    }
    return best;
  }
  _bind() {
    const cv = this.canvas;
    cv.style.touchAction = 'none';
    cv.addEventListener('contextmenu', e => e.preventDefault());
    cv.addEventListener('wheel', e => {
      e.preventDefault(); const c = this.cam;
      c.dist = clamp(c.dist * Math.exp(e.deltaY * 0.0012), 0.4, 60); this._dirty = true;
    }, { passive: false });
    cv.addEventListener('pointerdown', e => {
      cv.setPointerCapture(e.pointerId);
      this._ptr.set(e.pointerId, { x: e.clientX, y: e.clientY, sx: e.clientX, sy: e.clientY, t: performance.now(), btn: e.button, shift: e.shiftKey });
    });
    cv.addEventListener('pointermove', e => {
      const p = this._ptr.get(e.pointerId); if (!p) return;
      const dx = e.clientX - p.x, dy = e.clientY - p.y; p.x = e.clientX; p.y = e.clientY;
      const c = this.cam;
      if (this._ptr.size === 2) {
        const [a, b] = [...this._ptr.values()];
        const d = Math.hypot(a.x - b.x, a.y - b.y);
        if (this._pinch) { c.dist = clamp(c.dist * this._pinch / Math.max(20, d), 0.4, 60); this._pan((dx) / 2, (dy) / 2); }
        this._pinch = d; this._dirty = true; p.moved = true; return;
      }
      if (p.btn === 2 || p.btn === 1 || p.shift) this._pan(dx, dy);
      else { c.az -= dx * 0.35; c.el = clamp(c.el + dy * 0.35, -89, 89); }
      if (Math.hypot(e.clientX - p.sx, e.clientY - p.sy) > 4) p.moved = true;
      this._dirty = true;
    });
    const up = e => {
      const p = this._ptr.get(e.pointerId); this._ptr.delete(e.pointerId); this._pinch = 0;
      if (!p || p.moved || !this.ds || e.type === 'pointercancel' || p.btn !== 0 || performance.now() - p.t > 600) return;
      const r = cv.getBoundingClientRect(), i = this.pick(e.clientX - r.left, e.clientY - r.top);
      if (this.linkMode) {
        if (i < 0) this._pending = -1;
        else if (this._pending < 0) this._pending = i;
        else { if (i !== this._pending) this.toggleLink(this._pending, i); this._pending = -1; }
        this._dirty = true;
      } else this.select(i);
    };
    cv.addEventListener('pointerup', up); cv.addEventListener('pointercancel', up);
  }
  _pan(dx, dy) {
    const c = this.cam, s = c.dist / this.focal, r = this.br, u = this.bu;
    for (let k = 0; k < 3; k++) c.target[k] += -r[k] * dx * s + u[k] * dy * s;
  }
  setLinkMode(on) { this.linkMode = !!on; this._pending = -1; this._dirty = true; }
}

/* ---------------------------------------------------------------- chart */
function niceTicks(min, max, count) {
  const span = (max - min) || 1, raw = span / count, p = Math.pow(10, Math.floor(Math.log10(raw))), e = raw / p;
  const step = p * (e >= 7.5 ? 10 : e >= 3.5 ? 5 : e >= 1.5 ? 2 : 1);
  const out = []; for (let v = Math.ceil(min / step) * step; v <= max + step * 1e-6; v += step) out.push(Math.abs(v) < step * 1e-9 ? 0 : v);
  return { ticks: out, step };
}
class ChartView {
  constructor(canvas) {
    this.canvas = canvas; this.ctx = canvas.getContext('2d');
    this.series = []; this.rate = 100; this.n = 0; this.events = []; this.cursor = 0; this.title = ''; this.unit = '';
    this.theme = { fg: '#272030', fg2: '#4E2780', line: '#D0C3F1', accent: '#AB1D79', panel: '#F8F6F0' };
    this.onseek = null; this.W = 0; this.H = 0; this.dpr = 1;
    this.m = { l: 54, r: 14, t: 10, b: 26 };
    canvas.style.touchAction = 'none';
    const seek = e => {
      if (!this.n || !this.onseek) return;
      const r = canvas.getBoundingClientRect(), x = e.clientX - r.left, w = this.W - this.m.l - this.m.r;
      this.onseek(clamp(Math.round((x - this.m.l) / w * (this.n - 1)), 0, this.n - 1));
    };
    /* hover read-out: the sample under the pointer (mouse: while moving; touch/pen: while dragging) */
    this.hover = null;
    const hov = e => {
      if (!this.n) return;
      const r = canvas.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top, w = this.W - this.m.l - this.m.r;
      const inside = x >= this.m.l - 4 && x <= this.W - this.m.r + 4 && y >= this.m.t && y <= this.H - this.m.b;
      const h = inside ? clamp(Math.round((x - this.m.l) / w * (this.n - 1)), 0, this.n - 1) : null;
      if (h !== this.hover) { this.hover = h; this.draw(); }
    };
    const unhov = () => { if (this.hover != null) { this.hover = null; this.draw(); } };
    canvas.addEventListener('pointerdown', e => { canvas.setPointerCapture(e.pointerId); this._drag = true; seek(e); hov(e); });
    canvas.addEventListener('pointermove', e => { if (this._drag) seek(e); if (this._drag || e.pointerType === 'mouse') hov(e); });
    canvas.addEventListener('pointerup', e => { this._drag = false; if (e.pointerType !== 'mouse') unhov(); });
    canvas.addEventListener('pointercancel', () => { this._drag = false; unhov(); });
    canvas.addEventListener('pointerleave', e => { if (!this._drag) unhov(); });
    this._ro = new ResizeObserver(() => this._resize()); this._ro.observe(canvas); this._resize();
  }
  _resize() {
    const r = this.canvas.getBoundingClientRect(); this.dpr = Math.min(2, window.devicePixelRatio || 1);
    this.W = Math.max(1, Math.round(r.width)); this.H = Math.max(1, Math.round(r.height));
    this.canvas.width = Math.round(this.W * this.dpr); this.canvas.height = Math.round(this.H * this.dpr); this.draw();
  }
  set(o) { Object.assign(this, o); this.draw(); }
  draw() {
    const ctx = this.ctx, W = this.W, H = this.H, m = this.m, th = this.theme;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.fillStyle = th.panel; ctx.fillRect(0, 0, W, H);
    const pw = W - m.l - m.r, ph = H - m.t - m.b;
    ctx.font = '500 10.5px ' + MONO;
    if (!this.series.length || !this.n || pw < 20 || ph < 20) {
      ctx.fillStyle = th.fg2; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(this.empty || 'คลิก marker ในมุมมอง 3D หรือในรายการ เพื่อดูกราฟ', W / 2, H / 2); return;
    }
    let mn = Infinity, mx = -Infinity;
    for (const s of this.series) for (let i = 0; i < this.n; i++) { const v = s.data[i]; if (v === v) { if (v < mn) mn = v; if (v > mx) mx = v; } }
    if (!isFinite(mn)) { mn = -1; mx = 1; }
    if (mx - mn < 1e-9) { mn -= 0.5; mx += 0.5; }
    const pad = (mx - mn) * 0.06; mn -= pad; mx += pad;
    const dur = (this.n - 1) / this.rate;
    const X = i => m.l + i / (this.n - 1) * pw, Y = v => m.t + (1 - (v - mn) / (mx - mn)) * ph;
    // grid
    const yt = niceTicks(mn, mx, Math.max(2, Math.floor(ph / 34)));
    const dec = Math.max(0, Math.min(4, -Math.floor(Math.log10(yt.step)) + (yt.step < 1 ? 0 : 0)));
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle'; ctx.lineWidth = 1;
    for (const v of yt.ticks) {
      const y = Math.round(Y(v)) + 0.5;
      ctx.strokeStyle = th.line; ctx.globalAlpha = v === 0 ? 1 : 0.55; ctx.beginPath(); ctx.moveTo(m.l, y); ctx.lineTo(W - m.r, y); ctx.stroke(); ctx.globalAlpha = 1;
      ctx.fillStyle = th.fg2; ctx.fillText(v.toFixed(dec), m.l - 6, y);
    }
    const t0 = this.t0 || 0, xt = niceTicks(t0, t0 + dur, Math.max(2, Math.floor(pw / 70)));
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    for (const t of xt.ticks) {
      const x = Math.round(m.l + (t - t0) / dur * pw) + 0.5;
      ctx.strokeStyle = th.line; ctx.globalAlpha = 0.35; ctx.beginPath(); ctx.moveTo(x, m.t); ctx.lineTo(x, m.t + ph); ctx.stroke(); ctx.globalAlpha = 1;
      ctx.fillStyle = th.fg2; ctx.fillText(t.toFixed(xt.step < 1 ? 1 : 0) + ' s', x, m.t + ph + 6);
    }
    // phase bands + horizontal reference lines
    if (this.bands) for (const b of this.bands) {
      const x0 = X(clamp(b.from, 0, this.n - 1)), x1 = X(clamp(b.to, 0, this.n - 1)); if (x1 <= x0) continue;
      ctx.fillStyle = b.color; ctx.fillRect(x0, m.t, x1 - x0, ph);
      if (b.label && x1 - x0 > 26) { ctx.fillStyle = th.fg2; ctx.textAlign = 'center'; ctx.textBaseline = 'top'; ctx.fillText(b.label, (x0 + x1) / 2, m.t + 3); }
    }
    if (this.hlines) for (const h of this.hlines) {
      if (h.v < mn || h.v > mx) continue; const y = Math.round(Y(h.v)) + 0.5;
      ctx.setLineDash([6, 4]); ctx.strokeStyle = th.fg2; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(m.l, y); ctx.lineTo(W - m.r, y); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = th.fg2; ctx.textAlign = 'right'; ctx.textBaseline = 'bottom'; ctx.fillText(h.label, W - m.r - 4, y - 2);
    }
    // events
    ctx.setLineDash([3, 3]);
    for (const ev of this.events) {
      const x = Math.round(X(ev.frame)) + 0.5;
      ctx.strokeStyle = th.fg2; ctx.beginPath(); ctx.moveTo(x, m.t); ctx.lineTo(x, m.t + ph); ctx.stroke();
      if (!this.noEventLabels) { ctx.fillStyle = th.fg2; ctx.textAlign = 'left'; ctx.textBaseline = 'bottom'; ctx.fillText(ev.label, x + 4, m.t + ph - 2); }
    }
    ctx.setLineDash([]);
    // series
    const stride = Math.max(1, Math.floor(this.n / (pw * 1.5)));
    ctx.lineWidth = 1.6; ctx.lineJoin = 'round';
    for (const s of this.series) {
      ctx.beginPath(); let pen = false;
      for (let i = 0; i < this.n; i += stride) {
        const v = s.data[i]; if (v !== v) { pen = false; continue; }
        if (pen) ctx.lineTo(X(i), Y(v)); else ctx.moveTo(X(i), Y(v)); pen = true;
      }
      ctx.strokeStyle = s.color; ctx.stroke();
    }
    // cursor + legend values
    const cf = clamp(Math.round(this.cursor), 0, this.n - 1), cx = Math.round(X(cf)) + 0.5;
    ctx.strokeStyle = th.accent; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(cx, m.t); ctx.lineTo(cx, m.t + ph); ctx.stroke();
    let lx = m.l + 8; ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
    const ly = m.t + ph - 6;
    for (const s of this.series) {
      const v = s.data[cf];
      ctx.fillStyle = s.color; ctx.fillRect(lx, ly - 9, 9, 3);
      ctx.fillStyle = th.fg; const txt = s.name + ' ' + (v === v ? v.toFixed(this.dec == null ? 3 : this.dec) : '—');
      ctx.fillText(txt, lx + 13, ly + 2); lx += ctx.measureText(txt).width + 30;
    }
    if (this.hover != null && this.hover < this.n) this._drawHover(X, Y, mn, mx, pw, ph);
  }
  _drawHover(X, Y, mn, mx, pw, ph) {
    const ctx = this.ctx, m = this.m, th = this.theme, i = this.hover, hx = Math.round(X(i)) + 0.5;
    const dec = this.dec == null ? 3 : this.dec;
    ctx.save();
    ctx.setLineDash([4, 3]); ctx.strokeStyle = th.fg; ctx.globalAlpha = 0.55; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(hx, m.t); ctx.lineTo(hx, m.t + ph); ctx.stroke(); ctx.setLineDash([]); ctx.globalAlpha = 1;
    const rows = [];
    for (const s of this.series) {
      const v = s.data[i];
      if (v === v && v >= mn && v <= mx) { ctx.beginPath(); ctx.arc(X(i), Y(v), 3.5, 0, Math.PI * 2); ctx.fillStyle = s.color; ctx.fill(); ctx.lineWidth = 1.5; ctx.strokeStyle = th.panel; ctx.stroke(); }
      rows.push([s.color, s.name, v === v ? v.toFixed(dec) : '—']);
    }
    const t = (this.t0 || 0) + i / this.rate;
    const head = 't = ' + t.toFixed(3) + ' s' + (this.frameOffset != null ? ' · frame ' + (i + this.frameOffset) : '');
    ctx.font = '500 11px ' + MONO;
    let wName = 0, wVal = 0; for (const r of rows) { wName = Math.max(wName, ctx.measureText(r[1]).width); wVal = Math.max(wVal, ctx.measureText(r[2]).width); }
    const lh = 15, bw = Math.max(ctx.measureText(head).width, 14 + wName + 10 + wVal) + 16, bh = 8 + lh * (rows.length + 1) + 4;
    let bx = hx + 10; if (bx + bw > this.W - m.r) bx = hx - 10 - bw; if (bx < 2) bx = 2;
    const by = m.t + 4;
    ctx.globalAlpha = 0.94; ctx.fillStyle = th.panel; ctx.fillRect(bx, by, bw, bh); ctx.globalAlpha = 1;
    ctx.strokeStyle = th.fg2; ctx.lineWidth = 1; ctx.strokeRect(Math.round(bx) + 0.5, Math.round(by) + 0.5, Math.round(bw), Math.round(bh));
    ctx.textBaseline = 'middle'; ctx.textAlign = 'left'; ctx.fillStyle = th.fg2;
    ctx.fillText(head, bx + 8, by + 4 + lh / 2);
    rows.forEach((r, k) => {
      const yy = by + 4 + lh * (k + 1) + lh / 2;
      ctx.fillStyle = r[0]; ctx.fillRect(bx + 8, yy - 1.5, 9, 3);
      ctx.fillStyle = th.fg; ctx.textAlign = 'left'; ctx.fillText(r[1], bx + 22, yy);
      ctx.textAlign = 'right'; ctx.fillText(r[2], bx + bw - 8, yy);
    });
    ctx.restore();
  }
}


/* ---------------------------------------------------------------- lower-body kinematics (independent implementation)
   - Coda-type pelvis (ASIS + sacrum mid-PSIS), HJC by Bell regression (0.36 / -0.19 / -0.30 x ASIS distance)
   - virtual medial knee/ankle = lateral marker shifted by (width + 2*marker radius) along static pelvis ML
   - thigh/shank/foot: axial = proximal - distal, ML from lateral-medial markers (foot: ankle marker)
   - segment local axes: X anterior, Y axial (up), Z medio-lateral toward subject's RIGHT for both sides
   - static = mean of the first N frames; motion = rigid fit of the tracking set (Horn), anatomical frame carried along
   - joint angle: Cardan Z-X-Y, R = Rz(a) Rx(b) Ry(g); a = about Z (flex/ext), b = about X (ab/adduction), g = about Y (axial)
   Not cross-checked against other software: compare with an independent reference before relying on the numbers. */
const V = {
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]], add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  mul: (a, s) => [a[0] * s, a[1] * s, a[2] * s], dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  norm: a => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; },
  mid: (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2]
};
const M3 = {
  cols: (X, Y, Z) => [X[0], Y[0], Z[0], X[1], Y[1], Z[1], X[2], Y[2], Z[2]],
  mul: (A, B) => { const o = new Array(9); for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) o[r * 3 + c] = A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c]; return o; },
  T: A => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]],
  vec: (A, v) => [A[0] * v[0] + A[1] * v[1] + A[2] * v[2], A[3] * v[0] + A[4] * v[1] + A[5] * v[2], A[6] * v[0] + A[7] * v[1] + A[8] * v[2]],
  col: (A, c) => [A[c], A[3 + c], A[6 + c]]
};
function eigMaxSym4(N) {
  const a = N.map(r => r.slice()), v = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
  for (let sw = 0; sw < 60; sw++) {
    let off = 0; for (let i = 0; i < 4; i++) for (let j = i + 1; j < 4; j++) off += a[i][j] * a[i][j];
    if (off < 1e-26) break;
    for (let p = 0; p < 3; p++) for (let q = p + 1; q < 4; q++) {
      if (Math.abs(a[p][q]) < 1e-300) continue;
      const th = (a[q][q] - a[p][p]) / (2 * a[p][q]);
      const t = (th >= 0 ? 1 : -1) / (Math.abs(th) + Math.sqrt(th * th + 1)), c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < 4; k++) { const x = a[k][p], y = a[k][q]; a[k][p] = c * x - s * y; a[k][q] = s * x + c * y; }
      for (let k = 0; k < 4; k++) { const x = a[p][k], y = a[q][k]; a[p][k] = c * x - s * y; a[q][k] = s * x + c * y; }
      for (let k = 0; k < 4; k++) { const x = v[k][p], y = v[k][q]; v[k][p] = c * x - s * y; v[k][q] = s * x + c * y; }
    }
  }
  let m = 0; for (let i = 1; i < 4; i++) if (a[i][i] > a[m][m]) m = i;
  return [v[0][m], v[1][m], v[2][m], v[3][m]];
}
/* rigid transform B ~ R*A + t  (Horn 1987). A,B: arrays of [x,y,z] */
function rigidFit(A, B) {
  const n = A.length, ca = [0, 0, 0], cb = [0, 0, 0];
  for (let i = 0; i < n; i++) for (let k = 0; k < 3; k++) { ca[k] += A[i][k] / n; cb[k] += B[i][k] / n; }
  let Sxx = 0, Sxy = 0, Sxz = 0, Syx = 0, Syy = 0, Syz = 0, Szx = 0, Szy = 0, Szz = 0;
  for (let i = 0; i < n; i++) {
    const a = [A[i][0] - ca[0], A[i][1] - ca[1], A[i][2] - ca[2]], b = [B[i][0] - cb[0], B[i][1] - cb[1], B[i][2] - cb[2]];
    Sxx += a[0] * b[0]; Sxy += a[0] * b[1]; Sxz += a[0] * b[2]; Syx += a[1] * b[0]; Syy += a[1] * b[1]; Syz += a[1] * b[2];
    Szx += a[2] * b[0]; Szy += a[2] * b[1]; Szz += a[2] * b[2];
  }
  const q = eigMaxSym4([[Sxx + Syy + Szz, Syz - Szy, Szx - Sxz, Sxy - Syx], [Syz - Szy, Sxx - Syy - Szz, Sxy + Syx, Szx + Sxz],
    [Szx - Sxz, Sxy + Syx, -Sxx + Syy - Szz, Syz + Szy], [Sxy - Syx, Szx + Sxz, Syz + Szy, -Sxx - Syy + Szz]]);
  const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1, w = q[0] / l, x = q[1] / l, y = q[2] / l, z = q[3] / l;
  const R = [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y), 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
    2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)];
  const Rca = M3.vec(R, ca);
  return { R, t: [cb[0] - Rca[0], cb[1] - Rca[1], cb[2] - Rca[2]] };
}
/* R = Rz(a) Rx(b) Ry(g)  ->  degrees */
function cardanZXY(R) {
  return [Math.atan2(-R[1], R[4]) / D2R, Math.asin(clamp(R[7], -1, 1)) / D2R, Math.atan2(-R[6], R[8]) / D2R];
}
/* R = Ry(a) Rx(b) Rz(g)  ->  degrees  (Plug-in Gait style: a about lateral Y, b about anterior X, g about vertical Z) */
function cardanYXZ(R) {
  return [Math.atan2(R[2], R[8]) / D2R, Math.asin(clamp(-R[5], -1, 1)) / D2R, Math.atan2(R[3], R[4]) / D2R];
}
function rotYXZ(aDeg, bDeg, gDeg) {
  const a = aDeg * D2R, b = bDeg * D2R, g = gDeg * D2R, Ry = [Math.cos(a), 0, Math.sin(a), 0, 1, 0, -Math.sin(a), 0, Math.cos(a)],
    Rx = [1, 0, 0, 0, Math.cos(b), -Math.sin(b), 0, Math.sin(b), Math.cos(b)], Rz = [Math.cos(g), -Math.sin(g), 0, Math.sin(g), Math.cos(g), 0, 0, 0, 1];
  return M3.mul(M3.mul(Ry, Rx), Rz);
}
function rotZXY(aDeg, bDeg, gDeg) {
  const a = aDeg * D2R, b = bDeg * D2R, g = gDeg * D2R, Rz = [Math.cos(a), -Math.sin(a), 0, Math.sin(a), Math.cos(a), 0, 0, 0, 1],
    Rx = [1, 0, 0, 0, Math.cos(b), -Math.sin(b), 0, Math.sin(b), Math.cos(b)], Ry = [Math.cos(g), 0, Math.sin(g), 0, 1, 0, -Math.sin(g), 0, Math.cos(g)];
  return M3.mul(M3.mul(Rz, Rx), Ry);
}

const LB_MARKERS = ['LASI', 'RASI', 'LPSI', 'RPSI', 'LKNE', 'RKNE', 'LTHI', 'RTHI', 'LANK', 'RANK', 'LTIB', 'RTIB', 'LTOE', 'RTOE', 'LHEE', 'RHEE'];
const LB_DEFAULTS = { kneeW: 0.10, ankleW: 0.07, markerR: 0.007, staticStart: 0, staticN: 10, normalize: false, fitTol: 0.015, convention: 'zxy', tracking: 'anat' };
/* Marker sets for the lower-body model.
   conv16 : Conventional Lower 16 (tibia wand TIB, virtual medial points from caliper widths)
   bad18  : 18 markers with real medial KNEM/ANKM, no TIB (e.g. badminton set). Joint centres = midpoint of lateral/medial,
            segments are tracked with a robust rigid fit that drops one inconsistent marker (e.g. a mislabelled KNEM). */
const LB_SETS = {
  conv16: { label: 'Conventional Lower 16', markers: LB_MARKERS, medial: false },
  bad18: { label: 'Lower 18 (KNEM/ANKM, no TIB)', medial: true,
    markers: ['LASI', 'RASI', 'LPSI', 'RPSI', 'LKNE', 'RKNE', 'LKNEM', 'RKNEM', 'LTHI', 'RTHI', 'LANK', 'RANK', 'LANKM', 'RANKM', 'LTOE', 'RTOE', 'LHEE', 'RHEE'] }
};
function detectLBSet(ds) {
  const have = new Set(ds.base.map(b => b.toUpperCase()));
  for (const id of ['bad18', 'conv16']) if (LB_SETS[id].markers.every(n => have.has(n))) return id;   // medial markers present -> measured knee/ankle widths
  return null;
}
const LB_JOINTS = [['RHip', 'RTH', 'RPV'], ['RKnee', 'RSK', 'RTH'], ['RAnkle', 'RFT', 'RSK'], ['LHip', 'LTH', 'RPV'], ['LKnee', 'LSK', 'LTH'], ['LAnkle', 'LFT', 'LSK']];

/* Joint angle of segment sg relative to reference rf. Shared by segment optimisation, IK and the moment sign rules of ID.
   zxy: raw Cardan Z-X-Y.  pig: Euler Y-X-Z, a = flexion(+) / dorsiflexion(+), b = adduction(+) / varus(+), g = internal rotation(+). */
function jointEuler(jn, Rrf, Rsg, PIG, pigOffset) {
  const Rrel = M3.mul(M3.T(Rrf), Rsg);
  if (!PIG) return cardanZXY(Rrel);
  const e = cardanYXZ(Rrel), t = jn.slice(1), sd = jn[0] === 'L' ? -1 : 1;
  if (t === 'Hip') return [-e[0], sd * e[1], sd * e[2]];
  if (t === 'Knee') return [e[0], sd * e[1], sd * e[2]];
  return [-(e[0] - pigOffset[jn[0]]), sd * e[1], sd * e[2]];
}
/* unwrap a/g in place, optional static zero offset over [n0,n1); returns max |b| per joint (gimbal check) */
function finishAngles(angles, nF, normRange) {
  const gimbal = {};
  for (const jn in angles) {
    const A = angles[jn];
    for (const c of ['a', 'g']) { let prev = NaN, off = 0; const arr = A[c];
      for (let f = 0; f < nF; f++) { let v = arr[f]; if (v !== v) continue; if (prev === prev) { const d = v + off - prev; if (d > 180) off -= 360 * Math.round(d / 360); else if (d < -180) off -= 360 * Math.round(d / 360); } v += off; arr[f] = v; prev = v; } }
    let mxb = 0; for (let f = 0; f < nF; f++) { const v = Math.abs(A.b[f]); if (v === v && v > mxb) mxb = v; } gimbal[jn] = mxb;
    if (normRange) for (const c of ['a', 'b', 'g']) { let s = 0, n = 0; for (let f = normRange[0]; f < normRange[1]; f++) { const v = A[c][f]; if (v === v) { s += v; n++; } } const m = n ? s / n : 0; for (let f = 0; f < nF; f++) A[c][f] -= m; }
  }
  return gimbal;
}
function angleRanges(angles, nF) {
  const ranges = {};
  for (const jn in angles) { ranges[jn] = {}; for (const c of ['a', 'b', 'g']) { let lo = Infinity, hi = -Infinity; for (let f = 0; f < nF; f++) { const v = angles[jn][c][f]; if (v === v) { if (v < lo) lo = v; if (v > hi) hi = v; } } ranges[jn][c] = [lo, hi]; } }
  return ranges;
}
/* rotation matrix (row-major) helpers for the solvers */
function expRot(w) {   // Rodrigues: exp([w]x)
  const th = Math.hypot(w[0], w[1], w[2]);
  if (th < 1e-12) return [1, -w[2], w[1], w[2], 1, -w[0], -w[1], w[0], 1];
  const k = [w[0] / th, w[1] / th, w[2] / th], c = Math.cos(th), s = Math.sin(th), v = 1 - c;
  return [c + k[0] * k[0] * v, k[0] * k[1] * v - k[2] * s, k[0] * k[2] * v + k[1] * s,
    k[1] * k[0] * v + k[2] * s, c + k[1] * k[1] * v, k[1] * k[2] * v - k[0] * s,
    k[2] * k[0] * v - k[1] * s, k[2] * k[1] * v + k[0] * s, c + k[2] * k[2] * v];
}
function frameAt(arr, f) {   // stored segment frame (origin + 3 columns) -> {o, R} or null
  const b = f * 12; if (arr[b] !== arr[b]) return null;
  return { o: [arr[b], arr[b + 1], arr[b + 2]], R: M3.cols([arr[b + 3], arr[b + 4], arr[b + 5]], [arr[b + 6], arr[b + 7], arr[b + 8]], [arr[b + 9], arr[b + 10], arr[b + 11]]) };
}
function frameput(arr, f, o, R) {
  const b = f * 12; arr[b] = o[0]; arr[b + 1] = o[1]; arr[b + 2] = o[2];
  for (let c = 0; c < 3; c++) { arr[b + 3 + c * 3] = R[c]; arr[b + 4 + c * 3] = R[3 + c]; arr[b + 5 + c * 3] = R[6 + c]; }
}
function solveLin(A, b) {   // dense Gaussian elimination with partial pivoting (small n)
  const n = b.length, M = A.map((r, i) => r.concat([b[i]]));
  for (let c = 0; c < n; c++) {
    let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-300) return null;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = c + 1; r < n; r++) { const q = M[r][c] / M[c][c]; if (q) for (let k = c; k <= n; k++) M[r][k] -= q * M[c][k]; }
  }
  const x = new Array(n);
  for (let r = n - 1; r >= 0; r--) { let s = M[r][n]; for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k]; x[r] = s / M[r][r]; }
  return x;
}

/* ---------------------------------------------------------------- Anthropometry estimation
   - Segment lengths: Winter (2009) / Drillis & Contini (1966) (Thigh 0.245*H, Shank 0.246*H, Foot 0.152*H)
   - Segment masses: De Leva (1996) percentages of total body mass
   - Knee/ankle widths: Rough empirical / clinical heuristics (~0.058*H male / ~0.055*H female,
     ~0.041*H male / ~0.038*H female). These are NOT published regressions from Winter or De Leva;
     they serve solely as rough initial placeholders when caliper measurements are unavailable.
     Direct measurement (calipers) is strongly recommended for accurate kinematics.
----------------------------------------------------------------------------------------- */
function estimateAnthropometry(heightM, massKg, sex) {
  const H = parseFloat(heightM);
  if (!isFinite(H) || H < 0.5 || H > 2.5) return null;
  const rawM = parseFloat(massKg);
  const M = isFinite(rawM) && rawM >= 10 && rawM <= 350 ? rawM : 70;
  const isF = String(sex || 'M').toUpperCase().startsWith('F');

  // Rough empirical heuristics (direct measurement recommended)
  const kneeW = isF ? 0.055 * H : 0.058 * H;
  const ankleW = isF ? 0.038 * H : 0.041 * H;
  const asisDist = isF ? 0.148 * H : 0.136 * H;

  // Published segment lengths (Winter 2009 / Drillis & Contini 1966)
  const thighL = 0.245 * H;
  const shankL = 0.246 * H;
  const footL = 0.152 * H;

  // Published segment mass ratios (De Leva 1996)
  const thighMass = M * (isF ? 0.1478 : 0.1416);
  const shankMass = M * (isF ? 0.0481 : 0.0433);
  const footMass = M * (isF ? 0.0129 : 0.0137);
  const pelvisMass = M * (isF ? 0.1247 : 0.1117);
  const trunkMass = M * (isF ? 0.4257 : 0.4346);

  return {
    heightM: H,
    massKg: M,
    sex: isF ? 'F' : 'M',
    kneeW: +kneeW.toFixed(4),
    ankleW: +ankleW.toFixed(4),
    asisDist: +asisDist.toFixed(4),
    isRoughEstimate: true,
    lengths: {
      thigh: +thighL.toFixed(4),
      shank: +shankL.toFixed(4),
      foot: +footL.toFixed(4)
    },
    masses: {
      thigh: +thighMass.toFixed(2),
      shank: +shankMass.toFixed(2),
      foot: +footMass.toFixed(2),
      pelvis: +pelvisMass.toFixed(2),
      trunk: +trunkMass.toFixed(2)
    }
  };
}

function computeLowerBody(ds, params) {
  const p = Object.assign({}, LB_DEFAULTS, params || {});
  const kw = s => (p.kneeW && typeof p.kneeW === 'object') ? p.kneeW[s] : p.kneeW, aw = s => (p.ankleW && typeof p.ankleW === 'object') ? p.ankleW[s] : p.ankleW;
  const sds = p.staticDs || ds;               // separate static trial (optional); markers matched by name
  const setId = (p.markerSet && LB_SETS[p.markerSet]) ? p.markerSet : detectLBSet(ds);
  if (!setId) {
    const have = new Set(ds.base.map(b => b.toUpperCase()));
    return { ok: false, error: 'ไม่พบ marker: ' + LB_MARKERS.filter(n => !have.has(n)).join(', ') + ' · (รองรับชุด Conventional Lower 16 หรือชุด 18 ตัวที่มี KNEM/ANKM ไม่มี TIB)' };
  }
  const MK = LB_SETS[setId].markers, REAL_MEDIAL = LB_SETS[setId].medial, ROBUST = setId === 'bad18';
  const mkIdx = d => { const o = {}, miss = []; for (const n of MK) { const i = d.base.findIndex(b => b.toUpperCase() === n); if (i < 0) miss.push(n); o[n] = i; } return { o, miss }; };
  const di = mkIdx(ds), si = mkIdx(sds), idx = di.o, sidx = si.o;
  if (di.miss.length) return { ok: false, error: 'ไม่พบ marker: ' + di.miss.join(', ') };
  if (si.miss.length) return { ok: false, error: 'ไฟล์ static ไม่พบ marker: ' + si.miss.join(', ') };
  const nF = ds.nFrames, nM = ds.nM, warnings = [];
  const sameStatic = sds === ds;
  if (!sameStatic && Math.abs(sds.rate - ds.rate) > 1e-6) warnings.push('อัตราสุ่มตัวอย่างของ static (' + sds.rate + ' Hz) ต่างจาก dynamic (' + ds.rate + ' Hz) (ไม่กระทบการคำนวณมุม)');
  const at = (n, f) => { const k = f * nM + idx[n]; return ds.vis[k] ? [ds.pts[k * 3], ds.pts[k * 3 + 1], ds.pts[k * 3 + 2]] : null; };
  const atS = (n, f) => { const k = f * sds.nM + sidx[n]; return sds.vis[k] ? [sds.pts[k * 3], sds.pts[k * 3 + 1], sds.pts[k * 3 + 2]] : null; };
  /* static mean + stability */
  const s0 = clamp(p.staticStart | 0, 0, sds.nFrames - 1), s1 = Math.min(sds.nFrames, s0 + Math.max(1, p.staticN | 0));
  const S = {}, staticMove = {};
  for (const n of MK) {
    const sum = [0, 0, 0]; let c = 0;
    for (let f = s0; f < s1; f++) { const q = atS(n, f); if (q) { sum[0] += q[0]; sum[1] += q[1]; sum[2] += q[2]; c++; } }
    if (!c) return { ok: false, error: 'marker ' + n + ' ไม่มีข้อมูลในช่วง static' };
    S[n] = [sum[0] / c, sum[1] / c, sum[2] / c];
    let mx = 0; for (let f = s0; f < s1; f++) { const q = atS(n, f); if (q) mx = Math.max(mx, Math.hypot(q[0] - S[n][0], q[1] - S[n][1], q[2] - S[n][2])); }
    staticMove[n] = mx;
  }
  const maxMove = Math.max(...Object.values(staticMove));
  if (maxMove > 0.005) warnings.push('marker ขยับในช่วง static สูงสุด ' + (maxMove * 1000).toFixed(1) + ' mm (>5 mm) ช่วง static อาจไม่นิ่ง');
  /* static pelvis + landmarks */
  const ML = V.norm(V.sub(S.RASI, S.LASI)), am = V.mid(S.RASI, S.LASI), sacrum = V.mid(S.RPSI, S.LPSI), D = Math.hypot(...V.sub(S.RASI, S.LASI));
  const apRaw = V.sub(am, sacrum), AP = V.norm(V.sub(apRaw, V.mul(ML, V.dot(apRaw, ML)))), AX = V.cross(ML, AP);
  const L = {};
  L.RHJC = V.add(am, V.add(V.mul(ML, 0.36 * D), V.add(V.mul(AP, -0.19 * D), V.mul(AX, -0.30 * D))));
  L.LHJC = V.add(am, V.add(V.mul(ML, -0.36 * D), V.add(V.mul(AP, -0.19 * D), V.mul(AX, -0.30 * D))));
  let measured = null;
  if (REAL_MEDIAL) {   // real medial markers: joint centre = midpoint lateral/medial marker
    L.RKNM = S.RKNEM; L.LKNM = S.LKNEM; L.RANM = S.RANKM; L.LANM = S.LANKM;
    const gap = (a, b) => Math.hypot(...V.sub(S[a], S[b])) - 2 * p.markerR;
    measured = { kneeW: { R: gap('RKNE', 'RKNEM'), L: gap('LKNE', 'LKNEM') }, ankleW: { R: gap('RANK', 'RANKM'), L: gap('LANK', 'LANKM') } };
    for (const s of ['R', 'L']) {   // medial marker must lie on the medial side of the lateral one
      if (V.dot(V.sub(S[s + 'KNE'], S[s + 'KNEM']), ML) * (s === 'R' ? 1 : -1) <= 0) warnings.push(s + 'KNEM ไม่อยู่ด้านใน (medial) ของ ' + s + 'KNE ในช่วง static ตรวจ label');
      if (V.dot(V.sub(S[s + 'ANK'], S[s + 'ANKM']), ML) * (s === 'R' ? 1 : -1) <= 0) warnings.push(s + 'ANKM ไม่อยู่ด้านใน (medial) ของ ' + s + 'ANK ในช่วง static ตรวจ label');
    }
  } else {
    L.RKNM = V.add(S.RKNE, V.mul(ML, -(kw('R') + 2 * p.markerR))); L.LKNM = V.add(S.LKNE, V.mul(ML, (kw('L') + 2 * p.markerR)));
    L.RANM = V.add(S.RANK, V.mul(ML, -(aw('R') + 2 * p.markerR))); L.LANM = V.add(S.LANK, V.mul(ML, (aw('L') + 2 * p.markerR)));
  }
  L.RKJC = V.mid(S.RKNE, L.RKNM); L.LKJC = V.mid(S.LKNE, L.LKNM); L.RAJC = V.mid(S.RANK, L.RANM); L.LAJC = V.mid(S.LANK, L.LANM);
  /* segment definitions: tracking set (markers or landmarks), static anatomical frame */
  const frameFrom = (prox, dist, zRef) => {
    const Y = V.norm(V.sub(prox, dist)); let Z = V.sub(zRef, V.mul(Y, V.dot(zRef, Y))); Z = V.norm(Z); return { o: prox, R: M3.cols(V.cross(Y, Z), Y, Z) };
  };
  const SEG = {};
  SEG.RPV = { track: ['RASI', 'LASI', 'RPSI', 'LPSI'], frame: { o: V.mid(L.RHJC, L.LHJC), R: M3.cols(AP, AX, ML) } };
  /* tracking sets. 'anat' (default): every tracking point moves with its segment, joint centres chained from the parent pose
     (as HJC from the pelvis): thigh HJC + THI + KNE (+ KNEM, femoral epicondyles); shank KJC(thigh) + ANK (+ ANKM, malleoli) or TIB;
     foot AJC(shank) + HEE + TOE.  'mixed' (v0.2 behaviour): shank also tracked by the femoral epicondyles (KNE/KNEM) and foot by the
     malleoli (ANK/ANKM), which are not rigid with those segments (soft tissue + joint rotation). */
  const ANAT = p.tracking !== 'mixed';
  for (const s of ['R', 'L']) {
    const rs = s === 'R' ? 1 : -1;
    SEG[s + 'TH'] = { track: ANAT && REAL_MEDIAL ? [s + 'HJC', s + 'KNE', s + 'KNEM', s + 'THI'] : [s + 'HJC', s + 'KNE', s + 'THI'], frame: frameFrom(L[s + 'HJC'], L[s + 'KJC'], V.mul(V.sub(S[s + 'KNE'], L[s + 'KNM']), rs)) };
    SEG[s + 'SK'] = { track: ANAT ? (REAL_MEDIAL ? [s + 'KJC', s + 'ANK', s + 'ANKM'] : [s + 'KJC', s + 'TIB', s + 'ANK']) : (REAL_MEDIAL ? [s + 'KNE', s + 'KNEM', s + 'ANK', s + 'ANKM'] : [s + 'TIB', s + 'ANK', s + 'KNE']), frame: frameFrom(L[s + 'KJC'], L[s + 'AJC'], V.mul(V.sub(S[s + 'ANK'], L[s + 'ANM']), rs)) };
    SEG[s + 'FT'] = { track: ANAT ? [s + 'AJC', s + 'HEE', s + 'TOE'] : (REAL_MEDIAL ? [s + 'HEE', s + 'TOE', s + 'ANK', s + 'ANKM'] : [s + 'HEE', s + 'TOE', s + 'ANK']), frame: frameFrom(L[s + 'AJC'], S[s + 'TOE'], V.mul(V.sub(S[s + 'ANK'], L[s + 'AJC']), rs)) };
  }
  // names: thigh/shank/foot tracking uses SEG keys RTH,RSK,RFT / LTH,LSK,LFT; fix keys
  const fix = { RTH: 'RTH', RSK: 'RSK', RFT: 'RFT', LTH: 'LTH', LSK: 'LSK', LFT: 'LFT', RPV: 'RPV' };
  const segNames = Object.keys(fix);
 /* ---- Plug-in Gait style convention (bad18 only): segment axes X anterior, Y left, Z proximal(up); angles Euler Y-X-Z.
        Shank has no TIB marker: a virtual TIB (midpoint of lateral KNE-ANK in static) fixes the shank frontal plane, as the real TIB does in PiG.
        The virtual TIB is not an independent measurement; shank pose comes from the robust fit of KNE/KNEM/ANK/ANKM. */
  const PIG = p.convention === 'pig';
  if (PIG && !REAL_MEDIAL) return { ok: false, error: 'โหมด Plug-in Gait style ใช้ได้กับชุด 18 marker (KNEM/ANKM) เท่านั้น' };
  let pigOffset = null;
  if (PIG) {
    const orth = (v, z) => V.norm(V.sub(v, V.mul(z, V.dot(v, z))));
    const mk = (o, z, leftRef) => { const Y = orth(leftRef, z); return { o, R: M3.cols(V.cross(Y, z), Y, z) }; };
    const Ypel = V.norm(V.sub(S.LASI, S.RASI)), Xpel = V.norm(V.sub(apRaw, V.mul(Ypel, V.dot(apRaw, Ypel))));
    SEG.RPV.frame = { o: V.mid(L.RHJC, L.LHJC), R: M3.cols(Xpel, Ypel, V.cross(Xpel, Ypel)) };
    L.RTIBv = V.mid(S.RKNE, S.RANK); L.LTIBv = V.mid(S.LKNE, S.LANK);
    for (const s of ['R', 'L']) {
      const R_ = s === 'R', lat = (a, b) => V.sub(R_ ? S[b] : S[a], R_ ? S[a] : S[b]);   // pointing to subject's left
      SEG[s + 'TH'].frame = mk(L[s + 'HJC'], V.norm(V.sub(L[s + 'HJC'], L[s + 'KJC'])), lat(s + 'KNE', s + 'KNEM'));
      const l = V.sub(L[s + 'TIBv'], V.mid(L[s + 'KJC'], L[s + 'AJC']));
      SEG[s + 'SK'].frame = mk(L[s + 'KJC'], V.norm(V.sub(L[s + 'KJC'], L[s + 'AJC'])), R_ ? V.mul(l, -1) : l);
      SEG[s + 'FT'].frame = mk(L[s + 'AJC'], V.norm(V.sub(L[s + 'AJC'], S[s + 'TOE'])), lat(s + 'ANK', s + 'ANKM'));
    }
    // static plantar/dorsiflexion offset of the foot (as PiG's static calibration): foot-vs-shank angle in the static pose
    pigOffset = {};
    for (const s of ['R', 'L']) pigOffset[s] = cardanYXZ(M3.mul(M3.T(SEG[s + 'SK'].frame.R), SEG[s + 'FT'].frame.R))[0];
  }
  const stat = n => (n in S ? S[n] : L[n]);
  for (const k of segNames) SEG[k].A = SEG[k].track.map(stat);
  /* static frame sanity: Y axis vs lab up */
  const upTilt = {};
  for (const k of segNames) { const Y = M3.col(SEG[k].frame.R, 1); upTilt[k] = Math.acos(clamp(Y[2], -1, 1)) / D2R; }
  /* dynamic */
  const out = { frames: {}, jc: {}, angles: {} };
  for (const k of segNames) out.frames[k] = new Float32Array(nF * 12).fill(NaN);
  for (const j of ['RHJC', 'LHJC', 'RKJC', 'LKJC', 'RAJC', 'LAJC']) out.jc[j] = new Float32Array(nF * 3).fill(NaN);
  const pose = {}; let skipped = 0;
  const fitStats = {}; for (const k of segNames) fitStats[k] = { sum: 0, n: 0, max: 0, short: 0, dropped: {} };
  for (let f = 0; f < nF; f++) {
    const cur = {}; let allOK = true;
    for (const n of MK) { cur[n] = at(n, f); }
    pose.RPV = null;
    const fitSeg = (k, getter) => {
      const nm = [], Ai = [], B = [];
      SEG[k].track.forEach((n, i) => { const q = getter(n); if (q) { nm.push(n); Ai.push(SEG[k].A[i]); B.push(q); } });
      if (!ROBUST) {
        if (nm.length === SEG[k].track.length) return rigidFit(SEG[k].A, B);
        if (!ANAT || nm.length < 3) return null;   // anatomical mode: a segment with >= 3 of its points still tracks (e.g. a pelvis marker lost)
        fitStats[k].partial = (fitStats[k].partial || 0) + 1; return rigidFit(Ai, B);
      }
      if (nm.length < 3) { fitStats[k].short++; return null; }
      const run = (a, b) => { const T = rigidFit(a, b); let ss = 0; for (let i = 0; i < a.length; i++) { const r = V.add(M3.vec(T.R, a[i]), T.t), d = V.sub(r, b[i]); ss += d[0] * d[0] + d[1] * d[1] + d[2] * d[2]; } return { T, rms: Math.sqrt(ss / a.length) }; };
      let best = run(Ai, B);
      if (nm.length >= 4 && best.rms > p.fitTol) {   // leave-one-out: drop the marker whose removal fits best
        let alt = null, drop = -1;
        for (let j = 0; j < nm.length; j++) { const r = run(Ai.filter((_, i) => i !== j), B.filter((_, i) => i !== j)); if (!alt || r.rms < alt.rms) { alt = r; drop = j; } }
        if (alt.rms < best.rms * 0.6) { best = alt; fitStats[k].dropped[nm[drop]] = (fitStats[k].dropped[nm[drop]] || 0) + 1; }
      }
      fitStats[k].sum += best.rms; fitStats[k].n++; if (best.rms > fitStats[k].max) fitStats[k].max = best.rms;
      return best.T;
    };
    const T = {};
    T.RPV = fitSeg('RPV', n => cur[n]);
    const lm = {};
    if (T.RPV) { lm.RHJC = M3.vec(T.RPV.R, L.RHJC); lm.RHJC = V.add(lm.RHJC, T.RPV.t); lm.LHJC = V.add(M3.vec(T.RPV.R, L.LHJC), T.RPV.t); }
    for (const s of ['R', 'L']) {   // landmark points (joint centres) come from the parent segment's pose of this frame
      const g = n => (n in lm || n.endsWith('JC') ? (lm[n] || null) : cur[n]);
      T[s + 'TH'] = fitSeg(s + 'TH', g);
      if (T[s + 'TH']) lm[s + 'KJC'] = V.add(M3.vec(T[s + 'TH'].R, L[s + 'KJC']), T[s + 'TH'].t);
      T[s + 'SK'] = fitSeg(s + 'SK', g);
      if (T[s + 'SK']) lm[s + 'AJC'] = V.add(M3.vec(T[s + 'SK'].R, L[s + 'AJC']), T[s + 'SK'].t);
      T[s + 'FT'] = fitSeg(s + 'FT', g);
    }
    const RA = {};
    for (const k of segNames) {
      const t = T[k]; if (!t) continue;
      const Ra = M3.mul(t.R, SEG[k].frame.R), o = V.add(M3.vec(t.R, SEG[k].frame.o), t.t);
      RA[k] = Ra; const b = f * 12;
      out.frames[k][b] = o[0]; out.frames[k][b + 1] = o[1]; out.frames[k][b + 2] = o[2];
      for (let c = 0; c < 3; c++) { const col = M3.col(Ra, c); out.frames[k][b + 3 + c * 3] = col[0]; out.frames[k][b + 4 + c * 3] = col[1]; out.frames[k][b + 5 + c * 3] = col[2]; }
    }
    const put = (name, v) => { out.jc[name][f * 3] = v[0]; out.jc[name][f * 3 + 1] = v[1]; out.jc[name][f * 3 + 2] = v[2]; };
    if (lm.RHJC) { put('RHJC', lm.RHJC); put('LHJC', lm.LHJC); }
    for (const s of ['R', 'L']) {
      if (T[s + 'TH']) put(s + 'KJC', V.add(M3.vec(T[s + 'TH'].R, L[s + 'KJC']), T[s + 'TH'].t));
      if (T[s + 'SK']) put(s + 'AJC', V.add(M3.vec(T[s + 'SK'].R, L[s + 'AJC']), T[s + 'SK'].t));
    }
    pose[f] = null;
    for (const [jn, sg, rf] of LB_JOINTS) {
      if (!out.angles[jn]) out.angles[jn] = { a: new Float32Array(nF).fill(NaN), b: new Float32Array(nF).fill(NaN), g: new Float32Array(nF).fill(NaN) };
      if (!RA[sg] || !RA[rf]) { if (jn === 'RHip') skipped++; continue; }
      const e = jointEuler(jn, RA[rf], RA[sg], PIG, pigOffset);
      out.angles[jn].a[f] = e[0]; out.angles[jn].b[f] = e[1]; out.angles[jn].g[f] = e[2];
    }
  }
  /* unwrap + normalise */
  const gimbal = finishAngles(out.angles, nF, p.normalize && sameStatic ? [s0, s1] : null);
  if (ROBUST) for (const k of segNames) {
    const st = fitStats[k], dr = Object.entries(st.dropped);
    for (const [m, c] of dr) warnings.push(k + ': ตัด ' + m + ' ออกจากการ fit ' + c + ' frame (ระยะไม่คงที่ > ' + (p.fitTol * 1000).toFixed(0) + ' mm จาก static) · ตรวจ label/ตำแหน่ง marker');
    if (st.short) warnings.push(k + ': marker ไม่พอ (<3) ' + st.short + ' frame ไม่มีค่ามุม');
  }
  for (const jn in gimbal) if (gimbal[jn] > 75) warnings.push(jn + ': มุมแกนกลาง (ab/adduction) สูงสุด ' + gimbal[jn].toFixed(0) + '° ใกล้ gimbal lock ค่ามุมสามแกนอาจไม่น่าเชื่อถือ');
  const ranges = angleRanges(out.angles, nF);
  const fitSummary = {}; for (const k of segNames) { const st = fitStats[k]; fitSummary[k] = { meanRms: st.n ? st.sum / st.n : NaN, maxRms: st.max, dropped: st.dropped, short: st.short }; }
  /* static calibration kept for inverse kinematics / inverse dynamics (anatomical frames in the static pose, static markers + landmarks) */
  const model = { setId, PIG, pigOffset, S, L, seg: {}, normRange: p.normalize && sameStatic ? [s0, s1] : null };
  for (const k of segNames) model.seg[k] = { o: SEG[k].frame.o.slice(), R: SEG[k].frame.R.slice() };
  return { ok: true, source: '6dof', tracking: ANAT ? 'anat' : 'mixed', convention: PIG ? 'pig' : 'zxy', pigOffset, markerSet: setId, markerSetLabel: LB_SETS[setId].label, staticSource: sameStatic ? 'trial' : (sds.name || 'static file'), measured, fitSummary, params: Object.assign({}, p, { staticDs: undefined }), anthro: p.anthro || null, warnings, angles: out.angles, frames: out.frames, jc: out.jc, ranges, upTilt, staticMove: maxMove, landmarks: L, staticRange: [s0, s1 - 1], asisDistance: D, segNames, model };
}

/* ---------------------------------------------------------------- Upper body (thorax, head, arms) · ISB, Wu et al. 2005
   Markers (Plug-in Gait upper body + medial epicondyles): head LFHD RFHD LBHD RBHD; thorax C7 T10 CLAV STRN; per arm SHO UPA ELB
   ELBM FRM WRA WRB FIN; pelvis RASI LASI RPSI LPSI optional (trunk angle). All segment frames: X anterior, Y superior/proximal,
   Z right (left limbs too, so left angles are mirrored where noted).
   Landmarks from the static: thorax IJ = CLAV, PX = STRN, T8 ≈ T10; shoulder joint centre (GH) 17 % of the inter-acromion
   distance below the acromion marker along the thorax long axis (Rab et al. 2002); elbow centre = mid ELB/ELBM; wrist centre =
   mid WRA/WRB (radial / ulnar styloid); hand = wrist centre → FIN.
   Tracking (anatomically consistent, as the lower body): thorax CLAV STRN C7 T10; head 4 markers; humerus UPA ELB ELBM (GH fixed
   in the humerus, CAST); forearm built directly each frame from the elbow centre (humerus chain) and both styloids, because the
   forearm is not rigid (static-cluster fit of EJC WRA WRB FRM only as fallback); hand: wrist centre (from the forearm) + FIN with
   the forearm's axial orientation (one hand marker: wrist flexion/deviation only, no axial rotation).
   Angles: thorax vs global (lab vertical, forward = static thorax facing), trunk = thorax vs pelvis, neck = head vs thorax:
   Cardan Z-X-Y, a = flexion(+), b = lateral bend to the right(+), g = axial rotation to the left(+).
   Shoulder (humerus vs thorax) Euler Y-X-Y (ISB): a = plane of elevation (0 = abduction, 90 = forward flexion), b = elevation(+),
   g = axial rotation, internal(+); undefined near 0 elevation (gimbal lock).
   Elbow (forearm vs humerus) Z-X-Y: a = flexion(+), b = varus(+) (carrying angle), g = pronation(+).
   Wrist (hand vs forearm) Z-X-Y: a = flexion(+) / extension(−), b = ulnar deviation(+). Left b and g mirrored to the same meaning. */
const UB_MARKERS = { thorax: ['C7', 'T10', 'CLAV', 'STRN'], head: ['LFHD', 'RFHD', 'LBHD', 'RBHD'], arm: ['SHO', 'UPA', 'ELB', 'ELBM', 'FRM', 'WRA', 'WRB'] };   // FIN optional (wrist angle)
/* ---------------------------------------------------------------- marker-set definitions (config) + name mapping
   Which marker names each model reads. The segment geometry that uses them stays in code
   (computeLowerBody / computeUpperBody); this table drives detection, the mapping UI and reports. */
const MODEL_DEFS = {
  lb16: { label: 'Lower body · Conventional 16 (TIB wand)', required: LB_MARKERS.slice(), optional: [] },
  lb18: { label: 'Lower body · 18 (KNEM/ANKM, no TIB)', required: ['LASI', 'RASI', 'LPSI', 'RPSI', 'LKNE', 'RKNE', 'LKNEM', 'RKNEM', 'LTHI', 'RTHI', 'LANK', 'RANK', 'LANKM', 'RANKM', 'LTOE', 'RTOE', 'LHEE', 'RHEE'], optional: [] },
  ub: { label: 'Upper body · ISB (thorax + arms)', required: UB_MARKERS.thorax.concat(...['R', 'L'].map(s => UB_MARKERS.arm.map(m => s + m))), optional: UB_MARKERS.head.concat(['RFIN', 'LFIN']), anySide: true }
};
/* Common alternative names (side removed). Keys are model stems; values are other stems seen in labs / other marker sets. */
const MARKER_ALIASES = {
  ASI: ['ASIS', 'ASIS', 'AS'], PSI: ['PSIS', 'PS'], THI: ['THIGH', 'THG', 'TH'], KNE: ['KNEE', 'KNEELAT', 'LATKNEE', 'LKNEE', 'LFE', 'FLE', 'LEPI', 'LFC', 'KNL'],
  KNEM: ['MKNE', 'KNEEMED', 'MEDKNEE', 'MKNEE', 'MFE', 'FME', 'MEPI', 'MFC', 'KNM'], TIB: ['TIBIA', 'SHANK', 'SHN', 'TBA', 'TIBIAL'],
  ANK: ['ANKLE', 'LMAL', 'LMALL', 'LATMAL', 'LM', 'MALL', 'FAL', 'ANKLAT'], ANKM: ['MANK', 'MMAL', 'MMALL', 'MEDMAL', 'MM', 'TAM', 'ANKMED'],
  TOE: ['MT2', 'MTH2', 'MH2', 'FM2', 'TOE2', 'M2', 'MET2'], HEE: ['HEEL', 'CAL', 'CALC', 'FCC', 'HE'],
  SHO: ['ACR', 'ACROMION', 'SHOULDER', 'SAE', 'AC'], UPA: ['ARM', 'UARM', 'HUM', 'UPPERARM'], ELB: ['ELBOW', 'LEL', 'ELBLAT', 'LATELB', 'EPL', 'LHE', 'ELL'],
  ELBM: ['MELB', 'MEL', 'ELBMED', 'MEDELB', 'EPM', 'MHE', 'ELM'], FRM: ['FOREARM', 'FARM', 'FRA'], WRA: ['RS', 'RSP', 'WRR', 'WRIRAD', 'RADST', 'WRAD'],
  WRB: ['US', 'USP', 'WRU', 'WRIULN', 'ULNST', 'WULN'], FIN: ['MCP2', 'MC2', 'HAND', 'MH2', 'MCP', 'FIN2'], FHD: ['FHEAD', 'FRONTHEAD', 'HEADF'], BHD: ['BHEAD', 'BACKHEAD', 'HEADB'],
  C7: ['CV7', 'VC7', 'C07'], T10: ['TV10', 'T010'], CLAV: ['IJ', 'JN', 'SJN', 'CLA', 'MAN', 'STRNU'], STRN: ['PX', 'XP', 'XIPH', 'STER', 'SXS', 'STRNL']
};
/* label -> { side: 'L' | 'R' | '', stem } with side cues at the start or end (LASI, L_ASIS, ASIS_L, Left ASIS, l asis) */
function markerStem(label) {
  let b = splitLabel(String(label)).base.trim().toUpperCase();
  b = b.split(/[-]/).length > 1 && /^[A-Z_ ]+-/.test(b) && b.length > 6 ? b.slice(b.lastIndexOf('-') + 1) : b;   // "Bust-ASI" style group prefixes
  let side = '';
  let m;
  if ((m = b.match(/^(LEFT|RIGHT)[\s_.\-]*/))) { side = m[1][0]; b = b.slice(m[0].length); }
  else if ((m = b.match(/[\s_.\-]+(L|R|LEFT|RIGHT)$/))) { side = m[1][0]; b = b.slice(0, b.length - m[0].length); }
  else if ((m = b.match(/^(L|R)[\s_.\-]+/))) { side = m[1]; b = b.slice(m[0].length); }
  const stem = b.replace(/[\s_.\-]/g, '');
  if (!side && /^(L|R)[A-Z]/.test(stem) && stem.length >= 3) {
    const rest = stem.slice(1);
    if (MARKER_ALIASES[rest] || Object.values(MARKER_ALIASES).some(v => v.includes(rest)) || !MARKER_ALIASES[stem]) return { side: stem[0], stem: rest };
  }
  return { side, stem };
}
const canonStem = n => { const S = /^(L|R)/.test(n) && !MARKER_ALIASES[n] ? n.slice(1) : n; return { side: S === n ? '' : n[0], stem: S }; };
/* suggest {modelName: fileBase} for the model markers missing in ds */
function suggestMarkerMap(ds, modelIds) {
  const have = new Set(ds.base.map(b => b.toUpperCase()));
  const used = new Set();
  const parsed = ds.base.map((b, i) => ({ i, b, s: markerStem(b) }));
  const out = {};
  for (const id of modelIds || Object.keys(MODEL_DEFS)) {
    const D = MODEL_DEFS[id];
    for (const n of D.required.concat(D.optional)) {
      if (have.has(n) || out[n]) continue;
      const c = canonStem(n), alts = [c.stem].concat(MARKER_ALIASES[c.stem] || []);
      const hit = parsed.find(p => !used.has(p.i) && !have.has(n) && p.s.side === c.side && alts.includes(p.s.stem) && !(MODEL_DEFS.lb16.required.concat(MODEL_DEFS.lb18.required, MODEL_DEFS.ub.required, MODEL_DEFS.ub.optional).includes(p.b.toUpperCase())));
      if (hit) { out[n] = ds.baseFile ? ds.baseFile[hit.i] : hit.b; used.add(hit.i); }
    }
  }
  return out;
}
/* per model: which names are present / missing (after mapping) */
function markerReport(ds) {
  const have = new Set(ds.base.map(b => b.toUpperCase()));
  const R = {};
  for (const id in MODEL_DEFS) {
    const D = MODEL_DEFS[id];
    let req = D.required;
    if (D.anySide) {   // upper body: thorax + at least one complete arm
      const sides = ['R', 'L'].filter(s => UB_MARKERS.arm.every(m => have.has(s + m)));
      req = UB_MARKERS.thorax.concat(...(sides.length ? sides : ['R', 'L']).map(s => UB_MARKERS.arm.map(m => s + m)));
    }
    const missing = req.filter(n => !have.has(n));
    R[id] = { label: D.label, n: req.length, found: req.length - missing.length, missing, optionalMissing: D.optional.filter(n => !have.has(n)), ok: !missing.length };
  }
  return R;
}
const UB_JOINTS = ['Thorax', 'Trunk', 'Neck', 'RShoulder', 'RElbow', 'RWrist', 'LShoulder', 'LElbow', 'LWrist'];
const UB_LABELS = {
  Thorax: ['flexion(+)', 'lat. bend R(+)', 'axial rot L(+)'], Trunk: ['flexion(+)', 'lat. bend R(+)', 'axial rot L(+)'], Neck: ['flexion(+)', 'lat. bend R(+)', 'axial rot L(+)'],
  Shoulder: ['plane of elevation', 'elevation(+)', 'axial rot int(+)'], Elbow: ['flexion(+)', 'varus(+)', 'pronation(+)'], Wrist: ['flexion(+)/ext', 'ulnar dev(+)', 'axial (≈0)']
};
function detectUB(ds) {
  const have = new Set(ds.base.map(b => b.toUpperCase()));
  const sides = ['R', 'L'].filter(s => UB_MARKERS.arm.every(m => have.has(s + m)));
  return { thorax: UB_MARKERS.thorax.every(m => have.has(m)), head: UB_MARKERS.head.every(m => have.has(m)), sides, fin: sides.filter(s => have.has(s + 'FIN')), pelvis: ['RASI', 'LASI', 'RPSI', 'LPSI'].every(m => have.has(m)) };
}
const frameXYZ = (o, X, Y, Z) => ({ o, R: M3.cols(X, Y, Z) });
/* frame from a long axis Y and a vector pointing roughly right (Z): X = Y × Zr, Z = X × Y */
function frameYZ(o, Y, Zr) { const y = V.norm(Y), x = V.norm(V.cross(y, Zr)), z = V.cross(x, y); return frameXYZ(o, x, y, z); }
function eulerYXYr(R) {   // R = Ry(a) Rx(b) Ry(g); b in [0, π]
  const b = Math.acos(clamp(R[4], -1, 1)), a = Math.atan2(R[1], R[7]), g = Math.atan2(R[3], -R[5]);
  return [a, b, g];
}
function computeUpperBody(ds, opt) {
  opt = Object.assign({ staticStart: 0, staticN: 10, normalize: false, fitTol: 0.015 }, opt || {});
  const det = detectUB(ds);
  if (!det.thorax || !det.sides.length) return { ok: false, error: 'ต้องมี marker thorax (C7 T10 CLAV STRN) และแขนอย่างน้อย 1 ข้าง (SHO UPA ELB ELBM FRM WRA WRB; FIN ถ้าต้องการมุมข้อมือ)' };
  const sds = opt.staticDs || ds, sameStatic = sds === ds, nF = ds.nFrames, warnings = [];
  const sdet = detectUB(sds);
  if (!sdet.thorax || det.sides.some(s => !sdet.sides.includes(s))) return { ok: false, error: 'ไฟล์ static ไม่มี marker ส่วนบนครบ' };
  const useHead = det.head && sdet.head, usePelvis = det.pelvis && sdet.pelvis;
  const idx = d => { const m = {}; d.base.forEach((b, i) => { m[b.toUpperCase()] = i; }); return m; };
  const di = idx(ds), si = idx(sds);
  const get = (d, I, n, f) => { const i = I[n]; if (i == null) return null; const k = f * d.nM + i; return d.vis[k] ? [d.pts[k * 3], d.pts[k * 3 + 1], d.pts[k * 3 + 2]] : null; };
  /* optional label correction in the trial: radial / ulnar styloid (WRA / WRB) swapped on one side (opt.swapWrist = { R, L }) */
  for (const s of ['R', 'L']) if (opt.swapWrist && opt.swapWrist[s] && di[s + 'WRA'] != null && di[s + 'WRB'] != null) { const t = di[s + 'WRA']; di[s + 'WRA'] = di[s + 'WRB']; di[s + 'WRB'] = t; warnings.push(s + 'WRA/' + s + 'WRB สลับ label ใน trial นี้ (ตามที่เลือก)'); }
  /* static averages */
  const s0 = clamp(opt.staticStart | 0, 0, sds.nFrames - 1), s1 = Math.min(sds.nFrames, s0 + Math.max(1, opt.staticN | 0));
  const S = {}, need = [...UB_MARKERS.thorax, ...(useHead ? UB_MARKERS.head : []), ...(usePelvis ? ['RASI', 'LASI', 'RPSI', 'LPSI'] : [])];
  const finSides = det.fin.filter(s => sdet.fin.includes(s));
  for (const s of det.sides) for (const m of UB_MARKERS.arm) need.push(s + m);
  for (const s of finSides) need.push(s + 'FIN');
  for (const s of det.sides) if (!finSides.includes(s)) warnings.push(s + ': ไม่มี marker ' + s + 'FIN (เช่นมือที่จับไม้) · ไม่คำนวณมุมข้อมือข้างนี้');
  for (const n of need) { const a = [0, 0, 0]; let c = 0; for (let f = s0; f < s1; f++) { const p = get(sds, si, n, f); if (p) { a[0] += p[0]; a[1] += p[1]; a[2] += p[2]; c++; } } if (!c) return { ok: false, error: 'static ไม่มีข้อมูล ' + n }; S[n] = a.map(v => v / c); }
  const mid = (a, b) => V.mul(V.add(a, b), 0.5);
  const trunkLike = e => [-e[0], e[1], e[2]];   // Z rotation of a superior-pointing Y: forward lean is negative about Z -> flexion(+)
  /* static anatomical frames + landmarks */
  const thoraxFrame = (P) => { const up = V.sub(mid(P.CLAV, P.C7), mid(P.STRN, P.T10)), zr = V.cross(V.sub(P.C7, P.CLAV), V.sub(mid(P.STRN, P.T10), P.CLAV)); return frameYZ(P.CLAV, up, zr); };
  const F0 = {}, L = {};
  F0.TX = thoraxFrame(S);
  const yTx = M3.col(F0.TX.R, 1), dAA = det.sides.length === 2 ? Math.hypot(...V.sub(S.RSHO, S.LSHO)) : null;
  if (!dAA) warnings.push('มีแขนข้างเดียว: ใช้ระยะ acromion–acromion ประมาณ 0.35 m สำหรับ shoulder joint centre');
  for (const s of det.sides) {
    const zr = s === 'R' ? 1 : -1;
    L[s + 'SJC'] = V.sub(S[s + 'SHO'], V.mul(yTx, 0.17 * (dAA || 0.35)));
    L[s + 'EJC'] = mid(S[s + 'ELB'], S[s + 'ELBM']);
    L[s + 'WJC'] = mid(S[s + 'WRA'], S[s + 'WRB']);
    F0[s + 'UA'] = frameYZ(L[s + 'SJC'], V.sub(L[s + 'SJC'], L[s + 'EJC']), V.mul(V.sub(S[s + 'ELB'], S[s + 'ELBM']), zr));
    F0[s + 'FA'] = frameYZ(L[s + 'WJC'], V.sub(L[s + 'EJC'], L[s + 'WJC']), V.mul(V.sub(S[s + 'WRA'], S[s + 'WRB']), zr));
  }
  /* head: tracked by its 4 markers; its anatomical orientation is set equal to the thorax in the static (head offset, as Plug-in
     Gait), because head-band markers are rarely symmetric: neck angles are relative to the static (neutral) head posture */
  let headOffset = null;
  if (useHead) {
    const o = mid(mid(S.LFHD, S.RFHD), mid(S.LBHD, S.RBHD)), zr = V.sub(mid(S.RFHD, S.RBHD), mid(S.LFHD, S.LBHD)), xf = V.sub(mid(S.LFHD, S.RFHD), mid(S.LBHD, S.RBHD));
    const z = V.norm(zr), x = V.norm(V.sub(xf, V.mul(z, V.dot(xf, z)))), Rm = M3.cols(x, V.cross(z, x), z);
    headOffset = trunkLike(cardanZXY(M3.mul(M3.T(F0.TX.R), Rm)));
    F0.HD = { o, R: F0.TX.R.slice() };
  }
  if (usePelvis) { const o = mid(S.RASI, S.LASI), z = V.norm(V.sub(S.RASI, S.LASI)), xf = V.sub(o, mid(S.RPSI, S.LPSI)), x = V.norm(V.sub(xf, V.mul(z, V.dot(xf, z)))); F0.PV = frameXYZ(o, x, V.cross(z, x), z); }
  /* global reference: lab vertical, forward = static thorax facing projected on the floor */
  const up = [0, 0, 1], fx = M3.col(F0.TX.R, 0), gx = V.norm([fx[0], fx[1], 0]), G = M3.cols(gx, up, V.cross(gx, up));
  /* tracking definitions: static lab positions of tracking points (joint centres included) */
  const TRK = { TX: UB_MARKERS.thorax.slice() };
  if (useHead) TRK.HD = UB_MARKERS.head.slice();
  if (usePelvis) TRK.PV = ['RASI', 'LASI', 'RPSI', 'LPSI'];
  for (const s of det.sides) { TRK[s + 'UA'] = [s + 'UPA', s + 'ELB', s + 'ELBM']; TRK[s + 'FA'] = [s + 'EJC', s + 'WRA', s + 'WRB', s + 'FRM']; }
  const SP = n => (n.endsWith('JC') ? L[n] : S[n]);
  const segNames = Object.keys(TRK).concat(finSides.map(s => s + 'HA'));
  const frames = {}, jc = {}, fitStats = {};
  for (const k of segNames) { frames[k] = new Float32Array(nF * 12).fill(NaN); fitStats[k] = { sum: 0, n: 0, max: 0, short: 0, dropped: {} }; }
  for (const s of det.sides) for (const j of ['SJC', 'EJC', 'WJC'].concat(finSides.includes(s) ? ['FIN'] : [])) jc[s + j] = new Float32Array(nF * 3).fill(NaN);
  const angles = {}; for (const j of UB_JOINTS) angles[j] = { a: new Float32Array(nF).fill(NaN), b: new Float32Array(nF).fill(NaN), g: new Float32Array(nF).fill(NaN) };
  const fit = (k, getter) => {
    const A = [], B = [], nm = [];
    TRK[k].forEach(n => { const q = getter(n); if (q) { A.push(SP(n)); B.push(q); nm.push(n); } });
    if (A.length < 3) { fitStats[k].short++; return null; }
    const run = (a, b) => { const T = rigidFit(a, b); let ss = 0; for (let i = 0; i < a.length; i++) { const d = V.sub(V.add(M3.vec(T.R, a[i]), T.t), b[i]); ss += V.dot(d, d); } return { T, rms: Math.sqrt(ss / a.length) }; };
    let best = run(A, B);
    if (A.length >= 4 && best.rms > opt.fitTol) { let alt = null, drop = -1; for (let j = 0; j < A.length; j++) { const r = run(A.filter((_, i) => i !== j), B.filter((_, i) => i !== j)); if (!alt || r.rms < alt.rms) { alt = r; drop = j; } } if (alt.rms < best.rms * 0.6) { best = alt; fitStats[k].dropped[nm[drop]] = (fitStats[k].dropped[nm[drop]] || 0) + 1; } }
    fitStats[k].sum += best.rms; fitStats[k].n++; if (best.rms > fitStats[k].max) fitStats[k].max = best.rms;
    const T = best.T; return { o: V.add(M3.vec(T.R, F0[k].o), T.t), R: M3.mul(T.R, F0[k].R), T };
  };
  const put3 = (arr, f, p) => { arr[f * 3] = p[0]; arr[f * 3 + 1] = p[1]; arr[f * 3 + 2] = p[2]; };
  const zxy = (Rrel, mirror) => { const e = cardanZXY(Rrel); return mirror ? [e[0], -e[1], -e[2]] : e; };
  let lowElev = { R: 0, L: 0 }; const faDirect = { R: 0, L: 0 };
  for (let f = 0; f < nF; f++) {
    const cur = n => get(ds, di, n, f), P = {};
    for (const k of ['TX', 'HD', 'PV']) if (TRK[k]) P[k] = fit(k, cur);
    for (const s of det.sides) {
      const ua = fit(s + 'UA', cur); P[s + 'UA'] = ua;
      const ejc = ua ? V.add(M3.vec(ua.T.R, L[s + 'EJC']), ua.T.t) : null;
      /* forearm: not rigid (the radius turns around the ulna), so its frame is built directly each frame from the elbow centre
         (humerus chain) and the two styloids, as ISB / Plug-in Gait; the static cluster fit is only a fallback for a lost styloid */
      let fa = null; const ra = cur(s + 'WRA'), ul = cur(s + 'WRB');
      if (ejc && ra && ul) { const w = mid(ra, ul), F = frameYZ(w, V.sub(ejc, w), V.mul(V.sub(ra, ul), s === 'R' ? 1 : -1)); fa = { o: w, R: F.R, T: { R: M3.mul(F.R, M3.T(F0[s + 'FA'].R)), t: V.sub(w, M3.vec(M3.mul(F.R, M3.T(F0[s + 'FA'].R)), F0[s + 'FA'].o)) } }; faDirect[s]++; }
      else fa = fit(s + 'FA', n => (n === s + 'EJC' ? ejc : cur(n)));
      P[s + 'FA'] = fa;
      const fin = finSides.includes(s) ? cur(s + 'FIN') : null;
      if (fa && fin) { const w = V.add(M3.vec(fa.T.R, L[s + 'WJC']), fa.T.t), y = V.sub(w, fin); if (Math.hypot(...y) > 0.02) P[s + 'HA'] = frameYZ(w, y, M3.col(fa.R, 2)); put3(jc[s + 'FIN'], f, fin); }
      if (ua) { put3(jc[s + 'SJC'], f, ua.o); put3(jc[s + 'EJC'], f, ejc); }
      if (fa) put3(jc[s + 'WJC'], f, fa.o);
    }
    for (const k in P) if (P[k]) frameput(frames[k], f, P[k].o, P[k].R);
    const set = (j, e) => { angles[j].a[f] = e[0]; angles[j].b[f] = e[1]; angles[j].g[f] = e[2]; };
    if (P.TX) {
      set('Thorax', trunkLike(cardanZXY(M3.mul(M3.T(G), P.TX.R))));
      if (P.PV) set('Trunk', trunkLike(cardanZXY(M3.mul(M3.T(P.PV.R), P.TX.R))));
      if (P.HD) set('Neck', trunkLike(cardanZXY(M3.mul(M3.T(P.TX.R), P.HD.R))));
      for (const s of det.sides) {
        if (P[s + 'UA']) {
          const e = eulerYXYr(M3.mul(M3.T(P.TX.R), P[s + 'UA'].R)).map(v => v / D2R), w = v => ((v % 360) + 540) % 360 - 180;
          // right arm: elevation is the negative-β branch -> plane = α+180, axial = γ+180; left arm: mirrored (−α, −γ)
          const sh = s === 'R' ? [w(e[0] + 180), e[1], w(e[2] + 180)] : [-e[0], e[1], -e[2]];
          set(s + 'Shoulder', sh); if (e[1] < 15) lowElev[s]++;
        }
        if (P[s + 'UA'] && P[s + 'FA']) set(s + 'Elbow', zxy(M3.mul(M3.T(P[s + 'UA'].R), P[s + 'FA'].R), s === 'L'));
        if (P[s + 'FA'] && P[s + 'HA']) set(s + 'Wrist', zxy(M3.mul(M3.T(P[s + 'FA'].R), P[s + 'HA'].R), s === 'L'));
      }
    }
  }
  const sideJ = det.sides.flatMap(s => ['Shoulder', 'Elbow'].concat(finSides.includes(s) ? ['Wrist'] : []).map(t => s + t));
  for (const j of UB_JOINTS) if (!sideJ.includes(j) && !['Thorax', 'Trunk', 'Neck'].includes(j)) delete angles[j];
  if (!usePelvis) delete angles.Trunk; if (!useHead) delete angles.Neck;
  const zxyJ = {}, shJ = {}; for (const j in angles) (j.endsWith('Shoulder') ? shJ : zxyJ)[j] = angles[j];
  const normRange = opt.normalize && sameStatic ? [s0, s1] : null;
  const gimbal = finishAngles(zxyJ, nF, normRange); finishAngles(shJ, nF, null);
  if (opt.normalize && !sameStatic) warnings.push('ลบ offset static ได้เฉพาะเมื่อ static อยู่ใน trial เดียวกัน');
  /* plausibility: elbow pronation is 0 at full supination; values well below 0 are beyond the anatomical range and usually mean
     the radial / ulnar styloid labels (WRA / WRB) are swapped relative to the static */
  for (const s of det.sides) { const A = angles[s + 'Elbow']; if (!A) continue; const v = Array.from(A.g).filter(x => x === x).sort((x, y) => x - y); const med = v.length ? v[v.length >> 1] : NaN;
    if (med < -15) warnings.push(s + 'Elbow: pronation median ' + med.toFixed(0) + '° (เกิน full supination) · ' + s + 'WRA/' + s + 'WRB อาจสลับ label เทียบกับ static ตรวจในภาพ 3D ว่า WRA อยู่ฝั่งนิ้วโป้ง แล้วติ๊ก “สลับ WRA/WRB” ถ้าผิด'); }
  for (const s of det.sides) if (lowElev[s] > 0.1 * nF) warnings.push(s + 'Shoulder: elevation < 15° ใน ' + Math.round(100 * lowElev[s] / nF) + '% ของเฟรม · plane of elevation และ axial rotation ไม่มีนิยามชัดที่ elevation ต่ำ (gimbal lock ของ Y-X-Y)');
  for (const j in gimbal) if (gimbal[j] > 75) warnings.push(j + ': มุมแกนกลางสูงสุด ' + gimbal[j].toFixed(0) + '° ใกล้ gimbal lock');
  const fitSummary = {}; for (const k in fitStats) { const st = fitStats[k]; if (k.endsWith('HA') || !st.n) continue; /* forearm built directly: no fit */ fitSummary[k] = { meanRms: st.n ? st.sum / st.n : NaN, maxRms: st.max, dropped: st.dropped, short: st.short }; if (st.n && st.sum / st.n > 0.010) warnings.push(k + ': fit RMS เฉลี่ย ' + (st.sum / st.n * 1000).toFixed(1) + ' mm (> 10 mm) · ตรวจ marker ด้วย QC'); }
  if (!useHead) warnings.push('ไม่มี marker ศีรษะครบ: ไม่คำนวณ neck');
  else if (Math.max(...headOffset.map(Math.abs)) > 15) warnings.push('marker ศีรษะเอียงจาก thorax ใน static (flex ' + headOffset[0].toFixed(0) + '°, bend ' + headOffset[1].toFixed(0) + '°, rot ' + headOffset[2].toFixed(0) + '°) · ใช้ head offset: มุม neck เทียบกับท่า static');
  if (!usePelvis) warnings.push('ไม่มี marker pelvis ครบ: ไม่คำนวณ trunk (thorax เทียบ pelvis)');
  const labels = {}; for (const j in angles) labels[j] = UB_LABELS[j.replace(/^[RL]/, '')];
  return { ok: true, sides: det.sides, angles, labels, ranges: angleRanges(angles, nF), frames, jc, segNames, landmarks: L, fitSummary, warnings,
    staticSource: sameStatic ? 'trial' : (sds.name || 'static file'), staticRange: [s0, s1 - 1], dAA, headOffset, finSides, swapWrist: opt.swapWrist || null, normalize: !!normRange, model: { F0, L, S, G } };
}

/* ---------------------------------------------------------------- Marker quality control
   Rigid-body check: the distance between two markers on the same bone should stay at its static value. For each pair,
   Δ(t) = d(t) − d_static; reported: median Δ (bias: marker re-placed / static not representative), p95 |Δ| (soft-tissue
   artefact or mislabel), max |Δ| and its frame. Cross-joint pairs (lateral/medial knee–ankle) are not rigid but change
   little because the epicondyles lie near the flexion axis; a large lateral/medial mismatch flags a wrong gap-fill or label.
   Per marker: missing frames and jumps (frame-to-frame speed > maxSpeed). A marker present in several flagged pairs is the
   likely culprit. Tolerances in metres, [warn, bad]: 'p95' for p95 |Δ|, 'bias' for |median Δ|. Pelvis and thigh allow for
   soft-tissue artefact in deep hip flexion (thigh STA up to ~30 mm, Leardini et al. 2005); shank and foot markers sit on
   bone or shoe, so a constant shift there means a marker was re-placed or the static is not from this session. */
const QC_TOL = {
  pelvis: { p95: [0.012, 0.025], bias: [0.010, 0.020] }, thigh: { p95: [0.025, 0.040], bias: [0.015, 0.025] },
  shank: { p95: [0.010, 0.020], bias: [0.008, 0.015] }, foot: { p95: [0.010, 0.020], bias: [0.008, 0.015] },
  cross: { p95: [0.035, 0.060], bias: [0.025, 0.045] }, mismatch: [0.030, 0.050]
};
function qcPairs(setId) {
  const P = [['pelvis', 'RASI', 'LASI'], ['pelvis', 'RPSI', 'LPSI'], ['pelvis', 'RASI', 'RPSI'], ['pelvis', 'LASI', 'LPSI'], ['pelvis', 'RASI', 'LPSI'], ['pelvis', 'LASI', 'RPSI']];
  for (const s of ['R', 'L']) {
    if (setId === 'bad18') P.push(['thigh', s + 'THI', s + 'KNE'], ['thigh', s + 'THI', s + 'KNEM'], ['thigh', s + 'KNE', s + 'KNEM'], ['shank', s + 'ANK', s + 'ANKM'], ['foot', s + 'HEE', s + 'TOE'], ['cross', s + 'KNE', s + 'ANK'], ['cross', s + 'KNEM', s + 'ANKM']);
    else P.push(['thigh', s + 'THI', s + 'KNE'], ['shank', s + 'TIB', s + 'ANK'], ['foot', s + 'HEE', s + 'TOE'], ['cross', s + 'KNE', s + 'ANK'], ['cross', s + 'KNE', s + 'TIB']);
  }
  return P;
}
function markerQC(ds, opt) {
  opt = Object.assign({ staticStart: 0, staticN: 10, maxSpeed: 8 }, opt || {});
  const setId = detectLBSet(ds);
  if (!setId) return { ok: false, error: 'QC รองรับชุด Conventional Lower 16 หรือชุด 18 ตัว (KNEM/ANKM)' };
  const sds = opt.staticDs || ds, nF = ds.nFrames, MK = LB_SETS[setId].markers;
  const ix = d => Object.fromEntries(MK.map(n => [n, d.base.findIndex(b => b.toUpperCase() === n)]));
  const di = ix(ds), si = ix(sds);
  const miss = MK.filter(n => si[n] < 0); if (miss.length) return { ok: false, error: 'ไฟล์ static ไม่พบ marker: ' + miss.join(', ') };
  const at = (d, I, n, f) => { const k = f * d.nM + I[n]; return d.vis[k] ? [d.pts[k * 3], d.pts[k * 3 + 1], d.pts[k * 3 + 2]] : null; };
  const s0 = clamp(opt.staticStart | 0, 0, sds.nFrames - 1), s1 = Math.min(sds.nFrames, s0 + Math.max(1, opt.staticN | 0));
  const q = (v, p) => v[Math.min(v.length - 1, Math.floor(p * (v.length - 1)))];
  const pairs = [];
  for (const [grp, a, b] of qcPairs(setId)) {
    let s = 0, c = 0; for (let f = s0; f < s1; f++) { const A = at(sds, si, a, f), B = at(sds, si, b, f); if (A && B) { s += Math.hypot(A[0] - B[0], A[1] - B[1], A[2] - B[2]); c++; } }
    if (!c) { pairs.push({ grp, a, b, ok: false }); continue; }
    const d0 = s / c, delta = new Float32Array(nF).fill(NaN), v = [], av = [];
    let mx = 0, mxf = -1;
    for (let f = 0; f < nF; f++) { const A = at(ds, di, a, f), B = at(ds, di, b, f); if (!A || !B) continue; const d = Math.hypot(A[0] - B[0], A[1] - B[1], A[2] - B[2]) - d0; delta[f] = d; v.push(d); av.push(Math.abs(d)); if (Math.abs(d) > mx) { mx = Math.abs(d); mxf = f; } }
    if (!v.length) { pairs.push({ grp, a, b, d0, ok: false }); continue; }
    v.sort((x, y) => x - y); av.sort((x, y) => x - y);
    const med = q(v, 0.5), p95 = q(av, 0.95), tol = QC_TOL[grp];
    const lvl = (x, t) => (x > t[1] ? 2 : x > t[0] ? 1 : 0), level = Math.max(lvl(p95, tol.p95), lvl(Math.abs(med), tol.bias));
    pairs.push({ grp, a, b, ok: true, d0, median: med, p95, max: mx, maxFrame: mxf, n: v.length, level, delta });
  }
  /* cross-joint consistency: lateral vs medial (or KNE–ANK vs KNE–TIB) knee–ankle spans should vary alike */
  const cross = [];
  for (const s of ['R', 'L']) {
    const c = pairs.filter(p => p.grp === 'cross' && p.ok && p.a[0] === s);
    if (c.length === 2) {
      const r = x => x.delta, A = r(c[0]), B = r(c[1]), dv = [];
      for (let f = 0; f < nF; f++) if (A[f] === A[f] && B[f] === B[f]) dv.push(Math.abs(A[f] - B[f]));
      dv.sort((x, y) => x - y);
      if (dv.length) {
        const p95 = q(dv, 0.95), t = QC_TOL.mismatch, worse = c[0].p95 >= c[1].p95 ? c[0] : c[1];
        cross.push({ side: s, pairs: [c[0].a + '–' + c[0].b, c[1].a + '–' + c[1].b], p95, level: p95 > t[1] ? 2 : p95 > t[0] ? 1 : 0, worse: [worse.a, worse.b] });
      }
    }
  }
  /* per marker: gaps, jumps, involvement in flagged pairs */
  const markers = MK.map(n => {
    let gap = 0, jump = 0, prev = null, pf = -1, jf = -1;
    for (let f = 0; f < nF; f++) { const A = at(ds, di, n, f); if (!A) { gap++; continue; } if (prev && f - pf === 1) { const sp = Math.hypot(A[0] - prev[0], A[1] - prev[1], A[2] - prev[2]) * ds.rate; if (sp > opt.maxSpeed) { jump++; if (jf < 0) jf = f; } } prev = A; pf = f; }
    const mine = pairs.filter(p => p.ok && (p.a === n || p.b === n)), flagged = mine.filter(p => p.level > 0);
    const score = mine.length ? flagged.reduce((s2, p) => s2 + p.level, 0) / mine.length : 0;
    return { name: n, gap, gapPct: 100 * gap / nF, jump, firstJump: jf, pairs: mine.length, flagged: flagged.length, score };
  });
  /* culprits, with the reason: jumps; a lateral/medial knee–ankle mismatch (of the two markers of the worse span, the one with
     more flagged pairs elsewhere); a marker flagged in >= 2 pairs and >= 2/3 of its pairs, with a BAD pair or all pairs flagged. */
  const M = Object.fromEntries(markers.map(m => [m.name, m])), sus = new Map();
  const add = (n, why) => { if (!sus.has(n)) sus.set(n, []); sus.get(n).push(why); };
  for (const m of markers) if (m.jump) add(m.name, 'jump');
  for (const c of cross) if (c.level > 0) { const [a, b] = c.worse, pick = M[a].flagged - (a === c.worse[0] ? 1 : 0) >= M[b].flagged - 1 ? a : b; add(pick, 'mismatch'); }
  for (const m of markers) {   // >= 2/3 of its pairs flagged, and either one of them BAD or all of them flagged (WARN-only pelvis/thigh deviations are STA-sized)
    const mine = pairs.filter(p => p.ok && (p.a === m.name || p.b === m.name));
    if (m.flagged >= 2 && 3 * m.flagged >= 2 * m.pairs && (mine.some(p => p.level === 2) || m.flagged === m.pairs)) add(m.name, 'pairs');
  }
  const suspects = [...sus.entries()].map(([name, why]) => ({ name, why, score: M[name].score })).sort((x, y) => y.why.length - x.why.length || y.score - x.score);
  const level = Math.max(0, ...pairs.filter(p => p.ok).map(p => p.level), ...cross.map(c => c.level), ...markers.map(m => (m.jump ? 2 : 0)));
  return { ok: true, setId, staticSource: opt.staticDs ? (sds.name || 'static file') : 'trial', staticRange: [s0, s1 - 1], tol: QC_TOL, pairs, cross, markers, suspects, level };
}

/* ---------------------------------------------------------------- Movement cycles
   Time normalisation of one cycle [f0, f1] (frames, inclusive) to npts points (0–100 %), linear interpolation.
   A point whose two neighbouring frames are not both valid is NaN (gaps are not bridged). */
function cycleNormalize(data, f0, f1, npts) {
  npts = npts || 101; const out = new Float64Array(npts), L = f1 - f0;
  for (let k = 0; k < npts; k++) {
    const x = f0 + L * k / (npts - 1), i = Math.floor(x), w = x - i;
    const a = data[i], b = i + 1 <= f1 ? data[i + 1] : a;
    out[k] = w < 1e-9 ? (a === a ? a : NaN) : (a === a && b === b ? a + (b - a) * w : NaN);
  }
  return out;
}
/* mean and sample SD across cycles at each point (NaN ignored; SD needs >= 2 values) */
function cycleStats(curves) {
  const n = curves[0].length, mean = new Float64Array(n), sd = new Float64Array(n), cnt = new Uint16Array(n);
  for (let k = 0; k < n; k++) {
    let s = 0, c = 0; for (const cv of curves) { const v = cv[k]; if (v === v) { s += v; c++; } }
    const m = c ? s / c : NaN; let q = 0; for (const cv of curves) { const v = cv[k]; if (v === v) q += (v - m) ** 2; }
    mean[k] = m; sd[k] = c > 1 ? Math.sqrt(q / (c - 1)) : NaN; cnt[k] = c;
  }
  return { mean, sd, n: cnt };
}
/* cycles from a 0/1 contact signal: 'stance' = contact on -> off, 'stride' = contact on -> next contact on.
   Contacts shorter than minLen frames and gaps shorter than minGap frames are ignored (flicker). */
function cyclesFromContact(on, kind, minLen, minGap) {
  const n = on.length, segs = []; minLen = minLen || 1; minGap = minGap || 1;
  for (let i = 0; i < n;) { if (!on[i]) { i++; continue; } let j = i; while (j < n && on[j]) j++; segs.push([i, j - 1]); i = j; }
  const merged = []; for (const s of segs) { const p = merged[merged.length - 1]; if (p && s[0] - p[1] - 1 < minGap) p[1] = s[1]; else merged.push(s.slice()); }
  const st = merged.filter(s => s[1] - s[0] + 1 >= minLen);
  if (kind === 'stance') return st.map(s => [s[0], s[1]]);
  const out = []; for (let k = 0; k + 1 < st.length; k++) out.push([st[k][0], st[k + 1][0]]); return out;
}

/* ---------------------------------------------------------------- Inverse kinematics (global optimisation)
   Lu & O'Connor (1999) type multibody optimisation: pelvis -> thigh -> shank -> foot for both legs. Joint centres are
   forced to coincide (no joint dislocation, unlike segment optimisation), and all segment poses of a frame are found
   together by minimising Σ|measured − model marker|² (Levenberg–Marquardt, warm-started from the previous frame).
   - Geometry = the static calibration of computeLowerBody (same anatomical frames, joint centres, angle convention).
   - Joint rotations are parametrised in the same Euler sequence the angles are reported in (Z-X-Y or Y-X-Z), so a
     locked DOF (hinge knee, universal ankle) stays at its static value in the reported angle.
   - Pelvis 6 DOF; equal marker weights. OpenSim IK uses the same least-squares principle on a scaled generic model;
     this solver uses subject-specific static geometry instead, so it is not a numerical replica of OpenSim. */
const IK_MODELS = {
  sph: { label: 'Ball joints · hip 3 / knee 3 / ankle 3 DOF', dof: { Hip: [1, 1, 1], Knee: [1, 1, 1], Ankle: [1, 1, 1] } },
  lu: { label: "Lu & O'Connor 1999 · hip 3 / knee 1 (hinge) / ankle 2 (universal) DOF", dof: { Hip: [1, 1, 1], Knee: [1, 0, 0], Ankle: [1, 0, 1] } }
};
/* marker -> segment assignment for IK (each marker drives exactly one segment) */
const IK_SEGMARK = {
  conv16: { PV: ['RASI', 'LASI', 'RPSI', 'LPSI'], TH: ['THI', 'KNE'], SK: ['TIB', 'ANK'], FT: ['HEE', 'TOE'] },
  bad18: { PV: ['RASI', 'LASI', 'RPSI', 'LPSI'], TH: ['THI', 'KNE', 'KNEM'], SK: ['ANK', 'ANKM'], FT: ['HEE', 'TOE'] }
};
function rotZXYr(a, b, g) {   // Rz(a) Rx(b) Ry(g), radians
  const ca = Math.cos(a), sa = Math.sin(a), cb = Math.cos(b), sb = Math.sin(b), cg = Math.cos(g), sg = Math.sin(g);
  return [ca * cg - sa * sb * sg, -sa * cb, ca * sg + sa * sb * cg, sa * cg + ca * sb * sg, ca * cb, sa * sg - ca * sb * cg, -cb * sg, sb, cb * cg];
}
function rotYXZr(a, b, g) {   // Ry(a) Rx(b) Rz(g), radians
  const ca = Math.cos(a), sa = Math.sin(a), cb = Math.cos(b), sb = Math.sin(b), cg = Math.cos(g), sg = Math.sin(g);
  return [ca * cg + sa * sb * sg, -ca * sg + sa * sb * cg, sa * cb, cb * sg, cb * cg, -sb, -sa * cg + ca * sb * sg, sa * sg + ca * sb * cg, ca * cb];
}
function buildIKModel(kin, modelId) {
  const md = kin && kin.model; if (!md) return null;
  const PIG = md.PIG, dec = PIG ? cardanYXZ : cardanZXY, id = IK_MODELS[modelId] ? modelId : 'sph', mdef = IK_MODELS[id], sm = IK_SEGMARK[md.setId];
  const loc = (k, P) => M3.vec(M3.T(md.seg[k].R), V.sub(P, md.seg[k].o));
  const marks = [];
  for (const n of sm.PV) marks.push({ name: n, seg: 'RPV', p: loc('RPV', md.S[n]) });
  for (const s of ['R', 'L']) for (const [g, list] of [['TH', sm.TH], ['SK', sm.SK], ['FT', sm.FT]]) for (const m of list) marks.push({ name: s + m, seg: s + g, p: loc(s + g, md.S[s + m]) });
  const joints = [];
  for (const s of ['R', 'L']) for (const [t, ref, seg, jc] of [['Hip', 'RPV', s + 'TH', s + 'HJC'], ['Knee', s + 'TH', s + 'SK', s + 'KJC'], ['Ankle', s + 'SK', s + 'FT', s + 'AJC']]) {
    const q0 = dec(M3.mul(M3.T(md.seg[ref].R), md.seg[seg].R)).map(v => v * D2R);
    const free = mdef.dof[t].slice();
    joints.push({ jn: s + t, ref, seg, jc, d: loc(ref, md.L[jc]), q0, free, need: free.reduce((a, b) => a + b, 0) >= 3 ? 2 : 1 });
  }
  const map = []; joints.forEach((j, ji) => j.free.forEach((fr, c) => { if (fr) map.push([ji, c]); }));
  return { id, label: mdef.label, PIG, rot: PIG ? rotYXZr : rotZXYr, dec, marks, joints, map, nPar: 6 + map.length, setId: md.setId };
}
/* forward kinematics: pelvis pose (Rp, tp) + joint angles q[j] = [a,b,g] (rad) -> {seg: {R, o}} */
function ikForward(M, Rp, tp, q) {
  const P = { RPV: { R: Rp, o: tp } };
  M.joints.forEach((j, i) => { const pr = P[j.ref]; P[j.seg] = { R: M3.mul(pr.R, M.rot(q[i][0], q[i][1], q[i][2])), o: V.add(pr.o, M3.vec(pr.R, j.d)) }; });
  return P;
}
/* one frame: obs = [{k: marker index in M.marks, p: [x,y,z] lab}] */
function ikSolveFrame(M, obs, Rp0, tp0, q0, maxIt) {
  const nP = M.nPar, nR = obs.length * 3, q = q0.map(a => a.slice());
  let Rp = Rp0.slice();
  const x = new Float64Array(nP); x[3] = tp0[0]; x[4] = tp0[1]; x[5] = tp0[2];
  M.map.forEach(([ji, c], i) => { x[6 + i] = q[ji][c]; });
  const resid = (xv, out) => {
    M.map.forEach(([ji, c], i) => { q[ji][c] = xv[6 + i]; });
    const P = ikForward(M, M3.mul(Rp, expRot([xv[0], xv[1], xv[2]])), [xv[3], xv[4], xv[5]], q);
    let ss = 0;
    for (let i = 0; i < obs.length; i++) {
      const mk = M.marks[obs[i].k], S = P[mk.seg], r = V.add(S.o, M3.vec(S.R, mk.p));
      for (let a = 0; a < 3; a++) { const d = r[a] - obs[i].p[a]; out[i * 3 + a] = d; ss += d * d; }
    }
    return ss;
  };
  const r = new Float64Array(nR), rt = new Float64Array(nR), J = Array.from({ length: nP }, () => new Float64Array(nR)), h = 1e-7;
  let cost = resid(x, r), lam = 1e-3, it = 0;
  for (; it < (maxIt || 40); it++) {
    for (let j = 0; j < nP; j++) { const s = x[j]; x[j] = s + h; resid(x, rt); x[j] = s; for (let i = 0; i < nR; i++) J[j][i] = (rt[i] - r[i]) / h; }
    resid(x, r);
    const A = [], g = [];
    for (let a = 0; a < nP; a++) { const row = new Array(nP); for (let b = 0; b <= a; b++) { let s = 0; const Ja = J[a], Jb = J[b]; for (let i = 0; i < nR; i++) s += Ja[i] * Jb[i]; row[b] = s; if (b < a) A[b][a] = s; } A.push(row); let s = 0; for (let i = 0; i < nR; i++) s += J[a][i] * r[i]; g.push(-s); }
    let accepted = false, step = 0;
    for (let tr = 0; tr < 10 && !accepted; tr++) {
      const Ad = A.map((row, i) => row.map((v, j) => (i === j ? v + lam * Math.max(v, 1e-9) : v)));
      const dx = solveLin(Ad, g); if (!dx) { lam *= 10; continue; }
      const xn = Float64Array.from(x); for (let i = 0; i < nP; i++) xn[i] += dx[i];
      const cn = resid(xn, rt);
      if (cn <= cost) { x.set(xn); cost = cn; lam = Math.max(1e-9, lam / 3); accepted = true; step = Math.max(...dx.map(Math.abs)); }
      else lam *= 4;
    }
    // fold the pelvis rotation increment into Rp so the next linearisation is about δ = 0
    Rp = M3.mul(Rp, expRot([x[0], x[1], x[2]])); x[0] = x[1] = x[2] = 0;
    cost = resid(x, r);
    if (!accepted || step < 1e-8) break;
  }
  M.map.forEach(([ji, c], i) => { q[ji][c] = x[6 + i]; });
  return { Rp, tp: [x[3], x[4], x[5]], q, cost, it, rms: Math.sqrt(cost / Math.max(1, obs.length)) };
}
function computeIK(ds, kin, opt) {
  opt = opt || {};
  if (!kin || !kin.ok || !kin.model) return { ok: false, error: 'กด “คำนวณมุมข้อต่อ” ก่อน (IK ใช้ static calibration ชุดเดียวกัน)' };
  const M = buildIKModel(kin, opt.model), md = kin.model, nF = ds.nFrames, nM = ds.nM, warnings = [];
  const idx = M.marks.map(m => ds.base.findIndex(b => b.toUpperCase() === m.name));
  const miss = M.marks.filter((m, i) => idx[i] < 0).map(m => m.name);
  if (miss.length) return { ok: false, error: 'ไม่พบ marker: ' + miss.join(', ') };
  const segNames = kin.segNames, frames = {}, jc = {}, angles = {};
  for (const k of segNames) frames[k] = new Float32Array(nF * 12).fill(NaN);
  for (const j of ['RHJC', 'LHJC', 'RKJC', 'LKJC', 'RAJC', 'LAJC']) jc[j] = new Float32Array(nF * 3).fill(NaN);
  for (const [jn] of LB_JOINTS) angles[jn] = { a: new Float32Array(nF).fill(NaN), b: new Float32Array(nF).fill(NaN), g: new Float32Array(nF).fill(NaN) };
  const residual = new Float32Array(nF).fill(NaN), mkSum = new Float64Array(M.marks.length), mkN = new Uint32Array(mkSum.length), mkMax = new Float64Array(mkSum.length);
  const pvIdx = M.marks.map((m, i) => (m.seg === 'RPV' ? i : -1)).filter(i => i >= 0);
  const needOf = {}; for (const j of M.joints) needOf[j.seg] = j.need;
  let prev = null, skipped = 0, partial = 0, iters = 0, solved = 0;
  for (let f = 0; f < nF; f++) {
    const obs = [], cnt = {};
    M.marks.forEach((m, k) => { const q = f * nM + idx[k]; if (ds.vis[q]) { obs.push({ k, p: [ds.pts[q * 3], ds.pts[q * 3 + 1], ds.pts[q * 3 + 2]] }); cnt[m.seg] = (cnt[m.seg] || 0) + 1; } });
    if ((cnt.RPV || 0) < 3) { skipped++; continue; }
    /* cold-start candidates: the 6DOF pose of this frame, and the static pose carried by the pelvis markers.
       Used on the first frame, after a pelvis gap, and whenever the warm start ends > 30 mm RMS (local minimum guard). */
    const coldInits = () => {
      const out = [], fr = {};
      for (const k of segNames) fr[k] = frameAt(kin.frames[k], f);
      if (segNames.every(k => fr[k])) out.push({ Rp: fr.RPV.R, tp: fr.RPV.o, q: M.joints.map(j => { const e = M.dec(M3.mul(M3.T(fr[j.ref].R), fr[j.seg].R)).map(v => v * D2R); return j.free.map((fz, c) => (fz ? e[c] : j.q0[c])); }) });
      const A = [], B = []; for (const k of pvIdx) { const o = obs.find(u => u.k === k); if (o) { A.push(md.S[M.marks[k].name]); B.push(o.p); } }
      const T = rigidFit(A, B);
      out.push({ Rp: M3.mul(T.R, md.seg.RPV.R), tp: V.add(M3.vec(T.R, md.seg.RPV.o), T.t), q: M.joints.map(j => j.q0.slice()) });
      return out;
    };
    let sol = prev ? ikSolveFrame(M, obs, prev.Rp, prev.tp, prev.q, opt.maxIt) : null;
    if (sol) iters += sol.it;
    if (!sol || sol.rms > 0.03) for (const init of coldInits()) { const s2 = ikSolveFrame(M, obs, init.Rp, init.tp, init.q, opt.maxIt); iters += s2.it; if (!sol || s2.cost < sol.cost) sol = s2; }
    solved++;
    prev = sol;
    const P = ikForward(M, sol.Rp, sol.tp, sol.q), ok = { RPV: true };
    for (const j of M.joints) ok[j.seg] = ok[j.ref] && (cnt[j.seg] || 0) >= j.need;
    if (segNames.some(k => !ok[k])) partial++;
    for (const k of segNames) if (ok[k]) frameput(frames[k], f, P[k].o, P[k].R);
    for (const j of M.joints) if (ok[j.ref]) { const o = P[j.seg].o; jc[j.jc][f * 3] = o[0]; jc[j.jc][f * 3 + 1] = o[1]; jc[j.jc][f * 3 + 2] = o[2]; }
    for (const [jn, sg, rf] of LB_JOINTS) if (ok[sg] && ok[rf]) { const e = jointEuler(jn, P[rf].R, P[sg].R, M.PIG, md.pigOffset); angles[jn].a[f] = e[0]; angles[jn].b[f] = e[1]; angles[jn].g[f] = e[2]; }
    let ss = 0;
    for (const o of obs) { const mk = M.marks[o.k], S = P[mk.seg], d = Math.hypot(...V.sub(V.add(S.o, M3.vec(S.R, mk.p)), o.p)); ss += d * d; mkSum[o.k] += d; mkN[o.k]++; if (d > mkMax[o.k]) mkMax[o.k] = d; }
    residual[f] = Math.sqrt(ss / obs.length);
  }
  if (!solved) return { ok: false, error: 'ไม่มี frame ที่ pelvis มี marker ≥ 3 ตัว' };
  const gimbal = finishAngles(angles, nF, md.normRange);
  const perMarker = {}; M.marks.forEach((m, k) => { perMarker[m.name] = { mean: mkN[k] ? mkSum[k] / mkN[k] : NaN, max: mkMax[k], n: mkN[k] }; });
  let rs = 0, rn = 0, rmax = 0; for (let f = 0; f < nF; f++) { const v = residual[f]; if (v === v) { rs += v; rn++; if (v > rmax) rmax = v; } }
  const bad = Object.entries(perMarker).filter(([, v]) => v.mean > 0.010).sort((a, b) => b[1].mean - a[1].mean);
  if (bad.length) warnings.push('marker ที่ห่างจากโมเดลเฉลี่ย > 10 mm: ' + bad.map(([n, v]) => n + ' ' + (v.mean * 1000).toFixed(1) + ' mm').join(', ') + ' · ตรวจ label หรือ soft-tissue artefact');
  if (skipped) warnings.push('pelvis มี marker < 3 ตัว ' + skipped + ' frame (ไม่มีผล IK)');
  if (partial) warnings.push(partial + ' frame มีบาง segment marker ไม่พอ (segment นั้นไม่แสดงผล)');
  for (const jn in gimbal) if (gimbal[jn] > 75) warnings.push(jn + ': มุมแกนกลาง (ab/adduction) สูงสุด ' + gimbal[jn].toFixed(0) + '° ใกล้ gimbal lock ค่ามุมสามแกนอาจไม่น่าเชื่อถือ');
  return {
    ok: true, source: 'ik', ikModel: M.id, ikLabel: M.label, convention: kin.convention, pigOffset: kin.pigOffset, markerSet: kin.markerSet, markerSetLabel: kin.markerSetLabel,
    staticSource: kin.staticSource, staticRange: kin.staticRange, staticMove: kin.staticMove, measured: kin.measured, params: kin.params, anthro: kin.anthro, landmarks: kin.landmarks,
    model: md, segNames, frames, jc, angles, ranges: angleRanges(angles, nF), warnings,
    residual: { frame: residual, mean: rn ? rs / rn : NaN, max: rmax, perMarker }, iterMean: iters / solved, nPar: M.nPar
  };
}

/* ---------------------------------------------------------------- Inverse dynamics (Newton–Euler, bottom-up)
   foot -> shank -> thigh for each leg, in the lab frame, from the kinematic result (6DOF or IK) and the C3D force plates.
   - BSIP: De Leva (1996) mass fraction, CoM position and radii of gyration (sagittal, transverse, longitudinal).
     Segment ends from the static calibration: thigh HJC–KJC, shank KJC–AJC (De Leva: KJC–lateral malleolus),
     foot HEE–TOE markers (De Leva: heel–longest toe tip; the TOE marker sits proximal to it, so foot length is short).
   - Kinematics: segment origin + orientation low-pass filtered (2nd-order Butterworth, dual pass, cutoff corrected
     for the double pass, Winter 2009), rotation re-orthonormalised; a = d²c/dt², ω from Ṙ Rᵀ, α = dω/dt (central differences).
   - GRF: Fz < threshold set to zero, F and the plate wrench about the lab origin (COP × F + free moment) filtered with
     the same cutoff by default (Kristianslund et al. 2012), then summed per foot. A plate is assigned to the foot that is
     over it; a plate with both feet over it cannot be split and those frames get no result.
   - Output: internal net joint moment and force acting on the distal segment, resolved in the proximal (default)
     or distal segment frame; components follow the angle sign rules (positive moment acts to increase the positive angle).
     Joint power = M · (ω_distal − ω_proximal). Validated on synthetic data only, not yet cross-checked against other software. */
const BSIP_DELEVA = {   // [mass fraction, CoM from proximal end, r_sagittal, r_transverse, r_longitudinal] (fractions of segment length)
  M: { TH: [0.1416, 0.4095, 0.329, 0.329, 0.149], SK: [0.0433, 0.4459, 0.255, 0.249, 0.103], FT: [0.0137, 0.4415, 0.257, 0.245, 0.124] },
  F: { TH: [0.1478, 0.3612, 0.369, 0.364, 0.162], SK: [0.0481, 0.4416, 0.271, 0.267, 0.093], FT: [0.0129, 0.4014, 0.299, 0.279, 0.139] }
};
const ID_GRAV = [0, 0, -9.80665];
/* 2nd-order Butterworth applied forward and backward on each run of valid samples (cutoff corrected for the double pass) */
function lowpassRuns(x, valid, fs, fc) {
  const n = x.length, out = Float64Array.from(x);
  if (!(fc > 0)) return out;
  const wn = fc / 0.802 / (fs / 2); if (wn >= 0.99) return out;
  const { b, a } = FA.butter(2, Math.max(1e-4, wn));
  for (let i = 0; i < n;) {
    if (!valid[i]) { i++; continue; }
    let j = i; while (j < n && valid[j]) j++;
    const len = j - i;
    if (len >= 12) { const y = FA.filtfilt(b, a, x.subarray(i, j), Math.min(len - 1, Math.max(9, Math.round(fs / fc)))); out.set(y, i); }
    i = j;
  }
  return out;
}
function bsipFor(md, mass, sex) {
  const tab = BSIP_DELEVA[sex], LONG = md.PIG ? 2 : 1, ML = md.PIG ? 1 : 2, segs = {};
  for (const s of ['R', 'L']) for (const [g, prox, dist] of [['TH', md.L[s + 'HJC'], md.L[s + 'KJC']], ['SK', md.L[s + 'KJC'], md.L[s + 'AJC']], ['FT', md.S[s + 'HEE'], md.S[s + 'TOE']]]) {
    const t = tab[g], k = s + g, Ls = Math.hypot(...V.sub(dist, prox)), m = t[0] * mass, com0 = V.add(prox, V.mul(V.sub(dist, prox), t[1]));
    const I = [0, 0, 0]; I[ML] = m * (t[2] * Ls) ** 2; I[0] = m * (t[3] * Ls) ** 2; I[LONG] = m * (t[4] * Ls) ** 2;
    segs[k] = { m, L: Ls, I, cLoc: M3.vec(M3.T(md.seg[k].R), V.sub(com0, md.seg[k].o)) };
  }
  return segs;
}
/* filtered pose + derivatives for one segment */
function segKinematics(arr, nF, fs, fc, cLoc, longIdx, mlIdx) {
  const valid = new Uint8Array(nF), raw = Array.from({ length: 12 }, () => new Float64Array(nF));
  for (let f = 0; f < nF; f++) { const b = f * 12; if (arr[b] === arr[b]) { valid[f] = 1; for (let c = 0; c < 12; c++) raw[c][f] = arr[b + c]; } }
  for (let f = 0; f < nF; f++) if (!valid[f]) { let k = f - 1; while (k >= 0 && !valid[k]) k--; if (k >= 0) for (let c = 0; c < 12; c++) raw[c][f] = raw[c][k]; }
  const fl = raw.map(x => lowpassRuns(x, valid, fs, fc));
  const o = new Float64Array(nF * 3).fill(NaN), R = new Float64Array(nF * 9).fill(NaN), com = new Float64Array(nF * 3).fill(NaN);
  for (let f = 0; f < nF; f++) {
    if (!valid[f]) continue;
    const col = c => [fl[3 + c * 3][f], fl[4 + c * 3][f], fl[5 + c * 3][f]];
    const cl = V.norm(col(longIdx)); let cm = col(mlIdx); cm = V.norm(V.sub(cm, V.mul(cl, V.dot(cm, cl))));
    const cols = []; cols[longIdx] = cl; cols[mlIdx] = cm; cols[0] = V.cross(cols[1], cols[2]);
    const Rm = M3.cols(cols[0], cols[1], cols[2]), oo = [fl[0][f], fl[1][f], fl[2][f]];
    for (let i = 0; i < 9; i++) R[f * 9 + i] = Rm[i];
    for (let a = 0; a < 3; a++) o[f * 3 + a] = oo[a];
    if (cLoc) { const c = V.add(oo, M3.vec(Rm, cLoc)); for (let a = 0; a < 3; a++) com[f * 3 + a] = c[a]; }
  }
  const dt = 1 / fs, acc = new Float64Array(nF * 3).fill(NaN), om = new Float64Array(nF * 3).fill(NaN), al = new Float64Array(nF * 3).fill(NaN);
  for (let f = 1; f < nF - 1; f++) {
    if (!valid[f - 1] || !valid[f] || !valid[f + 1]) continue;
    if (cLoc) for (let a = 0; a < 3; a++) acc[f * 3 + a] = (com[(f + 1) * 3 + a] - 2 * com[f * 3 + a] + com[(f - 1) * 3 + a]) / (dt * dt);
    const Rd = new Array(9); for (let i = 0; i < 9; i++) Rd[i] = (R[(f + 1) * 9 + i] - R[(f - 1) * 9 + i]) / (2 * dt);
    const W = M3.mul(Rd, M3.T(Array.from(R.subarray(f * 9, f * 9 + 9))));
    om[f * 3] = (W[7] - W[5]) / 2; om[f * 3 + 1] = (W[2] - W[6]) / 2; om[f * 3 + 2] = (W[3] - W[1]) / 2;
  }
  for (let f = 1; f < nF - 1; f++) { if (om[(f - 1) * 3] !== om[(f - 1) * 3] || om[(f + 1) * 3] !== om[(f + 1) * 3]) continue; for (let a = 0; a < 3; a++) al[f * 3 + a] = (om[(f + 1) * 3 + a] - om[(f - 1) * 3 + a]) / (2 * dt); }
  const g3 = (A, f) => [A[f * 3], A[f * 3 + 1], A[f * 3 + 2]];
  return { valid, o, R, com, acc, om, al, get: { o: f => g3(o, f), com: f => g3(com, f), acc: f => g3(acc, f), om: f => g3(om, f), al: f => g3(al, f), R: f => Array.from(R.subarray(f * 9, f * 9 + 9)) } };
}
function plateGeom(pl) {
  const C = pl.corners, cen = [0, 1, 2].map(i => (C[0][i] + C[1][i] + C[2][i] + C[3][i]) / 4), ex = V.sub(C[0], C[1]), ey = V.sub(C[0], C[3]);
  return { cen, ex: V.norm(ex), ey: V.norm(ey), hx: Math.hypot(...ex) / 2, hy: Math.hypot(...ey) / 2 };
}
/* moment components with the angle sign rules (resolved in the chosen segment frame) */
function momentComps(jn, Mlab, Rres, PIG) {
  const m = M3.vec(M3.T(Rres), Mlab);
  if (!PIG) return [m[2], m[0], m[1]];   // Z-X-Y: a about Z, b about X, g about Y
  const sd = jn[0] === 'L' ? -1 : 1, t = jn.slice(1);
  return [t === 'Knee' ? m[1] : -m[1], sd * m[0], sd * m[2]];
}
function computeID(ds, kin, opt) {
  opt = Object.assign({ cutoff: 6, cutoffGRF: null, fzThr: 20, resolve: 'prox', normalize: true, margin: 0.03 }, opt || {});
  if (!kin || !kin.ok || !kin.model) return { ok: false, error: 'กด “คำนวณมุมข้อต่อ” ก่อน' };
  const mass = +opt.mass;
  if (!(mass >= 20 && mass <= 300)) return { ok: false, error: 'ระบุน้ำหนัก (มวลกาย) 20–300 kg ในช่อง Anthropometry' };
  const plates = ds.plates.filter(p => p.supported && p.F && p.cop && p.corners);
  if (!plates.length) return { ok: false, error: 'ไม่มีข้อมูล force plate' };
  const sex = String(opt.sex || 'M').toUpperCase().startsWith('F') ? 'F' : 'M', md = kin.model, PIG = md.PIG, nF = ds.nFrames, fs = ds.rate, warnings = [];
  const fcK = +opt.cutoff || 0, fcF = opt.cutoffGRF == null || opt.cutoffGRF === '' ? fcK : +opt.cutoffGRF || 0;
  const LONG = PIG ? 2 : 1, ML = PIG ? 1 : 2, segs = bsipFor(md, mass, sex);
  const SK = {};
  for (const k of kin.segNames) SK[k] = segKinematics(kin.frames[k], nF, fs, fcK, segs[k] ? segs[k].cLoc : null, LONG, ML);
  /* GRF: threshold, wrench about the lab origin, filter */
  const G = plates.map(pl => {
    const on = new Uint8Array(nF), F = [0, 1, 2].map(() => new Float64Array(nF)), Mo = [0, 1, 2].map(() => new Float64Array(nF)), all = new Uint8Array(nF).fill(1);
    for (let f = 0; f < nF; f++) {
      if (!(pl.F[f * 3 + 2] > opt.fzThr)) continue;
      on[f] = 1;
      const Fv = [pl.F[f * 3], pl.F[f * 3 + 1], pl.F[f * 3 + 2]], cp = [pl.cop[f * 3], pl.cop[f * 3 + 1], pl.cop[f * 3 + 2]], T = pl.T ? [pl.T[f * 3], pl.T[f * 3 + 1], pl.T[f * 3 + 2]] : [0, 0, 0];
      const m = V.add(V.cross(cp, Fv), T);
      for (let a = 0; a < 3; a++) { F[a][f] = Fv[a]; Mo[a][f] = m[a]; }
    }
    return { pl, geo: plateGeom(pl), on, F: F.map(x => lowpassRuns(x, all, fs, fcF)), Mo: Mo.map(x => lowpassRuns(x, all, fs, fcF)) };
  });
  /* plate -> foot assignment */
  const load = { R: { F: new Float64Array(nF * 3), M: new Float64Array(nF * 3), on: new Uint8Array(nF) }, L: { F: new Float64Array(nF * 3), M: new Float64Array(nF * 3), on: new Uint8Array(nF) } };
  const amb = new Uint8Array(nF); let nAmb = 0, nUn = 0, freeMissing = plates.some(p => !p.T);
  for (let f = 0; f < nF; f++) {
    const feet = {};
    for (const s of ['R', 'L']) { const fr = frameAt(kin.frames[s + 'FT'], f); if (fr) feet[s] = [fr.o, V.add(fr.o, M3.vec(fr.R, segs[s + 'FT'].cLoc))]; }
    for (const g of G) {
      if (!g.on[f]) continue;
      /* foot CoM over the plate: strictly inside, or within the margin band around its edge */
      const over = (s, mg) => { if (!feet[s]) return false; const d = V.sub(feet[s][1], g.geo.cen); return Math.abs(V.dot(d, g.geo.ex)) <= g.geo.hx + mg && Math.abs(V.dot(d, g.geo.ey)) <= g.geo.hy + mg; };
      const cp = [g.pl.cop[f * 3], g.pl.cop[f * 3 + 1]], dC = t => (feet[t] ? Math.hypot(feet[t][1][0] - cp[0], feet[t][1][1] - cp[1]) : Infinity);
      let s = null;
      if (over('R', 0) && over('L', 0)) { amb[f] = 1; continue; }   // both feet on one plate: cannot be split
      const inR = over('R', opt.margin), inL = over('L', opt.margin);
      if (inR && inL) {   // one foot only in the edge band: the COP must sit clearly under one foot
        const dr = dC('R'), dl = dC('L');
        if (Math.min(dr, dl) < 0.15 && Math.min(dr, dl) < 0.5 * Math.max(dr, dl)) s = dr < dl ? 'R' : 'L'; else { amb[f] = 1; continue; }
      } else if (inR) s = 'R'; else if (inL) s = 'L';
      else {   // neither foot over the plate (tracking gap / edge): nearest foot CoM to the COP within 0.25 m
        let best = 0.25;
        for (const t of ['R', 'L']) { const d = dC(t); if (d < best) { best = d; s = t; } }
      }
      if (!s) { nUn++; continue; }
      const Ld = load[s]; Ld.on[f] = 1;
      for (let a = 0; a < 3; a++) { Ld.F[f * 3 + a] += g.F[a][f]; Ld.M[f * 3 + a] += g.Mo[a][f]; }
    }
    if (amb[f]) nAmb++;
  }
  /* Newton–Euler */
  const sc = opt.normalize ? 1 / mass : 1, joints = {}, grf = {};
  const newJ = () => ({ a: new Float32Array(nF).fill(NaN), b: new Float32Array(nF).fill(NaN), g: new Float32Array(nF).fill(NaN), Fx: new Float32Array(nF).fill(NaN), Fy: new Float32Array(nF).fill(NaN), Fz: new Float32Array(nF).fill(NaN), P: new Float32Array(nF).fill(NaN) });
  for (const [jn] of LB_JOINTS) joints[jn] = newJ();
  for (const s of ['R', 'L']) {
    grf[s] = { x: new Float32Array(nF).fill(NaN), y: new Float32Array(nF).fill(NaN), z: new Float32Array(nF).fill(NaN) };
    const chain = [[s + 'FT', s + 'Ankle', s + 'SK'], [s + 'SK', s + 'Knee', s + 'TH'], [s + 'TH', s + 'Hip', 'RPV']];
    for (let f = 0; f < nF; f++) {
      if (amb[f]) continue;
      const Ld = load[s], g3 = A => [A[f * 3], A[f * 3 + 1], A[f * 3 + 2]];
      let Fd = g3(Ld.F), Md = g3(Ld.M);   // distal wrench about the lab origin
      for (let a = 0; a < 3; a++) grf[s][['x', 'y', 'z'][a]][f] = Fd[a] * sc;
      for (const [k, jn, ref] of chain) {
        const K = SK[k], B = segs[k];
        if (!K.valid[f] || K.acc[f * 3] !== K.acc[f * 3] || K.al[f * 3] !== K.al[f * 3]) break;
        const R = K.get.R(f), c = K.get.com(f), a = K.get.acc(f), w = K.get.om(f), al = K.get.al(f), rp = K.get.o(f);
        const Il = M3.mul(M3.mul(R, [B.I[0], 0, 0, 0, B.I[1], 0, 0, 0, B.I[2]]), M3.T(R));
        const Fp = V.sub(V.mul(V.sub(a, ID_GRAV), B.m), Fd);
        const Hd = V.add(M3.vec(Il, al), V.cross(w, M3.vec(Il, w)));
        const Mdc = V.sub(Md, V.cross(c, Fd));
        const Mp = V.sub(V.sub(Hd, Mdc), V.cross(V.sub(rp, c), Fp));
        const J = joints[jn], Kr = SK[ref];
        if (Kr && Kr.valid[f]) {
          const Rr = Kr.get.R(f), Rres = opt.resolve === 'dist' ? R : Rr, mc = momentComps(jn, Mp, Rres, PIG), fl = M3.vec(M3.T(Rres), Fp);
          J.a[f] = mc[0] * sc; J.b[f] = mc[1] * sc; J.g[f] = mc[2] * sc; J.Fx[f] = fl[0] * sc; J.Fy[f] = fl[1] * sc; J.Fz[f] = fl[2] * sc;
          const wr = Kr.get.om(f); if (wr[0] === wr[0]) J.P[f] = V.dot(Mp, V.sub(w, wr)) * sc;
        }
        Fd = V.mul(Fp, -1); Md = V.sub(V.mul(Mp, -1), V.cross(rp, Fp));   // reaction on the next proximal segment, about the lab origin
      }
    }
  }
  if (nAmb) warnings.push(nAmb + ' frame มีเท้าทั้งสองข้างอยู่บน plate เดียวกัน แยกแรงไม่ได้ (ไม่มีผล ID) · ต้องใช้ plate แยกซ้าย/ขวา');
  if (nUn) warnings.push(nUn + ' plate-frame มีแรงแต่หาเท้าที่เหยียบไม่ได้ (ไม่นำมาคิด)');
  if (freeMissing) warnings.push('บาง plate ไม่มี free moment (Tz) ใช้ค่า 0');
  if (fcF !== fcK) warnings.push('cutoff GRF (' + fcF + ' Hz) ≠ kinematics (' + fcK + ' Hz) อาจทำให้ joint moment มี artefact ช่วง impact (Kristianslund et al. 2012)');
  /* Total support moment (Winter 1980): Ms = M_hip,ext + M_knee,ext + M_ankle,plantarflexor, sagittal, extensor positive.
     Sign of the reported flexion component: hip and ankle a > 0 = flexor / dorsiflexor in both conventions; knee a > 0 = flexor
     in PiG style but extensor in raw Cardan Z-X-Y (knee flexion angle is negative there). Contribution = joint / Ms at the Ms peak.
     Support moment is a stance-phase quantity: frames without ground contact on that foot are left empty. In swing the joint
     moments are not zero (the hip holds and accelerates the leg, ~16 % of body mass) but they no longer "support" the body. */
  const extSign = { Hip: -1, Knee: PIG ? -1 : 1, Ankle: -1 }, support = {};
  for (const s of ['R', 'L']) {
    const S = { Ms: new Float32Array(nF).fill(NaN), Hip: new Float32Array(nF).fill(NaN), Knee: new Float32Array(nF).fill(NaN), Ankle: new Float32Array(nF).fill(NaN) };
    let pk = -Infinity, at = -1;
    for (let f = 0; f < nF; f++) {
      if (!load[s].on[f]) continue;
      const h = extSign.Hip * joints[s + 'Hip'].a[f], k = extSign.Knee * joints[s + 'Knee'].a[f], a = extSign.Ankle * joints[s + 'Ankle'].a[f];
      if (a === a) S.Ankle[f] = a; if (k === k) S.Knee[f] = k; if (h === h) S.Hip[f] = h;
      if (h === h && k === k && a === a) { const m = h + k + a; S.Ms[f] = m; if (m > pk) { pk = m; at = f; } }
    }
    S.peak = at >= 0 ? { frame: at, t: at / fs, Ms: pk, pct: { Hip: 100 * S.Hip[at] / pk, Knee: 100 * S.Knee[at] / pk, Ankle: 100 * S.Ankle[at] / pk } } : null;
    support[s] = S;
  }
  const peaks = {};
  for (const jn in joints) { peaks[jn] = {}; for (const c of ['a', 'b', 'g', 'P']) { let lo = Infinity, hi = -Infinity; for (const v of joints[jn][c]) if (v === v) { if (v < lo) lo = v; if (v > hi) hi = v; } peaks[jn][c] = [lo, hi]; } }
  return {
    ok: true, source: kin.source, convention: kin.convention, mass, sex, normalize: !!opt.normalize, resolve: opt.resolve, cutoff: fcK, cutoffGRF: fcF, fzThr: opt.fzThr,
    units: opt.normalize ? { M: 'Nm/kg', P: 'W/kg', F: 'N/kg' } : { M: 'Nm', P: 'W', F: 'N' },
    bsip: Object.fromEntries(Object.entries(segs).map(([k, v]) => [k, { m: v.m, L: v.L, I: v.I }])),
    joints, grf, support, contact: { R: load.R.on, L: load.L.on }, ambiguous: nAmb, peaks, warnings, _seg: opt.debug ? SK : undefined, _load: opt.debug ? load : undefined, _bsip: opt.debug ? segs : undefined
  };
}

/* ---------------------------------------------------------------- Force-analysis engine
   JavaScript port of RatTongiam/Force-Analysis (biomechanics.py, impact_frequency.py): CMJ phase detection,
   metrics (Anicic et al. 2023 classification), landing impact frequency vs plate natural frequency.
   Input = force plates embedded in the C3D (full analog rate Fz), never an external force file.
   Numerics follow scipy: butter + filtfilt (odd padding, steady-state zi), rolling mean, tukey window, uniform_filter1d. */
const FA = (() => {
  const G = 9.80665;
  /* ---- complex helpers for filter design */
  const cmul = (a, b) => [a[0] * b[0] - a[1] * b[1], a[0] * b[1] + a[1] * b[0]];
  const cdiv = (a, b) => { const d = b[0] * b[0] + b[1] * b[1]; return [(a[0] * b[0] + a[1] * b[1]) / d, (a[1] * b[0] - a[0] * b[1]) / d]; };
  function polyFromRoots(r) {
    let p = [[1, 0]];
    for (const z of r) { const q = Array.from({ length: p.length + 1 }, () => [0, 0]); for (let i = 0; i < p.length; i++) { q[i][0] += p[i][0]; q[i][1] += p[i][1]; const m = cmul(p[i], z); q[i + 1][0] -= m[0]; q[i + 1][1] -= m[1]; } p = q; }
    return p.map(c => c[0]);
  }
  function butter(order, Wn) {
    const fs2 = 4, om = 4 * Math.tan(Math.PI * Wn / 2), pz = [];
    let den = [1, 0];
    for (let k = 0; k < order; k++) {
      const m = -order + 1 + 2 * k, ang = Math.PI * m / (2 * order), p = [-Math.cos(ang) * om, -Math.sin(ang) * om];
      pz.push(cdiv([fs2 + p[0], p[1]], [fs2 - p[0], -p[1]]));
      den = cmul(den, [fs2 - p[0], -p[1]]);
    }
    const kz = Math.pow(om, order) * cdiv([1, 0], den)[0];
    const a = polyFromRoots(pz), b = polyFromRoots(new Array(order).fill([-1, 0])).map(v => v * kz);
    return { b, a };
  }
  function lfilter(b, a, x, zi) {
    const n = Math.max(a.length, b.length) - 1, a0 = a[0], bb = b.map(v => v / a0), aa = a.map(v => v / a0);
    const z = zi ? Array.from(zi) : new Array(n).fill(0), y = new Float64Array(x.length);
    for (let i = 0; i < x.length; i++) {
      const xi = x[i], yi = z[0] + bb[0] * xi;
      for (let k = 0; k < n - 1; k++) z[k] = z[k + 1] + bb[k + 1] * xi - aa[k + 1] * yi;
      z[n - 1] = bb[n] * xi - aa[n] * yi; y[i] = yi;
    }
    return y;
  }
  function solve(A, B) {
    const n = B.length, M = A.map((r, i) => r.concat([B[i]]));
    for (let c = 0; c < n; c++) {
      let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
      [M[c], M[p]] = [M[p], M[c]];
      for (let r = 0; r < n; r++) if (r !== c) { const f = M[r][c] / M[c][c]; for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k]; }
    }
    return M.map((r, i) => r[n] / r[i]);
  }
  function lfilterZi(b, a) {
    const n = a.length - 1, a0 = a[0], aa = a.map(v => v / a0), bb = b.map(v => v / a0);
    const A = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));
    // companion(a)^T : C[0][j] = -a[j+1], C[i][i-1] = 1 ; subtract transpose
    for (let j = 0; j < n; j++) A[j][0] += aa[j + 1];            // (C^T)[j][0] = C[0][j] = -aa[j+1]
    for (let i = 1; i < n; i++) A[i - 1][i] -= 1;                // (C^T)[i-1][i] = C[i][i-1] = 1
    const B = []; for (let i = 0; i < n; i++) B.push(bb[i + 1] - aa[i + 1] * bb[0]);
    return solve(A, B);
  }
  function filtfilt(b, a, x, padlen) {
    const n = x.length, e = padlen, ext = new Float64Array(n + 2 * e);
    for (let i = 0; i < e; i++) ext[i] = 2 * x[0] - x[e - i];
    for (let i = 0; i < n; i++) ext[e + i] = x[i];
    for (let i = 0; i < e; i++) ext[e + n + i] = 2 * x[n - 1] - x[n - 2 - i];
    const zi = lfilterZi(b, a);
    let y = lfilter(b, a, ext, zi.map(v => v * ext[0]));
    const r = new Float64Array(y.length); for (let i = 0; i < y.length; i++) r[i] = y[y.length - 1 - i];
    y = lfilter(b, a, r, zi.map(v => v * r[0]));
    const out = new Float64Array(n); for (let i = 0; i < n; i++) out[i] = y[y.length - 1 - e - i];
    return out;
  }
  function butterLowpass(data, cutoff, fs, order = 4) {
    const n = data.length; if (n <= order * 3) return Float64Array.from(data);
    const wn = Math.max(0.01, Math.min(0.99, cutoff / (0.5 * fs))), { b, a } = butter(order, wn);
    return filtfilt(b, a, data, Math.min(n - 1, Math.trunc(1.0 * fs)));
  }
  function movingAvg(x, w) {
    const n = x.length, out = new Float64Array(n), lo = Math.floor((w - 1) / 2), hi = w - 1 - lo;
    for (let i = 0; i < n; i++) { let s = 0, c = 0; for (let k = Math.max(0, i - lo); k <= Math.min(n - 1, i + hi); k++) { s += x[k]; c++; } out[i] = s / c; }
    return out;
  }
  function applyFilter(x, type, cutoff, fs, win = 15) {
    if (!x.length) return Float64Array.from(x);
    if (type === 'butter') return butterLowpass(x, cutoff, fs, 4);
    if (type === 'ma') return win <= 1 ? Float64Array.from(x) : movingAvg(x, win);
    return Float64Array.from(x);
  }
  /* ---- small numerics */
  const mean = (a, s = 0, e = a.length) => { if (e <= s) return 0; let t = 0; for (let i = s; i < e; i++) t += a[i]; return t / (e - s); };
  const std = (a, s, e) => { const m = mean(a, s, e); let t = 0; for (let i = s; i < e; i++) t += (a[i] - m) ** 2; return Math.sqrt(t / Math.max(1, e - s)); };
  const maxOf = (a, s, e) => { let m = -Infinity; for (let i = s; i < e; i++) if (a[i] > m) m = a[i]; return isFinite(m) ? m : 0; };
  const argmax = (a, s, e) => { let k = s, m = -Infinity; for (let i = s; i < e; i++) if (a[i] > m) { m = a[i]; k = i; } return k; };
  const argmin = (a, s, e) => { let k = s, m = Infinity; for (let i = s; i < e; i++) if (a[i] < m) { m = a[i]; k = i; } return k; };
  const median = a => { const b = Array.from(a).sort((x, y) => x - y), n = b.length; return n % 2 ? b[(n - 1) / 2] : (b[n / 2 - 1] + b[n / 2]) / 2; };
  function trap(a, s, e, dt) { const n = e - s; if (n < 2) return n === 1 ? a[s] * dt : 0; let t = 0; for (let i = s + 1; i < e; i++) t += 0.5 * (a[i - 1] + a[i]); return t * dt; }
  const sl_ = (a, s, e) => a.subarray ? a.subarray(Math.max(0, s), Math.max(0, e)) : a.slice(Math.max(0, s), Math.max(0, e));
  const deficit = (l, r) => { const m = Math.max(Math.abs(l), Math.abs(r)); return m === 0 ? '-' : (((l - r) / m * 100) >= 0 ? '+' : '') + ((l - r) / m * 100).toFixed(1) + '%'; };

  function findQuietWindow(f, fs, winS = 0.5, minBw = 200, maxCv = 3, relTol = 0.15) {
    const n = f.length, w = Math.round(winS * fs), out = { ok: false, start: null, end: null, bw: null, sd: null, cv: null, warnings: [] };
    if (n < w + 1 || w < 2) { out.warnings.push('ไฟล์สั้นเกินไปสำหรับหา quiet window'); return out; }
    const loaded = []; for (let i = 0; i < n; i++) if (f[i] > 100) loaded.push(f[i]);
    if (loaded.length < w) { out.warnings.push('แทบไม่มีช่วงที่มีโหลดบนแผ่น (> 100 N)'); return out; }
    const ref = median(loaded), step = Math.max(1, Math.round(0.02 * fs));
    const c1 = new Float64Array(n + 1), c2 = new Float64Array(n + 1);
    for (let i = 0; i < n; i++) { c1[i + 1] = c1[i] + f[i]; c2[i + 1] = c2[i] + f[i] * f[i]; }
    let best = -1, bestCv = Infinity, bm = 0, bs = 0;
    for (let s = 0; s <= n - w; s += step) {
      const m = (c1[s + w] - c1[s]) / w, sd = Math.sqrt(Math.max((c2[s + w] - c2[s]) / w - m * m, 0)), cv = sd / Math.max(m, 1e-9) * 100;
      if (m >= minBw && Math.abs(m - ref) <= relTol * ref && cv < bestCv) { best = s; bestCv = cv; bm = m; bs = sd; }
    }
    if (best < 0) { out.warnings.push(`ไม่พบช่วง ${winS.toFixed(1)} s ที่แรงรวมนิ่งใกล้ระดับโหลดปกติ (${ref.toFixed(0)} N)`); return out; }
    Object.assign(out, { start: best, end: best + w, bw: bm, sd: bs, cv: bestCv });
    if (bm < minBw) out.warnings.push(`BW ${bm.toFixed(0)} N ต่ำกว่า ${minBw} N`);
    else if (bestCv > maxCv) out.warnings.push(`ช่วงยืนนิ่งที่ดีที่สุดยังมี CV ${bestCv.toFixed(1)}% (> ${maxCv}%)`);
    else out.ok = true;
    return out;
  }
  function bwReference(sf, sl, sr, q0, q1, bwOverride) {
    q0 = Math.max(0, q0 | 0); q1 = Math.max(q0 + 1, q1 | 0);
    const bw = mean(sf, q0, q1);
    if (bwOverride) { let sh = bw > 50 ? mean(sl, q0, q1) / bw : 0.5; sh = Math.min(Math.max(sh, 0), 1); return { bw: bwOverride, sd: 0, l: bwOverride * sh, r: bwOverride * (1 - sh) }; }
    return { bw, sd: std(sf, q0, q1), l: mean(sl, q0, q1), r: mean(sr, q0, q1) };
  }
  /* ---- phase detection (detect_phases_sequential) */
  function detectPhases(sfRaw, dt, o) {
    const n = sfRaw.length, fs = 1 / dt;
    const sf = applyFilter(sfRaw, o.filterType, o.cutoff, fs);
    const q0 = Math.max(0, o.quiet[0] | 0), q1 = Math.min(n, o.quiet[1] | 0);
    const bw = o.bwOverride ? o.bwOverride : mean(sf, q0, q1), fsd = std(sf, q0, q1), mass = bw > 0 ? bw / G : 70;
    // flight blocks: sf < 25 N
    let tAuto = null, lAuto = null, bestLen = 0;
    for (let i = 0; i < n;) {
      if (sf[i] < 25) { let j = i; while (j + 1 < n && sf[j + 1] < 25) j++; const len = j - i + 1; if (len > Math.trunc(0.10 * fs) && i > Math.trunc(0.5 * fs) && len > bestLen) { bestLen = len; tAuto = i; lAuto = j; } i = j + 1; } else i++;
    }
    if (tAuto === null) { const gp = Math.trunc(0.5 * fs) + argmax(sf, Math.trunc(0.5 * fs), n) - Math.trunc(0.5 * fs); tAuto = Math.max(Math.trunc(0.5 * fs), gp - Math.trunc(0.2 * fs)); lAuto = Math.min(n - 1, tAuto + Math.trunc(0.4 * fs)); }
    let offL = 0, offR = 0;
    if (o.slRaw && o.srRaw && lAuto > tAuto) {
      const a = tAuto + Math.trunc((lAuto - tAuto) * 0.25), b = tAuto + Math.trunc((lAuto - tAuto) * 0.75);
      if (b > a) { offL = mean(o.slRaw, a, b); offR = mean(o.srRaw, a, b); }
    }
    const ps = Math.max(0, tAuto - Math.trunc(1.5 * fs));
    const peakProp = tAuto > ps ? argmax(sf, ps, tAuto) : Math.max(0, tAuto - 1);
    const bAuto = peakProp > ps ? argmin(sf, ps, peakProp) : Math.max(0, peakProp - Math.trunc(0.2 * fs));
    const thr = Math.max(bw * 0.98, bw - 3 * fsd);
    let sAuto = -1; for (let i = 0; i < bAuto; i++) if (sf[i] >= thr) sAuto = i;
    if (sAuto < 0) sAuto = Math.max(0, bAuto - Math.trunc(0.4 * fs));
    const len = tAuto - sAuto + 1, vel = new Float64Array(Math.max(0, len));
    for (let k = 1; k < len; k++) vel[k] = vel[k - 1] + 0.5 * ((sf[sAuto + k - 1] - bw) / mass + (sf[sAuto + k] - bw) / mass) * dt;
    const bRel = Math.max(0, bAuto - sAuto); let zAuto = bAuto;
    for (let k = bRel; k < len; k++) if (vel[k] >= 0) { zAuto = bAuto + (k - bRel); break; }
    let lL = lAuto, lR = lAuto;
    if (o.slRaw && o.srRaw) {
      const e = Math.min(n, tAuto + Math.trunc(1.2 * fs));
      for (let i = tAuto + Math.trunc(0.05 * fs); i < e; i++) if (o.slRaw[i] - offL >= 15) { lL = i; break; }
      for (let i = tAuto + Math.trunc(0.05 * fs); i < e; i++) if (o.srRaw[i] - offR >= 15) { lR = i; break; }
    }
    return { s: Math.min(sAuto, bAuto), b: Math.min(bAuto, zAuto), z: Math.min(zAuto, tAuto), t: Math.min(tAuto, n - 1), l: Math.min(lAuto, n - 1), lL, lR, offL, offR };
  }
  function rfdStart(a, s, dt, ms) { if (!a.length || s < 0 || s >= a.length) return 0; const k = Math.trunc((ms / 1000) / dt); if (k <= 0) return 0; return s + k < a.length ? (a[s + k] - a[s]) / (ms / 1000) : 0; }
  function rfdWin(a, s, e, dt, winMs = 20) {
    if (!a.length || s < 0 || e >= a.length || e <= s) return 0;
    const w = Math.max(1, Math.trunc((winMs / 1000) / dt)); if (e - s <= w) return 0;
    let m = -Infinity; for (let i = s; i + w <= e; i++) { const v = (a[i + w] - a[i]) / (w * dt); if (v > m) m = v; } return isFinite(m) ? m : 0;
  }
  /* ---- metrics (calculate_metrics, group_by="phase") */
  function calcMetrics(sfRaw, slRaw, srRaw, dt, P, o) {
    const n = sfRaw.length, fs = 1 / dt, t = i => i * dt;
    const slz = slRaw.map(v => v - o.offL), srz = srRaw.map(v => v - o.offR), sfz = slz.map((v, i) => v + srz[i]);
    const sf = applyFilter(sfz, o.filterType, o.cutoff, fs), sl = applyFilter(slz, o.filterType, o.cutoff, fs), sr = applyFilter(srz, o.filterType, o.cutoff, fs);
    const q0 = Math.max(0, o.quiet[0] | 0), q1 = Math.min(n, o.quiet[1] | 0);
    const ref = bwReference(sf, sl, sr, q0, q1, o.bwOverride), bw = ref.bw, mass = bw > 0 ? bw / G : 70;
    const { s: sI, b: bI, z: zI, t: tI, l: lI } = P;
    const vel = new Float64Array(n), disp = new Float64Array(n);
    for (let i = sI + 1; i < Math.min(tI + 1, n); i++) {
      vel[i] = vel[i - 1] + 0.5 * ((sf[i - 1] - bw) / mass + (sf[i] - bw) / mass) * dt;
      disp[i] = disp[i - 1] + 0.5 * (vel[i - 1] + vel[i]) * dt;
    }
    const contraction = Math.max(dt, t(tI) - t(sI)), propDur = Math.max(0, t(tI) - t(zI)), unwDur = Math.max(0, t(bI) - t(sI)), brkDur = Math.max(0, t(zI) - t(bI)), fltDur = Math.max(0, t(lI) - t(tI));
    const jhFlight = G * fltDur * fltDur / 8 * 100, vTO = vel[tI], jhImp = vTO * vTO / (2 * G) * 100;
    const rsi = contraction > 0 ? jhImp / 100 / contraction : 0;
    const e1 = Math.min(tI + 1, n), e2 = Math.min(zI + 1, n), e3 = Math.min(bI + 1, n);
    const peakVprop = e1 > zI ? maxOf(vel, zI, e1) : 0;
    let peakVneg = 0; if (e2 > sI) { peakVneg = Infinity; for (let i = sI; i < e2; i++) if (vel[i] < peakVneg) peakVneg = vel[i]; }
    const uImp = trap(sf, sI, e3, dt), uImpL = trap(sl, sI, e3, dt), uImpR = trap(sr, sI, e3, dt);
    const bF = sl_(sf, bI, e2), bFL = sl_(sl, bI, e2), bFR = sl_(sr, bI, e2), bP = bF.map((v, i) => v * vel[bI + i]);
    const avgBP = mean(bP, 0, bP.length), bImp = trap(sf, bI, e2, dt), bImpL = trap(sl, bI, e2, dt), bImpR = trap(sr, bI, e2, dt);
    const rfdB = bF.length > 1 ? (bF[bF.length - 1] - bF[0]) / (bF.length * dt) : 0, rfdBL = bFL.length > 1 ? (bFL[bFL.length - 1] - bFL[0]) / (bFL.length * dt) : 0, rfdBR = bFL.length > 1 ? (bFR[bFR.length - 1] - bFR[0]) / (bFL.length * dt) : 0;
    const pF = sl_(sf, zI, e1), pFL = sl_(sl, zI, e1), pFR = sl_(sr, zI, e1), pP = pF.map((v, i) => v * vel[zI + i]);
    const mx = a => a.length ? maxOf(a, 0, a.length) : 0, mn = a => mean(a, 0, a.length);
    const pImp = trap(sf, zI, e1, dt), pImpL = trap(sl, zI, e1, dt), pImpR = trap(sr, zI, e1, dt);
    const posImp = bImp + pImp, posImpL = bImpL + pImpL, posImpR = bImpR + pImpR;
    const pkIdx = argmax(sf, sI, tI + 1), ttp = (t(pkIdx) - t(sI)) * 1000;
    const i50 = Math.min(n, zI + Math.trunc(0.05 / dt)), i100 = Math.min(n, zI + Math.trunc(0.10 / dt));
    const sub = (a, s, e, b) => { let r = 0; if (e > s) { const tmp = new Float64Array(e - s); for (let i = s; i < e; i++) tmp[i - s] = a[i] - b; r = trap(tmp, 0, tmp.length, dt); } return r; };
    const p1 = sub(sf, zI, i50, bw), p1L = sub(sl, zI, i50, ref.l), p1R = sub(sr, zI, i50, ref.r);
    const p2 = sub(sf, i50, i100, bw), p2L = sub(sl, i50, i100, ref.l), p2R = sub(sr, i50, i100, ref.r);
    let lI0 = 0, lIL = 0, lIR = 0;
    if (lI > tI && lI < n) { const le = Math.min(n, lI + Math.trunc(0.5 / dt)); lI0 = trap(sf, lI, le, dt); lIL = trap(sl, lI, le, dt); lIR = trap(sr, lI, le, dt); }
    const comDepth = Math.abs(disp[zI] - disp[sI]) * 100, comTO = disp[tI] * 100, stiff = comDepth > 0 ? mx(bF) / (comDepth / 100) : 0, fjr = contraction > 0 ? fltDur / contraction : 0;
    const tdDelay = (o.lIdxL != null && o.lIdxR != null) ? ((t(o.lIdxL) - t(o.lIdxR)) * 1000) : null;
    const f0 = (v, d) => Number(v).toFixed(d), row = (name, l, r, tot, d) => ({ name, l, r, t: tot, d: (l === '-' || r === '-') ? '-' : deficit(parseFloat(l), parseFloat(r)) });
    const dash = '-', LR = (a, b, d) => [f0(a, d), f0(b, d)];
    const R = (name, l, r, tot, d) => ({ name, l: l === null ? dash : f0(l, d), r: r === null ? dash : f0(r, d), t: tot, d: (l === null || r === null) ? dash : deficit(l, r) });
    const T = (v, d) => f0(v, d);
    const groups = [
      { title: '1. Weighing & Onset', rows: [
        R('Body Mass (kg)', ref.l / G, ref.r / G, T(mass, 1), 1), R('Body Weight (N)', ref.l, ref.r, T(bw, 0), 0), R('Time to Peak Force (ms)', null, null, T(ttp, 0), 0)] },
      { title: '2. Unweighting', rows: [
        R('Unloading Duration (s)', null, null, T(unwDur, 3), 3), R('Unloading Impulse (N·s)', uImpL, uImpR, T(uImp, 0), 0), R('Peak Negative Velocity (m/s)', null, null, T(peakVneg, 2), 2)] },
      { title: '3. Braking (Eccentric)', rows: [
        R('Braking Duration (s)', null, null, T(brkDur, 3), 3), R('Peak Force (N)', mx(bFL), mx(bFR), T(mx(bF), 0), 0), R('Mean Force (N)', mn(bFL), mn(bFR), T(mn(bF), 0), 0),
        R('Braking Impulse (N·s)', bImpL, bImpR, T(bImp, 0), 0), R('Eccentric Braking RFD (N/s)', rfdBL, rfdBR, T(rfdB, 0), 0), R('Mean Power (W)', null, null, T(Math.abs(avgBP), 0), 0)] },
      { title: '4. Propulsive (Concentric)', rows: [
        R('Propulsive Duration (s)', null, null, T(propDur, 3), 3), R('Peak Force (N)', mx(pFL), mx(pFR), T(mx(pF), 0), 0), R('Mean Force (N)', mn(pFL), mn(pFR), T(mn(pF), 0), 0),
        R('Propulsive Impulse (N·s)', pImpL, pImpR, T(pImp, 0), 0), R('Peak Propulsive Power (W)', null, null, T(mx(pP), 0), 0), R('Mean Propulsive Power (W)', null, null, T(mn(pP), 0), 0),
        R('P1 Concentric Impulse 0–50 ms (N·s)', p1L, p1R, T(p1, 1), 1), R('P2 Concentric Impulse 50–100 ms (N·s)', p2L, p2R, T(p2, 1), 1)] },
      { title: '5. Flight & Performance', rows: [
        R('Jump Height – Impulse-Momentum (cm)', null, null, T(jhImp, 1), 1), R('Jump Height – Flight Time (cm)', null, null, T(jhFlight, 1), 1), R('Flight Duration (s)', null, null, T(fltDur, 3), 3),
        R('Take-off Velocity (m/s)', null, null, T(vTO, 2), 2), R('Peak Propulsive Velocity (m/s)', null, null, T(peakVprop, 2), 2), R('RSI Modified (AU)', null, null, T(rsi, 2), 2),
        R('Positive Impulse (N·s)', posImpL, posImpR, T(posImp, 0), 0), R('COM Height at Take-off (cm)', null, null, T(comTO, 1), 1)] },
      { title: '6. Landing & Strategy', rows: [
        R('Landing Impulse (N·s)', lIL, lIR, T(lI0, 0), 0),
        { name: 'Initial Touchdown Delay (L−R)', l: o.lIdxL ? f0(t(o.lIdxL), 3) + ' s' : dash, r: o.lIdxR ? f0(t(o.lIdxR), 3) + ' s' : dash, t: tdDelay === null ? dash : (tdDelay >= 0 ? '+' : '') + f0(tdDelay, 1) + ' ms', d: dash },
        R('Countermovement COM Depth (cm)', null, null, T(comDepth, 1), 1), R('Leg Stiffness (N/m)', null, null, stiff > 0 ? T(stiff, 0) : 'N/A', 0), R('Flight-to-Jump Time Ratio (AU)', null, null, T(fjr, 2), 2)] }
    ];
    return { groups, sf, sl, sr, vel, bw, bwL: ref.l, bwR: ref.r, mass, jhImp, jhFlight, vTO, fltDur, num: { ttp, unwDur, brkDur, propDur, jhImp, jhFlight, vTO, rsi, comDepth, stiff, bImp, pImp, lI0, peakBF: mx(bF), peakPF: mx(pF), posImp, uImp, rfdB, avgBP, peakPP: mx(pP) } };
  }
  /* ---- impact frequency (impact_frequency.py) */
  function fftReal(x, nfft) {
    const re = new Float64Array(nfft), im = new Float64Array(nfft); re.set(x.subarray ? x : Float64Array.from(x));
    for (let i = 1, j = 0; i < nfft; i++) { let bit = nfft >> 1; for (; j & bit; bit >>= 1) j ^= bit; j ^= bit; if (i < j) { [re[i], re[j]] = [re[j], re[i]]; } }
    for (let len = 2; len <= nfft; len <<= 1) {
      const ang = -2 * Math.PI / len, wr = Math.cos(ang), wi = Math.sin(ang);
      for (let i = 0; i < nfft; i += len) { let cr = 1, ci = 0; for (let k = 0; k < len / 2; k++) { const a = i + k, b = i + k + len / 2, tr = re[b] * cr - im[b] * ci, ti = re[b] * ci + im[b] * cr; re[b] = re[a] - tr; im[b] = im[a] - ti; re[a] += tr; im[a] += ti; const nr = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = nr; } }
    }
    return { re, im };
  }
  function tukey(M, alpha) {
    const w = new Float64Array(M), width = Math.floor(alpha * (M - 1) / 2);
    for (let n = 0; n < M; n++) {
      if (n <= width) w[n] = 0.5 * (1 + Math.cos(Math.PI * (-1 + 2 * n / alpha / (M - 1))));
      else if (n < M - width - 1) w[n] = 1;
      else w[n] = 0.5 * (1 + Math.cos(Math.PI * (-2 / alpha + 1 + 2 * n / alpha / (M - 1))));
    }
    return w;
  }
  function ampSpectrum(x, dt, nfft) {
    const m = mean(x, 0, x.length), w = tukey(x.length, 0.2), y = new Float64Array(x.length);
    for (let i = 0; i < x.length; i++) y[i] = (x[i] - m) * w[i];
    const { re, im } = fftReal(y, nfft), nb = nfft / 2 + 1, f = new Float64Array(nb), a = new Float64Array(nb);
    for (let k = 0; k < nb; k++) { f[k] = k / (nfft * dt); a[k] = Math.hypot(re[k], im[k]) * dt; }
    return { f, a };
  }
  function uniform1d(x, size) {
    const n = x.length, lo = Math.floor(size / 2), hi = size - 1 - lo, out = new Float64Array(n);
    const at = i => { if (i < 0) i = -i - 1; if (i >= n) i = 2 * n - i - 1; return x[Math.min(Math.max(i, 0), n - 1)]; };
    let s = 0; for (let k = -lo; k <= hi; k++) s += at(k);
    for (let i = 0; i < n; i++) { out[i] = s / size; s += at(i + hi + 1) - at(i - lo); }
    return out;
  }
  function crossTime(y, k, level, dt) { const y0 = y[k], y1 = y[k + 1], d = y1 !== y0 ? y1 - y0 : 1e-12; return (k + (level - y0) / d) * dt; }
  function lastBelow(y, level, ipk) { let r = -1; for (let i = 0; i <= ipk; i++) if (y[i] < level) r = i; return r >= 0 && r < ipk ? r : null; }
  function landingImpact(f, fs, tI, lI, o = {}) {
    const pre = o.preMs ?? 40, post = o.postMs ?? 250, pk = o.peakMs ?? 100, snrK = o.snrK ?? 3, onsetFloor = o.onsetFloor ?? 10;
    const n = f.length, dt = 1 / fs, out = { notes: [] };
    const i0 = Math.max(0, lI - Math.trunc(pre * fs / 1000)), i1 = Math.min(n, lI + Math.trunc(post * fs / 1000));
    if (i1 - i0 < 32) { out.notes.push('ช่วง landing สั้นเกินไปสำหรับ FFT'); return out; }
    const fl0 = tI + Math.trunc(0.25 * (lI - tI)), fl1 = tI + Math.trunc(0.75 * (lI - tI));
    const noise = (fl1 - fl0) >= 30 ? f.subarray(fl0, fl1) : null, sigma = noise ? std(f, fl0, fl1) : null;
    if (!noise) out.notes.push('flight สั้นเกินไป ใช้ noise floor ไม่ได้ (ข้าม f_snr)');
    const jEnd = Math.min(n, lI + Math.trunc(pk * fs / 1000)), iPk = argmax(f, i0, jEnd), peak = f[iPk];
    const onsetLvl = sigma ? Math.max(onsetFloor, 5 * sigma) : onsetFloor;
    const k10 = lastBelow(f, 0.10 * peak, iPk), k90 = lastBelow(f, 0.90 * peak, iPk), kOn = lastBelow(f, onsetLvl, iPk);
    out.peak = peak; out.tPeakIdx = iPk;
    if (k10 !== null && k90 !== null && kOn !== null) {
      const t10 = crossTime(f, k10, 0.10 * peak, dt), t90 = crossTime(f, k90, 0.90 * peak, dt), tOn = crossTime(f, kOn, onsetLvl, dt), tr = t90 - t10, tp = iPk * dt - tOn;
      out.tRiseMs = tr * 1000; out.tPeakMs = tp * 1000; out.fRise = tr > 0 ? 0.35 / tr : null; out.fDur = tp > 0 ? 1 / (4 * tp) : null;
      if (tr < 5 * dt) out.notes.push(`t_rise ${(tr * 1000).toFixed(1)} ms < 5 sample (${(5 * dt * 1000).toFixed(0)} ms): f_rise เป็นค่าประมาณหยาบ`);
    } else out.notes.push('หา onset/10%/90% ไม่ได้ (สัญญาณเริ่มสูงอยู่แล้ว หรือไม่มี impact ชัด)');
    const seg = f.subarray(i0, i1), nfft = 1 << Math.ceil(Math.log2(seg.length * 4)), sp = ampSpectrum(seg, dt, nfft), df = sp.f[1] - sp.f[0];
    let tot = 0; const cum = new Float64Array(sp.a.length); for (let i = 0; i < cum.length; i++) { tot += sp.a[i] ** 2; cum[i] = tot; }
    let ic = 0; while (ic < cum.length && cum[ic] / tot < 0.99) ic++; out.fC99 = sp.f[Math.min(sp.f.length - 1, ic)];
    out.freqs = sp.f; out.amp = sp.a;
    if (noise) {
      const ns = ampSpectrum(noise, dt, nfft), vals = []; for (let i = 0; i < sp.f.length; i++) if (sp.f[i] > 20) vals.push(ns.a[i]);
      const floor = median(vals) * Math.sqrt(seg.length / noise.length), thr = snrK * floor, sm = uniform1d(sp.a, Math.max(1, Math.round(5 / df)));
      let start = 0, bm = -Infinity; for (let i = 0; i < sm.length; i++) { const v = sp.f[i] >= 1 ? sm[i] : 0; if (v > bm) { bm = v; start = i; } }
      let found = -1; for (let i = start; i < sm.length; i++) if (sm[i] < thr) { found = i; break; }
      out.noiseThr = thr;
      if (found >= 0) out.fSnr = sp.f[found]; else { out.fSnr = fs / 2; out.notes.push('spectrum ไม่ตกต่ำกว่า noise floor ก่อน Nyquist: f_snr = Nyquist (อาจมี aliasing/noise สูง)'); }
    }
    return out;
  }
  function plateCheck(f, fn, lowRatio = 10, flagRatio = 4.5) {
    const r = f / fn; if (r >= 1) return { ratio: 1 / r, err: Infinity, status: 'Flag' };
    const err = (1 / (1 - r * r) - 1) * 100, ratio = fn / f;
    return { ratio, err, status: ratio >= lowRatio ? 'Low' : (ratio >= flagRatio ? 'Monitor' : 'Flag') };
  }
  function impactRows(channels, fs, tI, lI, fn) {
    const rows = [], fm = (v, d = 1) => v == null ? '-' : Number(v).toFixed(d);
    for (const [name, x] of Object.entries(channels)) {
      const r = landingImpact(x, fs, tI, lI), cand = [r.fC99, r.fRise, r.fDur].filter(v => v), worst = cand.length ? Math.max(...cand) : null, chk = (fn && worst) ? plateCheck(worst, fn) : null;
      rows.push({ channel: name, peak: fm(r.peak, 0), rise: fm(r.tRiseMs), fRise: fm(r.fRise), fDur: fm(r.fDur), fC99: fm(r.fC99), fUsed: fm(worst), ratio: chk ? fm(chk.ratio) : '-', err: chk ? (isFinite(chk.err) ? fm(chk.err, 2) : '>100') : '-', status: chk ? chk.status : 'fn not set', notes: r.notes, _worst: worst, _chk: chk });
    }
    return rows;
  }
  /* ---- orchestration for plates inside a C3D dataset */
  function analyzePlates(ds, o = {}) {
    const pls = ds.plates.filter(p => p.supported && p.fzFull);
    if (!pls.length) return { ok: false, error: 'ไม่มี force plate ที่ใช้ได้ใน C3D นี้ (ต้องเป็น FP_TYPE 2/4 และมี analog)' };
    const fs = ds.info.analogRate; if (!(fs > 0)) return { ok: false, error: 'ไม่พบ analog rate' };
    const dt = 1 / fs;
    let L = pls[0], Rr = null, lr = false;
    if (pls.length >= 2) {
      const iL = o.leftPlate != null ? pls.findIndex(p => p.index === o.leftPlate) : 0, iR = o.rightPlate != null ? pls.findIndex(p => p.index === o.rightPlate) : 1;
      if (iL < 0 || iR < 0 || iL === iR) return { ok: false, error: 'เลือก plate ซ้าย/ขวาไม่ถูกต้อง (ต้องเป็นคนละแผ่น)' };
      L = pls[iL]; Rr = pls[iR]; lr = true;
    }
    const n = L.fzFull.length, fl = Float64Array.from(L.fzFull), fr = lr ? Float64Array.from(Rr.fzFull) : new Float64Array(n), ft = fl.map((v, i) => v + fr[i]);
    const g = G, ftype = o.filterType || 'butter', cutoff = o.cutoff || 50, warnings = [];
    const auto = findQuietWindow(ft, fs);
    let q0, q1, bwOverride = null, src;
    if (o.bwMethod === 'mass' && o.mass > 0) { bwOverride = o.mass * g; q0 = auto.start ?? 0; q1 = auto.end ?? Math.min(n, Math.round(0.5 * fs)); src = 'น้ำหนักที่ระบุ ' + o.mass.toFixed(1) + ' kg'; }
    else if (o.bwMethod === 'window' && o.window) { q0 = Math.round(o.window[0] * fs); q1 = Math.round(o.window[1] * fs) + 1; src = `window ${o.window[0].toFixed(2)}–${o.window[1].toFixed(2)} s`; }
    else { q0 = auto.start ?? 0; q1 = auto.end ?? Math.min(n, Math.round(0.5 * fs)); src = auto.start != null ? `auto window ${(q0 * dt).toFixed(2)}–${((q1 - 1) * dt).toFixed(2)} s` : 'ไม่พบ window อัตโนมัติ (ใช้ 0.5 s แรก)'; if (!auto.ok) warnings.push('หา quiet-standing window อัตโนมัติไม่ผ่านเกณฑ์: ' + auto.warnings.join('; ')); }
    q0 = Math.max(0, Math.min(q0, n - 1)); q1 = Math.max(q0 + 1, Math.min(q1, n));
    const qm = mean(ft, q0, q1), qcv = qm > 0 ? std(ft, q0, q1) / qm * 100 : Infinity, bwRef = bwOverride || qm;
    const block = [];
    if (bwRef < 200) block.push(`BW ที่ใช้ = ${bwRef.toFixed(1)} N (< 200 N) ช่วงที่เลือกน่าจะไม่มีคนยืนบนแผ่น`);
    if (!bwOverride && (q1 - q0) * dt < 0.25) block.push('ช่วง quiet window สั้นกว่า 0.25 s');
    if (block.length) return { ok: false, error: 'หยุดการคำนวณเพื่อไม่ให้ได้ค่าที่ผิดปกติ: ' + block.join('; ') + ' · เลือกช่วงยืนนิ่งเอง หรือใส่น้ำหนักตัว', warnings, fs, dt, ft, fl, fr, lr, qWin: [q0, q1] };
    if (!bwOverride && qcv > 3) warnings.push(`ช่วงที่เลือกยังไม่นิ่งมาก (CV ${qcv.toFixed(1)}% > 3%)`);
    const quiet = [q0, q1];
    const ph = detectPhases(ft, dt, { filterType: ftype, cutoff, slRaw: fl, srRaw: fr, quiet, bwOverride });
    let P = { s: ph.s, b: ph.b, z: ph.z, t: ph.t, l: ph.l };
    if (o.phaseTimes) { const pt = o.phaseTimes, c = v => Math.min(n - 1, Math.max(0, Math.round(v * fs))); P = { s: c(pt.s), b: c(pt.b), z: c(pt.z), t: c(pt.t), l: c(pt.l) }; }
    const bad = !(P.s < P.b && P.b < P.z && P.z < P.t && P.t < P.l); if (bad) warnings.push('ลำดับเวลา phase ไม่เรียงจาก start < braking < split < take-off < landing');
    const m = calcMetrics(ft, fl, fr, dt, P, { offL: ph.offL, offR: ph.offR, filterType: ftype, cutoff, quiet, bwOverride, lIdxL: lr ? ph.lL : null, lIdxR: lr ? ph.lR : null });
    if (!lr) for (const gp of m.groups) for (const r of gp.rows) { r.l = '-'; r.r = '-'; r.d = '-'; }
    const flz = fl.map(v => v - ph.offL), frz = fr.map(v => v - ph.offR), ftz = flz.map((v, i) => v + frz[i]);
    let impact = [], impactErr = null;
    try { const ch = lr ? { Left: flz, Right: frz, Total: ftz } : { Total: ftz }; impact = impactRows(ch, fs, P.t, P.l, o.fn > 0 ? o.fn : null); } catch (e) { impactErr = e.message; }
    if (!(m.fltDur > 0.05)) warnings.push('ไม่พบ flight phase ที่ชัดเจน (ช่วงที่แรง < 25 N ยาว > 0.1 s หลัง 0.5 s แรก) · ข้อมูลนี้อาจไม่ใช่การกระโดด CMJ ค่า phase และ metric จึงอาจไม่มีความหมาย');
    return { ok: true, fs, dt, n, lr, plates: lr ? [L.index, Rr.index] : [L.index], phases: P, auto: ph, quiet, bwSrc: src, bw: m.bw, mass: m.mass, groups: m.groups, traces: { sf: m.sf, sl: m.sl, sr: m.sr, vel: m.vel }, raw: { ft, fl, fr }, num: m.num, impact, impactErr, warnings, params: { filterType: ftype, cutoff } };
  }
  return { butter, filtfilt, butterLowpass, findQuietWindow, detectPhases, calcMetrics, landingImpact, impactRows, analyzePlates, G };
})();

const api = { MODEL_DEFS, MARKER_ALIASES, markerStem, suggestMarkerMap, markerReport, detectUpAxis, upRotation, FA, computeUpperBody, detectUB, UB_JOINTS, UB_LABELS, eulerYXYr, markerQC, QC_TOL, cycleNormalize, cycleStats, cyclesFromContact, computeIK, computeID, buildIKModel, ikForward, IK_MODELS, BSIP_DELEVA, jointEuler, rotZXYr, rotYXZr, cardanYXZ, rotYXZ, computeLowerBody, detectLBSet, LB_SETS, estimateAnthropometry, rigidFit, cardanZXY, rotZXY, LB_JOINTS, LB_DEFAULTS, readC3D, makeSyntheticCMJ, defaultLinks, splitLabel, sideOf, validRatio, MocapViewer, ChartView, AXIS, SIDE_COLOR, finishDataset };
if (typeof module !== 'undefined' && module.exports) module.exports = api; else root.MocapEngine = api;
})(typeof window !== 'undefined' ? window : globalThis);
