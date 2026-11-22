/* Minimal C3D writer for tests: float storage, Intel (84) / DEC (85) / MIPS (86) processor formats.
   Only what the reader needs: POINT, ANALOG, FORCE_PLATFORM groups. */
'use strict';

function writeC3D(o) {
  const ptype = o.processor || 84, LE = ptype !== 86;
  const nP = o.labels.length, nF = o.nFrames, na = o.analogLabels ? o.analogLabels.length : 0, spf = na ? o.spf : 0;
  const bytes = [];
  // number encoders
  const tmp = new DataView(new ArrayBuffer(4));
  const i16 = v => { tmp.setInt16(0, v, LE); return [tmp.getUint8(0), tmp.getUint8(1)]; };
  const u16 = v => { tmp.setUint16(0, v, LE); return [tmp.getUint8(0), tmp.getUint8(1)]; };
  const f32 = v => {
    if (ptype === 85) { tmp.setFloat32(0, v * 4, true); const b = [0, 1, 2, 3].map(i => tmp.getUint8(i)); return [b[2], b[3], b[0], b[1]]; }
    tmp.setFloat32(0, v, LE); return [0, 1, 2, 3].map(i => tmp.getUint8(i));
  };
  // ---- parameters
  const groups = [['POINT', 1], ['ANALOG', 2], ['FORCE_PLATFORM', 3]];
  const params = [];
  const P = (g, name, type, dims, values) => params.push({ g, name, type, dims, values });
  const chars = (g, name, list, w) => {
    w = w || Math.max(1, ...list.map(s => s.length));
    const flat = []; for (const s of list) for (let i = 0; i < w; i++) flat.push(i < s.length ? s.charCodeAt(i) : 32);
    P(g, name, -1, list.length === 1 && o.scalarChars ? [w] : [w, list.length], flat);
  };
  P(1, 'USED', 2, [], [nP]); P(1, 'SCALE', 4, [], [-1]); P(1, 'RATE', 4, [], [o.rate]);
  P(1, 'FRAMES', 2, [], [nF]); chars(1, 'LABELS', o.labels); chars(1, 'UNITS', [o.units || 'mm']);
  const dataStartParamIdx = params.length; P(1, 'DATA_START', 2, [], [0]);
  if (o.yScreen) chars(1, 'Y_SCREEN', [o.yScreen]);
  P(2, 'USED', 2, [], [na]);
  if (na) {
    P(2, 'RATE', 4, [], [o.rate * spf]); P(2, 'GEN_SCALE', 4, [], [1]);
    P(2, 'SCALE', 4, [na], o.analogScale || new Array(na).fill(1)); P(2, 'OFFSET', 2, [na], new Array(na).fill(0));
    chars(2, 'LABELS', o.analogLabels);
  }
  const fp = o.plates || [];
  P(3, 'USED', 2, [], [fp.length]);
  if (fp.length) {
    const nch = Math.max(...fp.map(p => p.channels.length));
    P(3, 'TYPE', 2, [fp.length], fp.map(p => p.type));
    P(3, 'CORNERS', 4, [3, 4, fp.length], [].concat(...fp.map(p => [].concat(...p.corners))));
    P(3, 'ORIGIN', 4, [3, fp.length], [].concat(...fp.map(p => p.origin)));
    P(3, 'CHANNEL', 2, [nch, fp.length], [].concat(...fp.map(p => p.channels.concat(new Array(nch - p.channels.length).fill(0)))));
    P(3, 'ZERO', 2, [2], o.zero || [0, 0]);
    if (fp.some(p => p.cal)) P(3, 'CAL_MATRIX', 4, [6, 6, fp.length], [].concat(...fp.map(p => p.cal || new Array(36).fill(0))));
  }
  const encParam = (p, id) => {
    const nm = [...p.name].map(c => c.charCodeAt(0));
    const body = [p.type & 0xff, p.dims.length, ...p.dims];
    for (const v of p.values) {
      if (p.type === -1) body.push(v);
      else if (p.type === 1) body.push(v & 0xff);
      else if (p.type === 2) body.push(...i16(v));
      else body.push(...f32(v));
    }
    body.push(0);   // empty description
    return [nm.length, id, ...nm, ...i16(body.length + 2), ...body];
  };
  const encGroup = (name, id) => { const nm = [...name].map(c => c.charCodeAt(0)); return [nm.length, (-id) & 0xff, ...nm, ...i16(3), 0]; };
  const buildParams = () => {
    const out = [];
    for (const [n, id] of groups) out.push(...encGroup(n, id));
    params.forEach(p => out.push(...encParam(p, p.g)));
    return out;
  };
  let pb = buildParams();
  const nPB = Math.ceil((pb.length + 4 + 4) / 512);
  params[dataStartParamIdx].values = [2 + nPB];
  pb = buildParams();
  // ---- header (block 1)
  const H = new Array(512).fill(0);
  const put = (off, arr) => arr.forEach((b, i) => { H[off + i] = b; });
  H[0] = 2; H[1] = 80;
  put(2, u16(nP)); put(4, u16(na * spf)); put(6, u16(1)); put(8, u16(nF)); put(10, u16(0));
  put(12, f32(-1)); put(16, u16(2 + nPB)); put(18, u16(spf)); put(20, f32(o.rate));
  bytes.push(...H);
  // ---- parameter section
  const PS = [1, 80, nPB, 83 + (ptype - 83)].concat(pb, [0, 0, 0, 0]);
  while (PS.length < nPB * 512) PS.push(0);
  bytes.push(...PS);
  // ---- data
  for (let f = 0; f < nF; f++) {
    for (let i = 0; i < nP; i++) {
      const p = o.points[f][i];
      if (p) bytes.push(...f32(p[0]), ...f32(p[1]), ...f32(p[2]), ...f32(0));
      else bytes.push(...f32(0), ...f32(0), ...f32(0), ...f32(-1));
    }
    for (let s = 0; s < spf; s++) for (let c = 0; c < na; c++) bytes.push(...f32(o.analog[f * spf + s][c]));
  }
  while (bytes.length % 512) bytes.push(0);
  return new Uint8Array(bytes).buffer;
}
module.exports = { writeC3D };
