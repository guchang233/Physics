import { analyzeMotion, sampleAnalyticPath } from '../analysis/motionAnalysis.ts';
import { mountMathQuill } from '../ui/mathquill.ts';
import { mountMathKeyboard } from '../ui/mathKeyboard.ts';
import {
  FieldSystem2,
  sourcesFromSceneObjects,
  StubContactWorld,
  createRapierContactWorld,
  evalForceExpr,
  FORCE_EXPR_HELP,
} from '../engine/index.ts';


/* =====================================================================
 *  物理运动模拟器 —— 单文件版
 *  坐标：世界坐标单位 m，y 轴向上；角度以 +x 方向为 0°，逆时针为正
 *  积分：每帧拆成若干子步，速度更新使用 Boris 推进（磁场中能量严格守恒），
 *        接触/碰撞使用冲量法（含恢复系数、动/静摩擦）
 * ===================================================================== */
const $ = s => document.querySelector(s);
const DEG = Math.PI / 180;
const K_COULOMB = 8.99e9;
const STORE_KEY = 'physim.scene.v1';

/* ---------------- 场景数据 ---------------- */
let nextId = 1;
const BODY_KINDS = ['ball', 'particle', 'box'];
const KIND_NAME = { ball: '小球', particle: '带电粒子', box: '滑块', ground: '地面', wall: '挡板', incline: '斜面',
  efield: '电场', bfield: '磁场', spring: '弹簧', pointCharge: '点电荷' };
const PALETTE = ['#2f6fed', '#e5484d', '#1f9d55', '#f59e0b', '#8b5cf6', '#0ea5e9', '#ec4899', '#14b8a6'];

function defaultSettings() {
  return { gOn: true, g: 9.8, drag: 0, coulomb: false, kc: K_COULOMB, maxDt: 0.001, vPx: 20, FPx: 12, snap: true, snapStep: 0.1,
    showLabels: true, showGrid: true, showTrail: true, showVel: true, showForce: false, showAnalytic: true, showFieldLines: true, coulombLaw: 'coulomb3d', contactBackend: 'native', speed: 1 };
}
let scene = { settings: defaultSettings(), objects: [], view: { cx: 0, cy: 2, scale: 60 } };

function isBody(o) { return o && BODY_KINDS.includes(o.kind); }
function bodies() { return scene.objects.filter(isBody); }
function byId(id) { return scene.objects.find(o => o.id === id); }

function mkBody(kind, p = {}) {
  const n = bodies().length;
  const base = { id: nextId++, kind, name: KIND_NAME[kind] + (n + 1), color: PALETTE[n % PALETTE.length],
    m: 1, q: 0, r: 0.25, w: 0.6, h: 0.4, e: 0.6, mu: 0, x0: 0, y0: 0, vx0: 0, vy0: 0, ang0: 0,
    /* 外力：const 恒定 | expr 表达式 | aim 指向目标 */
    F: 0, Fang: 0, Fmode: 'const', Fexpr: '', FangExpr: '0',
    FaimId: null, FaimX: 0, FaimY: 0, _Ferr: '',
    fixed: false, trail: true, collide: true };
  if (kind === 'particle') Object.assign(base, { m: 0.01, q: 0.01, r: 0.08, color: '#e5484d', collide: false, e: 1 });
  if (kind === 'box') Object.assign(base, { mu: 0.3, e: 0.2, color: '#f59e0b', m: 1 });
  const b = Object.assign(base, p);
  b.st = null; return b;
}
function mkGround(y = 0, p = {}) { return Object.assign({ id: nextId++, kind: 'ground', name: '地面', y, smooth: false, e: 0.6 }, p); }
function mkWall(x1, y1, x2, y2, p = {}) { return Object.assign({ id: nextId++, kind: 'wall', name: '挡板', x1, y1, x2, y2, smooth: true, e: 0.8 }, p); }
function mkIncline(x, y, p = {}) { return Object.assign({ id: nextId++, kind: 'incline', name: '斜面', x, y, L: 4, ang: 37, dir: 1, smooth: false, e: 0.1 }, p); }
function mkE(p = {}) { return Object.assign({ id: nextId++, kind: 'efield', name: '电场', E: 1, ang: -90, all: false, x: 0, y: -1, w: 4, h: 2 }, p); }
function mkB(p = {}) { return Object.assign({ id: nextId++, kind: 'bfield', name: '磁场', B: 1, dir: -1, all: false, x: 0, y: -2, w: 4, h: 4 }, p); }
function mkPointCharge(x = 0, y = 0, p = {}) {
  return Object.assign({
    id: nextId++, kind: 'pointCharge', name: '点电荷', x, y, q: 1e-6, softRadius: 0.08, fixed: true, color: '#e5484d',
  }, p);
}
function mkSpring(a, b, p = {}) { return Object.assign({ id: nextId++, kind: 'spring', name: '弹簧', a, b, ax: 0, ay: 0, k: 20, L0: 1, rope: false }, p); }

/* ---------------- 仿真状态 ---------------- */
const sim = { t: 0, running: false, accum: 0 };
let leftTab = 'objects'; // objects | add | presets
let leftQuery = '';
let leftChip = 'all';
let propsTab = 'init';
let mobileDock = 'stage';
const fieldSys = new FieldSystem2({ law: 'coulomb3d', k: K_COULOMB, softening: 0.02 });
let contactWorld = new StubContactWorld();
let fieldsDirty = true;


function initState(b) {
  b.st = { x: b.x0, y: b.y0, vx: b.vx0, vy: b.vy0, ang: b.ang0 * DEG, trail: [[b.x0, b.y0]], hist: [], forces: [],
    contactN: [0, 0], contactF: [0, 0], cN: 0, ax: 0, ay: 0, pvx: b.vx0, pvy: b.vy0 };
  pushHist(b);
}
function resetSim(msg) {
  sim.t = 0; sim.running = false;
  bodies().forEach(initState);
  updatePlayBtn();
  if (msg) toast(msg);
}
function ensureStates() { bodies().forEach(b => { if (!b.st) initState(b); }); }

/* ---------------- 几何工具 ---------------- */
function inRect(f, x, y) { return f.all || (x >= f.x && x <= f.x + f.w && y >= f.y && y <= f.y + f.h); }
function closestOnSeg(px, py, s) {
  const dx = s.x2 - s.x1, dy = s.y2 - s.y1, L2 = dx * dx + dy * dy;
  let t = L2 > 0 ? ((px - s.x1) * dx + (py - s.y1) * dy) / L2 : 0;
  t = Math.max(0, Math.min(1, t));
  return [s.x1 + t * dx, s.y1 + t * dy];
}
function inclinePts(o) {
  const L = o.L, H = L * Math.tan(o.ang * DEG), d = o.dir;
  return [[o.x, o.y], [o.x + d * L, o.y], [o.x + d * L, o.y + H]];
}
const GROUND_HALF = 1e4;
function getSegments() {
  const segs = [];
  for (const o of scene.objects) {
    if (o.kind === 'ground') segs.push({ x1: -GROUND_HALF, y1: o.y, x2: GROUND_HALF, y2: o.y, smooth: o.smooth, e: o.e, src: o, ground: true });
    else if (o.kind === 'wall') segs.push({ x1: o.x1, y1: o.y1, x2: o.x2, y2: o.y2, smooth: o.smooth, e: o.e, src: o });
    else if (o.kind === 'incline') {
      const p = inclinePts(o);
      for (let i = 0; i < 3; i++) { const a = p[i], c = p[(i + 1) % 3]; segs.push({ x1: a[0], y1: a[1], x2: c[0], y2: c[1], smooth: o.smooth, e: o.e, src: o }); }
    }
  }
  return segs;
}
function bodyRadius(b) { return b.kind === 'box' ? (b.w + b.h) / 4 : b.r; }
function support(b, nx, ny) {
  if (b.kind !== 'box') return b.r;
  const c = Math.cos(b.st.ang), s = Math.sin(b.st.ang);
  return b.w / 2 * Math.abs(nx * c + ny * s) + b.h / 2 * Math.abs(-nx * s + ny * c);
}
function rebuildFieldSources() {
  fieldSys.k = scene.settings.kc || K_COULOMB;
  fieldSys.law = scene.settings.coulombLaw || 'coulomb3d';
  fieldSys.softening = 0.02;
  // Dynamic charged bodies emit field when coulomb toggle is on (unified with F=qE path)
  fieldSys.setSources(sourcesFromSceneObjects(scene.objects, {
    includeBodyCharges: !!scene.settings.coulomb,
  }));
  const v = scene.view;
  const halfW = (W || 800) / (2 * (v.scale || 60));
  const halfH = (H || 600) / (2 * (v.scale || 60));
  fieldSys.setLineOptions({
    enabled: !!scene.settings.showFieldLines,
    linesPerUnitCharge: 10,
    step: Math.max(0.02, 4 / (v.scale || 60)),
    maxSteps: 600,
    bounds: { x0: v.cx - halfW * 1.4, y0: v.cy - halfH * 1.4, x1: v.cx + halfW * 1.4, y1: v.cy + halfH * 1.4 },
  });
  fieldsDirty = false;
}
function ensureFields() {
  if (fieldsDirty) rebuildFieldSources();
}
function markFieldsDirty() { fieldsDirty = true; fieldSys.markDirty(); }
/** Update emitter positions in-place so multi-substep Coulomb stays consistent without full rebuild. */
let lastLineRemesh = 0;
function syncBodyEmitterPositions(opts = {}) {
  if (!scene.settings.coulomb) return;
  ensureFields();
  let moved = false;
  for (const s of fieldSys.sources) {
    const o = byId(s.id);
    if (!o || !o.st) continue;
    if (o.kind !== 'particle' && o.kind !== 'ball' && o.kind !== 'box') continue;
    if (typeof s.x === 'number' && typeof s.y === 'number') {
      if (s.x !== o.st.x || s.y !== o.st.y || s.q !== o.q) moved = true;
      s.x = o.st.x;
      s.y = o.st.y;
      s.q = o.q;
    }
  }
  // Remesh field lines at most ~8 Hz while emitters move (forces still use live positions)
  if (moved && scene.settings.showFieldLines) {
    const now = performance.now();
    if (opts.forceLines || now - lastLineRemesh > 120) {
      fieldSys.markDirty();
      lastLineRemesh = now;
    }
  }
}
function fieldAt(x, y) {
  ensureFields();
  const s = fieldSys.sample(x, y);
  return { Ex: s.Ex, Ey: s.Ey, Bz: s.Bz, phi: s.phi };
}
function fieldAtExcluding(x, y, excludeId) {
  ensureFields();
  // Temporary filter: rebuild without excludeId if needed — cheap enough for few sources
  const all = fieldSys.sources;
  const filtered = all.filter(s => s.id !== excludeId);
  if (filtered.length === all.length) return fieldAt(x, y);
  let Ex = 0, Ey = 0, Bz = 0, phi = 0;
  for (const s of filtered) {
    const e = s.sampleE(x, y, fieldSys.law, fieldSys.k);
    Ex += e.x; Ey += e.y;
    if (s.sampleB) Bz += s.sampleB(x, y).Bz;
    if (s.samplePhi) phi += s.samplePhi(x, y, fieldSys.law, fieldSys.k);
  }
  return { Ex, Ey, Bz, phi };
}

/* ---------------- 力的计算（不含磁场力和接触力） ---------------- */
function springEnds(s) {
  const A = byId(s.a); if (!A || !A.st) return null;
  const B = s.b != null ? byId(s.b) : null;
  if (s.b != null && (!B || !B.st)) return null;
  return { A, B, x1: A.st.x, y1: A.st.y, x2: B ? B.st.x : s.ax, y2: B ? B.st.y : s.ay };
}

function ensureForceFields(b) {
  if (b.Fmode == null) b.Fmode = 'const';
  if (b.Fexpr == null) b.Fexpr = '';
  if (b.FangExpr == null) b.FangExpr = '0';
  if (b.FaimId === undefined) b.FaimId = null;
  if (b.FaimX == null) b.FaimX = 0;
  if (b.FaimY == null) b.FaimY = 0;
  if (b._Ferr == null) b._Ferr = '';
}

function forceEvalCtx(b) {
  const st = b.st || { x: b.x0, y: b.y0, vx: b.vx0 || 0, vy: b.vy0 || 0 };
  return {
    t: sim.t,
    x: st.x, y: st.y,
    vx: st.vx || 0, vy: st.vy || 0,
    v: Math.hypot(st.vx || 0, st.vy || 0),
    m: b.m, q: b.q || 0,
    g: scene.settings.gOn ? scene.settings.g : 0,
  };
}

function resolveAimPoint(b) {
  ensureForceFields(b);
  if (b.FaimId != null) {
    const o = byId(b.FaimId);
    if (o) {
      if (isBody(o)) {
        const st = o.st || { x: o.x0, y: o.y0 };
        return { x: st.x, y: st.y, label: o.name };
      }
      if (o.kind === 'pointCharge') return { x: o.x, y: o.y, label: o.name };
    }
  }
  return { x: b.FaimX || 0, y: b.FaimY || 0, label: '定点' };
}

/** 用户外力（恒定 / 表达式 / 指向目标）→ { fx, fy, F, ang, ok, err } */
function appliedForceOn(b) {
  ensureForceFields(b);
  const ctx = forceEvalCtx(b);
  const st = b.st || { x: b.x0, y: b.y0 };
  let F = b.F || 0;
  let ang = b.Fang || 0;
  let err = '';

  const magFrom = (constVal, expr, useExpr) => {
    if (!useExpr) return { ok: true, value: constVal };
    const r = evalForceExpr(expr, ctx);
    if (!r.ok) return { ok: false, error: r.error, value: constVal };
    return { ok: true, value: r.value };
  };

  if (b.Fmode === 'expr') {
    const m = magFrom(F, b.Fexpr, true);
    const a = magFrom(ang, b.FangExpr, true);
    if (!m.ok) err = 'F: ' + m.error;
    else if (!a.ok) err = 'θ: ' + a.error;
    else { F = m.value; ang = a.value; }
  } else if (b.Fmode === 'aim') {
    // 大小：若填写了 Fexpr 则用表达式，否则用恒定 F；方向指向目标
    const e = String(b.Fexpr || '').trim();
    if (e) {
      const r = evalForceExpr(e, ctx);
      if (!r.ok) err = 'F: ' + r.error;
      else F = r.value;
    }
    const aim = resolveAimPoint(b);
    const dx = aim.x - st.x, dy = aim.y - st.y;
    const L = Math.hypot(dx, dy);
    if (L < 1e-12) {
      b._Ferr = err || '目标与物体重合';
      return { fx: 0, fy: 0, F: 0, ang: b.Fang || 0, ok: false, err: b._Ferr };
    }
    ang = Math.atan2(dy, dx) / DEG;
    const fx = F * dx / L, fy = F * dy / L;
    b.Fang = ang; // live readout
    b._Ferr = err;
    return { fx, fy, F, ang, ok: !err, err };
  } else {
    // const
    F = b.F || 0;
    ang = b.Fang || 0;
  }

  // expr path sets F,ang above; const too
  if (b.Fmode === 'expr' && !err) {
    b.F = F;
    b.Fang = ang;
  }
  b._Ferr = err;
  if (err && b.Fmode === 'expr') {
    // keep last finite vector from stored F/Fang if expr failed mid-run
    F = b.F || 0; ang = b.Fang || 0;
  }
  const fx = F * Math.cos(ang * DEG), fy = F * Math.sin(ang * DEG);
  return { fx, fy, F, ang, ok: !err, err };
}

function computeForces(list, record) {
  const S = scene.settings, out = new Map();
  for (const b of list) {
    const f = { x: 0, y: 0, parts: record ? [] : null }; out.set(b, f);
    const add = (fx, fy, label, color) => { f.x += fx; f.y += fy; if (record && (fx || fy)) f.parts.push({ fx, fy, label, color }); };
    if (S.gOn) add(0, -b.m * S.g, 'G', '#e5484d');
    if (b.q) {
      // Exclude self-field if this body is also an emitter
      const F = fieldAtExcluding(b.st.x, b.st.y, b.id);
      if (F.Ex || F.Ey) add(b.q * F.Ex, b.q * F.Ey, 'F电', '#e08600');
    }
    const af = appliedForceOn(b);
    if (af.fx || af.fy) add(af.fx, af.fy, 'F', '#8b5cf6');
    if (S.drag) add(-S.drag * b.st.vx, -S.drag * b.st.vy, 'F阻', '#6b7385');
  }
  // 弹簧/轻绳
  for (const s of scene.objects) {
    if (s.kind !== 'spring') continue;
    const e = springEnds(s); if (!e) continue;
    const dx = e.x2 - e.x1, dy = e.y2 - e.y1, L = Math.hypot(dx, dy) || 1e-9;
    let T = s.k * (L - s.L0); if (s.rope && T < 0) T = 0;
    const fx = T * dx / L, fy = T * dy / L, lab = s.rope ? 'T' : 'F弹';
    const fa = out.get(e.A); if (fa) { fa.x += fx; fa.y += fy; if (record && T) fa.parts.push({ fx, fy, label: lab, color: '#14b8a6' }); }
    const fb = e.B && out.get(e.B); if (fb) { fb.x -= fx; fb.y -= fy; if (record && T) fb.parts.push({ fx: -fx, fy: -fy, label: lab, color: '#14b8a6' }); }
  }
  // 库仑力：由 FieldSystem 中的点电荷/带电体源经 F=qE 统一计入（见上方电场力）。
  // 保留开关 scene.settings.coulomb → includeBodyCharges。
  return out;
}

/* ---------------- 单个子步 ---------------- */
function substep(h, segs) {
  const S = scene.settings;
  const movers = bodies().filter(b => !b.fixed);
  const F = computeForces(movers, false);
  for (const b of movers) {
    const st = b.st, f = F.get(b), ax = f.x / b.m, ay = f.y / b.m;
    // Boris：半步电/重力 → 磁场旋转 → 半步
    let vx = st.vx + ax * h / 2, vy = st.vy + ay * h / 2;
    if (b.q) {
      const Bz = fieldAt(st.x, st.y).Bz;
      if (Bz) {
        const t = b.q * Bz / b.m * h / 2, s = 2 * t / (1 + t * t);
        const px = vx + vy * t, py = vy - vx * t;
        vx = vx + py * s; vy = vy - px * s;
      }
    }
    st.vx = vx + ax * h / 2; st.vy = vy + ay * h / 2;
    st.x += st.vx * h; st.y += st.vy * h;
  }
  // 与静止表面接触
  for (const b of movers) {
    for (const sg of segs) contactSeg(b, sg, h);
  }
  // 物体间碰撞
  const bl = bodies().filter(b => b.collide);
  for (let i = 0; i < bl.length; i++) for (let j = i + 1; j < bl.length; j++) collidePair(bl[i], bl[j]);
  // 轨迹
  for (const b of movers) {
    const tr = b.st.trail, last = tr[tr.length - 1];
    const minD = 2 / scene.view.scale;
    if (!last || Math.abs(last[0] - b.st.x) + Math.abs(last[1] - b.st.y) > minD) {
      tr.push([b.st.x, b.st.y]); if (tr.length > 4000) tr.splice(0, tr.length - 4000);
    }
  }
}
const REST_V = 0.15; // 低于此法向速度视为静止接触，不反弹
function contactSeg(b, sg, h) {
  const st = b.st;
  let nx, ny, d, reff;
  if (sg.ground) {
    // 地面视为下方实心的半空间：法线恒为 +y，防止物体被挤到地面以下后“穿透”
    nx = 0; ny = 1; d = st.y - sg.y1; reff = support(b, 0, 1);
    if (d >= reff || d < -Math.max(2 * reff, 0.5)) return;
  } else {
    const cp = closestOnSeg(st.x, st.y, sg);
    const dx = st.x - cp[0], dy = st.y - cp[1]; d = Math.hypot(dx, dy);
    if (d < 1e-9) return;
    nx = dx / d; ny = dy / d; reff = support(b, nx, ny);
    if (d >= reff) return;
  }
  st.x += nx * (reff - d); st.y += ny * (reff - d);
  const vn = st.vx * nx + st.vy * ny;
  if (b.kind === 'box' && ny > 0.5) st.ang = Math.atan2(-nx, ny);
  if (vn >= 0) return;
  const e = -vn < REST_V ? 0 : Math.min(b.e, sg.e);
  const dvn = -(1 + e) * vn;
  st.vx += nx * dvn; st.vy += ny * dvn;
  st.contactN[0] += b.m * nx * dvn / h; st.contactN[1] += b.m * ny * dvn / h;
  if (!sg.smooth && b.mu > 0) {
    const tx = -ny, ty = nx, vt = st.vx * tx + st.vy * ty, maxdv = b.mu * dvn;
    const dvt = Math.abs(vt) <= maxdv ? -vt : -Math.sign(vt) * maxdv;
    st.vx += tx * dvt; st.vy += ty * dvt;
    st.contactF[0] += b.m * tx * dvt / h; st.contactF[1] += b.m * ty * dvt / h;
  }
}
function collidePair(a, b) {
  if (a.fixed && b.fixed) return;
  const A = a.st, B = b.st;
  let dx = B.x - A.x, dy = B.y - A.y; const d = Math.hypot(dx, dy), R = bodyRadius(a) + bodyRadius(b);
  if (d >= R || d < 1e-9) return;
  const nx = dx / d, ny = dy / d;
  const ia = a.fixed ? 0 : 1 / a.m, ib = b.fixed ? 0 : 1 / b.m, is = ia + ib;
  const pen = R - d;
  A.x -= nx * pen * ia / is; A.y -= ny * pen * ia / is; B.x += nx * pen * ib / is; B.y += ny * pen * ib / is;
  const vrel = (B.vx - A.vx) * nx + (B.vy - A.vy) * ny;
  if (vrel >= 0) return;
  const e = -vrel < REST_V * 0.5 ? 0 : Math.min(a.e, b.e);
  const j = -(1 + e) * vrel / is;
  A.vx -= j * ia * nx; A.vy -= j * ia * ny; B.vx += j * ib * nx; B.vy += j * ib * ny;
}

/* ---------------- 前进一帧 ---------------- */
function advance(dtFrame) {
  ensureStates();
  const segs = getSegments();
  const n = Math.min(20000, Math.max(1, Math.ceil(dtFrame / scene.settings.maxDt)));
  const h = dtFrame / n;
  for (const b of bodies()) { b.st.contactN = [0, 0]; b.st.contactF = [0, 0]; b.st.pvx = b.st.vx; b.st.pvy = b.st.vy; }
  for (let i = 0; i < n; i++) {
    syncBodyEmitterPositions();
    substep(h, segs);
  }
  sim.t += dtFrame;
  for (const b of bodies()) {
    const st = b.st; st.contactN[0] /= n; st.contactN[1] /= n; st.contactF[0] /= n; st.contactF[1] /= n;
    st.ax = (st.vx - st.pvx) / dtFrame; st.ay = (st.vy - st.pvy) / dtFrame;
    pushHist(b);
  }
}
function pushHist(b) {
  const st = b.st, v2 = st.vx * st.vx + st.vy * st.vy;
  st.hist.push([sim.t, st.x, st.y, st.vx, st.vy, Math.sqrt(v2), 0.5 * b.m * v2]);
  if (st.hist.length > 6000) st.hist.splice(0, st.hist.length - 6000);
}

/* =====================================================================
 *  渲染
 * ===================================================================== */
const cv = $('#cv'), ctx = cv.getContext('2d');
let W = 0, H = 0, DPR = 1;
function css(v) { return getComputedStyle(document.body).getPropertyValue(v).trim(); }
let C = {};
function readColors() { ['--canvas', '--grid', '--grid2', '--axis', '--text', '--muted', '--efield', '--efield-line', '--bfield', '--bfield-line', '--surface', '--accent', '--panel', '--border'].forEach(k => C[k] = css(k)); }
function resize() {
  const r = cv.getBoundingClientRect(); DPR = window.devicePixelRatio || 1;
  W = r.width; H = r.height; cv.width = Math.round(W * DPR); cv.height = Math.round(H * DPR);
  markFieldsDirty();
  draw();
}
function w2s(x, y) { const v = scene.view; return [W / 2 + (x - v.cx) * v.scale, H / 2 - (y - v.cy) * v.scale]; }
function s2w(sx, sy) { const v = scene.view; return [v.cx + (sx - W / 2) / v.scale, v.cy - (sy - H / 2) / v.scale]; }
function niceStep(raw) { const p = Math.pow(10, Math.floor(Math.log10(raw))), f = raw / p; return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * p; }
function fmt(v, d = 3) {
  if (!isFinite(v)) return '—';
  const a = Math.abs(v);
  if (a < 1e-12) return '0';
  if (a !== 0 && (a < 1e-3 || a >= 1e5)) return v.toExponential(2);
  return (+v.toFixed(d)).toString();
}
function arrow(x1, y1, x2, y2, color, width = 2, head = 9, dash) {
  const dx = x2 - x1, dy = y2 - y1, L = Math.hypot(dx, dy); if (L < 1) return;
  ctx.save(); ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width; ctx.lineCap = 'round';
  if (dash) ctx.setLineDash(dash);
  const hh = Math.min(head, L * 0.45), ux = dx / L, uy = dy / L;
  ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2 - ux * hh * 0.8, y2 - uy * hh * 0.8); ctx.stroke();
  ctx.setLineDash([]);
  ctx.beginPath(); ctx.moveTo(x2, y2); ctx.lineTo(x2 - ux * hh - uy * hh * 0.45, y2 - uy * hh + ux * hh * 0.45);
  ctx.lineTo(x2 - ux * hh + uy * hh * 0.45, y2 - uy * hh - ux * hh * 0.45); ctx.closePath(); ctx.fill();
  ctx.restore();
}
function label(text, x, y, color, align = 'left', bg = true) {
  ctx.save(); ctx.font = '12px -apple-system,"PingFang SC","Microsoft YaHei",sans-serif'; ctx.textAlign = align; ctx.textBaseline = 'middle';
  if (bg) {
    const w = ctx.measureText(text).width + 8, x0 = align === 'center' ? x - w / 2 : align === 'right' ? x - w + 4 : x - 4;
    ctx.globalAlpha = 0.85; ctx.fillStyle = C['--panel']; ctx.fillRect(x0, y - 9, w, 18); ctx.globalAlpha = 1;
  }
  ctx.fillStyle = color; ctx.fillText(text, x, y); ctx.restore();
}

function drawGrid() {
  const v = scene.view, step = niceStep(80 / v.scale), minor = step / 5;
  const [x0, y1] = s2w(0, 0), [x1, y0] = s2w(W, H);
  ctx.lineWidth = 1;
  if (minor * v.scale > 10) {
    ctx.strokeStyle = C['--grid']; ctx.beginPath();
    for (let x = Math.floor(x0 / minor) * minor; x <= x1; x += minor) { const s = w2s(x, 0)[0]; ctx.moveTo(s + .5, 0); ctx.lineTo(s + .5, H); }
    for (let y = Math.floor(y0 / minor) * minor; y <= y1; y += minor) { const s = w2s(0, y)[1]; ctx.moveTo(0, s + .5); ctx.lineTo(W, s + .5); }
    ctx.stroke();
  }
  ctx.strokeStyle = C['--grid2']; ctx.beginPath();
  for (let x = Math.floor(x0 / step) * step; x <= x1; x += step) { const s = w2s(x, 0)[0]; ctx.moveTo(s + .5, 0); ctx.lineTo(s + .5, H); }
  for (let y = Math.floor(y0 / step) * step; y <= y1; y += step) { const s = w2s(0, y)[1]; ctx.moveTo(0, s + .5); ctx.lineTo(W, s + .5); }
  ctx.stroke();
  // 坐标轴
  const [ox, oy] = w2s(0, 0);
  ctx.strokeStyle = C['--axis']; ctx.lineWidth = 1.3; ctx.beginPath();
  if (oy > 0 && oy < H) { ctx.moveTo(0, oy); ctx.lineTo(W, oy); }
  if (ox > 0 && ox < W) { ctx.moveTo(ox, 0); ctx.lineTo(ox, H); }
  ctx.stroke();
  ctx.fillStyle = C['--muted']; ctx.font = '10.5px sans-serif';
  const ly = Math.min(Math.max(oy + 12, 12), H - 6), lx = Math.min(Math.max(ox + 4, 4), W - 40);
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  for (let x = Math.floor(x0 / step) * step; x <= x1; x += step) { if (Math.abs(x) < step / 2) continue; ctx.fillText(fmt(x, 4), w2s(x, 0)[0], ly); }
  ctx.textAlign = 'left';
  for (let y = Math.floor(y0 / step) * step; y <= y1; y += step) { if (Math.abs(y) < step / 2) continue; ctx.fillText(fmt(y, 4), lx, w2s(0, y)[1]); }
  ctx.fillText('O', lx, ly); ctx.fillText('x/m', W - 30, ly - 12); ctx.fillText('y/m', lx + 14, 10);
}
function fieldScreenRect(f) {
  if (f.all) return [0, 0, W, H];
  const [a, b] = w2s(f.x, f.y + f.h), [c, d] = w2s(f.x + f.w, f.y); return [a, b, c - a, d - b];
}
function drawField(f) {
  const [x, y, w, h] = fieldScreenRect(f), sel = f === selected();
  const isE = f.kind === 'efield', line = isE ? C['--efield-line'] : C['--bfield-line'];
  ctx.save();
  ctx.fillStyle = isE ? C['--efield'] : C['--bfield']; ctx.fillRect(x, y, w, h);
  ctx.beginPath(); ctx.rect(x, y, w, h); ctx.clip();
  if (isE) {
    // 电场线：沿 E 方向的平行箭头
    const a = f.ang * DEG, ux = Math.cos(a), uy = -Math.sin(a), sp = 46;
    const cx = x + w / 2, cy = y + h / 2, R = Math.hypot(w, h) / 2 + sp;
    ctx.globalAlpha = 0.55;
    for (let k = -R; k <= R; k += sp) {
      const px = cx - uy * k, py = cy + ux * k;
      ctx.strokeStyle = line; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(px - ux * R, py - uy * R); ctx.lineTo(px + ux * R, py + uy * R); ctx.stroke();
      for (let m = -R; m <= R; m += 90) arrow(px + ux * (m - 1), py + uy * (m - 1), px + ux * (m + 7), py + uy * (m + 7), line, 1, 8);
    }
  } else {
    const v = scene.view, sp = niceStep(42 / v.scale);
    const [wx0, wy1] = s2w(x, y), [wx1, wy0] = s2w(x + w, y + h);
    ctx.strokeStyle = line; ctx.fillStyle = line; ctx.lineWidth = 1.2; ctx.globalAlpha = 0.7;
    for (let gx = Math.ceil(wx0 / sp) * sp; gx <= wx1; gx += sp) for (let gy = Math.ceil(wy0 / sp) * sp; gy <= wy1; gy += sp) {
      const [sx, sy] = w2s(gx, gy), r = 5;
      ctx.beginPath(); ctx.arc(sx, sy, r, 0, 7); ctx.stroke();
      if (f.dir < 0) { ctx.beginPath(); ctx.moveTo(sx - 3.3, sy - 3.3); ctx.lineTo(sx + 3.3, sy + 3.3); ctx.moveTo(sx + 3.3, sy - 3.3); ctx.lineTo(sx - 3.3, sy + 3.3); ctx.stroke(); }
      else { ctx.beginPath(); ctx.arc(sx, sy, 1.6, 0, 7); ctx.fill(); }
    }
  }
  ctx.restore();
  ctx.save(); ctx.strokeStyle = line; ctx.lineWidth = sel ? 2 : 1.2; ctx.setLineDash(sel ? [] : [6, 4]);
  if (!f.all) ctx.strokeRect(x + .5, y + .5, w, h);
  ctx.restore();
  const txt = isE ? `E = ${fmt(f.E)} V/m` : `B = ${fmt(f.B)} T ${f.dir < 0 ? '⊗' : '⊙'}`;
  if (f.all) label(txt + '（全空间）', 10, isE ? 16 : 36, line);
  else label(txt, x + 6, y + 12, line);
  if (sel && !f.all) { for (const [hx, hy] of rectHandles(f)) handleDot(hx, hy, line); }
}
function rectHandles(f) { const [x, y, w, h] = fieldScreenRect(f); return [[x, y], [x + w, y], [x + w, y + h], [x, y + h]]; }
function handleDot(x, y, color, r = 5) { ctx.save(); ctx.fillStyle = C['--panel']; ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.fill(); ctx.stroke(); ctx.restore(); }
function hatchLine(x1, y1, x2, y2, side, color) {
  const dx = x2 - x1, dy = y2 - y1, L = Math.hypot(dx, dy); if (L < 1) return;
  const ux = dx / L, uy = dy / L, nx = -uy * side, ny = ux * side;
  ctx.save(); ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.globalAlpha = 0.6; ctx.beginPath();
  for (let s = 0; s < L; s += 9) { const px = x1 + ux * s, py = y1 + uy * s; ctx.moveTo(px, py); ctx.lineTo(px + nx * 8 - ux * 6, py + ny * 8 - uy * 6); }
  ctx.stroke(); ctx.restore();
}
function drawSurface(o) {
  const sel = o === selected(), col = sel ? C['--accent'] : C['--surface'];
  ctx.save(); ctx.lineCap = 'round';
  if (o.kind === 'ground') {
    const y = w2s(0, o.y)[1];
    ctx.strokeStyle = col; ctx.lineWidth = sel ? 3 : 2; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
    hatchLine(0, y, W, y, -1, col);
    if (o.smooth) label('光滑', 8, y + 14, C['--muted'], 'left', false);
  } else if (o.kind === 'wall') {
    const [a, b] = w2s(o.x1, o.y1), [c, d] = w2s(o.x2, o.y2);
    ctx.strokeStyle = col; ctx.lineWidth = sel ? 4 : 3; ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
    if (sel) { handleDot(a, b, col); handleDot(c, d, col); }
  } else if (o.kind === 'incline') {
    const p = inclinePts(o).map(q => w2s(q[0], q[1]));
    ctx.beginPath(); ctx.moveTo(p[0][0], p[0][1]); ctx.lineTo(p[1][0], p[1][1]); ctx.lineTo(p[2][0], p[2][1]); ctx.closePath();
    ctx.fillStyle = C['--surface']; ctx.globalAlpha = 0.16; ctx.fill(); ctx.globalAlpha = 1;
    ctx.strokeStyle = col; ctx.lineWidth = sel ? 3 : 2; ctx.stroke();
    // 角度标注
    const r = 26, a0 = o.dir > 0 ? 0 : Math.PI, a1 = o.dir > 0 ? -o.ang * DEG : Math.PI + o.ang * DEG;
    ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(p[0][0], p[0][1], r, Math.min(a0, a1), Math.max(a0, a1)); ctx.stroke();
    label(`${fmt(o.ang, 1)}°`, p[0][0] + o.dir * (r + 16), p[0][1] - 9, C['--muted'], 'center', false);
    if (o.smooth) label('光滑', (p[0][0] + p[2][0]) / 2 - o.dir * 14, (p[0][1] + p[2][1]) / 2 - 12, C['--muted'], 'center', false);
  }
  ctx.restore();
}
function drawSpring(s) {
  const e = springEnds(s); if (!e) return;
  const [x1, y1] = w2s(e.x1, e.y1), [x2, y2] = w2s(e.x2, e.y2), sel = s === selected();
  ctx.save(); ctx.strokeStyle = sel ? C['--accent'] : '#14b8a6'; ctx.lineWidth = sel ? 2.5 : 1.8; ctx.lineJoin = 'round';
  const dx = x2 - x1, dy = y2 - y1, L = Math.hypot(dx, dy) || 1, ux = dx / L, uy = dy / L;
  ctx.beginPath(); ctx.moveTo(x1, y1);
  if (s.rope) ctx.lineTo(x2, y2);
  else {
    const n = 12, lead = Math.min(10, L * 0.15), amp = 6;
    ctx.lineTo(x1 + ux * lead, y1 + uy * lead);
    for (let i = 1; i < n * 2; i++) { const t = lead + (L - 2 * lead) * i / (n * 2), sgn = i % 2 ? 1 : -1; ctx.lineTo(x1 + ux * t - uy * amp * sgn, y1 + uy * t + ux * amp * sgn); }
    ctx.lineTo(x2 - ux * lead, y2 - uy * lead); ctx.lineTo(x2, y2);
  }
  ctx.stroke();
  if (s.b == null) { ctx.fillStyle = C['--surface']; ctx.fillRect(x2 - 8, y2 - 3, 16, 6); hatchLine(x2 - 8, y2 - 3, x2 + 8, y2 - 3, 1, C['--surface']); }
  ctx.restore();
}
function bodyScreenR(b) { return Math.max(bodyRadius(b) * scene.view.scale, b.kind === 'particle' ? 5 : 3); }
function drawBodyShape(b, x, y, ang, ghost) {
  const s = scene.view.scale;
  ctx.save(); ctx.globalAlpha = ghost ? 0.35 : 1;
  ctx.fillStyle = b.color; ctx.strokeStyle = 'rgba(0,0,0,.35)'; ctx.lineWidth = 1;
  if (ghost) ctx.setLineDash([4, 3]);
  if (b.kind === 'box') {
    ctx.translate(x, y); ctx.rotate(-ang);
    const w = Math.max(b.w * s, 4), h = Math.max(b.h * s, 4);
    ctx.fillRect(-w / 2, -h / 2, w, h); ctx.strokeRect(-w / 2, -h / 2, w, h);
  } else {
    const r = bodyScreenR(b);
    ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.fill(); ctx.stroke();
    if (!ghost && r > 6) { ctx.fillStyle = 'rgba(255,255,255,.35)'; ctx.beginPath(); ctx.arc(x - r * 0.3, y - r * 0.3, r * 0.3, 0, 7); ctx.fill(); }
  }
  ctx.restore();
  if (b.q && !ghost) {
    ctx.save(); ctx.translate(x, y); ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.6;
    const k = Math.min(Math.max(bodyScreenR(b) * 0.5, 2.5), 7);
    ctx.beginPath(); ctx.moveTo(-k, 0); ctx.lineTo(k, 0); if (b.q > 0) { ctx.moveTo(0, -k); ctx.lineTo(0, k); } ctx.stroke(); ctx.restore();
  }
}
function drawBody(b) {
  const st = b.st, [x, y] = w2s(st.x, st.y), sel = b === selected(), S = scene.settings;
  if (S.showTrail && b.trail && st.trail.length > 1) {
    ctx.save(); ctx.strokeStyle = b.color; ctx.globalAlpha = 0.6; ctx.lineWidth = 1.6; ctx.beginPath();
    st.trail.forEach((p, i) => { const [a, c] = w2s(p[0], p[1]); i ? ctx.lineTo(a, c) : ctx.moveTo(a, c); }); ctx.stroke(); ctx.restore();
  }
  drawBodyShape(b, x, y, st.ang, false);
  if (b.fixed) { ctx.save(); ctx.fillStyle = C['--text']; ctx.beginPath(); ctx.arc(x, y, 2.5, 0, 7); ctx.fill(); ctx.restore(); }
  if (sel) {
    ctx.save(); ctx.strokeStyle = C['--accent']; ctx.lineWidth = 2; ctx.setLineDash([4, 3]);
    const r = b.kind === 'box' ? Math.hypot(b.w, b.h) / 2 * scene.view.scale + 4 : bodyScreenR(b) + 5;
    ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.stroke(); ctx.restore();
  }
  if (S.showLabels) label(b.name, x, y + bodyScreenR(b) + 13, C['--muted'], 'center', false);
}
function drawVectors(b) {
  const st = b.st, [x, y] = w2s(st.x, st.y), S = scene.settings;
  if (S.showVel && (sim.t > 0 || b !== selected())) {
    const vx = st.vx * S.vPx, vy = -st.vy * S.vPx;
    if (Math.hypot(vx, vy) > 2) { arrow(x, y, x + vx, y + vy, '#1f9d55', 2.2); label(`v=${fmt(Math.hypot(st.vx, st.vy), 2)}`, x + vx + 6, y + vy - 8, '#1f9d55', 'left', false); }
  }
  if (S.showForce) {
    const parts = forceParts(b); let mx = 0; parts.forEach(p => mx = Math.max(mx, Math.hypot(p.fx, p.fy)));
    if (mx > 0) {
      const sc = 70 / mx;
      for (const p of parts) {
        // 用户外力 F 用专用手柄绘制，避免与自动缩放叠画
        if (p.label === 'F' && b === selected()) continue;
        const ex = x + p.fx * sc, ey = y - p.fy * sc; if (Math.hypot(p.fx, p.fy) * sc < 3) continue;
        arrow(x, y, ex, ey, p.color, 1.8, 8);
        label(`${p.label} ${fmt(Math.hypot(p.fx, p.fy), 2)}N`, ex + 4, ey + (p.fy > 0 ? -8 : 8), p.color, 'left', false);
      }
    }
  }
}

/** 外力箭头末端手柄（开启「受力」且选中物体时） */
function forceHandle(b) {
  ensureForceFields(b);
  const st = b.st || { x: b.x0, y: b.y0 };
  const [x, y] = w2s(st.x, st.y);
  const af = appliedForceOn(b);
  const FPx = scene.settings.FPx || 12;
  let fx = af.fx, fy = af.fy;
  let zero = Math.hypot(fx, fy) < 1e-9;
  // 零力时给出默认可拖方向（沿 +x）
  if (zero) { fx = 1; fy = 0; }
  let hx = x + fx * FPx, hy = y - fy * FPx;
  const minL = bodyScreenR(b) + 16;
  const L = Math.hypot(hx - x, hy - y);
  if (L < minL) {
    const ux = (hx - x) / (L || 1), uy = (hy - y) / (L || 1);
    hx = x + ux * minL; hy = y + uy * minL;
    zero = true;
  }
  return { x, y, hx, hy, zero, af, FPx };
}
function drawForceHandle(b) {
  if (!scene.settings.showForce) return;
  const H = forceHandle(b);
  const ghost = H.zero;
  if (!ghost) arrow(H.x, H.y, H.hx, H.hy, '#8b5cf6', 3, 12, null);
  else {
    ctx.save(); ctx.setLineDash([5, 4]);
    arrow(H.x, H.y, H.hx, H.hy, '#8b5cf6', 2, 10, [5, 4]);
    ctx.restore();
  }
  handleDot(H.hx, H.hy, '#8b5cf6', 7);
  const mode = b.Fmode || 'const';
  const modeLab = mode === 'expr' ? '表达式' : mode === 'aim' ? '指向' : '恒定';
  const txt = ghost
    ? `拖末端设置外力 F（${modeLab}）`
    : `F=${fmt(H.af.F, 2)} N  ${fmt(H.af.ang, 1)}° · ${modeLab}`;
  label(txt, H.hx + 12, H.hy - 12, '#8b5cf6');
  if (b._Ferr) label(b._Ferr, H.hx + 12, H.hy + 6, '#e5484d');
  if (mode === 'aim') {
    const aim = resolveAimPoint(b);
    const [ax, ay] = w2s(aim.x, aim.y);
    ctx.save(); ctx.strokeStyle = 'rgba(139,92,246,.45)'; ctx.setLineDash([3, 3]); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(H.x, H.y); ctx.lineTo(ax, ay); ctx.stroke();
    ctx.fillStyle = '#8b5cf6'; ctx.beginPath(); ctx.arc(ax, ay, 3, 0, 7); ctx.fill();
    ctx.restore();
  }
}
function forceParts(b) {
  const f = computeForces([b], true).get(b), parts = f.parts.slice(), st = b.st;
  if (b.q) {
    const Bz = fieldAt(st.x, st.y).Bz;
    if (Bz) parts.push({ fx: b.q * st.vy * Bz, fy: -b.q * st.vx * Bz, label: 'F洛', color: '#2f6fed' });
  }
  if (sim.t > 0) {
    if (Math.hypot(...st.contactN) > 1e-6) parts.push({ fx: st.contactN[0], fy: st.contactN[1], label: 'N', color: '#0ea5e9' });
    if (Math.hypot(...st.contactF) > 1e-6) parts.push({ fx: st.contactF[0], fy: st.contactF[1], label: 'f', color: '#a16207' });
  }
  return parts;
}
// 初速度拖拽手柄：返回屏幕坐标 [起点x,起点y,终点x,终点y]
function v0Handle(b) {
  const [x, y] = w2s(b.x0, b.y0), S = scene.settings;
  let hx = x + b.vx0 * S.vPx, hy = y - b.vy0 * S.vPx;
  const L = Math.hypot(hx - x, hy - y), minL = bodyScreenR(b) + 14;
  const zero = L < 1;
  if (zero) { hx = x + minL; hy = y; }
  return { x, y, hx, hy, zero };
}
function drawV0(b) {
  const H0 = v0Handle(b), ghost = sim.t > 0;
  if (ghost) drawBodyShape(b, H0.x, H0.y, b.ang0 * DEG, true);
  const v = Math.hypot(b.vx0, b.vy0), ang = Math.atan2(b.vy0, b.vx0) / DEG;
  if (!H0.zero) arrow(H0.x, H0.y, H0.hx, H0.hy, '#1f9d55', 3, 12, ghost ? [5, 4] : null);
  handleDot(H0.hx, H0.hy, '#1f9d55', 7);
  const txt = H0.zero ? '拖动设置 v₀' : `v₀ = ${fmt(v, 2)} m/s  ${fmt(ang, 1)}°`;
  label(txt, H0.hx + 12, H0.hy - 12, '#1f9d55');
}


function drawElectricFieldLines() {
  const lines = fieldSys.getFieldLines();
  if (!lines.length) return;
  ctx.save();
  ctx.strokeStyle = C['--efield-line'] || '#b45f14';
  ctx.globalAlpha = 0.55;
  ctx.lineWidth = 1.2;
  for (const line of lines) {
    ctx.beginPath();
    line.forEach((p, i) => {
      const [sx, sy] = w2s(p.x, p.y);
      if (i) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy);
    });
    ctx.stroke();
    // arrowhead near 60% mark
    if (line.length > 8) {
      const a = line[Math.floor(line.length * 0.55)];
      const b = line[Math.floor(line.length * 0.58)];
      const [x1, y1] = w2s(a.x, a.y), [x2, y2] = w2s(b.x, b.y);
      arrow(x1, y1, x2, y2, C['--efield-line'] || '#b45f14', 1.2, 6);
    }
  }
  ctx.restore();
}
function drawPointCharge(o) {
  const [sx, sy] = w2s(o.x, o.y);
  const r = Math.max(8, (o.softRadius || 0.08) * scene.view.scale);
  const pos = o.q >= 0;
  ctx.save();
  ctx.fillStyle = o.color || (pos ? '#e5484d' : '#2f6fed');
  ctx.strokeStyle = 'rgba(0,0,0,.35)';
  ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(sx, sy, r, 0, 7); ctx.fill(); ctx.stroke();
  ctx.strokeStyle = '#fff'; ctx.lineWidth = 2;
  const k = r * 0.45;
  ctx.beginPath(); ctx.moveTo(sx - k, sy); ctx.lineTo(sx + k, sy);
  if (pos) { ctx.moveTo(sx, sy - k); ctx.lineTo(sx, sy + k); }
  ctx.stroke();
  if (o === selected()) {
    ctx.setLineDash([4, 3]); ctx.strokeStyle = C['--accent']; ctx.beginPath(); ctx.arc(sx, sy, r + 5, 0, 7); ctx.stroke();
  }
  if (scene.settings.showLabels) label(o.name + ` (${pos ? '+' : ''}${fmt(o.q, 2)} C)`, sx, sy + r + 12, C['--muted'], 'center', false);
  ctx.restore();
}
function drawAnalyticPath(b) {
  if (!b || !b.st) return;
  const sampled = sampleAnalyticPath(buildAnalysisContext(b), {});
  if (!sampled || sampled.points.length < 2) return;
  ctx.save();
  ctx.strokeStyle = C['--accent'] || '#2f6fed';
  ctx.globalAlpha = 0.55;
  ctx.lineWidth = 2;
  ctx.setLineDash([7, 5]);
  ctx.beginPath();
  sampled.points.forEach((p, i) => {
    const [sx, sy] = w2s(p.x, p.y);
    if (i) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy);
  });
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.globalAlpha = 0.9;
  const mid = sampled.points[Math.floor(sampled.points.length * 0.35)];
  const [lx, ly] = w2s(mid.x, mid.y);
  label('解析轨迹', lx + 8, ly - 10, C['--accent'], 'left', true);
  ctx.restore();
}
let pending = null; // 正在创建的矩形/线段/弹簧
function draw() {
  if (!W) return;
  ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
  ctx.fillStyle = C['--canvas']; ctx.fillRect(0, 0, W, H);
  if (scene.settings.showGrid) drawGrid();
  const objs = scene.objects;
  ensureFields();
  if (scene.settings.showFieldLines) drawElectricFieldLines();
  objs.filter(o => o.kind === 'efield' || o.kind === 'bfield').forEach(drawField);
  objs.filter(o => o.kind === 'pointCharge').forEach(drawPointCharge);
  objs.filter(o => ['ground', 'wall', 'incline'].includes(o.kind)).forEach(drawSurface);
  objs.filter(o => o.kind === 'spring').forEach(drawSpring);
  ensureStates();
  bodies().forEach(drawBody);
  bodies().forEach(drawVectors);
  const s = selected();
  // Analytic overlay is relatively expensive — refresh every other frame while running
  if (isBody(s) && scene.settings.showAnalytic && (!sim.running || ((performance.now() / 32) | 0) % 2 === 0)) {
    drawAnalyticPath(s);
  }
  if (isBody(s) && tool === 'select' && !sim.running) drawV0(s);
  if (isBody(s) && scene.settings.showForce) drawForceHandle(s);
  if (pending) {
    ctx.save(); ctx.strokeStyle = C['--accent']; ctx.setLineDash([5, 4]); ctx.lineWidth = 1.5;
    if (pending.type === 'rect') { const [a, b] = w2s(pending.x1, pending.y1), [c, d] = w2s(pending.x2, pending.y2); ctx.strokeRect(Math.min(a, c), Math.min(b, d), Math.abs(c - a), Math.abs(d - b)); label(`${fmt(Math.abs(pending.x2 - pending.x1), 2)} × ${fmt(Math.abs(pending.y2 - pending.y1), 2)} m`, Math.max(a, c) + 6, Math.max(b, d) + 10, C['--accent']); }
    if (pending.type === 'line') { const [a, b] = w2s(pending.x1, pending.y1), [c, d] = w2s(pending.x2, pending.y2); ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke(); label(`${fmt(Math.hypot(pending.x2 - pending.x1, pending.y2 - pending.y1), 2)} m`, c + 8, d, C['--accent']); }
    if (pending.type === 'spring') { const A = byId(pending.a); if (A) { const [a, b] = w2s(A.st.x, A.st.y), [c, d] = w2s(mouseW[0], mouseW[1]); ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke(); } }
    ctx.restore();
  }
  updateBadge();
}

/* =====================================================================
 *  交互
 * ===================================================================== */
let tool = 'select', selId = null, drag = null, mouseW = [0, 0], spaceDown = false;
function selected() { return selId != null ? byId(selId) : null; }
function select(id) {
  const prev = selId;
  selId = id;
  if (prev !== id) {
    // Cheap list highlight update; full rebuild only when structure changes
    document.querySelectorAll('.objlist li').forEach(li => {
      const lid = +li.dataset.id;
      li.classList.toggle('sel', lid === selId);
    });
    buildPanel();
  } else {
    buildPanel();
  }
  draw();
}
const TOOL_HINT = {
  select: '选择：拖动物体；绿箭头=v₀；开「受力」后紫箭头末端=外力 F；空白处平移',
  pan: '平移：拖动画布移动视图，滚轮缩放', ball: '点击画布放置小球（按住 Shift 可连续放置）',
  particle: '点击画布放置带电粒子（按住 Shift 可连续放置）', box: '点击画布放置滑块（按住 Shift 可连续放置）',
  ground: '点击画布确定地面高度', wall: '按住拖动画出挡板/极板（线段）', incline: '点击放置斜面（点击处为底角顶点）',
  pointCharge: '点击画布放置点电荷（右侧可改 q）', efield: '拖动画出电场矩形区域；单击 = 全空间电场', bfield_in: '拖动画出磁场 ⊗ 区域；单击 = 全空间',
  bfield_out: '拖动画出磁场 ⊙ 区域；单击 = 全空间', spring: '先点击物体 A，再点击物体 B（或空白处作为固定点）'
};
function setTool(t) {
  tool = t; pending = null;
  document.querySelectorAll('[data-tool]').forEach(b => b.classList.toggle('active', b.dataset.tool === t));
  const hint = $('#toolhint'); if (hint) hint.textContent = TOOL_HINT[t] || '';
  cv.style.cursor = t === 'pan' ? 'grab' : t === 'select' ? 'default' : 'crosshair';
  if (leftTab === 'add') refreshLeft();
  draw();
}
function snap(v) { const S = scene.settings; return S.snap ? Math.round(v / S.snapStep) * S.snapStep : v; }
function snapPt(p) { return [+snap(p[0]).toFixed(6), +snap(p[1]).toFixed(6)]; }
function evPos(e) { const r = cv.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; }

function hitBody(sx, sy) {
  const list = bodies();
  for (let i = list.length - 1; i >= 0; i--) {
    const b = list[i], [x, y] = w2s(b.st.x, b.st.y);
    if (b.kind === 'box') {
      const a = b.st.ang, dx = sx - x, dy = -(sy - y), c = Math.cos(a), s = Math.sin(a);
      const lx = dx * c + dy * s, ly = -dx * s + dy * c, sc = scene.view.scale;
      if (Math.abs(lx) <= Math.max(b.w * sc, 6) / 2 + 3 && Math.abs(ly) <= Math.max(b.h * sc, 6) / 2 + 3) return b;
    } else if (Math.hypot(sx - x, sy - y) <= bodyScreenR(b) + 4) return b;
  }
  return null;
}
function distSeg(px, py, x1, y1, x2, y2) { const c = closestOnSeg(px, py, { x1, y1, x2, y2 }); return Math.hypot(px - c[0], py - c[1]); }
function pointInTri(p, a, b, c) {
  const cr = (u, v) => (v[0] - u[0]) * (p[1] - u[1]) - (v[1] - u[1]) * (p[0] - u[0]);
  const A = cr(a, b), B = cr(b, c), D = cr(c, a);
  return (A >= 0 && B >= 0 && D >= 0) || (A <= 0 && B <= 0 && D <= 0);
}
function hitOther(sx, sy) {
  const objs = scene.objects;
  for (const o of objs) if (o.kind === 'pointCharge') {
    const [x, y] = w2s(o.x, o.y);
    const r = Math.max(8, (o.softRadius || 0.08) * scene.view.scale);
    if (Math.hypot(sx - x, sy - y) <= r + 4) return o;
  }
  for (const o of objs) if (o.kind === 'spring') { const e = springEnds(o); if (!e) continue; const [a, b] = w2s(e.x1, e.y1), [c, d] = w2s(e.x2, e.y2); if (distSeg(sx, sy, a, b, c, d) < 6) return o; }
  for (const o of objs) {
    if (o.kind === 'wall') { const [a, b] = w2s(o.x1, o.y1), [c, d] = w2s(o.x2, o.y2); if (distSeg(sx, sy, a, b, c, d) < 7) return o; }
    if (o.kind === 'ground') { if (Math.abs(sy - w2s(0, o.y)[1]) < 6) return o; }
    if (o.kind === 'incline') { const p = inclinePts(o).map(q => w2s(q[0], q[1])); if (pointInTri([sx, sy], p[0], p[1], p[2])) return o; }
  }
  let best = null, area = Infinity;
  for (const o of objs) if ((o.kind === 'efield' || o.kind === 'bfield') && !o.all) {
    const [x, y, w, h] = fieldScreenRect(o);
    if (sx >= x && sx <= x + w && sy >= y && sy <= y + h && w * h < area) { best = o; area = w * h; }
  }
  return best;
}

cv.addEventListener('contextmenu', e => e.preventDefault());
cv.addEventListener('pointerdown', e => {
  cv.setPointerCapture(e.pointerId);
  const [sx, sy] = evPos(e), wp = s2w(sx, sy); mouseW = wp;
  if (e.button === 1 || e.button === 2 || tool === 'pan' || spaceDown) { drag = { type: 'pan', sx, sy, cx: scene.view.cx, cy: scene.view.cy }; cv.style.cursor = 'grabbing'; return; }
  if (e.button !== 0) return;
  const sp = snapPt(wp);
  // ---- 添加工具 ----
  if (['ball', 'particle', 'box'].includes(tool)) {
    pushUndo(); if (sim.t > 0) resetSim('添加物体，已重置到 t=0');
    const b = mkBody(tool, { x0: sp[0], y0: sp[1] }); scene.objects.push(b); initState(b);
    afterStructure(); select(b.id); if (!e.shiftKey) setTool('select'); return;
  }
  if (tool === 'pointCharge') {
    pushUndo(); if (sim.t > 0) resetSim('添加点电荷，已重置');
    const q = 1e-6;
    const o = mkPointCharge(sp[0], sp[1], { q, color: q >= 0 ? '#e5484d' : '#2f6fed' });
    scene.objects.push(o); afterStructure(); select(o.id); if (!e.shiftKey) setTool('select'); return;
  }
  if (tool === 'ground') { pushUndo(); const g = mkGround(sp[1]); scene.objects.push(g); afterStructure(); select(g.id); setTool('select'); return; }
  if (tool === 'incline') { pushUndo(); const o = mkIncline(sp[0], sp[1]); scene.objects.push(o); afterStructure(); select(o.id); setTool('select'); return; }
  if (['efield', 'bfield_in', 'bfield_out'].includes(tool)) { pending = { type: 'rect', x1: sp[0], y1: sp[1], x2: sp[0], y2: sp[1], sx, sy }; drag = { type: 'create' }; return; }
  if (tool === 'wall') { pending = { type: 'line', x1: sp[0], y1: sp[1], x2: sp[0], y2: sp[1], sx, sy }; drag = { type: 'create' }; return; }
  if (tool === 'spring') {
    const hb = hitBody(sx, sy);
    if (!pending) { if (hb) { pending = { type: 'spring', a: hb.id }; draw(); } else toast('请先点击一个物体作为弹簧的一端'); return; }
    pushUndo();
    const A = byId(pending.a); let s;
    if (hb && hb.id !== pending.a) s = mkSpring(pending.a, hb.id, { L0: +Math.hypot(hb.st.x - A.st.x, hb.st.y - A.st.y).toFixed(3) });
    else s = mkSpring(pending.a, null, { ax: sp[0], ay: sp[1], L0: +Math.hypot(sp[0] - A.st.x, sp[1] - A.st.y).toFixed(3) });
    scene.objects.push(s); pending = null; afterStructure(); select(s.id); setTool('select'); return;
  }
  // ---- 选择工具 ----
  const cur = selected();
  if (isBody(cur) && scene.settings.showForce) {
    const HF = forceHandle(cur);
    if (Math.hypot(sx - HF.hx, sy - HF.hy) < 12) {
      pushUndo();
      // 拖末端 → 切回恒定模式，直接编辑 F / Fang
      cur.Fmode = 'const';
      drag = { type: 'F', b: cur };
      return;
    }
  }
  if (isBody(cur)) {
    const H0 = v0Handle(cur);
    if (Math.hypot(sx - H0.hx, sy - H0.hy) < 11) { pushUndo(); drag = { type: 'v0', b: cur }; return; }
  }
  if (cur && (cur.kind === 'efield' || cur.kind === 'bfield') && !cur.all) {
    const hs = rectHandles(cur);
    for (let i = 0; i < 4; i++) if (Math.hypot(sx - hs[i][0], sy - hs[i][1]) < 9) { pushUndo(); drag = { type: 'fieldCorner', o: cur, i }; return; }
  }
  if (cur && cur.kind === 'wall') {
    const [a, b] = w2s(cur.x1, cur.y1), [c, d] = w2s(cur.x2, cur.y2);
    if (Math.hypot(sx - a, sy - b) < 9) { pushUndo(); drag = { type: 'wallEnd', o: cur, end: 1 }; return; }
    if (Math.hypot(sx - c, sy - d) < 9) { pushUndo(); drag = { type: 'wallEnd', o: cur, end: 2 }; return; }
  }
  const hb = hitBody(sx, sy);
  if (hb) {
    select(hb.id); pushUndo();
    drag = { type: 'body', b: hb, ox: wp[0] - hb.st.x, oy: wp[1] - hb.st.y, moved: false, sx, sy };
    return;
  }
  const ho = hitOther(sx, sy);
  if (ho) { select(ho.id); pushUndo(); drag = { type: 'moveObj', o: ho, last: wp, snapLast: sp, moved: false }; return; }
  drag = { type: 'pan', sx, sy, cx: scene.view.cx, cy: scene.view.cy, click: true };
});
cv.addEventListener('pointermove', e => {
  const [sx, sy] = evPos(e), wp = s2w(sx, sy); mouseW = wp;
  updateStatusMouse();
  if (!drag) { if (pending && pending.type === 'spring') draw(); hoverCursor(sx, sy); return; }
  const sp = snapPt(wp);
  if (drag.type === 'pan') {
    const v = scene.view; v.cx = drag.cx - (sx - drag.sx) / v.scale; v.cy = drag.cy + (sy - drag.sy) / v.scale;
    if (Math.hypot(sx - drag.sx, sy - drag.sy) > 3) drag.click = false;
    draw(); return;
  }
  if (drag.type === 'create') { pending.x2 = sp[0]; pending.y2 = sp[1]; draw(); return; }
  if (drag.type === 'v0') {
    const b = drag.b, S = scene.settings, [x, y] = w2s(b.x0, b.y0);
    let vx = (sx - x) / S.vPx, vy = -(sy - y) / S.vPx;
    if (e.shiftKey) { let v = Math.round(Math.hypot(vx, vy)), a = Math.round(Math.atan2(vy, vx) / DEG / 15) * 15 * DEG; vx = v * Math.cos(a); vy = v * Math.sin(a); }
    else { vx = Math.round(vx * 10) / 10; vy = Math.round(vy * 10) / 10; }
    b.vx0 = clean(vx); b.vy0 = clean(vy);
    initChanged(); return;
  }
  if (drag.type === 'F') {
    const b = drag.b, S = scene.settings;
    const st = b.st || { x: b.x0, y: b.y0 };
    const [x, y] = w2s(st.x, st.y);
    const FPx = S.FPx || 12;
    let fx = (sx - x) / FPx, fy = -(sy - y) / FPx;
    if (e.shiftKey) {
      const mag = Math.round(Math.hypot(fx, fy) * 10) / 10;
      const a = Math.round(Math.atan2(fy, fx) / DEG / 15) * 15 * DEG;
      fx = mag * Math.cos(a); fy = mag * Math.sin(a);
    } else {
      fx = Math.round(fx * 100) / 100; fy = Math.round(fy * 100) / 100;
    }
    b.Fmode = 'const';
    b.F = clean(Math.hypot(fx, fy));
    b.Fang = clean(Math.atan2(fy, fx) / DEG);
    b._Ferr = '';
    liveEdit(); return;
  }
  if (drag.type === 'body') {
    if (!drag.moved && Math.hypot(sx - drag.sx, sy - drag.sy) < 3) return;
    drag.moved = true;
    const b = drag.b, p = snapPt([wp[0] - drag.ox, wp[1] - drag.oy]);
    b.x0 = p[0]; b.y0 = p[1]; initChanged(); return;
  }
  if (drag.type === 'fieldCorner') {
    const o = drag.o, i = drag.i; let x1 = o.x, y1 = o.y, x2 = o.x + o.w, y2 = o.y + o.h;
    // 屏幕角点 0:左上 1:右上 2:右下 3:左下
    if (i === 0 || i === 3) x1 = sp[0]; else x2 = sp[0];
    if (i === 0 || i === 1) y2 = sp[1]; else y1 = sp[1];
    o.x = Math.min(x1, x2); o.w = Math.abs(x2 - x1); o.y = Math.min(y1, y2); o.h = Math.abs(y2 - y1);
    if (x2 < x1) drag.i = [1, 0, 3, 2][drag.i]; if (y2 < y1) drag.i = [3, 2, 1, 0][drag.i];
    liveEdit(); return;
  }
  if (drag.type === 'wallEnd') { const o = drag.o; if (drag.end === 1) { o.x1 = sp[0]; o.y1 = sp[1]; } else { o.x2 = sp[0]; o.y2 = sp[1]; } liveEdit(); return; }
  if (drag.type === 'moveObj') {
    const dx = sp[0] - drag.snapLast[0], dy = sp[1] - drag.snapLast[1]; if (!dx && !dy) return;
    drag.snapLast = sp; drag.moved = true; const o = drag.o;
    if (o.kind === 'ground') o.y = clean(o.y + dy);
    else if (o.kind === 'wall') { o.x1 = clean(o.x1 + dx); o.x2 = clean(o.x2 + dx); o.y1 = clean(o.y1 + dy); o.y2 = clean(o.y2 + dy); }
    else if (o.kind === 'spring') { if (o.b == null) { o.ax = clean(o.ax + dx); o.ay = clean(o.ay + dy); } }
    else { o.x = clean(o.x + dx); o.y = clean(o.y + dy); }
    liveEdit(); return;
  }
});
function finishDrag(e) {
  if (!drag) return;
  const d = drag; drag = null;
  if (d.type === 'pan') { cv.style.cursor = tool === 'pan' ? 'grab' : 'default'; if (d.click && tool === 'select') select(null); markDirty(); return; }
  if (d.type === 'create' && pending) {
    const p = pending; pending = null;
    const tiny = Math.abs(p.x2 - p.x1) < 1e-9 && Math.abs(p.y2 - p.y1) < 1e-9;
    pushUndo();
    if (p.type === 'rect') {
      const r = { x: Math.min(p.x1, p.x2), y: Math.min(p.y1, p.y2), w: Math.abs(p.x2 - p.x1), h: Math.abs(p.y2 - p.y1), all: tiny };
      if (tiny) Object.assign(r, { x: p.x1 - 2, y: p.y1 - 2, w: 4, h: 4 });
      const o = tool === 'efield' ? mkE(r) : mkB(Object.assign(r, { dir: tool === 'bfield_in' ? -1 : 1 }));
      scene.objects.push(o); afterStructure(); select(o.id);
    } else {
      const o = tiny ? mkWall(p.x1, p.y1, p.x1, p.y1 + 2) : mkWall(p.x1, p.y1, p.x2, p.y2);
      scene.objects.push(o); afterStructure(); select(o.id);
    }
    if (!(e && e.shiftKey)) setTool('select');
    return;
  }
  if (d.type === 'body' && !d.moved) undoStack.pop();
  if (d.type === 'moveObj' && !d.moved) undoStack.pop();
  markDirty(); buildPanel(); draw();
}
cv.addEventListener('pointerup', finishDrag);
cv.addEventListener('pointercancel', finishDrag);
cv.addEventListener('wheel', e => {
  e.preventDefault();
  const [sx, sy] = evPos(e), before = s2w(sx, sy), v = scene.view;
  v.scale = Math.min(1e6, Math.max(0.5, v.scale * Math.exp(-e.deltaY * 0.0015)));
  const after = s2w(sx, sy); v.cx += before[0] - after[0]; v.cy += before[1] - after[1];
  draw(); markDirty();
}, { passive: false });
function hoverCursor(sx, sy) {
  if (tool !== 'select') return;
  const cur = selected();
  if (isBody(cur) && scene.settings.showForce) {
    const HF = forceHandle(cur);
    if (Math.hypot(sx - HF.hx, sy - HF.hy) < 12) { cv.style.cursor = 'crosshair'; return; }
  }
  if (isBody(cur)) { const H0 = v0Handle(cur); if (Math.hypot(sx - H0.hx, sy - H0.hy) < 11) { cv.style.cursor = 'crosshair'; return; } }
  cv.style.cursor = hitBody(sx, sy) || hitOther(sx, sy) ? 'move' : 'default';
}
function clean(v) { return +(+v).toFixed(6); }
/** 初始条件被修改：回到 t=0 */
function initChanged() {
  const wasRunning = sim.t > 0;
  resetSim(wasRunning ? '初始条件已修改，已重置到 t=0' : null);
  markFieldsDirty(); refreshInputs(); markDirty(); draw();
}
/** 非初始参数修改：实时生效 */
function liveEdit() { markFieldsDirty(); refreshInputs(); markDirty(); draw(); }
function afterStructure() { ensureStates(); markFieldsDirty(); refreshLeft(); markDirty(); draw(); }

/* ---------------- 键盘 ---------------- */
document.addEventListener('keydown', e => {
  const tag = (document.activeElement && document.activeElement.tagName) || '';
  if (['INPUT', 'SELECT', 'TEXTAREA'].includes(tag)) { if (e.key === 'Escape') document.activeElement.blur(); return; }
  const k = e.key.toLowerCase();
  if ((e.ctrlKey || e.metaKey) && k === 'z') { e.preventDefault(); undo(); return; }
  if ((e.ctrlKey || e.metaKey) && k === 'd') { e.preventDefault(); duplicateSel(); return; }
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  if (e.key === ' ') { e.preventDefault(); if (!spaceDown) togglePlay(); return; }
  if (k === 'delete' || k === 'backspace') { deleteSel(); return; }
  if (k === 'escape') {
    $('#moreMenu')?.classList.remove('open');
    closeMobilePanels();
    if (tool !== 'select') setTool('select');
    else if (selId != null) select(null);
    return;
  }
  if (k === 'r') { resetSim('已重置到 t=0'); draw(); return; }
  if (k === '.') { stepOnce(); return; }
  if (k === 'v') setTool('select'); if (k === 'h') setTool('pan'); if (k === 'f') fitView();
});

/* ---------------- 撤销 ---------------- */
const undoStack = [];
function pushUndo() { undoStack.push(serialize()); if (undoStack.length > 80) undoStack.shift(); }
function undo() { const s = undoStack.pop(); if (!s) { toast('没有可撤销的操作'); return; } loadScene(JSON.parse(s), true); toast('已撤销'); }

/* ---------------- 复制/删除 ---------------- */
function deleteSel() {
  const s = selected(); if (!s) return; pushUndo();
  scene.objects = scene.objects.filter(o => o !== s && !(o.kind === 'spring' && (o.a === s.id || o.b === s.id)));
  selId = null; afterStructure(); buildPanel(); toast(`已删除 ${s.name}`);
}
function duplicateSel() {
  const s = selected(); if (!s) return; pushUndo();
  const c = JSON.parse(JSON.stringify(Object.assign({}, s, { st: null }))); c.id = nextId++; c.name = s.name + '副本';
  const off = 0.5;
  if (isBody(c)) { c.x0 += off; c.y0 += off; } else if (c.kind === 'wall') { c.x1 += off; c.x2 += off; } else if (c.kind === 'ground') c.y += off; else if (c.kind !== 'spring') { c.x += off; c.y -= off; }
  scene.objects.push(c); if (isBody(c)) { if (sim.t > 0) resetSim(); initState(c); }
  afterStructure(); select(c.id); toast('已复制');
}

/* ---------------- 播放控制 ---------------- */
function updatePlayBtn() { const b = $('#bPlay'); if (b) b.textContent = sim.running ? '暂停' : '播放'; }
function togglePlay() { ensureStates(); sim.running = !sim.running; updatePlayBtn(); lastTs = 0; draw(); updateReadouts(); }
function stepOnce() { sim.running = false; updatePlayBtn(); advance(1 / 60 * scene.settings.speed); afterFrame(); }
let lastTs = 0;
let needsDraw = true;
function requestDraw() { needsDraw = true; }
function loop(ts) {
  if (sim.running) {
    const real = lastTs ? Math.min((ts - lastTs) / 1000, 1 / 20) : 1 / 60;
    advance(real * scene.settings.speed);
    needsDraw = true;
    updateReadouts();
  }
  if (needsDraw) {
    needsDraw = false;
    draw();
  }
  lastTs = ts;
  requestAnimationFrame(loop);
}
function afterFrame() { requestDraw(); updateReadouts(); }
function updateBadge() {
  $('#tbadge').innerHTML = `t = <b>${fmt(sim.t, 3)}</b> s &nbsp; <span class="${sim.running ? 'run' : 'pause'}">${sim.running ? '● 运行中' : sim.t === 0 ? '■ 初始状态' : '❚❚ 已暂停'}</span> &nbsp; ${scene.settings.speed}×`;
}
function fitView() {
  const pts = [];
  for (const o of scene.objects) {
    if (isBody(o)) { pts.push([o.st ? o.st.x : o.x0, o.st ? o.st.y : o.y0], [o.x0, o.y0]); }
    else if (o.kind === 'wall') pts.push([o.x1, o.y1], [o.x2, o.y2]);
    else if (o.kind === 'incline') pts.push(...inclinePts(o));
    else if ((o.kind === 'efield' || o.kind === 'bfield') && !o.all) pts.push([o.x, o.y], [o.x + o.w, o.y + o.h]);
    else if (o.kind === 'ground') pts.push([0, o.y]);
  }
  if (!pts.length) { scene.view.cx = 0; scene.view.cy = 0; scene.view.scale = 60; draw(); return; }
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  pts.forEach(p => { x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]); y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]); });
  const w = Math.max(x1 - x0, 1), h = Math.max(y1 - y0, 1);
  scene.view.scale = Math.min(W / (w * 1.35), H / (h * 1.45));
  scene.view.cx = (x0 + x1) / 2; scene.view.cy = (y0 + y1) / 2; draw(); markDirty();
}

/* =====================================================================
 *  右侧属性面板
 * ===================================================================== */
let binds = [];
function el(tag, attrs = {}, html) { const e = document.createElement(tag); for (const k in attrs) { if (k === 'class') e.className = attrs[k]; else if (k.startsWith('on')) e.addEventListener(k.slice(2), attrs[k]); else e.setAttribute(k, attrs[k]); } if (html != null) e.innerHTML = html; return e; }
function fmtIn(v) { if (typeof v !== 'number') return v; if (!isFinite(v)) return ''; return String(+v.toPrecision(6)); }
function refreshInputs() { binds.forEach(b => { if (document.activeElement !== b.el) { if (b.el.type === 'checkbox') b.el.checked = !!b.get(); else b.el.value = fmtIn(b.get()); } }); }
function section(parent, title, cls = '', extra) { const s = el('div', { class: 'sec ' + cls }); const h = el('h4', {}, title); if (extra) h.appendChild(extra); s.appendChild(h); parent.appendChild(s); return s; }
function numRow(parent, lab, get, set, unit = '', o = {}) {
  const r = el('div', { class: 'row' }); r.appendChild(el('span', { class: 'lab' }, lab));
  const i = el('input', { type: 'number', step: o.step || 'any' }); if (o.min != null) i.min = o.min; if (o.title) r.title = o.title;
  i.value = fmtIn(get()); let pushed = false;
  i.addEventListener('focus', () => pushed = false);
  i.addEventListener('input', () => {
    let v = parseFloat(i.value); if (!isFinite(v)) return; if (o.min != null && v < o.min) return;
    if (!pushed) { pushUndo(); pushed = true; }
    set(v); o.init ? initChanged() : liveEdit(); if (o.after) o.after();
  });
  i.addEventListener('change', () => { i.value = fmtIn(get()); });
  r.appendChild(i); r.appendChild(el('span', { class: 'unit' }, unit)); parent.appendChild(r);
  binds.push({ el: i, get }); return i;
}
function chkRow(parent, lab, get, set, o = {}) {
  const r = el('label', { class: 'chk', style: 'margin:4px 10px 4px 0' }); const i = el('input', { type: 'checkbox' }); i.checked = !!get();
  i.addEventListener('change', () => { pushUndo(); set(i.checked); o.init ? initChanged() : liveEdit(); if (o.rebuild) buildPanel(); });
  r.appendChild(i); r.appendChild(document.createTextNode(lab)); parent.appendChild(r); binds.push({ el: i, get }); return i;
}
function textRow(parent, lab, get, set, o = {}) {
  const r = el('div', { class: 'row expr-row' });
  r.appendChild(el('span', { class: 'lab' }, lab));
  const i = el('input', { type: 'text', class: 'expr-input', spellcheck: 'false', autocomplete: 'off' });
  if (o.placeholder) i.placeholder = o.placeholder;
  if (o.title) i.title = o.title;
  i.value = get() ?? '';
  let pushed = false;
  i.addEventListener('focus', () => { pushed = false; });
  i.addEventListener('input', () => {
    if (!pushed) { pushUndo(); pushed = true; }
    set(i.value);
    liveEdit();
    if (o.after) o.after();
  });
  i.addEventListener('change', () => { i.value = get() ?? ''; if (o.rebuild) buildPanel(); });
  r.appendChild(i);
  parent.appendChild(r);
  binds.push({ el: i, get: () => get() ?? '' });
  return i;
}
function selectRow(parent, lab, get, set, options, o = {}) {
  const r = el('div', { class: 'row' });
  r.appendChild(el('span', { class: 'lab' }, lab));
  const s = el('select', { class: 'force-mode' });
  options.forEach(([val, text]) => s.appendChild(el('option', { value: val }, text)));
  s.value = get();
  s.addEventListener('change', () => {
    pushUndo();
    set(s.value);
    liveEdit();
    if (o.rebuild !== false) buildPanel();
  });
  r.appendChild(s);
  parent.appendChild(r);
  return s;
}
function btn(parent, html, fn, cls = '', title = '') { const b = el('button', { class: cls, title, onclick: fn }, html); parent.appendChild(b); return b; }
function readRow(parent, pairs) { const g = el('div', { class: 'readout' }); pairs.forEach(([k, id]) => { g.appendChild(el('span', { class: 'k' }, k)); g.appendChild(el('span', { id })); }); parent.appendChild(g); return g; }


function makeTabs(items, active, onPick, attrs = {}) {
  const bar = el('div', { class: 'tabs', role: 'tablist' });
  items.forEach(([id, label]) => {
    const b = el('button', { type: 'button', class: id === active ? 'active' : '', 'data-tab': id }, label);
    b.addEventListener('click', () => onPick(id));
    bar.appendChild(b);
  });
  if (attrs.id) bar.id = attrs.id;
  return bar;
}

function panelBox(id, active) {
  return el('div', { class: 'tab-panel' + (id === active ? ' active' : ''), id: 'panel-' + id, 'data-panel': id });
}

function buildPanel() {
  binds = [];
  const P = $('#props'); P.innerHTML = '';
  const o = selected();
  if (!o) return buildScenePanel(P);

  const head = el('div', { class: 'insp-head' });
  const nm = el('input', { type: 'text', value: o.name, 'aria-label': '名称' });
  nm.addEventListener('input', () => { o.name = nm.value; refreshLeft(); draw(); markDirty(); });
  head.appendChild(nm);
  if (o.color != null) {
    const c = el('input', { type: 'color', value: o.color, title: '颜色' });
    c.addEventListener('input', () => { o.color = c.value; refreshLeft(); draw(); markDirty(); });
    head.appendChild(c);
  }
  P.appendChild(head);
  const kindExtra = o.kind === 'bfield' ? (o.dir < 0 ? ' · 向里' : ' · 向外') : '';
  P.appendChild(el('div', { class: 'insp-meta' }, KIND_NAME[o.kind] + kindExtra));

  if (isBody(o)) buildBodyPanel(P, o);
  else buildNonBodyPanel(P, o);
}

function buildNonBodyPanel(P, o) {
  const tabs = [['props', '属性'], ['actions', '操作']];
  if (!tabs.some(([id]) => id === propsTab)) propsTab = 'props';
  P.appendChild(makeTabs(tabs, propsTab, id => { propsTab = id; buildPanel(); }));
  const panels = el('div', { class: 'tab-panels' });
  const pProps = panelBox('props', propsTab);
  const pAct = panelBox('actions', propsTab);

  if (o.kind === 'pointCharge') buildPointChargePanel(pProps, o);
  else if (o.kind === 'efield') buildEPanel(pProps, o);
  else if (o.kind === 'bfield') buildBPanel(pProps, o);
  else if (o.kind === 'ground') {
    const s = section(pProps, '几何'); numRow(s, '高度 y', () => o.y, v => o.y = v, 'm'); surfOpts(s, o);
  } else if (o.kind === 'wall') {
    const s = section(pProps, '端点');
    numRow(s, '起点 x₁', () => o.x1, v => o.x1 = v, 'm');
    numRow(s, '起点 y₁', () => o.y1, v => o.y1 = v, 'm');
    numRow(s, '终点 x₂', () => o.x2, v => o.x2 = v, 'm');
    numRow(s, '终点 y₂', () => o.y2, v => o.y2 = v, 'm');
    s.appendChild(el('div', { class: 'hint' }, '可拖动两端控制点。'));
    surfOpts(s, o);
  } else if (o.kind === 'incline') {
    const s = section(pProps, '几何');
    numRow(s, '底角 x', () => o.x, v => o.x = v, 'm');
    numRow(s, '底角 y', () => o.y, v => o.y = v, 'm');
    numRow(s, '底边 L', () => o.L, v => o.L = v, 'm', { min: 0.01 });
    numRow(s, '倾角 θ', () => o.ang, v => o.ang = Math.min(85, v), '°', { min: 0 });
    const br = el('div', { class: 'btnrow' });
    [30, 37, 45, 53, 60].forEach(a => btn(br, a + '°', () => { pushUndo(); o.ang = a; liveEdit(); }));
    btn(br, o.dir > 0 ? '朝右' : '朝左', () => { pushUndo(); o.dir *= -1; liveEdit(); buildPanel(); });
    s.appendChild(br);
    s.appendChild(el('div', { class: 'hint' }, `斜面长 ${fmt(o.L / Math.cos(o.ang * DEG), 3)} m，高 ${fmt(o.L * Math.tan(o.ang * DEG), 3)} m`));
    surfOpts(s, o);
    const s2 = section(pProps, '快捷');
    btn(s2, '在顶端放置滑块', () => placeOnIncline(o));
  } else if (o.kind === 'spring') buildSpringPanel(pProps, o);

  const act = section(pAct, '操作');
  const br = el('div', { class: 'btnrow' });
  btn(br, '复制', duplicateSel); btn(br, '删除', deleteSel, 'danger');
  act.appendChild(br);

  panels.appendChild(pProps); panels.appendChild(pAct);
  P.appendChild(panels);
}

function buildAppliedForcePanel(P, b) {
  ensureForceFields(b);
  const f = section(P, '外力 F');
  selectRow(f, '模式', () => b.Fmode || 'const', v => { b.Fmode = v; b._Ferr = ''; }, [
    ['const', '恒定'],
    ['expr', '表达式'],
    ['aim', '指向目标'],
  ]);

  const mode = b.Fmode || 'const';
  if (mode === 'const') {
    numRow(f, '大小 F', () => b.F, v => b.F = v, 'N', { min: 0 });
    numRow(f, '方向角', () => b.Fang, v => b.Fang = v, '°');
    f.appendChild(el('div', { class: 'hint' }, '开启顶栏「受力」后，可直接拖紫色箭头末端设置 F。Shift 拖动按 15° 吸附。'));
  } else if (mode === 'expr') {
    textRow(f, '大小 F(t,…)', () => b.Fexpr, v => { b.Fexpr = v; }, {
      placeholder: '如 10*sin(2*pi*t)', title: FORCE_EXPR_HELP,
    });
    textRow(f, '方向 θ(°)', () => b.FangExpr, v => { b.FangExpr = v; }, {
      placeholder: '如 90 或 atan2(1,0)*180/pi', title: FORCE_EXPR_HELP,
    });
    f.appendChild(el('div', { class: 'hint' }, FORCE_EXPR_HELP));
    f.appendChild(el('div', { class: 'hint' }, `当前求值：F=${fmt(b.F, 3)} N，θ=${fmt(b.Fang, 2)}°${b._Ferr ? ' · 错误：' + b._Ferr : ''}`));
  } else {
    // aim
    textRow(f, '大小 F 表达式', () => b.Fexpr, v => { b.Fexpr = v; }, {
      placeholder: '留空则用下方恒定值；如 20+5*sin(t)', title: FORCE_EXPR_HELP,
    });
    numRow(f, '大小 F（备用）', () => b.F, v => b.F = v, 'N', { min: 0 });
    const targets = [['', '世界坐标']].concat(
      scene.objects
        .filter(o => o.id !== b.id && (isBody(o) || o.kind === 'pointCharge'))
        .map(o => [String(o.id), o.name]),
    );
    selectRow(f, '指向', () => (b.FaimId == null ? '' : String(b.FaimId)), v => {
      b.FaimId = v === '' ? null : +v;
    }, targets);
    if (b.FaimId == null) {
      numRow(f, '目标 x', () => b.FaimX, v => b.FaimX = v, 'm');
      numRow(f, '目标 y', () => b.FaimY, v => b.FaimY = v, 'm');
    }
    f.appendChild(el('div', { class: 'hint' }, '力始终指向目标；大小可用表达式随时间/状态变化。'));
    f.appendChild(el('div', { class: 'hint' }, `当前：F=${fmt(b.F, 3)} N，θ=${fmt(b.Fang, 2)}°${b._Ferr ? ' · ' + b._Ferr : ''}`));
  }

  const br = el('div', { class: 'btnrow' });
  btn(br, scene.settings.showForce ? '受力显示已开' : '开启受力显示', () => {
    scene.settings.showForce = true;
    syncToolbar();
    liveEdit();
    buildPanel();
    toast('拖紫色箭头末端可设置外力');
  }, scene.settings.showForce ? 'active' : '');
  btn(br, '清零外力', () => {
    pushUndo();
    b.F = 0; b.Fang = 0; b.Fmode = 'const'; b._Ferr = '';
    liveEdit(); buildPanel();
  }, 'danger');
  f.appendChild(br);
}

let chartMode = 'v';
function buildBodyPanel(P, b) {
  const tabs = [
    ['init', '初态'],
    ['props', '属性'],
    ['live', '实时'],
    ['analysis', '分析'],
    ['chart', '图像'],
    ['actions', '操作'],
  ];
  if (!tabs.some(([id]) => id === propsTab)) propsTab = 'init';
  P.appendChild(makeTabs(tabs, propsTab, id => { propsTab = id; buildPanel(); }));
  const panels = el('div', { class: 'tab-panels' });

  const pInit = panelBox('init', propsTab);
  {
    const s = section(pInit, '初始条件 t = 0', 'hl');
    numRow(s, '位置 x₀', () => b.x0, v => b.x0 = v, 'm', { init: true });
    numRow(s, '位置 y₀', () => b.y0, v => b.y0 = v, 'm', { init: true });
    const vmag = () => Math.hypot(b.vx0, b.vy0);
    const vang = () => (b.vx0 || b.vy0) ? Math.atan2(b.vy0, b.vx0) / DEG : (b._lastAng || 0);
    numRow(s, '速率 v₀', vmag, v => { const a = vang() * DEG; b.vx0 = clean(v * Math.cos(a)); b.vy0 = clean(v * Math.sin(a)); }, 'm/s', { init: true, min: 0 });
    numRow(s, '方向 θ', vang, a => { b._lastAng = a; const v = vmag(); b.vx0 = clean(v * Math.cos(a * DEG)); b.vy0 = clean(v * Math.sin(a * DEG)); }, '°', { init: true, title: '相对 +x，逆时针为正' });
    const br = el('div', { class: 'btnrow' });
    [[0, '0°'], [45, '45°'], [90, '90°'], [135, '135°'], [180, '180°'], [-90, '-90°']].forEach(([a, t]) =>
      btn(br, t, () => { pushUndo(); b._lastAng = a; const v = vmag() || 1; b.vx0 = clean(v * Math.cos(a * DEG)); b.vy0 = clean(v * Math.sin(a * DEG)); initChanged(); }));
    btn(br, '清零', () => { pushUndo(); b.vx0 = 0; b.vy0 = 0; initChanged(); }, 'danger');
    s.appendChild(br);
    numRow(s, '分量 vx₀', () => b.vx0, v => b.vx0 = v, 'm/s', { init: true });
    numRow(s, '分量 vy₀', () => b.vy0, v => b.vy0 = v, 'm/s', { init: true });
    s.appendChild(el('div', { class: 'hint' }, '也可拖动画布上的速度手柄；运行中修改初态会回到 t = 0。'));
  }

  const pProps = panelBox('props', propsTab);
  {
    const p = section(pProps, '物理量');
    numRow(p, '质量 m', () => b.m, v => b.m = v, 'kg', { min: 1e-40, title: '如 9.1e-31' });
    numRow(p, '电荷 q', () => b.q, v => b.q = v, 'C', { title: '如 -1.6e-19' });
    if (b.kind === 'box') {
      numRow(p, '宽 w', () => b.w, v => b.w = v, 'm', { min: 0.001 });
      numRow(p, '高 h', () => b.h, v => b.h = v, 'm', { min: 0.001 });
      numRow(p, '倾角', () => b.ang0, v => b.ang0 = v, '°', { init: true });
    } else numRow(p, '半径 r', () => b.r, v => b.r = v, 'm', { min: 0.0001 });
    numRow(p, '摩擦 μ', () => b.mu, v => b.mu = v, '', { step: 0.05, min: 0 });
    numRow(p, '恢复系数 e', () => b.e, v => b.e = Math.min(1, v), '', { step: 0.1, min: 0 });
    buildAppliedForcePanel(pProps, b);
    const o = section(pProps, '行为');
    chkRow(o, '固定', () => b.fixed, v => b.fixed = v, { init: true });
    chkRow(o, '显示轨迹', () => b.trail, v => b.trail = v);
    chkRow(o, '参与碰撞', () => b.collide, v => b.collide = v);
  }

  const pLive = panelBox('live', propsTab);
  {
    const r = section(pLive, '瞬时状态');
    readRow(r, [['位置', 'r_pos'], ['速度', 'r_vel'], ['速率', 'r_v'], ['方向', 'r_ang'], ['加速度', 'r_a'], ['动能', 'r_ek'], ['动量', 'r_p'], ['重力势能', 'r_ep'], ['机械能', 'r_e']]);
  }

  const pAn = panelBox('analysis', propsTab);
  {
    const th = section(pAn, '运动分析');
    th.appendChild(el('div', { id: 'r_theory', class: 'analysis-panel' }));
  }

  const pCh = panelBox('chart', propsTab);
  {
    const sel = el('select', { id: 'chartSel' });
    [['v', 'v–t'], ['vx', 'vx–t'], ['vy', 'vy–t'], ['x', 'x–t'], ['y', 'y–t'], ['ek', 'Ek–t'], ['xy', 'y–x']].forEach(([v, t]) => sel.appendChild(el('option', { value: v }, t)));
    sel.value = chartMode;
    sel.addEventListener('change', () => { chartMode = sel.value; drawChart(); });
    const g = section(pCh, '图像', '', sel);
    g.appendChild(el('canvas', { id: 'chart' }));
  }

  const pAct = panelBox('actions', propsTab);
  {
    const act = section(pAct, '操作');
    const br = el('div', { class: 'btnrow' });
    btn(br, '复制', duplicateSel);
    btn(br, '删除', deleteSel, 'danger');
    btn(br, '清除轨迹', () => { b.st.trail = [[b.st.x, b.st.y]]; draw(); });
    act.appendChild(br);
  }

  [pInit, pProps, pLive, pAn, pCh, pAct].forEach(x => panels.appendChild(x));
  P.appendChild(panels);
  updateReadouts();
}

function buildPointChargePanel(P, o) {
  const s = section(P, '点电荷');
  numRow(s, '位置 x', () => o.x, v => o.x = v, 'm');
  numRow(s, '位置 y', () => o.y, v => o.y = v, 'm');
  numRow(s, '电荷 q', () => o.q, v => { o.q = v; o.color = v >= 0 ? '#e5484d' : '#2f6fed'; }, 'C', { title: '如 1e-6；负号表示负电' });
  numRow(s, '软化半径', () => o.softRadius, v => o.softRadius = v, 'm', { min: 1e-4 });
  const br = el('div', { class: 'btnrow' });
  btn(br, '+1 μC', () => { pushUndo(); o.q = 1e-6; o.color = '#e5484d'; liveEdit(); buildPanel(); });
  btn(br, '−1 μC', () => { pushUndo(); o.q = -1e-6; o.color = '#2f6fed'; liveEdit(); buildPanel(); });
  btn(br, '取反', () => { pushUndo(); o.q = -o.q; o.color = o.q >= 0 ? '#e5484d' : '#2f6fed'; liveEdit(); buildPanel(); });
  s.appendChild(br);
  s.appendChild(el('div', { class: 'hint' }, `力律：${scene.settings.coulombLaw === 'coulomb2d' ? '二维/线电荷截面 (1/r)' : '三维点电荷 (1/r²)'}。电场线由 FieldSystem 积分绘制。`));
  const f = fieldAt(o.x + (o.softRadius || 0.08) * 2, o.y);
  s.appendChild(el('div', { class: 'hint' }, `旁侧抽样 |E| ≈ ${fmt(Math.hypot(f.Ex, f.Ey))} V/m`));
}
function buildEPanel(P, o) {
  const s = section(P, '匀强电场');
  numRow(s, '场强 E', () => o.E, v => o.E = v, 'V/m', { min: 0 });
  numRow(s, '方向角', () => o.ang, v => o.ang = v, '°');
  const br = el('div', { class: 'btnrow' });
  [[90, '上'], [-90, '下'], [180, '左'], [0, '右']].forEach(([a, t]) => btn(br, t, () => { pushUndo(); o.ang = a; liveEdit(); }));
  s.appendChild(br);
  regionSec(P, o);
  P.appendChild(el('div', { class: 'hint', style: 'padding:8px 12px' }, 'F = qE。平行板：E = U/d。'));
  const s2 = section(P, '由电压推算');
  const r1 = el('div', { class: 'row' }); r1.appendChild(el('span', { class: 'lab' }, 'U / d'));
  const iu = el('input', { type: 'number', value: '100', title: 'U (V)' });
  const id_ = el('input', { type: 'number', value: '1', title: 'd (m)' });
  r1.appendChild(iu); r1.appendChild(id_); s2.appendChild(r1);
  btn(s2, '设 E = U/d', () => { const u = +iu.value, dd = +id_.value; if (dd > 0) { pushUndo(); o.E = Math.abs(u / dd); liveEdit(); } });
}

function buildBPanel(P, o) {
  const s = section(P, '匀强磁场');
  numRow(s, '磁感应强度 B', () => o.B, v => o.B = v, 'T', { min: 0 });
  const br = el('div', { class: 'btnrow' });
  btn(br, '向里 ⊗', () => { pushUndo(); o.dir = -1; liveEdit(); buildPanel(); }, o.dir < 0 ? 'active' : '');
  btn(br, '向外 ⊙', () => { pushUndo(); o.dir = 1; liveEdit(); buildPanel(); }, o.dir > 0 ? 'active' : '');
  s.appendChild(br);
  regionSec(P, o);
  P.appendChild(el('div', { class: 'hint', style: 'padding:8px 12px' }, 'F = qv×B。r = mv/|qB|，T = 2πm/|qB|。'));
}

function regionSec(P, o) {
  const s = section(P, '作用区域');
  chkRow(s, '全空间', () => o.all, v => o.all = v, { rebuild: true });
  if (!o.all) {
    numRow(s, '左下角 x', () => o.x, v => o.x = v, 'm');
    numRow(s, '左下角 y', () => o.y, v => o.y = v, 'm');
    numRow(s, '宽度', () => o.w, v => o.w = v, 'm', { min: 0 });
    numRow(s, '高度', () => o.h, v => o.h = v, 'm', { min: 0 });
    s.appendChild(el('div', { class: 'hint' }, '可拖四角调整，拖内部移动。'));
  }
}
function buildSpringPanel(P, o) {
  const s = section(P, o.rope ? '轻绳' : '弹簧');
  chkRow(s, '轻绳模式（只拉不压）', () => o.rope, v => o.rope = v, { rebuild: true });
  numRow(s, o.rope ? '绳长 L' : '原长 L₀', () => o.L0, v => o.L0 = v, 'm', { min: 0 });
  numRow(s, o.rope ? '刚度' : '劲度 k', () => o.k, v => o.k = v, 'N/m', { min: 0 });
  if (o.b == null) {
    numRow(s, '固定点 x', () => o.ax, v => o.ax = v, 'm');
    numRow(s, '固定点 y', () => o.ay, v => o.ay = v, 'm');
  }
  const A = byId(o.a), B = o.b != null ? byId(o.b) : null;
  s.appendChild(el('div', { class: 'hint' }, `连接：${A ? A.name : '?'} — ${B ? B.name : '固定点'}`));
  btn(s, '原长 = 当前长度', () => { const e = springEnds(o); if (e) { pushUndo(); o.L0 = clean(Math.hypot(e.x2 - e.x1, e.y2 - e.y1)); liveEdit(); } });
  if (o.rope) s.appendChild(el('div', { class: 'hint' }, '轻绳用硬弹簧近似，建议 k ≥ 1000 N/m。'));
}

function buildScenePanel(P) {
  const S = scene.settings;
  propsTab = propsTab === 'init' ? 'scene' : propsTab;
  const tabs = [['scene', '场景'], ['display', '显示'], ['help', '指引']];
  if (!tabs.some(([id]) => id === propsTab)) propsTab = 'scene';
  const head = el('div', { class: 'insp-head' });
  head.appendChild(el('div', { style: 'font-weight:600' }, '场景'));
  P.appendChild(head);
  P.appendChild(makeTabs(tabs, propsTab, id => { propsTab = id; buildPanel(); }));
  const panels = el('div', { class: 'tab-panels' });

  const pScene = panelBox('scene', propsTab);
  {
    const s = section(pScene, '全局物理');
    chkRow(s, '开启重力', () => S.gOn, v => { S.gOn = v; syncToolbar(); });
    numRow(s, '重力加速度 g', () => S.g, v => { S.g = v; syncToolbar(); }, 'm/s²');
    const br = el('div', { class: 'btnrow' });
    btn(br, 'g = 9.8', () => { S.g = 9.8; syncToolbar(); liveEdit(); });
    btn(br, 'g = 10', () => { S.g = 10; syncToolbar(); liveEdit(); });
    s.appendChild(br);
    numRow(s, '空气阻力', () => S.drag, v => S.drag = v, 'N·s/m', { min: 0 });
    chkRow(s, '带电体发射库仑场', () => S.coulomb, v => { S.coulomb = v; markFieldsDirty(); });
    numRow(s, '静电力常量 k', () => S.kc, v => { S.kc = v; markFieldsDirty(); }, 'N·m²/C²');
    const lawRow = el('div', { class: 'btnrow' });
    btn(lawRow, '力律 1/r²', () => { S.coulombLaw = 'coulomb3d'; markFieldsDirty(); liveEdit(); buildPanel(); }, S.coulombLaw !== 'coulomb2d' ? 'active' : '');
    btn(lawRow, '力律 1/r', () => { S.coulombLaw = 'coulomb2d'; markFieldsDirty(); liveEdit(); buildPanel(); }, S.coulombLaw === 'coulomb2d' ? 'active' : '');
    s.appendChild(lawRow);
    chkRow(s, '显示电场线', () => S.showFieldLines, v => { S.showFieldLines = v; markFieldsDirty(); });
  }

  const pDisp = panelBox('display', propsTab);
  {
    const d = section(pDisp, '显示与精度');
    numRow(d, '速度箭头比例', () => S.vPx, v => S.vPx = v, 'px/(m/s)', { min: 0.0001 });
    numRow(d, '外力箭头比例', () => S.FPx || 12, v => S.FPx = v, 'px/N', { min: 0.0001 });
    numRow(d, '积分步长', () => S.maxDt, v => S.maxDt = v, 's', { min: 1e-9 });
    chkRow(d, '吸附网格', () => S.snap, v => S.snap = v);
    numRow(d, '吸附步长', () => S.snapStep, v => S.snapStep = v, 'm', { min: 1e-6 });
    chkRow(d, '显示名称', () => S.showLabels, v => S.showLabels = v);
    const view = el('div', { class: 'btnrow' });
    btn(view, '适配视图', fitView);
    btn(view, '回到原点', () => { scene.view.cx = 0; scene.view.cy = 0; draw(); });
    d.appendChild(view);
  }

  const pHelp = panelBox('help', propsTab);
  {
    const h = section(pHelp, '流程');
    h.appendChild(el('div', { class: 'hint' }, `
      1. 在左侧「添加」放入物体与场；<br>
      2. 在「初态」或画布上设置 v₀；<br>
      3. 播放观察，于「分析」查看参数方程；<br>
      4. 当前：${bodies().length} 个物体，${scene.objects.length - bodies().length} 个场/表面/约束。`));
  }

  [pScene, pDisp, pHelp].forEach(x => panels.appendChild(x));
  P.appendChild(panels);
}

const ADD_CATALOG = [
  { cat: '质点与刚体', items: [
    { tool: 'ball', name: '小球', desc: '质点' },
    { tool: 'particle', name: '带电粒子', desc: '可受电磁力' },
    { tool: 'box', name: '滑块', desc: '矩形刚体' },
  ]},
  { cat: '表面与边界', items: [
    { tool: 'ground', name: '地面', desc: '水平半空间' },
    { tool: 'wall', name: '挡板', desc: '线段/极板' },
    { tool: 'incline', name: '斜面', desc: '三角形楔' },
  ]},
  { cat: '场', items: [
    { tool: 'pointCharge', name: '点电荷', desc: '库仑场 · 力线' },
    { tool: 'efield', name: '匀强电场', desc: '矩形 / 全空间' },
    { tool: 'bfield_in', name: '磁场向里', desc: 'B ⊗' },
    { tool: 'bfield_out', name: '磁场向外', desc: 'B ⊙' },
  ]},
  { cat: '约束', items: [
    { tool: 'spring', name: '弹簧 / 绳', desc: '连接两点' },
  ]},
];

function matchQuery(text, q) {
  if (!q) return true;
  return String(text).toLowerCase().includes(q.trim().toLowerCase());
}

function refreshLeft() {
  const host = $('#leftBody');
  if (!host) return;
  host.innerHTML = '';
  const q = leftQuery;

  if (leftTab === 'objects') {
    const chips = el('div', { class: 'chips' });
    [['all', '全部'], ['body', '物体'], ['field', '场'], ['surface', '表面'], ['spring', '约束']].forEach(([id, lab]) => {
      const b = el('button', { type: 'button', class: 'chip' + (leftChip === id ? ' active' : '') }, lab);
      b.onclick = () => { leftChip = id; refreshLeft(); };
      chips.appendChild(b);
    });
    host.appendChild(chips);
    const wrap = el('div', { class: 'side-body pad' });
    const groups = [
      ['物体', 'body', o => isBody(o)],
      ['场', 'field', o => o.kind === 'efield' || o.kind === 'bfield' || o.kind === 'pointCharge'],
      ['表面', 'surface', o => ['ground', 'wall', 'incline'].includes(o.kind)],
      ['约束', 'spring', o => o.kind === 'spring'],
    ];
    let any = false;
    for (const [title, chip, pred] of groups) {
      if (leftChip !== 'all' && leftChip !== chip) continue;
      const items = scene.objects.filter(o => pred(o) && (matchQuery(o.name, q) || matchQuery(KIND_NAME[o.kind], q)));
      if (!items.length) continue;
      any = true;
      wrap.appendChild(el('div', { class: 'objgroup' }, `${title} · ${items.length}`));
      const ul = el('ul', { class: 'objlist' });
      for (const o of items) {
        const li = el('li', { class: o.id === selId ? 'sel' : '' });
        const col = o.color || (o.kind === 'efield' ? '#b45f14' : o.kind === 'bfield' ? '#0f6e6a' : o.kind === 'spring' ? '#0f766e' : '#6b7280');
        li.appendChild(el('span', { class: 'dot', style: `background:${col}` }));
        li.appendChild(el('span', {}, o.name));
        const kindLabel = o.kind === 'bfield' ? (o.dir < 0 ? 'B⊗' : 'B⊙')
          : o.kind === 'efield' ? 'E'
          : o.kind === 'spring' ? (o.rope ? '绳' : '簧')
          : KIND_NAME[o.kind];
        if (kindLabel && o.name !== kindLabel && !o.name.startsWith(KIND_NAME[o.kind] || '')) {
          li.appendChild(el('span', { class: 'ty' }, kindLabel));
        } else if (o.kind === 'bfield' || o.kind === 'efield' || o.kind === 'spring') {
          li.appendChild(el('span', { class: 'ty' }, kindLabel));
        }
        li.dataset.id = String(o.id);
        li.addEventListener('click', () => { select(o.id); closeMobilePanels(); });
        ul.appendChild(li);
      }
      wrap.appendChild(ul);
    }
    if (!any) wrap.appendChild(el('div', { class: 'empty-hint' }, scene.objects.length ? '无匹配结果。' : '场景为空。切换到「添加」或「预设」开始。'));
    host.appendChild(wrap);
    return;
  }

  if (leftTab === 'add') {
    const grid = el('div', { class: 'add-grid' });
    for (const block of ADD_CATALOG) {
      const visibleItems = block.items.filter(it => matchQuery(it.name + it.desc + block.cat, q));
      if (!visibleItems.length) continue;
      grid.appendChild(el('div', { class: 'add-cat' }, block.cat));
      for (const it of visibleItems) {
        const b = el('button', { type: 'button', title: it.desc }, `<div><b>${it.name}</b><div class="hint" style="margin:0">${it.desc}</div></div>`);
        b.onclick = () => { setTool(it.tool); closeMobilePanels(false); toast('在画布上点击或拖拽以放置'); };
        if (tool === it.tool) b.classList.add('active');
        grid.appendChild(b);
      }
    }
    if (!grid.childNodes.length) grid.appendChild(el('div', { class: 'empty-hint' }, '无匹配的工具。'));
    host.appendChild(grid);
    return;
  }

  if (leftTab === 'presets') {
    const chips = el('div', { class: 'chips' });
    const cats = ['全部', '力学', '电磁', '振动'];
    const cur = leftChip === 'all' ? '全部' : leftChip;
    cats.forEach(c => {
      const id = c === '全部' ? 'all' : c;
      const b = el('button', { type: 'button', class: 'chip' + (cur === c || (c === '全部' && leftChip === 'all') ? ' active' : '') }, c);
      b.onclick = () => { leftChip = id; refreshLeft(); };
      chips.appendChild(b);
    });
    // fix active state
    chips.querySelectorAll('.chip').forEach(ch => {
      const lab = ch.textContent;
      const id = lab === '全部' ? 'all' : lab;
      ch.classList.toggle('active', leftChip === id);
    });
    host.appendChild(chips);
    const wrap = el('div', { class: 'side-body pad' });
    let any = false;
    for (const key of Object.keys(PRESETS)) {
      const meta = PRESET_META[key] || { cat: '其他', tags: [] };
      if (leftChip !== 'all' && meta.cat !== leftChip) continue;
      const name = PRESETS[key].name;
      const blob = [name, meta.cat, ...(meta.tags || [])].join(' ');
      if (!matchQuery(blob, q)) continue;
      any = true;
      const item = el('div', { class: 'preset-item' });
      item.appendChild(el('div', { class: 'p-name' }, name));
      item.appendChild(el('div', { class: 'p-cat' }, `${meta.cat}${(meta.tags || []).length ? ' · ' + meta.tags.join(' ') : ''}`));
      item.onclick = () => { loadPreset(key); closeMobilePanels(); };
      wrap.appendChild(item);
    }
    if (!any) wrap.appendChild(el('div', { class: 'empty-hint' }, '无匹配预设。'));
    host.appendChild(wrap);
  }
}

function buildList() { refreshLeft(); }

function syncPanelToggleButtons() {
  const main = $('#main');
  if (!main) return;
  $('#bLeft')?.classList.toggle('active', !main.classList.contains('noleft'));
  $('#bRight')?.classList.toggle('active', !main.classList.contains('noright'));
}

function closeMobilePanels(resetDock = true) {
  $('#left')?.classList.remove('open');
  $('#right')?.classList.remove('open');
  $('#panelScrim')?.classList.remove('show');
  if (resetDock) {
    mobileDock = 'stage';
    document.querySelectorAll('#mobile-dock button').forEach(b => b.classList.toggle('active', b.dataset.dock === 'stage'));
  }
}

function openMobile(which) {
  const left = $('#left'), right = $('#right'), scrim = $('#panelScrim');
  closeMobilePanels(false);
  if (which === 'objects' || which === 'add' || which === 'presets') {
    leftTab = which;
    leftChip = 'all';
    document.querySelectorAll('#leftTabs button').forEach(b => b.classList.toggle('active', b.dataset.leftTab === leftTab));
    refreshLeft();
    left.classList.add('open');
    scrim?.classList.add('show');
  } else if (which === 'inspect') {
    right.classList.add('open');
    scrim?.classList.add('show');
  }
}

/* ---------------- 实时读数 ---------------- */
function setTxt(id, t) { const e = document.getElementById(id); if (e) e.innerHTML = t; }
function updateReadouts() {
  updateStatus();
  const b = selected(); if (!isBody(b) || !b.st) return;
  const st = b.st, S = scene.settings, v = Math.hypot(st.vx, st.vy), Ek = 0.5 * b.m * v * v, Ep = S.gOn ? b.m * S.g * st.y : 0;
  setTxt('r_pos', `(${fmt(st.x)}, ${fmt(st.y)}) m`); setTxt('r_vel', `(${fmt(st.vx)}, ${fmt(st.vy)}) m/s`);
  setTxt('r_v', `${fmt(v)} m/s`); setTxt('r_ang', v > 1e-9 ? `${fmt(Math.atan2(st.vy, st.vx) / DEG, 2)}°` : '—');
  setTxt('r_a', sim.t > 0 ? `${fmt(Math.hypot(st.ax, st.ay))} m/s²` : '—');
  setTxt('r_ek', `${fmt(Ek)} J`); setTxt('r_p', `${fmt(b.m * v)} kg·m/s`); setTxt('r_ep', S.gOn ? `${fmt(Ep)} J（以 y=0 为零势面）` : '—'); setTxt('r_e', `${fmt(Ek + Ep)} J`);
  // Throttle MathQuill remount while running — force only when paused / tab switch
  if (propsTab === 'analysis') renderMotionAnalysis(b, !sim.running);
  if (propsTab === 'chart' && (!sim.running || (performance.now() / 200 | 0) % 2 === 0)) drawChart();
}

let analysisRefMode = 'initial'; // 'initial' | 'current'

function collectSpringsFor(b) {
  const out = [];
  for (const s of scene.objects) {
    if (s.kind !== 'spring') continue;
    if (s.a !== b.id && s.b !== b.id) continue;
    let anchor;
    let otherBody = false;
    if (s.a === b.id) {
      if (s.b != null) {
        const o = byId(s.b); if (!o || !o.st) continue;
        anchor = { x: o.st.x, y: o.st.y }; otherBody = true;
      } else anchor = { x: s.ax, y: s.ay };
    } else {
      const o = byId(s.a); if (!o || !o.st) continue;
      anchor = { x: o.st.x, y: o.st.y }; otherBody = true;
    }
    out.push({ k: s.k, L0: s.L0, rope: !!s.rope, anchor, otherBody });
  }
  return out;
}

function sceneModelField() {
  let Ex = 0, Ey = 0, Bz = 0, nE = 0, nB = 0;
  for (const o of scene.objects) {
    if (o.kind === 'efield') {
      Ex += o.E * Math.cos(o.ang * DEG);
      Ey += o.E * Math.sin(o.ang * DEG);
      nE++;
    } else if (o.kind === 'bfield') {
      Bz += o.B * o.dir;
      nB++;
    }
  }
  if (!nE && !nB) return null;
  const bits = [];
  if (nE) bits.push(nE === 1 ? '场景匀强 E' : `场景中 ${nE} 个电场叠加`);
  if (nB) bits.push(nB === 1 ? '场景匀强 B' : `场景中 ${nB} 个磁场叠加`);
  return {
    field: { Ex, Ey, Bz },
    note: `参考点处场为零；下列方程按${bits.join(' + ')}给出（粒子进入对应场区后适用）。`,
  };
}

function collectPointChargesForAnalysis(excludeId) {
  const out = [];
  for (const o of scene.objects) {
    if (o.id === excludeId) continue;
    if (o.kind === 'pointCharge' && o.q) {
      out.push({ id: o.id, x: o.x, y: o.y, q: o.q, fixed: true });
    } else if (
      scene.settings.coulomb &&
      o.q &&
      o.st &&
      (o.kind === 'particle' || o.kind === 'ball' || o.kind === 'box')
    ) {
      out.push({
        id: o.id,
        x: o.st.x,
        y: o.st.y,
        q: o.q,
        fixed: !!o.fixed,
      });
    }
  }
  return out;
}

function buildAnalysisContext(b) {
  const useInit = analysisRefMode === 'initial';
  const st = b.st;
  const x = useInit ? b.x0 : st.x;
  const y = useInit ? b.y0 : st.y;
  const vx = useInit ? b.vx0 : st.vx;
  const vy = useInit ? b.vy0 : st.vy;
  const field = fieldAtExcluding(x, y, b.id);
  const model = sceneModelField();
  let hasUniformE = false, hasUniformB = false;
  for (const o of scene.objects) {
    if (o.kind === 'efield') hasUniformE = true;
    if (o.kind === 'bfield') hasUniformB = true;
  }
  return {
    gOn: scene.settings.gOn,
    g: scene.settings.g,
    drag: scene.settings.drag || 0,
    refMode: analysisRefMode,
    body: {
      id: b.id, name: b.name, kind: b.kind, m: b.m, q: b.q,
      x, y, vx, vy, t: sim.t, fixed: !!b.fixed,
    },
    field,
    modelField: model ? model.field : undefined,
    modelFieldNote: model ? model.note : undefined,
    Fext: (() => { const af = appliedForceOn(b); return { x: af.fx, y: af.fy }; })(),
    springs: collectSpringsFor(b),
    pointCharges: collectPointChargesForAnalysis(b.id),
    coulombLaw: scene.settings.coulombLaw || 'coulomb3d',
    kCoulomb: scene.settings.kc || K_COULOMB,
    hasUniformE,
    hasUniformB,
  };
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, c => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
  })[c]);
}

function attrLatex(s) {
  return escapeHtml(String(s ?? '')).replace(/\n/g, ' ');
}

function eqBlocksHtml(blocks, editableNumeric = false) {
  const numMode = editableNumeric ? 'edit' : 'static';
  return blocks.map(eq => `
    <div class="eqblock">
      ${eq.note ? `<div class="eq-note">${escapeHtml(eq.note)}</div>` : ''}
      <div class="eq-row eq-sym-row">
        <span class="eq-tag">符号</span>
        <div class="eq-sym" data-mq="static" data-latex="${attrLatex(eq.symbolic)}"></div>
      </div>
      <div class="eq-row eq-num-row">
        <span class="eq-tag">数值</span>
        <div class="eq-num" data-mq="${numMode}" data-latex="${attrLatex(eq.numeric)}" title="${editableNumeric ? '可编辑 · 复制为 LaTeX' : '暂停后可编辑数值式'}"></div>
      </div>
    </div>`).join('');
}

let lastAnalysisTs = 0;
let analysisScratchLatex = '';
function renderMotionAnalysis(b, force = false) {
  const host = document.getElementById('r_theory');
  if (!host || !b || !b.st) return;
  const now = performance.now();
  // Remounting MathQuill is costly — keep live updates sparse while playing
  if (!force && sim.running && now - lastAnalysisTs < 500) return;
  if (!force && !sim.running && now - lastAnalysisTs < 80) return;
  lastAnalysisTs = now;
  // Preserve scratch pad edits across live refreshes
  const prevScratch = host.querySelector('.an-scratch [data-mq="edit"]');
  if (prevScratch && prevScratch.mq && typeof prevScratch.mq.latex === 'function') {
    try { analysisScratchLatex = prevScratch.mq.latex(); } catch { /* keep */ }
  }
  const report = analyzeMotion(buildAnalysisContext(b));
  const editableNumeric = !sim.running;
  const derHtml = report.derived.map(d => `
    <div class="der-row">
      <span class="k">${escapeHtml(d.name)}</span>
      <span class="der-math">
        <span class="der-formula" data-mq="static" data-latex="${attrLatex(d.formula)}"></span>
        <span class="der-eq">=</span>
        <span class="der-value" data-mq="static" data-latex="${attrLatex(d.value)}"></span>
      </span>
    </div>`).join('');
  host.innerHTML = `
    <div class="an-head">
      <div class="an-title">${escapeHtml(report.title)}</div>
      <div class="an-summary">${escapeHtml(report.summary)}</div>
      <div class="an-regime">识别：${escapeHtml(report.regimeId)} · MathQuill</div>
    </div>
    <div class="an-ref btnrow">
      <span class="lab" style="width:auto;margin-right:4px">方程参考点</span>
      <button type="button" class="${analysisRefMode === 'initial' ? 'active' : ''}" data-ref="initial">初始条件</button>
      <button type="button" class="${analysisRefMode === 'current' ? 'active' : ''}" data-ref="current">当前状态</button>
      <button type="button" id="btnCopyEq" title="复制 LaTeX">复制 LaTeX</button>
    </div>
    <div class="an-sec-label">轨迹参数方程</div>
    ${eqBlocksHtml(report.parametric, editableNumeric)}
    ${report.extras.length ? `<div class="an-sec-label">补充关系</div>${eqBlocksHtml(report.extras, editableNumeric)}` : ''}
    <div class="an-sec-label">特征量（随选中物体与场状态更新）</div>
    <div class="der-grid">${derHtml || '<span class="k">—</span>'}</div>
    <div class="an-sec-label">演算草稿</div>
    <div class="an-scratch eqblock">
      <div data-mq="edit" data-latex="${attrLatex(analysisScratchLatex || 'E=')}" title="点击后用下方数学键盘输入"></div>
    </div>
    <div class="an-sec-label">数学键盘</div>
    <div id="mathKeyboard" class="math-kb-host"></div>
    <div class="an-meta">
      <div><b>计入的力：</b>${report.forcesInModel.map(escapeHtml).join('、') || '无'}</div>
      <div><b>假定：</b>${report.assumptions.map(escapeHtml).join('； ')}</div>
      <div class="hint">${escapeHtml(report.validity)}</div>
    </div>`;
  mountMathQuill(host);
  const kbHost = host.querySelector('#mathKeyboard');
  if (kbHost) mountMathKeyboard(kbHost);
  host.querySelectorAll('[data-ref]').forEach(btn => {
    btn.addEventListener('click', () => {
      analysisRefMode = btn.getAttribute('data-ref');
      renderMotionAnalysis(b, true);
      draw();
    });
  });
  const copyBtn = host.querySelector('#btnCopyEq');
  if (copyBtn) {
    copyBtn.addEventListener('click', async () => {
      const text = [
        `% ${report.title}`,
        `% ${report.summary}`,
        '',
        '% parametric',
        ...report.parametric.flatMap(e => [e.symbolic, e.numeric, '']),
        ...(report.extras.length ? ['% extras', ...report.extras.flatMap(e => [e.symbolic, e.numeric, ''])] : []),
        '% derived',
        ...report.derived.map(d => `${d.formula} = ${d.value}  % ${d.name}`),
      ].join('\n');
      try {
        await navigator.clipboard.writeText(text);
        toast('已复制 LaTeX');
      } catch {
        toast('复制失败');
      }
    });
  }
}

function drawChart() {
  const c = document.getElementById('chart'), b = selected(); if (!c || !isBody(b) || !b.st) return;
  const r = c.getBoundingClientRect(); if (!r.width) return;
  const d = window.devicePixelRatio || 1; c.width = r.width * d; c.height = r.height * d;
  const g = c.getContext('2d'); g.setTransform(d, 0, 0, d, 0, 0); const w = r.width, h = r.height;
  const hist = b.st.hist; const idx = { v: 5, vx: 3, vy: 4, x: 1, y: 2, ek: 6 };
  const pts = chartMode === 'xy' ? hist.map(p => [p[1], p[2]]) : hist.map(p => [p[0], p[idx[chartMode]]]);
  g.fillStyle = C['--muted']; g.font = '10px sans-serif';
  if (pts.length < 2) { g.fillText('播放后显示图像', 10, h / 2); return; }
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  pts.forEach(p => { x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]); y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]); });
  if (chartMode !== 'xy') { y0 = Math.min(y0, 0); y1 = Math.max(y1, 0); }
  if (y1 - y0 < 1e-12) { y1 += 1; y0 -= 1; } if (x1 - x0 < 1e-12) x1 = x0 + 1;
  const L = 38, R = 8, T = 8, B = 18, X = v => L + (v - x0) / (x1 - x0) * (w - L - R), Y = v => h - B - (v - y0) / (y1 - y0) * (h - T - B);
  g.strokeStyle = C['--border']; g.lineWidth = 1; g.beginPath(); g.moveTo(L, T); g.lineTo(L, h - B); g.lineTo(w - R, h - B); g.stroke();
  if (y0 < 0 && y1 > 0) { g.setLineDash([3, 3]); g.beginPath(); g.moveTo(L, Y(0)); g.lineTo(w - R, Y(0)); g.stroke(); g.setLineDash([]); }
  g.textAlign = 'right'; g.textBaseline = 'middle'; g.fillText(fmt(y1, 2), L - 3, T + 4); g.fillText(fmt(y0, 2), L - 3, h - B - 4);
  g.textAlign = 'center'; g.textBaseline = 'top'; g.fillText(fmt(x0, 2), L, h - B + 3); g.fillText(fmt(x1, 2) + (chartMode === 'xy' ? ' m' : ' s'), w - R - 12, h - B + 3);
  g.strokeStyle = b.color; g.lineWidth = 1.6; g.beginPath(); pts.forEach((p, i) => i ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1]))); g.stroke();
}
let lastStatusTs = 0;
function updateStatus(force = false) {
  const now = performance.now();
  if (!force && sim.running && now - lastStatusTs < 66) {
    updateStatusMouse();
    return;
  }
  lastStatusTs = now;
  const b = selected(), parts = [`<span><span class="k">t</span>${fmt(sim.t, 3)} s</span>`];
  if (isBody(b) && b.st) {
    const st = b.st, v = Math.hypot(st.vx, st.vy);
    parts.push('<span class="sep"></span>', `<span><b>${b.name}</b></span>`,
      `<span><span class="k">x</span>${fmt(st.x)} m</span>`, `<span><span class="k">y</span>${fmt(st.y)} m</span>`,
      `<span><span class="k">vx</span>${fmt(st.vx)}</span>`, `<span><span class="k">vy</span>${fmt(st.vy)} m/s</span>`,
      `<span><span class="k">|v|</span>${fmt(v)} m/s</span>`, `<span><span class="k">Ek</span>${fmt(0.5 * b.m * v * v)} J</span>`,
      `<span><span class="k">p</span>${fmt(b.m * v)} kg·m/s</span>`);
  } else parts.push('<span class="sep"></span>', '<span class="k">未选中</span>');
  parts.push('<span class="sep"></span>', `<span id="mousepos"></span>`);
  $('#status').innerHTML = parts.join(''); updateStatusMouse();
}
function updateStatusMouse() { const m = document.getElementById('mousepos'); if (m) m.innerHTML = `<span class="k">鼠标</span>(${fmt(mouseW[0], 2)}, ${fmt(mouseW[1], 2)}) m <span class="k" style="margin-left:8px">缩放</span>${fmt(scene.view.scale, 3)} px/m`; }

/* ---------------- 提示 ---------------- */
let toastT = 0;
function toast(msg) { const t = $('#toast'); t.textContent = msg; t.classList.add('show'); clearTimeout(toastT); toastT = setTimeout(() => t.classList.remove('show'), 2200); }

/* =====================================================================
 *  预设场景
 * ===================================================================== */
function sceneOf(objects, settings = {}, view = null) { return { settings: Object.assign(defaultSettings(), settings), objects, view }; }
const PRESET_META = {
  welcome: { cat: '力学', tags: ['斜抛', '摩擦'] },
  projectile: { cat: '力学', tags: ['平抛'] },
  vertical: { cat: '力学', tags: ['竖直上抛'] },
  incline: { cat: '力学', tags: ['斜面', '摩擦'] },
  collision: { cat: '力学', tags: ['碰撞'] },
  selector: { cat: '电磁', tags: ['速度选择器'] },
  edeflect: { cat: '电磁', tags: ['电场偏转'] },
  bbound: { cat: '电磁', tags: ['有界磁场'] },
  accel: { cat: '电磁', tags: ['加速', '偏转'] },
  cycloid: { cat: '电磁', tags: ['摆线'] },
  springosc: { cat: '振动', tags: ['弹簧'] },
  pendulum: { cat: '振动', tags: ['单摆'] },
  coulomb: { cat: '电磁', tags: ['库仑'] },
  dipole: { cat: '电磁', tags: ['偶极', '场线'] },
};
const PRESETS = {
  welcome: { name: '斜抛与滑块', make() {
    nextId = 1;
    return sceneOf([mkGround(0), mkBody('ball', { name: '小球', x0: -4, y0: 0.25, vx0: 6, vy0: 8, e: 0.5 }),
      mkBody('box', { name: '滑块', x0: -6, y0: 0.2, vx0: 5, vy0: 0, mu: 0.3, color: '#f59e0b' })], {}, { cx: 3, cy: 2.6, scale: 42 });
  } },
  projectile: { name: '平抛运动', make() {
    nextId = 1;
    return sceneOf([mkGround(0), mkWall(-3, 4.85, 0, 4.85, { name: '平台', smooth: true }),
      mkBody('ball', { name: '小球', x0: 0, y0: 5, r: 0.15, vx0: 3, vy0: 0, e: 0.5 })]);
  } },
  vertical: { name: '竖直上抛', make() {
    nextId = 1;
    return sceneOf([mkGround(0), mkBody('ball', { name: '小球', x0: 0, y0: 0.2, r: 0.2, vx0: 0, vy0: 10, e: 0.6 })], {}, null);
  } },
  incline: { name: '斜面滑块', make() {
    nextId = 1;
    const inc = mkIncline(-2, 0, { L: 4, ang: 37, dir: 1, smooth: false });
    const th = 37 * DEG, top = inclinePts(inc)[2], h = 0.4, along = 0.6;
    const b = mkBody('box', { name: '滑块', w: 0.6, h, m: 1, mu: 0.5, e: 0, ang0: 37,
      x0: +(top[0] - Math.sin(th) * h / 2 - Math.cos(th) * along).toFixed(3), y0: +(top[1] + Math.cos(th) * h / 2 - Math.sin(th) * along).toFixed(3) });
    return sceneOf([mkGround(0), inc, b], { showForce: true, g: 9.8 });
  } },
  collision: { name: '弹性碰撞', make() {
    nextId = 1;
    return sceneOf([mkGround(0, { smooth: true }),
      mkBody('ball', { name: 'A (1kg)', m: 1, r: 0.3, x0: -3, y0: 0.3, vx0: 4, e: 1, color: '#2f6fed' }),
      mkBody('ball', { name: 'B (3kg)', m: 3, r: 0.3, x0: 0, y0: 0.3, vx0: 0, e: 1, color: '#e5484d' })]);
  } },
  selector: { name: '速度选择器', make() {
    nextId = 1;
    return sceneOf([
      mkE({ name: '电场 E↓', E: 5, ang: -90, x: 0, y: -2, w: 10, h: 4 }), mkB({ name: '磁场 ⊗', B: 1, dir: -1, x: 0, y: -2, w: 10, h: 4 }),
      mkWall(0, 2, 10, 2, { name: '上极板(−)' }), mkWall(0, -2, 10, -2, { name: '下极板(+)' }),
      mkBody('particle', { name: 'v=5 (=E/B)', x0: -2, y0: 0, vx0: 5, color: '#1f9d55' }),
      mkBody('particle', { name: 'v=8', x0: -2, y0: 0, vx0: 8, color: '#e5484d' }),
      mkBody('particle', { name: 'v=3', x0: -2, y0: 0, vx0: 3, color: '#2f6fed' })], { gOn: false, showLabels: false });
  } },
  edeflect: { name: '电场偏转', make() {
    nextId = 1;
    return sceneOf([mkE({ name: '偏转电场', E: 1, ang: -90, x: 0, y: -1, w: 4, h: 2 }),
      mkWall(0, 1, 4, 1, { name: '上极板' }), mkWall(0, -1, 4, -1, { name: '下极板' }), mkWall(8, -3, 8, 3, { name: '荧光屏', e: 0 }),
      mkBody('particle', { name: '粒子', x0: -1, y0: 0, vx0: 4, m: 0.01, q: 0.01 })], { gOn: false });
  } },
  bbound: { name: '有界磁场', make() {
    nextId = 1;
    return sceneOf([mkB({ name: '磁场 ⊗', B: 1, dir: -1, x: 0, y: -3, w: 6, h: 9 }),
      mkBody('particle', { name: '粒子 q/m=1', m: 1, q: 1, r: 0.1, x0: -2, y0: 0, vx0: 2 }),
      mkBody('particle', { name: '斜射入', m: 1, q: 1, r: 0.1, x0: -2, y0: -2, vx0: 1.732, vy0: 1, color: '#8b5cf6' })], { gOn: false });
  } },
  accel: { name: '加速后偏转', make() {
    nextId = 1;
    return sceneOf([mkE({ name: '加速电场', E: 2, ang: 0, x: -4, y: -1, w: 3, h: 2 }),
      mkB({ name: '偏转磁场 ⊗', B: 1, dir: -1, x: 0, y: -1, w: 10, h: 9 }),
      mkBody('particle', { name: '粒子 q/m=1', m: 1, q: 1, r: 0.1, x0: -3.9, y0: 0 })], { gOn: false });
  } },
  cycloid: { name: '正交电磁场摆线', make() {
    nextId = 1;
    return sceneOf([mkE({ name: '电场 E↓', E: 2, ang: -90, all: true }), mkB({ name: '磁场 ⊗', B: 1, dir: -1, all: true }),
      mkBody('particle', { name: '粒子 q/m=1', m: 1, q: 1, r: 0.1, x0: 0, y0: 0 })], { gOn: false }, { cx: 8, cy: -2, scale: 45 });
  } },
  springosc: { name: '弹簧振子', make() {
    nextId = 1;
    const b = mkBody('box', { name: '振子', w: 0.5, h: 0.5, m: 1, mu: 0, x0: 1, y0: 0.25 });
    return sceneOf([mkGround(0, { smooth: true }), mkWall(-3, 0, -3, 1.5, { name: '墙' }), b,
      mkSpring(b.id, null, { ax: -3, ay: 0.25, k: 20, L0: 3 })]);
  } },
  pendulum: { name: '单摆', make() {
    nextId = 1;
    const b = mkBody('ball', { name: '摆球', m: 0.5, r: 0.15, x0: 1, y0: 2.268 });
    return sceneOf([b, mkSpring(b.id, null, { name: '轻绳', ax: 0, ay: 4, L0: 2, k: 5000, rope: true })], {}, { cx: 0, cy: 2.5, scale: 90 });
  } },
  dipole: { name: '电偶极子场线', make() {
    nextId = 1;
    return sceneOf([
      mkPointCharge(-1, 0, { name: '+q', q: 1e-6, color: '#e5484d' }),
      mkPointCharge(1, 0, { name: '−q', q: -1e-6, color: '#2f6fed' }),
      mkBody('particle', { name: '试探电荷', m: 0.01, q: 1e-8, r: 0.06, x0: 0, y0: 1.2, vx0: 0.8 }),
    ], { gOn: false, showFieldLines: true, coulomb: false }, { cx: 0, cy: 0, scale: 80 });
  } },
  coulomb: { name: '库仑绕核', make() {
    nextId = 1;
    // v = √(k|qQ|/(m r)) ≈ 2.120 with k=8.99e9, |qQ|=1e-9, m=1, r=2
    return sceneOf([
      mkBody('ball', { name: '+Q 固定', fixed: true, q: 1e-4, m: 1, r: 0.2, x0: 0, y0: 0, color: '#e5484d' }),
      mkBody('ball', { name: '−q', q: -1e-5, m: 1, r: 0.1, x0: 2, y0: 0, vx0: 0, vy0: 2.12, color: '#2f6fed' }),
    ], { gOn: false, coulomb: true, showFieldLines: true, showAnalytic: true }, { cx: 0, cy: 0, scale: 80 });
  } },
};
function loadPreset(key) {
  const p = PRESETS[key]; if (!p) return;
  pushUndo();
  const sc = p.make(); sc.objects.forEach(o => { if (isBody(o)) o.st = null; });
  loadScene(sc, false);
  if (!sc.view) fitView();
  toast(`已载入预设：${p.name}，按 ▶ 播放`);
}

/* =====================================================================
 *  保存 / 载入
 * ===================================================================== */
function serialize() {
  return JSON.stringify({ app: 'physim', version: 1, settings: scene.settings, view: scene.view,
    objects: scene.objects.map(o => { const c = Object.assign({}, o); delete c.st; return c; }) });
}
function loadScene(data, keepView) {
  if (!data || !Array.isArray(data.objects)) throw new Error('无效的场景文件');
  const oldView = scene.view;
  scene = {
    settings: Object.assign(defaultSettings(), data.settings || {}),
    objects: data.objects.map(o => {
      const c = Object.assign({}, o, { st: null });
      if (isBody(c)) ensureForceFields(c);
      return c;
    }),
    view: keepView ? oldView : Object.assign({ cx: 0, cy: 2, scale: 60 }, data.view || {}),
  };
  nextId = scene.objects.reduce((m, o) => Math.max(m, o.id || 0), 0) + 1;
  if (selId != null && !byId(selId)) selId = null;
  if (!keepView) selId = null;
  resetSim(); markFieldsDirty(); syncToolbar(); buildList(); buildPanel(); draw(); markDirty();
}
let saveT = 0;
function markDirty() { clearTimeout(saveT); saveT = setTimeout(() => { try { localStorage.setItem(STORE_KEY, serialize()); } catch (e) { } }, 400); }

/* =====================================================================
 *  工具栏绑定
 * ===================================================================== */
function syncToolbar() {
  const S = scene.settings;
  if ($('#chkG')) $('#chkG').checked = S.gOn;
  if ($('#numG')) $('#numG').value = S.g;
  if ($('#chkGrid')) $('#chkGrid').checked = S.showGrid;
  if ($('#chkTrail')) $('#chkTrail').checked = S.showTrail;
  if ($('#chkVel')) $('#chkVel').checked = S.showVel;
  if ($('#chkForce')) $('#chkForce').checked = S.showForce;
  if ($('#chkAnalytic')) $('#chkAnalytic').checked = !!S.showAnalytic;
  if ($('#chkFieldLines')) $('#chkFieldLines').checked = !!S.showFieldLines;
  const sel = $('#selSpeed');
  if (sel) {
    if (![...sel.options].some(o => +o.value === S.speed)) sel.appendChild(el('option', { value: S.speed }, S.speed + '×'));
    sel.value = String(S.speed);
  }
}

function initToolbar() {
  document.querySelectorAll('[data-tool]').forEach(b => b.addEventListener('click', () => setTool(b.dataset.tool)));
  $('#bPlay').onclick = togglePlay;
  $('#bStep').onclick = stepOnce;
  $('#bReset').onclick = () => { resetSim('已重置到 t=0'); draw(); updateReadouts(); };
  $('#selSpeed').onchange = e => { scene.settings.speed = +e.target.value; markDirty(); draw(); };
  $('#chkG').onchange = e => { scene.settings.gOn = e.target.checked; liveEdit(); };
  $('#numG').oninput = e => { const v = parseFloat(e.target.value); if (isFinite(v)) { scene.settings.g = v; liveEdit(); } };
  const flag = (id, k) => { const node = $(id); if (node) node.onchange = e => { scene.settings[k] = e.target.checked; markDirty(); draw(); }; };
  flag('#chkGrid', 'showGrid'); flag('#chkTrail', 'showTrail'); flag('#chkVel', 'showVel'); flag('#chkForce', 'showForce');
  flag('#chkAnalytic', 'showAnalytic');
  if ($('#chkFieldLines')) {
    $('#chkFieldLines').onchange = e => { scene.settings.showFieldLines = e.target.checked; markFieldsDirty(); markDirty(); draw(); };
  }

  // left tabs
  document.querySelectorAll('#leftTabs [data-left-tab]').forEach(btn => {
    btn.addEventListener('click', () => {
      leftTab = btn.dataset.leftTab;
      leftChip = 'all';
      document.querySelectorAll('#leftTabs button').forEach(b => b.classList.toggle('active', b === btn));
      refreshLeft();
    });
  });
  const q = $('#qLeft');
  if (q) {
    q.addEventListener('input', () => { leftQuery = q.value; refreshLeft(); });
  }
  document.addEventListener('keydown', e => {
    if (e.key === '/' && !['INPUT', 'TEXTAREA', 'SELECT'].includes((document.activeElement || {}).tagName)) {
      e.preventDefault();
      if (window.matchMedia('(max-width: 820px)').matches) openMobile('objects');
      $('#qLeft')?.focus();
    }
  });

  // more menu
  const moreBtn = $('#bMore'), moreMenu = $('#moreMenu');
  moreBtn?.addEventListener('click', e => {
    e.preventDefault();
    e.stopPropagation();
    moreMenu?.classList.toggle('open');
  });
  document.addEventListener('click', e => {
    if (moreMenu?.classList.contains('open') && !moreMenu.contains(e.target) && e.target !== moreBtn) {
      moreMenu.classList.remove('open');
    }
  });
  moreMenu?.addEventListener('click', e => e.stopPropagation());

  $('#bUndo').onclick = undo;
  $('#bSave').onclick = () => {
    const blob = new Blob([serialize()], { type: 'application/json' });
    const a = el('a', { href: URL.createObjectURL(blob), download: `物理场景_${new Date().toISOString().slice(0, 10)}.json` });
    document.body.appendChild(a); a.click(); a.remove();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
    toast('已导出');
    moreMenu?.classList.remove('open');
  };
  $('#bLoad').onclick = () => { $('#fileIn').click(); moreMenu?.classList.remove('open'); };
  $('#fileIn').onchange = e => {
    const f = e.target.files[0]; if (!f) return;
    const rd = new FileReader();
    rd.onload = () => { try { pushUndo(); loadScene(JSON.parse(rd.result), false); toast('已导入'); } catch (err) { toast('导入失败：' + err.message); } };
    rd.readAsText(f); e.target.value = '';
  };
  $('#bClear').onclick = () => {
    if (!confirm('确定清空场景？')) return;
    pushUndo();
    const s = scene.settings;
    loadScene({ objects: [], settings: s, view: scene.view }, true);
    toast('已清空');
    moreMenu?.classList.remove('open');
  };
  $('#bTheme').onclick = () => {
    document.body.classList.toggle('dark');
    try { localStorage.setItem('physim.theme', document.body.classList.contains('dark') ? 'dark' : 'light'); } catch (e) {}
    readColors(); draw(); drawChart();
    moreMenu?.classList.remove('open');
  };
  $('#bHelp').onclick = () => { $('#help').classList.add('show'); moreMenu?.classList.remove('open'); };
  $('#bHelpClose')?.addEventListener('click', () => $('#help').classList.remove('show'));
  $('#help').addEventListener('click', e => { if (e.target.id === 'help') e.target.classList.remove('show'); });

  $('#bLeft')?.addEventListener('click', () => {
    const main = $('#main');
    main.classList.toggle('noleft');
    syncPanelToggleButtons();
    toast(main.classList.contains('noleft') ? '已隐藏对象栏' : '已显示对象栏');
    requestAnimationFrame(() => { resize(); requestDraw(); });
  });
  $('#bRight')?.addEventListener('click', () => {
    const main = $('#main');
    main.classList.toggle('noright');
    syncPanelToggleButtons();
    toast(main.classList.contains('noright') ? '已隐藏检视栏' : '已显示检视栏');
    requestAnimationFrame(() => { resize(); requestDraw(); });
  });
  document.addEventListener('keydown', e => {
    if (e.key === 'Escape') {
      moreMenu?.classList.remove('open');
      closeMobilePanels();
    }
  });

  // mobile dock
  document.querySelectorAll('#mobile-dock [data-dock]').forEach(btn => {
    btn.addEventListener('click', () => {
      const dock = btn.dataset.dock;
      mobileDock = dock;
      document.querySelectorAll('#mobile-dock button').forEach(b => b.classList.toggle('active', b === btn));
      if (dock === 'stage') closeMobilePanels();
      else if (dock === 'objects') openMobile('objects');
      else if (dock === 'add') openMobile('add');
      else if (dock === 'inspect') openMobile('inspect');
    });
  });
  $('#panelScrim')?.addEventListener('click', () => closeMobilePanels());

  document.addEventListener('keydown', e => { if (e.key === ' ' && e.target === document.body) e.preventDefault(); });
  refreshLeft();
}

/* =====================================================================
 *  启动（仅由 main.ts 调用一次；禁止在模块底再自动 boot）
 * ===================================================================== */
function bootOnce() {
  try { if (localStorage.getItem('physim.theme') === 'dark') document.body.classList.add('dark'); } catch (e) { }
  readColors(); initToolbar();
  new ResizeObserver(resize).observe($('#stage'));
  resize();
  let loaded = false;
  try {
    const s = localStorage.getItem(STORE_KEY);
    if (s) { loadScene(JSON.parse(s), false); loaded = true; }
  } catch (e) { loaded = false; }
  if (!loaded) { const sc = PRESETS.welcome.make(); loadScene(sc, false); }
  setTool('select');
  syncPanelToggleButtons();
  toast(loaded ? '已恢复上次场景' : '选中物体后拖动绿色箭头设置初速度，空格播放');
  window.__physim = {
    scene: () => scene, sim, advance, loadPreset, select, resetSim, togglePlay, fieldSys,
    async enableRapier() {
      contactWorld = await createRapierContactWorld(0);
      scene.settings.contactBackend = 'rapier2d';
      toast('Rapier 接触后端已加载（试验）');
      return contactWorld.backend;
    },
  };
  requestAnimationFrame(loop);
}

let booted = false;
export function boot() {
  if (booted) return;
  booted = true;
  bootOnce();
}
