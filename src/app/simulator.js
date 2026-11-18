import { analyzeMotion, sampleAnalyticPath } from '../analysis/motionAnalysis.ts';
import { mountMathQuill } from '../ui/mathquill.ts';
import { syncDesmosKeypadForAnalysis, ensureDesmosKeypadDock } from '../ui/mathKeyboard.ts';
import {
  FieldSystem2,
  sourcesFromSceneObjects,
  StubContactWorld,
  createRapierContactWorld,
  evalForceExpr,
  FORCE_EXPR_HELP,
  rewriteExpr,
  orderExprNodes,
} from '../engine/index.ts';


/* =====================================================================
 *  物理运动模拟器 —— 单文件版
 *  坐标：世界坐标单位 m，y 轴向上；角度以 +x 方向为 0°，逆时针为正
 *  积分：每帧拆成若干子步，速度更新使用 Boris 推进（磁场中能量严格守恒），
 *        接触/碰撞使用冲量法（含恢复系数、动/静摩擦）
 * ===================================================================== */
const $ = s => document.querySelector(s);
const UI_FONT = '-apple-system,BlinkMacSystemFont,"Segoe UI","PingFang SC","Microsoft YaHei",sans-serif';
const MONO_FONT = '"JetBrains Mono",ui-monospace,Menlo,Consolas,monospace';
const DEG = Math.PI / 180;
const K_COULOMB = 8.99e9;
const STORE_KEY = 'physim.scene.v1';

/* ---------------- 场景数据 ---------------- */
let nextId = 1;
const BODY_KINDS = ['ball', 'particle', 'box'];
const KIND_NAME = { ball: '小球', particle: '带电粒子', box: '滑块', ground: '地面', wall: '挡板', incline: '斜面',
  efield: '电场', bfield: '磁场', spring: '弹簧', rope: '轻绳', rod: '轻杆', pointCharge: '点电荷' };
const LINK_KINDS = ['spring', 'rope', 'rod'];
function isLink(o) { return o && LINK_KINDS.includes(o.kind); }
const PALETTE = ['#d97757', '#4a78b0', '#5e8c4a', '#c2973e', '#8a6bbe', '#3d8f86', '#c46a7a', '#6b6a64'];

function defaultSettings() {
  return { gOn: true, g: 9.8, drag: 0, coulomb: false, kc: K_COULOMB, maxDt: 0.001, vPx: 20, FPx: 12, snap: true, snapStep: 0.1,
    showLabels: true, showGrid: true, showTrail: true, showVel: true, showForce: false, showAnalytic: true, showFieldLines: true, coulombLaw: 'coulomb3d', contactBackend: 'native', speed: 1,
    /* 物体间万有引力（教学用缩放常量） */
    gravity: false, Gc: 1, gravSoft: 0.05 };
}
let scene = { settings: defaultSettings(), objects: [], view: { cx: 0, cy: 2, scale: 60 } };

function isBody(o) { return o && BODY_KINDS.includes(o.kind); }
function bodies() { return scene.objects.filter(isBody); }
function byId(id) { return scene.objects.find(o => o.id === id); }

function mkBody(kind, p = {}) {
  const n = bodies().length;
  const base = { id: nextId++, kind, name: p.name || uniqueName(KIND_NAME[kind] + (n + 1)), color: PALETTE[n % PALETTE.length],
    m: 1, q: 0, r: 0.25, w: 0.6, h: 0.4, e: 0.6, mu: 0, x0: 0, y0: 0, vx0: 0, vy0: 0, ang0: 0,
    /* 外力：const 恒定 | expr 表达式 | aim 指向目标 */
    F: 0, Fang: 0, Fmode: 'const', Fexpr: '', FangExpr: '0',
    FaimId: null, FaimX: 0, FaimY: 0, _Ferr: '',
    fixed: false, trail: true, collide: true };
  if (kind === 'particle') Object.assign(base, { m: 0.01, q: 0.01, r: 0.08, color: '#c8553d', collide: false, e: 1 });
  if (kind === 'box') Object.assign(base, { mu: 0.3, e: 0.2, color: '#d08a3c', m: 1 });
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
    id: nextId++, kind: 'pointCharge', name: '点电荷', x, y, q: 1e-6, softRadius: 0.08, fixed: true, color: '#c8553d',
  }, p);
}
function mkSpring(a, b, p = {}) { return Object.assign({ id: nextId++, kind: 'spring', name: '弹簧', a, b, ax: 0, ay: 0, k: 20, L0: 1, m: 0 }, p); }
function mkRope(a, b, p = {}) { return Object.assign({ id: nextId++, kind: 'rope', name: '轻绳', a, b, ax: 0, ay: 0, k: 5000, L0: 1, m: 0 }, p); }
/** 轻杆：不可伸缩的刚性杆（长度约束），质量默认 0 */
function mkRod(a, b, p = {}) { return Object.assign({ id: nextId++, kind: 'rod', name: '轻杆', a, b, ax: 0, ay: 0, L0: 1, m: 0 }, p); }

/* ---------------- 仿真状态 ---------------- */
const sim = { t: 0, running: false, accum: 0,
  /* 功的账本：匀强电场做功、外力与阻力做功；能量历史 [t, Ek, Ep, E, E−W] */
  wE: 0, wExt: 0, eHist: [], E0: null, eDrop: 0,
  /* 时间轴：每帧一个快照，可在暂停时拖回任意时刻 */
  frames: [], frameIdx: 0 };
const MAX_FRAMES = 6000;
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
  resolveExprs();
  bodies().forEach(initState);
  resolveExprs();
  resetEnergyLedger();
  sim.frames = []; snapshotFrame();
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
function linkEnds(s) {
  const A = byId(s.a); if (!A || !A.st) return null;
  const B = s.b != null ? byId(s.b) : null;
  if (s.b != null && (!B || !B.st)) return null;
  return { A, B, x1: A.st.x, y1: A.st.y, x2: B ? B.st.x : s.ax, y2: B ? B.st.y : s.ay };
}
const springEnds = linkEnds;

/** 连接件自身质量的重力：均匀分布到两端（固定端不吃力）。 */
function applyLinkWeight(out, e, m, g, record) {
  if (!(m > 0) || !g) return;
  const w = m * g;
  const shareA = e.A && !e.A.fixed;
  const shareB = e.B && !e.B.fixed;
  const n = (shareA ? 1 : 0) + (shareB ? 1 : 0);
  if (!n) return;
  const each = w / n;
  const addW = (body, fy) => {
    const f = out.get(body); if (!f) return;
    f.y += fy;
    if (record && fy) f.parts.push({ fx: 0, fy, label: 'G连', color: '#a8a398' });
  };
  if (shareA) addW(e.A, -each);
  if (shareB) addW(e.B, -each);
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

const EXPR_DERIVED = {
  v: ['vx', 'vy'],
  v0: ['vx0', 'vy0'],
  Fx: ['F', 'Fang'],
  Fy: ['F', 'Fang'],
};

function exprNames() {
  return scene.objects.map(o => ({ id: o.id, name: o.name || '' }));
}
function slotIdOf(owner, key) {
  return owner === scene.settings ? `set:${key}` : `${owner.id}:${key}`;
}
function clearExpr(owner, keys) {
  if (!owner || !owner.expr) return;
  for (const k of keys) delete owner.expr[k];
}
function readQuantity(id, field) {
  if (field === 't') return sim.t;
  if (field === 'g') return scene.settings.gOn ? scene.settings.g : 0;
  const o = byId(id);
  if (!o) return NaN;
  const st = o.st;
  switch (field) {
    case 'x': return st ? st.x : (o.x0 ?? o.x ?? 0);
    case 'y': return st ? st.y : (o.y0 ?? o.y ?? 0);
    case 'vx': return st ? (st.vx || 0) : (o.vx0 || 0);
    case 'vy': return st ? (st.vy || 0) : (o.vy0 || 0);
    case 'v': return Math.hypot(readQuantity(id, 'vx'), readQuantity(id, 'vy'));
    case 'ax': return isBody(o) ? (st ? (st.ax || 0) : 0) : (o.ax || 0);
    case 'ay': return isBody(o) ? (st ? (st.ay || 0) : 0) : (o.ay || 0);
    case 'x0': return o.x0 ?? o.x ?? 0;
    case 'y0': return o.y0 ?? o.y ?? 0;
    case 'vx0': return o.vx0 || 0;
    case 'vy0': return o.vy0 || 0;
    case 'v0': return Math.hypot(o.vx0 || 0, o.vy0 || 0);
    case 'ang0': return o.ang0 || 0;
    case 'F': return o.F || 0;
    case 'Fang': return o.Fang || 0;
    case 'Fx': return (o.F || 0) * Math.cos((o.Fang || 0) * DEG);
    case 'Fy': return (o.F || 0) * Math.sin((o.Fang || 0) * DEG);
    case 'm': return o.m || 0;
    case 'q': return o.q || 0;
    case 'r': return o.r || 0;
    case 'w': return o.w || 0;
    case 'h': return o.h || 0;
    case 'mu': return o.mu || 0;
    case 'e': return o.e ?? 0;
    case 'k': return o.k || 0;
    case 'L0': return o.L0 || 0;
    case 'L': {
      if (isLink(o)) {
        const e = linkEnds(o);
        return e ? Math.hypot(e.x2 - e.x1, e.y2 - e.y1) : (o.L0 || 0);
      }
      return o.L || 0;
    }
    case 'ang': return o.ang || 0;
    case 'E': return o.E || 0;
    case 'B': return o.B || 0;
    case 'softRadius': return o.softRadius || 0;
    case 'FaimX': return o.FaimX || 0;
    case 'FaimY': return o.FaimY || 0;
    default: return o[field] ?? 0;
  }
}
function holdState(owner, key, value) {
  if (!owner.st) return;
  owner.st[key] = value;
  if (!owner._hold) owner._hold = {};
  owner._hold[key] = value;
}
function applySlot(owner, key, value) {
  if (!Number.isFinite(value)) return;
  if (owner === scene.settings) {
    if (key === 'maxDt') value = Math.max(1e-9, value);
    if (key === 'drag' || key === 'kc' || key === 'vPx' || key === 'FPx' || key === 'snapStep') value = Math.max(1e-9, value);
    owner[key] = value;
    return;
  }
  if ((owner.kind === 'efield' || owner.kind === 'bfield' || owner.kind === 'pointCharge') && owner[key] !== value) fieldsDirty = true;
  switch (key) {
    case 'x':
      if (isBody(owner)) { holdState(owner, 'x', value); if (sim.t === 0) owner.x0 = value; return; }
      owner.x = value; return;
    case 'y':
      if (isBody(owner)) { holdState(owner, 'y', value); if (sim.t === 0) owner.y0 = value; return; }
      owner.y = value; return;
    case 'vx': holdState(owner, 'vx', value); if (sim.t === 0) owner.vx0 = value; return;
    case 'vy': holdState(owner, 'vy', value); if (sim.t === 0) owner.vy0 = value; return;
    case 'v': {
      const ang = Math.atan2(readQuantity(owner.id, 'vy'), readQuantity(owner.id, 'vx'));
      applySlot(owner, 'vx', value * Math.cos(ang));
      applySlot(owner, 'vy', value * Math.sin(ang));
      return;
    }
    case 'vang': {
      const m = Math.hypot(readQuantity(owner.id, 'vx'), readQuantity(owner.id, 'vy'));
      applySlot(owner, 'vx', m * Math.cos(value * DEG));
      applySlot(owner, 'vy', m * Math.sin(value * DEG));
      return;
    }
    case 'v0': {
      const ang = Math.atan2(owner.vy0 || 0, owner.vx0 || 0);
      owner.vx0 = value * Math.cos(ang);
      owner.vy0 = value * Math.sin(ang);
      if (sim.t === 0 && owner.st) { owner.st.vx = owner.vx0; owner.st.vy = owner.vy0; }
      return;
    }
    case 'v0ang': {
      const m = Math.hypot(owner.vx0 || 0, owner.vy0 || 0);
      owner.vx0 = m * Math.cos(value * DEG);
      owner.vy0 = m * Math.sin(value * DEG);
      if (sim.t === 0 && owner.st) { owner.st.vx = owner.vx0; owner.st.vy = owner.vy0; }
      return;
    }
    case 'x0': owner.x0 = value; if (sim.t === 0 && owner.st) owner.st.x = value; return;
    case 'y0': owner.y0 = value; if (sim.t === 0 && owner.st) owner.st.y = value; return;
    case 'vx0': owner.vx0 = value; if (sim.t === 0 && owner.st) owner.st.vx = value; return;
    case 'vy0': owner.vy0 = value; if (sim.t === 0 && owner.st) owner.st.vy = value; return;
    case 'ang0': owner.ang0 = value; if (sim.t === 0 && owner.st) owner.st.ang = value * DEG; return;
    case 'm': owner.m = isBody(owner) ? Math.max(1e-40, value) : Math.max(0, value); return;
    case 'q':
      owner.q = value;
      if (owner.kind === 'pointCharge') owner.color = value >= 0 ? '#c8553d' : '#4a78b0';
      return;
    case 'r': owner.r = Math.max(1e-4, value); return;
    case 'w': owner.w = Math.max(0, value); return;
    case 'h': owner.h = Math.max(0, value); return;
    case 'mu': owner.mu = Math.max(0, value); return;
    case 'e': owner.e = Math.min(1, Math.max(0, value)); return;
    case 'F': owner.F = value; return;
    case 'Fang': owner.Fang = value; return;
    case 'k': owner.k = Math.max(0, value); return;
    case 'L0': owner.L0 = Math.max(0, value); return;
    case 'L': owner.L = Math.max(0.01, value); return;
    case 'ang':
      owner.ang = owner.kind === 'incline' ? Math.min(85, Math.max(0, value)) : value;
      return;
    case 'E': owner.E = Math.max(0, value); return;
    case 'B': owner.B = Math.max(0, value); return;
    case 'softRadius': owner.softRadius = Math.max(1e-4, value); return;
    default: owner[key] = value;
  }
}
function collectExprSlots() {
  const slots = [];
  const push = (owner, key, src) => {
    const s = String(src || '').trim();
    if (!s) return;
    slots.push({ owner, key, src: s, id: slotIdOf(owner, key) });
  };
  for (const o of scene.objects) {
    const bag = o.expr || {};
    const keys = new Set(Object.keys(bag));
    if (isBody(o) && (o.Fmode === 'expr' || o.Fmode === 'aim')) keys.add('F');
    if (isBody(o) && o.Fmode === 'expr') keys.add('Fang');
    for (const key of keys) {
      if (isBody(o) && key === 'F' && (o.Fmode === 'expr' || o.Fmode === 'aim') && String(o.Fexpr || '').trim()) {
        push(o, 'F', o.Fexpr);
        continue;
      }
      if (isBody(o) && key === 'Fang' && o.Fmode === 'expr') {
        push(o, 'Fang', o.FangExpr || '0');
        continue;
      }
      if (bag[key]) push(o, key, bag[key]);
    }
  }
  const sb = scene.settings.expr || {};
  for (const key of Object.keys(sb)) push(scene.settings, key, sb[key]);
  return slots;
}
function resolveExprs() {
  if (resolveExprs.busy) return;
  resolveExprs.busy = true;
  try {
    for (const b of bodies()) b._hold = null;
    const slots = collectExprSlots();
    const bySlot = new Map(slots.map(s => [s.id, s]));
    const names = exprNames();
    const nodes = slots.map(s => {
      const refs = rewriteExpr(s.src, names).refs || [];
      const deps = [];
      for (const r of refs) {
        const fields = EXPR_DERIVED[r.field] || [r.field];
        for (const f of fields) {
          const id = `${r.id}:${f}`;
          if (bySlot.has(id)) deps.push(id);
        }
      }
      return { id: s.id, deps };
    });
    const { order, cyclic } = orderExprNodes(nodes);
    const cyc = new Set(cyclic);
    for (const s of slots) {
      if (!s.owner._exprErr) s.owner._exprErr = {};
      if (cyc.has(s.id)) s.owner._exprErr[s.key] = '循环引用';
    }
    const ref = (id, field) => readQuantity(id, field);
    for (const id of order) {
      const s = bySlot.get(id);
      const ctx = isBody(s.owner) ? forceEvalCtx(s.owner) : {
        t: sim.t,
        x: s.owner.x || s.owner.x0 || 0,
        y: s.owner.y || s.owner.y0 || 0,
        vx: 0, vy: 0, v: 0,
        m: s.owner.m || 0,
        q: s.owner.q || 0,
        g: scene.settings.gOn ? scene.settings.g : 0,
      };
      const r = evalForceExpr(s.src, ctx, { names, ref });
      if (!s.owner._exprErr) s.owner._exprErr = {};
      if (!r.ok) { s.owner._exprErr[s.key] = r.error; continue; }
      s.owner._exprErr[s.key] = '';
      applySlot(s.owner, s.key, r.value);
    }
  } finally {
    resolveExprs.busy = false;
  }
}
function reapplyHolds() {
  for (const b of bodies()) {
    const h = b._hold;
    if (!h || !b.st) continue;
    if ('x' in h) b.st.x = h.x;
    if ('y' in h) b.st.y = h.y;
    if ('vx' in h) b.st.vx = h.vx;
    if ('vy' in h) b.st.vy = h.vy;
  }
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
function appliedForceOn(b, skipResolve = false) {
  ensureForceFields(b);
  if (!skipResolve) resolveExprs();
  const st = b.st || { x: b.x0, y: b.y0 };
  let F = b.F || 0;
  let ang = b.Fang || 0;
  const bag = b._exprErr || {};
  let err = bag.F || bag.Fang || '';
  if (b.Fmode === 'expr' && !String(b.Fexpr || '').trim()) err = err || '表达式为空';

  if (b.Fmode === 'aim') {
    const aim = resolveAimPoint(b);
    const dx = aim.x - st.x, dy = aim.y - st.y;
    const L = Math.hypot(dx, dy);
    if (L < 1e-12) {
      b._Ferr = err || '目标与物体重合';
      return { fx: 0, fy: 0, F: 0, ang: b.Fang || 0, ok: false, err: b._Ferr };
    }
    ang = Math.atan2(dy, dx) / DEG;
    const fx = F * dx / L, fy = F * dy / L;
    b.Fang = ang;
    b._Ferr = err;
    return { fx, fy, F, ang, ok: !err, err };
  }

  b._Ferr = err;
  const fx = F * Math.cos(ang * DEG), fy = F * Math.sin(ang * DEG);
  return { fx, fy, F, ang, ok: !err, err };
}

function computeForces(list, record) {
  const S = scene.settings, out = new Map();
  for (const b of list) {
    const f = { x: 0, y: 0, parts: record ? [] : null }; out.set(b, f);
    const add = (fx, fy, label, color) => { f.x += fx; f.y += fy; if (record && (fx || fy)) f.parts.push({ fx, fy, label, color }); };
    f.ux = 0; f.uy = 0; f.ex = 0; f.ey = 0;
    if (S.gOn) add(0, -b.m * S.g, 'G', '#c8553d');
    if (b.q) {
      const F = fieldAtExcluding(b.st.x, b.st.y, b.id);
      if (F.Ex || F.Ey) add(b.q * F.Ex, b.q * F.Ey, 'F电', '#c48a2a');
      const U = uniformEAt(b.st.x, b.st.y);
      f.ux = b.q * U[0]; f.uy = b.q * U[1];
    }
    const af = appliedForceOn(b, true);
    if (af.fx || af.fy) { add(af.fx, af.fy, 'F', '#8a6bbe'); f.ex += af.fx; f.ey += af.fy; }
    if (S.drag) { const dx = -S.drag * b.st.vx, dy = -S.drag * b.st.vy; add(dx, dy, 'F阻', '#7c796f'); f.ex += dx; f.ey += dy; }
    if (S.gravity) {
      let gx = 0, gy = 0;
      const G = S.Gc || 0, e2 = (S.gravSoft || 0) ** 2;
      for (const o of bodies()) {
        if (o === b || !(o.m > 0)) continue;
        const p = o.st || { x: o.x0, y: o.y0 };
        const dx = p.x - b.st.x, dy = p.y - b.st.y, r2 = dx * dx + dy * dy + e2;
        if (r2 < 1e-18) continue;
        const k = G * b.m * o.m / (r2 * Math.sqrt(r2));
        gx += k * dx; gy += k * dy;
      }
      if (gx || gy) add(gx, gy, 'F引', '#9b6b9e');
    }
  }
  // 弹簧：胡克定律（可拉可压）
  // 轻绳：只拉不压（松弛时张力为 0）
  // 轻杆：长度由约束投影保证，这里只计自重；张力/压力在 applyRodConstraints 中估出
  for (const s of scene.objects) {
    if (!isLink(s)) continue;
    const e = linkEnds(s); if (!e) continue;
    if (S.gOn) applyLinkWeight(out, e, s.m || 0, S.g, record);
    if (s.kind === 'rod') continue;
    const dx = e.x2 - e.x1, dy = e.y2 - e.y1, L = Math.hypot(dx, dy) || 1e-9;
    let T = (s.k || 0) * (L - s.L0);
    if (s.kind === 'rope' && T < 0) T = 0;
    const fx = T * dx / L, fy = T * dy / L;
    const lab = s.kind === 'rope' ? 'T' : 'F弹';
    const col = s.kind === 'rope' ? '#3f8fb0' : '#3d8f86';
    const fa = out.get(e.A); if (fa) { fa.x += fx; fa.y += fy; if (record && T) fa.parts.push({ fx, fy, label: lab, color: col }); }
    const fb = e.B && out.get(e.B); if (fb) { fb.x -= fx; fb.y -= fy; if (record && T) fb.parts.push({ fx: -fx, fy: -fy, label: lab, color: col }); }
  }
  return out;
}

/**
 * 轻杆刚性长度约束：位置 + 速度投影（可压可拉，长度恒为 L₀）。
 * 自重已在 computeForces 中处理；此处估出约束力供受力显示。
 */
function applyRodConstraints(h, recordMap) {
  for (const s of scene.objects) {
    if (s.kind !== 'rod') continue;
    const e = linkEnds(s); if (!e) continue;
    const mRod = s.m || 0;
    const invMass = (body) => {
      if (!body || body.fixed) return 0;
      return 1 / Math.max(body.m + mRod * 0.5, 1e-12);
    };
    const invA = invMass(e.A);
    const invB = e.B ? invMass(e.B) : 0;
    const invSum = invA + invB;
    if (invSum < 1e-18) continue;

    let dx = e.x2 - e.x1, dy = e.y2 - e.y1;
    let L = Math.hypot(dx, dy) || 1e-9;
    let nx = dx / L, ny = dy / L;
    const err = L - s.L0;
    // 位置 Baumgarte 修正
    const corr = err / invSum;
    if (invA) { e.A.st.x += corr * invA * nx; e.A.st.y += corr * invA * ny; }
    if (invB) { e.B.st.x -= corr * invB * nx; e.B.st.y -= corr * invB * ny; }

    // 刷新端点后做速度约束：相对速度沿杆方向为 0
    dx = (e.B ? e.B.st.x : s.ax) - e.A.st.x;
    dy = (e.B ? e.B.st.y : s.ay) - e.A.st.y;
    L = Math.hypot(dx, dy) || 1e-9;
    nx = dx / L; ny = dy / L;
    const vAx = e.A.st.vx, vAy = e.A.st.vy;
    const vBx = e.B ? e.B.st.vx : 0, vBy = e.B ? e.B.st.vy : 0;
    const vrel = (vBx - vAx) * nx + (vBy - vAy) * ny;
    const j = vrel / invSum;
    if (invA) { e.A.st.vx += j * invA * nx; e.A.st.vy += j * invA * ny; }
    if (invB) { e.B.st.vx -= j * invB * nx; e.B.st.vy -= j * invB * ny; }

    // 约束力估计（冲量 / Δt），供受力箭头
    if (recordMap && h > 0) {
      const Fx = (j / h) * nx, Fy = (j / h) * ny;
      if (Math.hypot(Fx, Fy) > 1e-9) {
        const fa = recordMap.get(e.A);
        if (fa && fa.parts) fa.parts.push({ fx: Fx, fy: Fy, label: 'N杆', color: '#4d6470' });
        const fb = e.B && recordMap.get(e.B);
        if (fb && fb.parts) fb.parts.push({ fx: -Fx, fy: -Fy, label: 'N杆', color: '#4d6470' });
      }
    }
    s._N = j / (h || 1e-6); // signed axial force estimate
  }
}

/**
 * SHAKE：沿上一子步的杆方向施加约束冲量，使 |B − A| = L₀，并把位移同步记入速度。
 * 与蛙跳积分配合是辛的，长期能量漂移远小于“投影 + 速度修正”。可处理多杆串联（迭代）。
 */
function shakeRods(h, reset) {
  const rods = scene.objects.filter(s => s.kind === 'rod');
  if (!rods.length) return;
  if (reset) rods.forEach(s => { s._N = 0; });
  for (let it = 0; it < 12; it++) {
    let worst = 0;
    for (const s of rods) {
      const e = linkEnds(s); if (!e) continue;
      const mRod = s.m || 0;
      const inv = body => (!body || body.fixed || body._grab) ? 0 : 1 / Math.max(body.m + mRod * 0.5, 1e-12);
      const iA = inv(e.A), iB = e.B ? inv(e.B) : 0, S = iA + iB;
      if (S < 1e-18) continue;
      const A = e.A.st, B = e.B ? e.B.st : null;
      const ox = (B ? (B._ox ?? B.x) : s.ax) - (A._ox ?? A.x), oy = (B ? (B._oy ?? B.y) : s.ay) - (A._oy ?? A.y);
      const dx = e.x2 - A.x, dy = e.y2 - A.y;
      const diff = dx * dx + dy * dy - s.L0 * s.L0;
      worst = Math.max(worst, Math.abs(diff) / Math.max(s.L0 * s.L0, 1e-12));
      const dr = dx * ox + dy * oy;
      if (Math.abs(dr) < 1e-12) continue;
      const g = diff / (2 * S * dr);
      if (iA) { A.x += g * iA * ox; A.y += g * iA * oy; A.vx += g * iA * ox / h; A.vy += g * iA * oy / h; }
      if (B && iB) { B.x -= g * iB * ox; B.y -= g * iB * oy; B.vx -= g * iB * ox / h; B.vy -= g * iB * oy / h; }
      s._N = (s._N || 0) + g * Math.hypot(ox, oy) / (h * h);
    }
    if (worst < 1e-12) break;
  }
}

/* ---------------- 单个子步 ---------------- */
function substep(h, segs) {
  resolveExprs();
  const S = scene.settings;
  const movers = bodies().filter(b => !b.fixed);
  for (const b of bodies()) { b.st._ox = b.st.x; b.st._oy = b.st.y; }
  const F = computeForces(movers, false);
  for (const b of movers) {
    const st = b.st, f = F.get(b), ax = f.x / b.m, ay = f.y / b.m;
    if (b._grab) { const g = b._grab; st.x = g.x; st.y = g.y; st.vx = g.vx; st.vy = g.vy; continue; }
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
    const ovx = st.vx, ovy = st.vy;
    st.vx = vx + ax * h / 2; st.vy = vy + ay * h / 2;
    const mvx = (ovx + st.vx) / 2, mvy = (ovy + st.vy) / 2;
    sim.wE += (f.ux * mvx + f.uy * mvy) * h;
    sim.wExt += (f.ex * mvx + f.ey * mvy) * h;
    st.x += st.vx * h; st.y += st.vy * h;
  }
  // 轻杆：刚性长度约束（SHAKE，在接触前满足约束）
  shakeRods(h, true);
  // 与静止表面接触
  for (const b of movers) {
    if (b._grab) continue;
    for (const sg of segs) contactSeg(b, sg, h);
  }
  // 物体间碰撞
  const bl = bodies().filter(b => b.collide);
  for (let i = 0; i < bl.length; i++) for (let j = i + 1; j < bl.length; j++) collidePair(bl[i], bl[j]);
  // 接触可能破坏杆长，再做一次 SHAKE
  shakeRods(h, false);
  reapplyHolds();
  // 轨迹
  for (const b of movers) {
    const tr = b.st.trail, last = tr[tr.length - 1];
    const minD = 2 / scene.view.scale;
    if (!last || Math.abs(last[0] - b.st.x) + Math.abs(last[1] - b.st.y) > minD) {
      tr.push([b.st.x, b.st.y]);
      if (tr.length > 8000) { const n = tr.length - 8000; tr.splice(0, n); b.st.trailDrop = (b.st.trailDrop || 0) + n; }
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
  if ((a.fixed || a._grab) && (b.fixed || b._grab)) return;
  const A = a.st, B = b.st;
  let dx = B.x - A.x, dy = B.y - A.y; const d = Math.hypot(dx, dy), R = bodyRadius(a) + bodyRadius(b);
  if (d >= R || d < 1e-9) return;
  const nx = dx / d, ny = dy / d;
  const ia = a.fixed || a._grab ? 0 : 1 / a.m, ib = b.fixed || b._grab ? 0 : 1 / b.m, is = ia + ib;
  if (is <= 0) return;
  const pen = R - d;
  A.x -= nx * pen * ia / is; A.y -= ny * pen * ia / is; B.x += nx * pen * ib / is; B.y += ny * pen * ib / is;
  const vrel = (B.vx - A.vx) * nx + (B.vy - A.vy) * ny;
  if (vrel >= 0) return;
  const e = -vrel < REST_V * 0.5 ? 0 : Math.min(a.e, b.e);
  const j = -(1 + e) * vrel / is;
  A.vx -= j * ia * nx; A.vy -= j * ia * ny; B.vx += j * ib * nx; B.vy += j * ib * ny;
  // 物体间库仑摩擦（两者都有 μ 时才生效，取较小值）
  const mu = Math.min(a.mu || 0, b.mu || 0);
  if (mu > 0) {
    const tx = -ny, ty = nx, vt = (B.vx - A.vx) * tx + (B.vy - A.vy) * ty;
    const jt = Math.max(-mu * j, Math.min(mu * j, -vt / is));
    A.vx -= jt * ia * tx; A.vy -= jt * ia * ty; B.vx += jt * ib * tx; B.vy += jt * ib * ty;
  }
}

/* ---------------- 能量与功 ---------------- */
function uniformEAt(x, y) {
  let Ex = 0, Ey = 0;
  for (const o of scene.objects) {
    if (o.kind !== 'efield' || !inRect(o, x, y)) continue;
    Ex += o.E * Math.cos(o.ang * DEG); Ey += o.E * Math.sin(o.ang * DEG);
  }
  return [Ex, Ey];
}
/**
 * 系统能量：动能 + 重力势能 + 弹性势能 + 静电势能(点电荷/带电体) + 引力势能。
 * 匀强电场区域有界，不存在全局势函数，改记为「电场力做功」W_E；外力与阻力做功记为 W_ext。
 * 若只有保守力，E 守恒；否则 ΔE = W_E + W_ext − 耗散（碰撞、摩擦）。
 */
function systemEnergy() {
  const S = scene.settings, list = bodies();
  let Ek = 0, Ug = 0, Us = 0, Ue = 0, Ugr = 0, px = 0, py = 0, M = 0;
  const pos = b => b.st || { x: b.x0, y: b.y0, vx: b.vx0 || 0, vy: b.vy0 || 0 };
  for (const b of list) {
    if (b.fixed) continue;
    const p = pos(b), v2 = (p.vx || 0) ** 2 + (p.vy || 0) ** 2;
    Ek += 0.5 * b.m * v2; px += b.m * (p.vx || 0); py += b.m * (p.vy || 0); M += b.m;
    if (S.gOn) Ug += b.m * S.g * p.y;
  }
  for (const s of scene.objects) {
    if (s.kind !== 'spring' && s.kind !== 'rope') continue;
    const e = linkEnds(s); if (!e) continue;
    const dL = Math.hypot(e.x2 - e.x1, e.y2 - e.y1) - s.L0;
    if (s.kind === 'rope' && dL <= 0) continue;
    Us += 0.5 * (s.k || 0) * dL * dL;
  }
  const law = S.coulombLaw || 'coulomb3d', kc = S.kc || K_COULOMB;
  const phi = (q, r) => law === 'coulomb2d' ? -2 * kc * q * Math.log(Math.max(r, 1e-6)) : kc * q / Math.max(r, 1e-9);
  const charges = scene.objects.filter(o => o.kind === 'pointCharge');
  for (const b of list) {
    if (!b.q) continue;
    const p = pos(b);
    for (const c of charges) Ue += b.q * phi(c.q, Math.hypot(p.x - c.x, p.y - c.y, c.softRadius || 0.08));
  }
  for (let i = 0; i < list.length; i++) for (let j = i + 1; j < list.length; j++) {
    const a = list[i], b = list[j];
    if (a.fixed && b.fixed) continue;
    const pa = pos(a), pb = pos(b), dx = pb.x - pa.x, dy = pb.y - pa.y;
    if (S.coulomb && a.q && b.q) Ue += a.q * phi(b.q, Math.hypot(dx, dy, 0.02));
    if (S.gravity) Ugr += -(S.Gc || 0) * a.m * b.m / Math.sqrt(dx * dx + dy * dy + (S.gravSoft || 0) ** 2);
  }
  const Ep = Ug + Us + Ue + Ugr, E = Ek + Ep;
  return { Ek, Ug, Us, Ue, Ugr, Ep, E, W: sim.wE + sim.wExt, wE: sim.wE, wExt: sim.wExt, px, py, M };
}
function resetEnergyLedger() {
  sim.wE = 0; sim.wExt = 0; sim.eHist = []; sim.eDrop = 0;
  const en = systemEnergy();
  sim.E0 = en.E;
  sim.eHist.push([0, en.Ek, en.Ep, en.E, en.E - en.W]);
}
function pushEnergy() {
  const en = systemEnergy();
  if (sim.E0 == null) sim.E0 = en.E;
  sim.eHist.push([sim.t, en.Ek, en.Ep, en.E, en.E - en.W]);
  if (sim.eHist.length > 8000) { const n = sim.eHist.length - 8000; sim.eHist.splice(0, n); sim.eDrop += n; }
}

/* ---------------- 时间轴快照 ---------------- */
function snapshotFrame() {
  sim.frames.push({ t: sim.t, wE: sim.wE, wExt: sim.wExt, e: sim.eHist.length + sim.eDrop,
    b: bodies().filter(b => b.st).map(b => { const st = b.st; return [b.id, st.x, st.y, st.vx, st.vy, st.ang, st.trail.length + (st.trailDrop || 0), st.hist.length + (st.histDrop || 0), st.ax || 0, st.ay || 0]; }) });
  if (sim.frames.length > MAX_FRAMES) sim.frames.shift();
  sim.frameIdx = sim.frames.length - 1;
}
function restoreFrame(i) {
  i = Math.max(0, Math.min(sim.frames.length - 1, i | 0));
  const f = sim.frames[i]; if (!f) return;
  sim.t = f.t; sim.wE = f.wE; sim.wExt = f.wExt;
  sim.eHist.length = Math.max(1, Math.min(sim.eHist.length, f.e - sim.eDrop));
  for (const r of f.b) {
    const o = byId(r[0]); if (!o || !o.st) continue;
    const st = o.st;
    st.x = r[1]; st.y = r[2]; st.vx = r[3]; st.vy = r[4]; st.ang = r[5]; st.ax = r[8]; st.ay = r[9];
    st.trail.length = Math.max(1, Math.min(st.trail.length, r[6] - (st.trailDrop || 0)));
    st.hist.length = Math.max(1, Math.min(st.hist.length, r[7] - (st.histDrop || 0)));
    st.contactN = [0, 0]; st.contactF = [0, 0];
  }
  sim.frameIdx = i;
  resolveExprs(); markFieldsDirty(); requestDraw(); updateReadouts(); updateEnergyReadout(true);
}
function isScrubbed() { return sim.frameIdx < sim.frames.length - 1; }
/** 从回看位置继续：丢弃之后的记录 */
function commitScrub() {
  if (!isScrubbed()) return;
  sim.frames.length = sim.frameIdx + 1;
}
function updateTimeline() {
  const box = $('#timeline'); if (!box) return;
  const n = sim.frames.length;
  box.classList.toggle('show', n > 1);
  if (n < 2) return;
  const r = $('#tlRange');
  r.max = String(n - 1);
  if (document.activeElement !== r) r.value = String(sim.frameIdx);
  const total = sim.frames[n - 1].t, t0 = sim.frames[0].t;
  r.style.setProperty('--p', `${(sim.frameIdx / (n - 1)) * 100}%`);
  $('#tlTime').textContent = `${fmt(sim.t, 2)} / ${fmt(total, 2)} s`;
  box.classList.toggle('scrubbed', isScrubbed());
  box.title = t0 > 0 ? `只保留最近 ${MAX_FRAMES} 帧（从 ${fmt(t0, 2)} s 起）` : '拖动回看；从回看处播放会丢弃之后的记录';
}

/* ---------------- 前进一帧 ---------------- */
function advance(dtFrame) {
  ensureStates();
  commitScrub();
  const segs = getSegments();
  const n = Math.min(20000, Math.max(1, Math.ceil(dtFrame / scene.settings.maxDt)));
  const h = dtFrame / n;
  for (const b of bodies()) { b.st.contactN = [0, 0]; b.st.contactF = [0, 0]; b.st.pvx = b.st.vx; b.st.pvy = b.st.vy; }
  for (let i = 0; i < n; i++) {
    syncBodyEmitterPositions();
    substep(h, segs);
  }
  sim.t += dtFrame;
  pushEnergy();
  for (const b of bodies()) {
    const st = b.st; st.contactN[0] /= n; st.contactN[1] /= n; st.contactF[0] /= n; st.contactF[1] /= n;
    st.ax = (st.vx - st.pvx) / dtFrame; st.ay = (st.vy - st.pvy) / dtFrame;
    pushHist(b);
  }
  snapshotFrame();
}
function pushHist(b) {
  const st = b.st, v2 = st.vx * st.vx + st.vy * st.vy;
  st.hist.push([sim.t, st.x, st.y, st.vx, st.vy, Math.sqrt(v2), 0.5 * b.m * v2, Math.hypot(st.ax || 0, st.ay || 0)]);
  if (st.hist.length > 8000) { const n = st.hist.length - 8000; st.hist.splice(0, n); st.histDrop = (st.histDrop || 0) + n; }
}

/* =====================================================================
 *  渲染
 * ===================================================================== */
const cv = $('#cv'), ctx = cv.getContext('2d');
let W = 0, H = 0, DPR = 1;
function css(v) { return getComputedStyle(document.body).getPropertyValue(v).trim(); }
let C = {};
function readColors() { ['--accent-soft', '--canvas', '--grid', '--grid2', '--axis', '--text', '--muted', '--efield', '--efield-line', '--bfield', '--bfield-line', '--surface', '--accent', '--panel', '--border'].forEach(k => C[k] = css(k)); }
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
  ctx.save(); ctx.font = '12px ' + UI_FONT; ctx.textAlign = align; ctx.textBaseline = 'middle';
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
  ctx.fillStyle = C['--muted']; ctx.font = '10.5px ' + MONO_FONT;
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
function drawCrossMark(sx, sy, size = 6.5) {
  ctx.beginPath();
  ctx.moveTo(sx - size, sy - size); ctx.lineTo(sx + size, sy + size);
  ctx.moveTo(sx + size, sy - size); ctx.lineTo(sx - size, sy + size);
  ctx.stroke();
}
function drawDotMark(sx, sy) {
  ctx.beginPath();
  ctx.arc(sx, sy, 2.6, 0, Math.PI * 2);
  ctx.fill();
}
function drawField(f) {
  const [x, y, w, h] = fieldScreenRect(f), sel = f === selected();
  const isE = f.kind === 'efield', line = isE ? C['--efield-line'] : C['--bfield-line'];
  ctx.save();
  ctx.fillStyle = isE ? C['--efield'] : C['--bfield']; ctx.fillRect(x, y, w, h);
  ctx.beginPath(); ctx.rect(x, y, w, h); ctx.clip();
  // 屏幕上等距、偏疏：磁场只画叉或点，电场只留一支短箭头表示方向
  const sp = 108;
  const pad = 22;
  const cx = x + w / 2, cy = y + h / 2;
  const startX = cx - Math.floor((cx - (x + pad)) / sp) * sp;
  const startY = cy - Math.floor((cy - (y + pad)) / sp) * sp;
  ctx.strokeStyle = line; ctx.fillStyle = line; ctx.lineWidth = 1.7; ctx.lineCap = 'round';
  ctx.globalAlpha = 0.9;
  for (let sx = startX; sx <= x + w - pad; sx += sp) {
    for (let sy = startY; sy <= y + h - pad; sy += sp) {
      if (!isE) {
        if (f.dir < 0) drawCrossMark(sx, sy);
        else drawDotMark(sx, sy);
      } else {
        const a = f.ang * DEG, ux = Math.cos(a), uy = -Math.sin(a), len = 16;
        arrow(sx - ux * len, sy - uy * len, sx + ux * len, sy + uy * len, line, 1.6, 7);
      }
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
function drawLink(s) {
  const e = linkEnds(s); if (!e) return;
  const [x1, y1] = w2s(e.x1, e.y1), [x2, y2] = w2s(e.x2, e.y2), sel = s === selected();
  const kind = s.kind;
  const base = kind === 'rod' ? '#4d6470' : kind === 'rope' ? '#4f7faf' : '#3d8f86';
  ctx.save();
  ctx.strokeStyle = sel ? C['--accent'] : base;
  ctx.lineWidth = sel ? (kind === 'rod' ? 3.2 : 2.5) : (kind === 'rod' ? 2.6 : 1.8);
  ctx.lineJoin = 'round';
  ctx.lineCap = kind === 'rod' ? 'round' : 'butt';
  const dx = x2 - x1, dy = y2 - y1, L = Math.hypot(dx, dy) || 1, ux = dx / L, uy = dy / L;
  ctx.beginPath(); ctx.moveTo(x1, y1);
  if (kind === 'spring') {
    const n = 12, lead = Math.min(10, L * 0.15), amp = 6;
    ctx.lineTo(x1 + ux * lead, y1 + uy * lead);
    for (let i = 1; i < n * 2; i++) {
      const t = lead + (L - 2 * lead) * i / (n * 2), sgn = i % 2 ? 1 : -1;
      ctx.lineTo(x1 + ux * t - uy * amp * sgn, y1 + uy * t + ux * amp * sgn);
    }
    ctx.lineTo(x2 - ux * lead, y2 - uy * lead); ctx.lineTo(x2, y2);
  } else {
    // 轻绳 / 轻杆：直线；轻杆略粗已在 lineWidth 体现
    ctx.lineTo(x2, y2);
  }
  ctx.stroke();
  if (kind === 'rod') {
    // 端点铰接小圆
    ctx.fillStyle = sel ? C['--accent'] : base;
    for (const [px, py] of [[x1, y1], [x2, y2]]) {
      ctx.beginPath(); ctx.arc(px, py, 3.2, 0, 7); ctx.fill();
    }
  }
  if (s.b == null) {
    ctx.fillStyle = C['--surface'];
    ctx.fillRect(x2 - 8, y2 - 3, 16, 6);
    hatchLine(x2 - 8, y2 - 3, x2 + 8, y2 - 3, 1, C['--surface']);
  }
  ctx.restore();
}
const drawSpring = drawLink;
function bodyScreenR(b) { return Math.max(bodyRadius(b) * scene.view.scale, b.kind === 'particle' ? 5 : 3); }
function drawBodyShape(b, x, y, ang, ghost) {
  const s = scene.view.scale;
  ctx.save(); ctx.globalAlpha = ghost ? 0.35 : 1;
  ctx.fillStyle = b.color; ctx.strokeStyle = 'rgba(20,20,19,.32)'; ctx.lineWidth = 1;
  if (ghost) ctx.setLineDash([4, 3]);
  if (b.kind === 'box') {
    ctx.translate(x, y); ctx.rotate(-ang);
    const w = Math.max(b.w * s, 4), h = Math.max(b.h * s, 4);
    ctx.fillRect(-w / 2, -h / 2, w, h); ctx.strokeRect(-w / 2, -h / 2, w, h);
  } else {
    const r = bodyScreenR(b);
    ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.fill(); ctx.stroke();
    if (!ghost && r > 6) { ctx.fillStyle = 'rgba(255,255,255,.22)'; ctx.beginPath(); ctx.arc(x - r * 0.3, y - r * 0.3, r * 0.3, 0, 7); ctx.fill(); }
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
/** 可单独选取的矢量：初速度、瞬时速度、外力，以及受力图里的分力。零矢量不画箭头。 */
function vectorMeta(key) {
  if (key === 'v0') return { key, name: '初速度', color: '#4a8a4f', editable: true, unit: 'm/s' };
  if (key === 'v') return { key, name: '速度', color: '#4a8a4f', editable: true, unit: 'm/s' };
  if (key === 'F') return { key, name: '外力', color: '#8a6bbe', editable: true, unit: 'N' };
  const label = String(key).startsWith('part:') ? key.slice(5) : key;
  return { key, name: label, color: '#6b6a64', editable: false, unit: 'N', part: true };
}
function vectorScreen(b, key) {
  const st = b.st || { x: b.x0, y: b.y0, vx: b.vx0 || 0, vy: b.vy0 || 0 };
  if (key === 'v0') {
    const [x, y] = w2s(b.x0, b.y0);
    const S = scene.settings;
    const vx = b.vx0 || 0, vy = b.vy0 || 0;
    const mag = Math.hypot(vx, vy);
    return { x, y, hx: x + vx * S.vPx, hy: y - vy * S.vPx, mag, zero: mag < 1e-9, color: '#4a8a4f', ang: Math.atan2(vy, vx) / DEG };
  }
  if (key === 'v') {
    const [x, y] = w2s(st.x, st.y);
    const S = scene.settings;
    const vx = st.vx || 0, vy = st.vy || 0;
    const mag = Math.hypot(vx, vy);
    return { x, y, hx: x + vx * S.vPx, hy: y - vy * S.vPx, mag, zero: mag < 1e-9, color: '#4a8a4f', ang: Math.atan2(vy, vx) / DEG };
  }
  if (key === 'F') {
    ensureForceFields(b);
    const [x, y] = w2s(st.x, st.y);
    const af = appliedForceOn(b);
    const FPx = scene.settings.FPx || 12;
    const mag = Math.hypot(af.fx, af.fy);
    return { x, y, hx: x + af.fx * FPx, hy: y - af.fy * FPx, mag, zero: mag < 1e-6, color: '#8a6bbe', ang: af.ang, af };
  }
  if (String(key).startsWith('part:')) {
    const label = key.slice(5);
    const parts = forceParts(b);
    const p = parts.find(q => q.label === label);
    if (!p) return null;
    let mx = 0;
    parts.forEach(q => { mx = Math.max(mx, Math.hypot(q.fx, q.fy)); });
    const sc = mx > 0 ? 70 / mx : 1;
    const [x, y] = w2s(st.x, st.y);
    const mag = Math.hypot(p.fx, p.fy);
    return { x, y, hx: x + p.fx * sc, hy: y - p.fy * sc, mag, zero: mag < 1e-6, color: p.color, ang: Math.atan2(p.fy, p.fx) / DEG, part: p };
  }
  return null;
}
function vectorIsSelected(b, key) {
  return !!(selVec && selVec.bodyId === b.id && selVec.key === key);
}
function visibleVectorKeys(b) {
  const keys = [];
  const bodyOn = selId === b.id && !selVec;
  const g0 = vectorScreen(b, 'v0');
  if (!sim.running && g0 && (!g0.zero || bodyOn || vectorIsSelected(b, 'v0'))) keys.push('v0');
  // t = 0 时速度就是初速度，避免两支箭头叠在一起
  if (scene.settings.showVel && (sim.t > 0 || !keys.includes('v0'))) {
    const gv = vectorScreen(b, 'v');
    if (gv && !gv.zero) keys.push('v');
  }
  const gF = vectorScreen(b, 'F');
  if (gF && !gF.zero && (scene.settings.showForce || vectorIsSelected(b, 'F'))) keys.push('F');
  if (scene.settings.showForce && b.st) {
    for (const p of forceParts(b)) {
      if (p.label === 'F') continue;
      if (Math.hypot(p.fx, p.fy) < 1e-6) continue;
      keys.push('part:' + p.label);
    }
  }
  return keys;
}
function listVectors(b) {
  const items = [
    { key: 'v0', name: '初速度', color: '#4a8a4f' },
    { key: 'v', name: '速度', color: '#4a8a4f' },
    { key: 'F', name: '外力', color: '#8a6bbe' },
  ];
  if (scene.settings.showForce && b.st) {
    for (const p of forceParts(b)) {
      if (p.label === 'F') continue;
      if (Math.hypot(p.fx, p.fy) < 1e-4) continue;
      items.push({ key: 'part:' + p.label, name: p.label, color: p.color });
    }
  }
  return items;
}
function drawOneVector(b, key) {
  const g = vectorScreen(b, key);
  if (!g) return;
  const sel = vectorIsSelected(b, key);
  const meta = vectorMeta(key);
  if (g.zero) {
    if (key !== 'v0') return;
    const minL = bodyScreenR(b) + 14;
    const hx = g.x + minL, hy = g.y;
    ctx.save(); ctx.setLineDash([5, 4]);
    arrow(g.x, g.y, hx, hy, g.color, 2, 10, [5, 4]);
    ctx.restore();
    handleDot(hx, hy, g.color, sel ? 8 : 7);
    label(sel ? '拖动设置 v₀' : '拖动设置 v₀', hx + 12, hy - 12, g.color);
    if (sel) drawVecSelection(hx, hy);
    return;
  }
  const len = Math.hypot(g.hx - g.x, g.hy - g.y);
  if (len < 2 && !sel) return;
  arrow(g.x, g.y, g.hx, g.hy, g.color, sel ? 3 : 2.2, sel ? 12 : 8);
  if (key === 'v0' || key === 'F' || key === 'v' || sel) handleDot(g.hx, g.hy, g.color, sel ? 8 : 6);
  const txt = key === 'F'
    ? `F=${fmt(g.mag, 2)} N  ${fmt(g.ang, 1)}°`
    : key === 'v0' ? `v₀=${fmt(g.mag, 2)} m/s  ${fmt(g.ang, 1)}°`
    : key === 'v' ? `v=${fmt(g.mag, 2)}`
    : `${meta.name} ${fmt(g.mag, 2)}N`;
  label(txt, g.hx + 10, g.hy - 10, g.color, 'left', false);
  if (sel) drawVecSelection(g.hx, g.hy);
  if (key === 'F' && (b.Fmode === 'aim') && sel) {
    const aim = resolveAimPoint(b);
    const [ax, ay] = w2s(aim.x, aim.y);
    ctx.save(); ctx.strokeStyle = 'rgba(138,107,190,.45)'; ctx.setLineDash([3, 3]); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(g.x, g.y); ctx.lineTo(ax, ay); ctx.stroke();
    ctx.fillStyle = '#8a6bbe'; ctx.beginPath(); ctx.arc(ax, ay, 3, 0, 7); ctx.fill();
    ctx.restore();
  }
}
function drawVecSelection(hx, hy) {
  ctx.save();
  ctx.strokeStyle = C['--accent'];
  ctx.lineWidth = 2;
  ctx.setLineDash([4, 3]);
  ctx.beginPath();
  ctx.arc(hx, hy, 12, 0, 7);
  ctx.stroke();
  ctx.restore();
}
function drawSelectableVectors() {
  for (const b of bodies()) {
    for (const key of visibleVectorKeys(b)) drawOneVector(b, key);
  }
}
function hitVector(sx, sy) {
  let best = null, bestD = 14;
  for (const b of bodies()) {
    const bodyR = bodyScreenR(b) + 3;
    for (const key of visibleVectorKeys(b)) {
      const g = vectorScreen(b, key);
      if (!g) continue;
      // 点在物体身上时交给物体拖拽，避免箭头从质心伸出时把拖动吃掉
      if (Math.hypot(sx - g.x, sy - g.y) <= bodyR) continue;
      let hx = g.hx, hy = g.hy;
      if (g.zero && key === 'v0') { hx = g.x + bodyR + 11; hy = g.y; }
      const dt = Math.hypot(sx - hx, sy - hy);
      if (dt < bestD) { best = { b, key, tip: true }; bestD = dt; }
      if (!g.zero && Math.hypot(g.hx - g.x, g.hy - g.y) > 10) {
        const ds = distSeg(sx, sy, g.x, g.y, g.hx, g.hy);
        if (ds < 8 && ds < bestD) { best = { b, key, tip: false }; bestD = ds; }
      }
    }
  }
  return best;
}
function forceParts(b) {
  const fmap = computeForces([b], true);
  const f = fmap.get(b), parts = f.parts.slice(), st = b.st;
  // 估测轻杆约束力（用最近一次 _N；无则瞬时投影一次）
  for (const s of scene.objects) {
    if (s.kind !== 'rod') continue;
    if (s.a !== b.id && s.b !== b.id) continue;
    const e = linkEnds(s); if (!e) continue;
    const dx = e.x2 - e.x1, dy = e.y2 - e.y1, L = Math.hypot(dx, dy) || 1e-9;
    const nx = dx / L, ny = dy / L;
    const N = s._N || 0;
    if (Math.abs(N) < 1e-9) continue;
    const sign = s.a === b.id ? 1 : -1;
    parts.push({ fx: sign * N * nx, fy: sign * N * ny, label: 'N杆', color: '#4d6470' });
  }
  if (b.q) {
    const Bz = fieldAt(st.x, st.y).Bz;
    if (Bz) parts.push({ fx: b.q * st.vy * Bz, fy: -b.q * st.vx * Bz, label: 'F洛', color: '#4a78b0' });
  }
  if (sim.t > 0) {
    if (Math.hypot(...st.contactN) > 1e-6) parts.push({ fx: st.contactN[0], fy: st.contactN[1], label: 'N', color: '#3f8fb0' });
    if (Math.hypot(...st.contactF) > 1e-6) parts.push({ fx: st.contactF[0], fy: st.contactF[1], label: 'f', color: '#9a6a2a' });
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
  if (!H0.zero) arrow(H0.x, H0.y, H0.hx, H0.hy, '#4a8a4f', 3, 12, ghost ? [5, 4] : null);
  handleDot(H0.hx, H0.hy, '#4a8a4f', 7);
  const txt = H0.zero ? '拖动设置 v₀' : `v₀ = ${fmt(v, 2)} m/s  ${fmt(ang, 1)}°`;
  label(txt, H0.hx + 12, H0.hy - 12, '#4a8a4f');
}


function drawElectricFieldLines() {
  const lines = fieldSys.getFieldLines();
  if (!lines.length) return;
  ctx.save();
  ctx.strokeStyle = C['--efield-line'] || '#b5762a';
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
      arrow(x1, y1, x2, y2, C['--efield-line'] || '#b5762a', 1.2, 6);
    }
  }
  ctx.restore();
}
function drawPointCharge(o) {
  const [sx, sy] = w2s(o.x, o.y);
  const r = Math.max(8, (o.softRadius || 0.08) * scene.view.scale);
  const pos = o.q >= 0;
  ctx.save();
  ctx.fillStyle = o.color || (pos ? '#c8553d' : '#4a78b0');
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
  ctx.strokeStyle = C['--accent'] || '#4a78b0';
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
/** 悬停反馈：被指到的对象描一圈半透明强调色 */
function drawHover() {
  const o = hoverObj;
  if (!o || (o === selected() && tool === 'select') || !scene.objects.includes(o)) return;
  ctx.save(); ctx.strokeStyle = C['--accent']; ctx.lineCap = 'round'; ctx.lineJoin = 'round';
  const sc = scene.view.scale;
  if (isBody(o) && o.st) {
    const [x, y] = w2s(o.st.x, o.st.y);
    const r = o.kind === 'box' ? Math.hypot(o.w, o.h) / 2 * sc + 4 : bodyScreenR(o) + 4;
    ctx.globalAlpha = 0.18; ctx.fillStyle = C['--accent']; ctx.beginPath(); ctx.arc(x, y, r + 2, 0, 7); ctx.fill();
    ctx.globalAlpha = 0.7; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(x, y, r + 2, 0, 7); ctx.stroke();
  } else if (o.kind === 'pointCharge') {
    const [x, y] = w2s(o.x, o.y), r = Math.max(8, (o.softRadius || 0.08) * sc) + 5;
    ctx.globalAlpha = 0.7; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.stroke();
  } else {
    ctx.globalAlpha = 0.28; ctx.lineWidth = 8; ctx.beginPath();
    if (o.kind === 'wall') { const [a, b] = w2s(o.x1, o.y1), [c, d] = w2s(o.x2, o.y2); ctx.moveTo(a, b); ctx.lineTo(c, d); }
    else if (isLink(o)) { const e = linkEnds(o); if (e) { const [a, b] = w2s(e.x1, e.y1), [c, d] = w2s(e.x2, e.y2); ctx.moveTo(a, b); ctx.lineTo(c, d); } }
    else if (o.kind === 'ground') { const y = w2s(0, o.y)[1]; ctx.moveTo(0, y); ctx.lineTo(W, y); }
    else if (o.kind === 'incline') { const p = inclinePts(o).map(q => w2s(q[0], q[1])); ctx.lineWidth = 5; ctx.moveTo(...p[0]); ctx.lineTo(...p[1]); ctx.lineTo(...p[2]); ctx.closePath(); }
    else if (o.kind === 'efield' || o.kind === 'bfield') { const [x, y, w, h] = fieldScreenRect(o); ctx.lineWidth = 4; ctx.rect(x, y, w, h); }
    ctx.stroke();
  }
  ctx.restore();
}
/** 放置预览：添加工具激活时，在吸附后的光标位置画出半透明的待放对象 */
function drawGhost() {
  if (!mouseIn || drag || tool === 'select' || tool === 'pan' || LINK_KINDS.includes(tool)) return;
  const p = snapPt(mouseW), [x, y] = w2s(p[0], p[1]), sc = scene.view.scale;
  ctx.save(); ctx.globalAlpha = 0.5; ctx.strokeStyle = C['--accent']; ctx.fillStyle = C['--accent-soft'] || 'rgba(217,119,87,.15)';
  ctx.lineWidth = 1.5; ctx.setLineDash([4, 3]);
  if (tool === 'ball' || tool === 'particle') {
    const r = Math.max((tool === 'ball' ? 0.25 : 0.08) * sc, tool === 'particle' ? 5 : 3);
    ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.fill(); ctx.stroke();
  } else if (tool === 'box') {
    const w = Math.max(0.6 * sc, 4), h = Math.max(0.4 * sc, 4); ctx.fillRect(x - w / 2, y - h / 2, w, h); ctx.strokeRect(x - w / 2, y - h / 2, w, h);
  } else if (tool === 'pointCharge') {
    ctx.beginPath(); ctx.arc(x, y, Math.max(8, 0.08 * sc), 0, 7); ctx.fill(); ctx.stroke();
  } else if (tool === 'ground') {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
  } else if (tool === 'incline') {
    const q = inclinePts({ x: p[0], y: p[1], L: 4, ang: 37, dir: 1 }).map(v => w2s(v[0], v[1]));
    ctx.beginPath(); ctx.moveTo(...q[0]); ctx.lineTo(...q[1]); ctx.lineTo(...q[2]); ctx.closePath(); ctx.fill(); ctx.stroke();
  } else {
    ctx.setLineDash([]); ctx.beginPath(); ctx.moveTo(x - 7, y); ctx.lineTo(x + 7, y); ctx.moveTo(x, y - 7); ctx.lineTo(x, y + 7); ctx.stroke();
  }
  ctx.restore();
  label(`(${fmt(p[0], 2)}, ${fmt(p[1], 2)})${snapOff ? ' · 不吸附' : ''}`, x + 12, y + 14, C['--muted'], 'left', true);
}
function draw() {
  if (!W) return;
  if (!drag) resolveExprs();
  ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
  ctx.fillStyle = C['--canvas']; ctx.fillRect(0, 0, W, H);
  if (scene.settings.showGrid) drawGrid();
  const objs = scene.objects;
  ensureFields();
  if (scene.settings.showFieldLines) drawElectricFieldLines();
  objs.filter(o => o.kind === 'efield' || o.kind === 'bfield').forEach(drawField);
  objs.filter(o => o.kind === 'pointCharge').forEach(drawPointCharge);
  objs.filter(o => ['ground', 'wall', 'incline'].includes(o.kind)).forEach(drawSurface);
  objs.filter(isLink).forEach(drawLink);
  ensureStates();
  bodies().forEach(drawBody);
  drawSelectableVectors();
  const s = selected();
  if (isBody(s) && !sim.running && sim.t > 0 && (!selVec || (selVec.bodyId === s.id && selVec.key === 'v0'))) {
    const [gx, gy] = w2s(s.x0, s.y0);
    drawBodyShape(s, gx, gy, s.ang0 * DEG, true);
  }
  // Analytic overlay is relatively expensive — refresh every other frame while running
  if (isBody(s) && !selVec && scene.settings.showAnalytic && (!sim.running || ((performance.now() / 32) | 0) % 2 === 0)) {
    drawAnalyticPath(s);
  }
  drawHover();
  drawGhost();
  if (pending) {
    ctx.save(); ctx.strokeStyle = C['--accent']; ctx.setLineDash([5, 4]); ctx.lineWidth = 1.5;
    if (pending.type === 'rect') { const [a, b] = w2s(pending.x1, pending.y1), [c, d] = w2s(pending.x2, pending.y2); ctx.strokeRect(Math.min(a, c), Math.min(b, d), Math.abs(c - a), Math.abs(d - b)); label(`${fmt(Math.abs(pending.x2 - pending.x1), 2)} × ${fmt(Math.abs(pending.y2 - pending.y1), 2)} m`, Math.max(a, c) + 6, Math.max(b, d) + 10, C['--accent']); }
    if (pending.type === 'line') { const [a, b] = w2s(pending.x1, pending.y1), [c, d] = w2s(pending.x2, pending.y2); ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke(); label(`${fmt(Math.hypot(pending.x2 - pending.x1, pending.y2 - pending.y1), 2)} m`, c + 8, d, C['--accent']); }
    if (pending && LINK_KINDS.includes(pending.type)) {
      const A = byId(pending.a);
      if (A) {
        const tgt = isBody(hoverObj) && hoverObj !== A ? [hoverObj.st.x, hoverObj.st.y] : snapPt(mouseW);
        const [a, b] = w2s(A.st.x, A.st.y), [c, d] = w2s(tgt[0], tgt[1]);
        ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
        ctx.setLineDash([]); handleDot(a, b, C['--accent'], 4);
        if (!isBody(hoverObj)) { ctx.fillStyle = C['--accent']; ctx.fillRect(c - 6, d - 2, 12, 4); }
        label(`${fmt(Math.hypot(tgt[0] - A.st.x, tgt[1] - A.st.y), 2)} m${isBody(hoverObj) && hoverObj !== A ? ' → ' + hoverObj.name : ' · 固定点'}`, c + 10, d - 12, C['--accent']);
      }
    }
    ctx.restore();
  }
  updateBadge();
}

/* =====================================================================
 *  交互
 * ===================================================================== */
let tool = 'select', selId = null, selVec = null, drag = null, mouseW = [0, 0];
let mouseIn = false, hoverObj = null, snapOff = false;
const touchPoints = new Map();
let pinch = null;
function selected() { return selId != null ? byId(selId) : null; }
function syncListSelection() {
  document.querySelectorAll('.objlist li').forEach(li => {
    if (li.classList.contains('vec-item')) {
      const on = selVec && li.dataset.vecBody === String(selVec.bodyId) && li.dataset.vecKey === selVec.key;
      li.classList.toggle('sel', !!on);
    } else {
      const lid = +li.dataset.id;
      li.classList.toggle('sel', !selVec && lid === selId);
    }
  });
}
function select(id, opts = {}) {
  const prev = selId;
  const prevVec = selVec ? selVec.bodyId + '\0' + selVec.key : '';
  if (!opts.keepVec) selVec = null;
  selId = id;
  const vecKey = selVec ? selVec.bodyId + '\0' + selVec.key : '';
  syncListSelection();
  if ((prev !== id || prevVec !== vecKey) && !opts.deferPanel) buildPanel();
  else if (!opts.deferPanel && opts.forcePanel) buildPanel();
  draw();
}
function selectVector(bodyId, key, opts = {}) {
  const prevVec = selVec ? selVec.bodyId + '\0' + selVec.key : '';
  const nextVec = bodyId + '\0' + key;
  selVec = { bodyId, key };
  const changed = selId !== bodyId || prevVec !== nextVec;
  select(bodyId, Object.assign({ keepVec: true, forcePanel: changed }, opts));
}
const TOOL_HINT = {
  select: '',
  pan: '拖动画布平移 · 滚轮缩放', ball: '点击画布放置小球（按住 Shift 可连续放置）',
  particle: '点击画布放置带电粒子（按住 Shift 可连续放置）', box: '点击画布放置滑块（按住 Shift 可连续放置）',
  ground: '点击画布确定地面高度', wall: '按住拖动画出挡板/极板（线段）', incline: '点击放置斜面（点击处为底角顶点）',
  pointCharge: '点击画布放置点电荷（右侧可改 q）', efield: '拖动画出电场矩形区域；单击 = 全空间电场', bfield_in: '拖动画出磁场 ⊗ 区域；单击 = 全空间',
  bfield_out: '拖动画出磁场 ⊙ 区域；单击 = 全空间',
  spring: '弹簧：先点物体 A，再点物体 B（或空白处作为固定点）',
  rope: '轻绳：只拉不压。先点物体 A，再点物体 B（或空白固定点）',
  rod: '轻杆：不可伸缩。先点物体 A，再点物体 B（或空白固定点）',
};
function setTool(t) {
  tool = t; pending = null;
  document.querySelectorAll('[data-tool]').forEach(b => b.classList.toggle('active', b.dataset.tool === t));
  const hint = $('#toolhint');
  if (hint) hint.innerHTML = TOOL_HINT[t] ? `${escapeHtml(TOOL_HINT[t])}${t !== 'select' && t !== 'pan' ? ' <span class="th-k">Alt 关闭吸附 · Esc 取消</span>' : ''}` : '';
  hoverObj = null;
  cv.style.cursor = t === 'pan' ? 'grab' : t === 'select' ? 'default' : 'crosshair';
  if (leftTab === 'add') refreshLeft();
  draw();
}
function snap(v) { const S = scene.settings; return S.snap && !snapOff ? Math.round(v / S.snapStep) * S.snapStep : v; }
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
  for (const o of objs) if (isLink(o)) { const e = linkEnds(o); if (!e) continue; const [a, b] = w2s(e.x1, e.y1), [c, d] = w2s(e.x2, e.y2); if (distSeg(sx, sy, a, b, c, d) < 6) return o; }
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

function touchPair() {
  if (touchPoints.size < 2) return null;
  return [...touchPoints.values()].slice(0, 2);
}
function pinchDistance(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}
function pinchMid(a, b) {
  return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
}
/** 第二根手指按下时，结束单指正在进行的拖拽，避免对象被双指手势带飞 */
function abortDragForPinch() {
  if (!drag) return;
  if (drag.type === 'create') {
    drag = null;
    pending = null;
    return;
  }
  if (drag.type === 'pan') {
    drag = null;
    cv.style.cursor = tool === 'pan' ? 'grab' : 'default';
    markDirty();
    return;
  }
  // 复用松手逻辑：未移动的操作回滚撤销栈，已移动的操作正常收尾
  finishDrag({});
}
function startPinchFromTouches() {
  const pair = touchPair();
  if (!pair || pinch) return;
  const [a, b] = pair;
  const dist = pinchDistance(a, b);
  if (dist < 12) return;
  abortDragForPinch();
  pending = null;
  const mid = pinchMid(a, b);
  pinch = { dist, scale: scene.view.scale, world: s2w(mid.x, mid.y) };
  cv.style.cursor = 'grabbing';
}
function updatePinch() {
  const pair = touchPair();
  if (!pair || !pinch) return;
  const [a, b] = pair;
  const dist = pinchDistance(a, b);
  if (dist < 1) return;
  const mid = pinchMid(a, b);
  const v = scene.view;
  const s = Math.min(1e6, Math.max(0.5, pinch.scale * dist / pinch.dist));
  v.scale = s;
  // 让双指中点下的世界坐标保持不动，同时支持双指平移
  v.cx = pinch.world[0] - (mid.x - W / 2) / s;
  v.cy = pinch.world[1] + (mid.y - H / 2) / s;
  draw(); markDirty();
}

cv.addEventListener('contextmenu', e => {
  e.preventDefault();
  const [sx, sy] = evPos(e);
  const hv = tool === 'select' ? hitVector(sx, sy) : null;
  if (hv) {
    selectVector(hv.b.id, hv.key);
    openVectorMenu(e.clientX, e.clientY, hv.b, hv.key);
    return;
  }
  const hb = hitBody(sx, sy);
  const ho = hb || hitOther(sx, sy);
  if (ho) select(ho.id);
  openContextMenu(e.clientX, e.clientY, ho || null, 'canvas');
});
cv.addEventListener('pointerdown', e => {
  // 右键不 capture，交给 contextmenu；中键/左键才拖拽
  if (e.button === 2) return;
  e.preventDefault();
  const [sx, sy] = evPos(e);
  if (e.pointerType === 'touch') {
    // 第三根及以后的手指忽略，保证始终是干净的双指手势
    if (pinch || touchPoints.size >= 2) return;
    touchPoints.set(e.pointerId, { x: sx, y: sy });
    try { cv.setPointerCapture(e.pointerId); } catch { /* ignore */ }
    if (touchPoints.size === 2) {
      startPinchFromTouches();
      return;
    }
  } else {
    try { cv.setPointerCapture(e.pointerId); } catch { /* ignore */ }
  }
  snapOff = !!e.altKey;
  const wp = s2w(sx, sy); mouseW = wp;
  if (e.button === 1 || tool === 'pan') {
    drag = { type: 'pan', sx, sy, cx: scene.view.cx, cy: scene.view.cy };
    cv.style.cursor = 'grabbing';
    return;
  }
  if (e.button !== 0) return;
  hideContextMenu();
  const sp = snapPt(wp);
  // ---- 添加工具 ----
  if (['ball', 'particle', 'box'].includes(tool)) {
    pushUndo('添加' + KIND_NAME[tool]); if (sim.t > 0) resetSim('添加物体，已重置到 t=0');
    const b = mkBody(tool, { x0: sp[0], y0: sp[1] }); scene.objects.push(b); initState(b);
    afterStructure(); select(b.id); if (!e.shiftKey) setTool('select'); return;
  }
  if (tool === 'pointCharge') {
    pushUndo('添加点电荷'); if (sim.t > 0) resetSim('添加点电荷，已重置');
    const q = 1e-6;
    const o = mkPointCharge(sp[0], sp[1], { q, color: q >= 0 ? '#c8553d' : '#4a78b0' });
    scene.objects.push(o); afterStructure(); select(o.id); if (!e.shiftKey) setTool('select'); return;
  }
  if (tool === 'ground') { pushUndo('添加地面'); const g = mkGround(sp[1]); scene.objects.push(g); afterStructure(); select(g.id); setTool('select'); return; }
  if (tool === 'incline') { pushUndo('添加斜面'); const o = mkIncline(sp[0], sp[1]); scene.objects.push(o); afterStructure(); select(o.id); setTool('select'); return; }
  if (['efield', 'bfield_in', 'bfield_out'].includes(tool)) { pending = { type: 'rect', x1: sp[0], y1: sp[1], x2: sp[0], y2: sp[1], sx, sy }; drag = { type: 'create' }; return; }
  if (tool === 'wall') { pending = { type: 'line', x1: sp[0], y1: sp[1], x2: sp[0], y2: sp[1], sx, sy }; drag = { type: 'create' }; return; }
  if (LINK_KINDS.includes(tool)) {
    const hb = hitBody(sx, sy);
    const label = KIND_NAME[tool] || tool;
    if (!pending) {
      if (hb) { pending = { type: tool, a: hb.id }; draw(); }
      else toast(`请先点击一个物体作为${label}的一端`);
      return;
    }
    if (hb && hb.id === pending.a) { toast('不能连接到自身：请点另一个物体，或点空白处作为固定点'); return; }
    pushUndo('添加' + (KIND_NAME[tool] || '连接'));
    const A = byId(pending.a);
    const L0 = hb && hb.id !== pending.a
      ? +Math.hypot(hb.st.x - A.st.x, hb.st.y - A.st.y).toFixed(3)
      : +Math.hypot(sp[0] - A.st.x, sp[1] - A.st.y).toFixed(3);
    let s;
    const ends = hb && hb.id !== pending.a
      ? { a: pending.a, b: hb.id, L0 }
      : { a: pending.a, b: null, ax: sp[0], ay: sp[1], L0 };
    if (tool === 'rope') s = mkRope(ends.a, ends.b, ends);
    else if (tool === 'rod') s = mkRod(ends.a, ends.b, ends);
    else s = mkSpring(ends.a, ends.b, ends);
    scene.objects.push(s); pending = null; afterStructure(); select(s.id); setTool('select'); return;
  }
  // ---- 选择工具 ----
  const cur = selected();
  const hv = hitVector(sx, sy);
  if (hv) {
    selectVector(hv.b.id, hv.key, { deferPanel: true });
    const meta = vectorMeta(hv.key);
    if (meta.editable) {
      pushUndo('调整' + meta.name);
      drag = { type: hv.key === 'F' ? 'F' : hv.key === 'v' ? 'v' : 'v0', b: hv.b, moved: false };
    }
    return;
  }
  if (cur && (cur.kind === 'efield' || cur.kind === 'bfield') && !cur.all) {
    const hs = rectHandles(cur);
    for (let i = 0; i < 4; i++) if (Math.hypot(sx - hs[i][0], sy - hs[i][1]) < 9) { pushUndo('调整区域'); drag = { type: 'fieldCorner', o: cur, i, moved: false }; return; }
  }
  if (cur && cur.kind === 'wall') {
    const [a, b] = w2s(cur.x1, cur.y1), [c, d] = w2s(cur.x2, cur.y2);
    if (Math.hypot(sx - a, sy - b) < 9) { pushUndo('调整端点'); drag = { type: 'wallEnd', o: cur, end: 1, moved: false }; return; }
    if (Math.hypot(sx - c, sy - d) < 9) { pushUndo('调整端点'); drag = { type: 'wallEnd', o: cur, end: 2, moved: false }; return; }
  }
  const hb = hitBody(sx, sy);
  if (hb && sim.running && !hb.fixed) {
    // 运行中：直接抓住物体，松手时按鼠标速度甩出（不改初始条件）
    select(hb.id, { deferPanel: true });
    hb._grab = { x: hb.st.x, y: hb.st.y, vx: 0, vy: 0, t: performance.now() };
    drag = { type: 'grab', b: hb, ox: wp[0] - hb.st.x, oy: wp[1] - hb.st.y };
    cv.style.cursor = 'grabbing';
    return;
  }
  if (hb) {
    // 拖拽开始时不要同步重建右侧面板（MathQuill 很重），松手后再建
    select(hb.id, { deferPanel: true }); pushUndo(`移动「${hb.name}」`);
    drag = { type: 'body', b: hb, ox: wp[0] - hb.st.x, oy: wp[1] - hb.st.y, moved: false, sx, sy, wasT: sim.t };
    return;
  }
  const ho = hitOther(sx, sy);
  if (ho) {
    select(ho.id, { deferPanel: true }); pushUndo(`移动「${ho.name}」`);
    drag = { type: 'moveObj', o: ho, last: wp, snapLast: sp, moved: false }; return;
  }
  drag = { type: 'pan', sx, sy, cx: scene.view.cx, cy: scene.view.cy, click: true };
});
cv.addEventListener('pointermove', e => {
  const [sx, sy] = evPos(e);
  if (e.pointerType === 'touch') {
    if (!touchPoints.has(e.pointerId)) return;
    touchPoints.set(e.pointerId, { x: sx, y: sy });
    if (!pinch && touchPoints.size === 2) startPinchFromTouches();
    if (pinch) { e.preventDefault(); updatePinch(); return; }
  }
  snapOff = !!e.altKey;
  const wp = s2w(sx, sy); mouseW = wp; mouseIn = true;
  updateStatusMouse();
  if (!drag) { hoverCursor(sx, sy); if (tool !== 'select' && tool !== 'pan') requestDraw(); return; }
  e.preventDefault();
  const sp = snapPt(wp);
  if (drag.type === 'grab') {
    const g = drag.b._grab; if (!g) return;
    const now = performance.now(), dt = Math.max((now - g.t) / 1000, 1e-3);
    const nx = wp[0] - drag.ox, ny = wp[1] - drag.oy;
    const k = Math.min(1, dt / 0.06); // 平滑后的鼠标速度
    g.vx += ((nx - g.x) / dt * scene.settings.speed ** -1 - g.vx) * k;
    g.vy += ((ny - g.y) / dt * scene.settings.speed ** -1 - g.vy) * k;
    g.x = nx; g.y = ny; g.t = now;
    if (!sim.running) { drag.b.st.x = nx; drag.b.st.y = ny; }
    requestDraw(); return;
  }
  if (drag.type === 'pan') {
    const v = scene.view; v.cx = drag.cx - (sx - drag.sx) / v.scale; v.cy = drag.cy + (sy - drag.sy) / v.scale;
    if (Math.hypot(sx - drag.sx, sy - drag.sy) > 3) drag.click = false;
    draw(); return;
  }
  if (drag.type === 'create') { pending.x2 = sp[0]; pending.y2 = sp[1]; draw(); return; }
  if (drag.type === 'v0' || drag.type === 'v') {
    const b = drag.b, S = scene.settings;
    const origin = drag.type === 'v0' ? w2s(b.x0, b.y0) : w2s(b.st.x, b.st.y);
    let vx = (sx - origin[0]) / S.vPx, vy = -(sy - origin[1]) / S.vPx;
    if (e.shiftKey) { const v = Math.round(Math.hypot(vx, vy)), a = Math.round(Math.atan2(vy, vx) / DEG / 15) * 15 * DEG; vx = v * Math.cos(a); vy = v * Math.sin(a); }
    else { vx = Math.round(vx * 10) / 10; vy = Math.round(vy * 10) / 10; }
    vx = clean(vx); vy = clean(vy);
    drag.moved = true;
    if (drag.type === 'v0') clearExpr(b, ['vx0', 'vy0', 'v0', 'v0ang']);
    else clearExpr(b, ['vx', 'vy', 'v', 'vang']);
    if (drag.type === 'v0' || sim.t === 0) { b.vx0 = vx; b.vy0 = vy; }
    if (drag.type === 'v' && b.st) { b.st.vx = vx; b.st.vy = vy; }
    if (drag.type === 'v0') syncInitialLive(b);
    else { refreshInputs(); markDirty(); draw(); }
    return;
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
    drag.moved = true;
    clearExpr(b, ['F', 'Fang']);
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
    b.x0 = p[0]; b.y0 = p[1];
    // Shift：拖动物体时同步拉长/缩短相连轻杆；默认保持杆长，把位置投影到圆周上
    if (e.shiftKey) stretchRodsToBody(b);
    else projectBodyOntoRods(b);
    syncInitialLive(b); return;
  }
  if (drag.type === 'fieldCorner') {
    const o = drag.o, i = drag.i; let x1 = o.x, y1 = o.y, x2 = o.x + o.w, y2 = o.y + o.h;
    // 屏幕角点 0:左上 1:右上 2:右下 3:左下
    if (i === 0 || i === 3) x1 = sp[0]; else x2 = sp[0];
    if (i === 0 || i === 1) y2 = sp[1]; else y1 = sp[1];
    o.x = Math.min(x1, x2); o.w = Math.abs(x2 - x1); o.y = Math.min(y1, y2); o.h = Math.abs(y2 - y1);
    if (x2 < x1) drag.i = [1, 0, 3, 2][drag.i]; if (y2 < y1) drag.i = [3, 2, 1, 0][drag.i];
    drag.moved = true; liveEditLight(); return;
  }
  if (drag.type === 'wallEnd') { const o = drag.o; drag.moved = true; if (drag.end === 1) { o.x1 = sp[0]; o.y1 = sp[1]; } else { o.x2 = sp[0]; o.y2 = sp[1]; } liveEditLight(); return; }
  if (drag.type === 'moveObj') {
    const dx = sp[0] - drag.snapLast[0], dy = sp[1] - drag.snapLast[1]; if (!dx && !dy) return;
    drag.snapLast = sp; const o = drag.o;
    if (translateObject(o, dx, dy)) drag.moved = true;
    liveEditLight(); return;
  }
});
function finishDrag(e) {
  if (!drag) return;
  const d = drag; drag = null;
  try { if (e && e.pointerId != null) cv.releasePointerCapture(e.pointerId); } catch { /* ignore */ }
  if (d.type === 'grab') {
    const b = d.b, g = b._grab; b._grab = null;
    if (g && b.st) {
      // 静止松手不额外加速；超过 30 m/s 的甩动限速，避免数值爆炸
      const sp = Math.hypot(g.vx, g.vy), cap = 30;
      const f = sp > cap ? cap / sp : 1;
      b.st.vx = (performance.now() - g.t > 80) ? 0 : g.vx * f;
      b.st.vy = (performance.now() - g.t > 80) ? 0 : g.vy * f;
    }
    cv.style.cursor = 'default'; buildPanel(); requestDraw(); return;
  }
  if (d.type === 'pan') { cv.style.cursor = tool === 'pan' ? 'grab' : 'default'; if (d.click && tool === 'select') select(null); markDirty(); return; }
  if (d.type === 'create' && pending) {
    const p = pending; pending = null;
    const tiny = Math.abs(p.x2 - p.x1) < 1e-9 && Math.abs(p.y2 - p.y1) < 1e-9;
    pushUndo(p.type === 'rect' ? '添加' + (tool === 'efield' ? '电场' : '磁场') : '添加挡板');
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
  if ((d.type === 'body' || d.type === 'moveObj' || d.type === 'v0' || d.type === 'v' || d.type === 'F' || d.type === 'fieldCorner' || d.type === 'wallEnd') && !d.moved) cancelLastUndo();
  // 拖拽结束后再做一次完整同步（场线、面板）
  if (d.type === 'body' && d.moved && d.wasT > 0) toast('已修改初始位置，回到 t = 0');
  if ((d.type === 'body' || d.type === 'v0') && d.moved !== false) initChanged();
  else if (d.type === 'v' && d.moved && sim.t === 0) initChanged();
  else { markFieldsDirty(); markDirty(); draw(); }
  buildPanel();
  refreshLeft();
}
function endPointer(e) {
  if (e.pointerType === 'touch') {
    const wasPinching = !!pinch;
    touchPoints.delete(e.pointerId);
    try { cv.releasePointerCapture(e.pointerId); } catch { /* ignore */ }
    if (wasPinching) {
      pinch = null;
      // 双指松开一根后，剩下那根可以继续单指平移
      if (e.type !== 'pointercancel' && touchPoints.size === 1) {
        const p = [...touchPoints.values()][0];
        drag = { type: 'pan', sx: p.x, sy: p.y, cx: scene.view.cx, cy: scene.view.cy, click: false };
        cv.style.cursor = 'grabbing';
      } else {
        drag = null;
        cv.style.cursor = tool === 'pan' ? 'grab' : 'default';
      }
      markDirty();
      return;
    }
  }
  finishDrag(e);
}
cv.addEventListener('pointerup', endPointer);
cv.addEventListener('pointercancel', endPointer);
cv.addEventListener('gesturestart', e => e.preventDefault());
function resetTouchGesture() {
  touchPoints.clear();
  pinch = null;
}
window.addEventListener('blur', resetTouchGesture);
document.addEventListener('visibilitychange', () => { if (document.hidden) resetTouchGesture(); });
cv.addEventListener('wheel', e => {
  e.preventDefault();
  const v = scene.view;
  const unit = e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? H : 1;
  const dx = e.deltaX * unit, dy = e.deltaY * unit;
  // 触控板双指滑动（有横向分量、非捏合）→ 平移；鼠标滚轮或捏合（ctrlKey）→ 以光标为中心缩放
  const trackpadPan = !e.ctrlKey && e.deltaMode === 0 && (Math.abs(dx) > 0.5 || (!Number.isInteger(dy) && Math.abs(dy) < 40));
  if (trackpadPan) {
    v.cx += dx / v.scale; v.cy -= dy / v.scale;
  } else {
    const [sx, sy] = evPos(e), before = s2w(sx, sy);
    const k = e.ctrlKey ? 0.01 : 0.0015;
    v.scale = Math.min(1e6, Math.max(0.5, v.scale * Math.exp(-dy * k)));
    const after = s2w(sx, sy); v.cx += before[0] - after[0]; v.cy += before[1] - after[1];
  }
  markFieldsDirty(); requestDraw(); markDirty();
}, { passive: false });
cv.addEventListener('pointerenter', () => { mouseIn = true; });
cv.addEventListener('pointerleave', () => { mouseIn = false; if (hoverObj) { hoverObj = null; } requestDraw(); });
function hoverCursor(sx, sy) {
  let h = null;
  if (tool === 'select') {
    const hv = hitVector(sx, sy);
    if (hv) { cv.style.cursor = 'pointer'; h = null; }
    else {
      h = hitBody(sx, sy) || hitOther(sx, sy);
      cv.style.cursor = h ? (sim.running && isBody(h) && !h.fixed ? 'grab' : 'move') : 'default';
    }
  } else if (LINK_KINDS.includes(tool)) {
    h = hitBody(sx, sy);
    cv.style.cursor = h ? 'pointer' : (pending ? 'crosshair' : 'not-allowed');
  }
  if (h !== hoverObj) { hoverObj = h; requestDraw(); }
}
function clean(v) { return +(+v).toFixed(6); }
/** 初始条件被修改：回到 t=0 */
function initChanged() {
  const wasRunning = sim.t > 0 || sim.running;
  resetSim(wasRunning ? '初始条件已修改，已重置到 t=0' : null);
  markFieldsDirty(); refreshInputs(); markDirty(); draw();
}
/** 拖拽过程中的轻量同步：不重建场线/不 toast，避免拖一下就卡死或中断 */
function syncInitialLive(b) {
  if (sim.running || sim.t > 0) {
    sim.t = 0; sim.running = false; updatePlayBtn();
  }
  if (b) {
    if (!b.st) initState(b);
    else {
      b.st.x = b.x0; b.st.y = b.y0; b.st.vx = b.vx0; b.st.vy = b.vy0; b.st.ang = b.ang0 * DEG;
      b.st.trail = [[b.x0, b.y0]]; b.st.hist = []; pushHist(b);
    }
  } else {
    bodies().forEach(bb => {
      if (!bb.st) initState(bb);
      else { bb.st.x = bb.x0; bb.st.y = bb.y0; bb.st.vx = bb.vx0; bb.st.vy = bb.vy0; }
    });
  }
  refreshInputs(); markDirty(); draw();
}
/** 轻杆：编辑初态时把物体投影到定长圆周（保持杆不变形） */
function projectBodyOntoRods(b) {
  for (let iter = 0; iter < 4; iter++) {
    for (const s of scene.objects) {
      if (s.kind !== 'rod') continue;
      let ox, oy, mine;
      if (s.a === b.id) {
        const other = s.b != null ? byId(s.b) : null;
        ox = other ? other.x0 : s.ax; oy = other ? other.y0 : s.ay; mine = true;
      } else if (s.b === b.id) {
        const other = byId(s.a); if (!other) continue;
        ox = other.x0; oy = other.y0; mine = true;
      } else continue;
      if (!mine) continue;
      const dx = b.x0 - ox, dy = b.y0 - oy;
      const L = Math.hypot(dx, dy) || 1e-9;
      const f = s.L0 / L;
      b.x0 = clean(ox + dx * f);
      b.y0 = clean(oy + dy * f);
    }
  }
}
/** Shift 拖：用当前距离更新相连轻杆长度 */
function stretchRodsToBody(b) {
  for (const s of scene.objects) {
    if (s.kind !== 'rod') continue;
    let ox, oy;
    if (s.a === b.id) {
      const other = s.b != null ? byId(s.b) : null;
      ox = other ? other.x0 : s.ax; oy = other ? other.y0 : s.ay;
    } else if (s.b === b.id) {
      const other = byId(s.a); if (!other) continue;
      ox = other.x0; oy = other.y0;
    } else continue;
    s.L0 = clean(Math.hypot(b.x0 - ox, b.y0 - oy));
  }
}
/** 非初始参数修改：实时生效 */
function liveEdit() { markFieldsDirty(); refreshInputs(); markDirty(); draw(); }
/** 拖拽场/墙等时的轻量刷新 */
function liveEditLight() { refreshInputs(); markDirty(); draw(); }
function afterStructure() { ensureStates(); markFieldsDirty(); refreshLeft(); markDirty(); draw(); }

/* ---------------- 键盘 ---------------- */
const TOOL_KEYS = { 1: 'ball', 2: 'particle', 3: 'box', 4: 'ground', 5: 'wall', 6: 'incline', 7: 'spring', 8: 'rope', 9: 'rod' };
/** 平移一个对象（不含连接件两端都连着物体的情况） */
function translateObject(o, dx, dy) {
  if (isBody(o)) { o.x0 = clean(o.x0 + dx); o.y0 = clean(o.y0 + dy); projectBodyOntoRods(o); return true; }
  if (o.kind === 'ground') { o.y = clean(o.y + dy); return true; }
  if (o.kind === 'wall') { o.x1 = clean(o.x1 + dx); o.x2 = clean(o.x2 + dx); o.y1 = clean(o.y1 + dy); o.y2 = clean(o.y2 + dy); return true; }
  if (isLink(o)) { if (o.b != null) return false; o.ax = clean(o.ax + dx); o.ay = clean(o.ay + dy); return true; }
  o.x = clean(o.x + dx); o.y = clean(o.y + dy); return true;
}
let lastNudge = 0;
function nudgeSel(dx, dy) {
  const o = selected(); if (!o) return false;
  const now = performance.now();
  if (now - lastNudge > 700) pushUndo(`移动「${o.name}」`);   // 连续按键合并为一步撤销
  lastNudge = now;
  if (!translateObject(o, dx, dy)) { toast('两端都连着物体的连接件不能单独移动'); return true; }
  if (isBody(o)) initChanged(); else liveEdit();
  updateStatus(true);
  return true;
}
function zoomAt(f, sx = W / 2, sy = H / 2) {
  const v = scene.view, before = s2w(sx, sy);
  v.scale = Math.min(1e6, Math.max(0.5, v.scale * f));
  const after = s2w(sx, sy); v.cx += before[0] - after[0]; v.cy += before[1] - after[1];
  markFieldsDirty(); requestDraw(); markDirty();
}
document.addEventListener('keydown', e => {
  const ae = document.activeElement, tag = (ae && ae.tagName) || '';
  const isRange = tag === 'INPUT' && (ae.type === 'range' || ae.type === 'checkbox');
  const typing = !isRange && (['INPUT', 'SELECT', 'TEXTAREA'].includes(tag) || (ae && ae.isContentEditable) || (ae && ae.closest && ae.closest('.mq-editable-field')));
  if (typing) { if (e.key === 'Escape') ae.blur(); return; }
  if (isRange && (e.key.startsWith('Arrow') || e.key === 'Home' || e.key === 'End')) return; // 滑块自身处理方向键
  if (isRange && e.key === ' ') ae.blur();
  const k = e.key.toLowerCase(), mod = e.ctrlKey || e.metaKey;
  if (mod && k === 'z') { e.preventDefault(); e.shiftKey ? redo() : undo(); return; }
  if (mod && k === 'y') { e.preventDefault(); redo(); return; }
  if (mod && k === 'd') { e.preventDefault(); duplicateSel(); return; }
  if (mod && k === 'c') { if (selected()) { e.preventDefault(); copySel(); } return; }
  if (mod && k === 'v') { e.preventDefault(); pasteClip(); return; }
  if (mod && k === 's') { e.preventDefault(); $('#bSave')?.click(); return; }
  if (mod && (k === '=' || k === '+')) { e.preventDefault(); zoomAt(1.25); return; }
  if (mod && k === '-') { e.preventDefault(); zoomAt(0.8); return; }
  if (mod || e.altKey) return;
  if (e.key === ' ') { e.preventDefault(); if (!e.repeat) togglePlay(); return; }
  if (k === 'delete' || k === 'backspace') { e.preventDefault(); deleteSel(); return; }
  if (k.startsWith('arrow')) {
    if (!selected() || selVec) return;
    e.preventDefault();
    const st = (scene.settings.snapStep || 0.1) * (e.shiftKey ? 10 : 1);
    nudgeSel(k === 'arrowleft' ? -st : k === 'arrowright' ? st : 0, k === 'arrowup' ? st : k === 'arrowdown' ? -st : 0);
    return;
  }
  if (k === 'escape') {
    if ($('#help').classList.contains('show')) { $('#help').classList.remove('show'); return; }
    $('#moreMenu')?.classList.remove('open');
    $('#viewFlags')?.classList.remove('show'); $('#bView')?.classList.remove('active');
    hideContextMenu();
    closeMobilePanels();
    if (pending) { pending = null; draw(); }
    if (tool !== 'select') setTool('select');
    else if (selVec) { selVec = null; syncListSelection(); buildPanel(); draw(); }
    else if (selId != null) select(null);
    return;
  }
  if (e.key === '?') { $('#help').classList.toggle('show'); return; }
  if (k === 'r') { resetSim('已重置到 t = 0'); draw(); updateReadouts(); return; }
  if (k === '.') { stepOnce(); return; }
  if (k === ',') { stepBack(); return; }
  if (k === '=' || k === '+') { zoomAt(1.25); return; }
  if (k === '-') { zoomAt(0.8); return; }
  if (k === 'v') { setTool('select'); return; }
  if (k === 'h') { setTool('pan'); return; }
  if (k === 'f') { fitView(); return; }
  if (TOOL_KEYS[e.key]) { setTool(TOOL_KEYS[e.key]); toast(`${KIND_NAME[TOOL_KEYS[e.key]]}：${TOOL_HINT[TOOL_KEYS[e.key]]}`); return; }
});
// 鼠标点过的按钮立刻失焦，避免随后按空格时按钮被再次“点击”
document.addEventListener('pointerup', e => {
  const b = e.target.closest && e.target.closest('button, select');
  if (b && b.tagName === 'BUTTON' && e.pointerType !== '') setTimeout(() => b.blur(), 0);
});

/* ---------------- 撤销 ---------------- */
const undoStack = [], redoStack = [];
/** 记录一步可撤销操作。label 用于提示「已撤销：xxx」。 */
function pushUndo(label = '修改参数') {
  undoStack.push({ s: serialize(), label, redo: redoStack.splice(0) });
  if (undoStack.length > 120) undoStack.shift();
  syncHistoryButtons();
}
/** 操作最终没有产生改动（例如点了一下没拖动）：撤回刚压入的记录，并恢复重做栈。 */
function cancelLastUndo() {
  const it = undoStack.pop();
  if (it && it.redo) redoStack.push(...it.redo);
  syncHistoryButtons();
}
function restoreSnapshot(json) {
  const keepSel = selId, keepVec = selVec;
  loadScene(JSON.parse(json), true);
  if (keepSel != null && byId(keepSel)) { selId = keepSel; selVec = keepVec && byId(keepVec.bodyId) ? keepVec : null; syncListSelection(); buildPanel(); draw(); }
}
function undo() {
  const it = undoStack.pop();
  if (!it) { toast('没有可撤销的操作'); return; }
  redoStack.push({ s: serialize(), label: it.label });
  restoreSnapshot(it.s);
  toast(it.label ? `已撤销：${it.label}` : '已撤销');
  syncHistoryButtons();
}
function redo() {
  const it = redoStack.pop();
  if (!it) { toast('没有可重做的操作'); return; }
  undoStack.push({ s: serialize(), label: it.label });
  restoreSnapshot(it.s);
  toast(it.label ? `已重做：${it.label}` : '已重做');
  syncHistoryButtons();
}
function syncHistoryButtons() {
  const u = $('#bUndo'), r = $('#bRedo');
  if (u) { u.disabled = !undoStack.length; u.title = undoStack.length ? `撤销${undoStack.at(-1).label ? '：' + undoStack.at(-1).label : ''} (Ctrl+Z)` : '没有可撤销的操作'; }
  if (r) { r.disabled = !redoStack.length; r.title = redoStack.length ? `重做${redoStack.at(-1).label ? '：' + redoStack.at(-1).label : ''} (Ctrl+Shift+Z)` : '没有可重做的操作'; }
}

/* ---------------- 复制/删除 ---------------- */
function clearVectorValue(b, key) {
  if (key === 'v0') {
    b.vx0 = 0; b.vy0 = 0;
    clearExpr(b, ['vx0', 'vy0', 'v0', 'v0ang']);
    if (b.st && sim.t === 0) { b.st.vx = 0; b.st.vy = 0; }
    initChanged();
    return '已清除初速度';
  }
  if (key === 'v') {
    clearExpr(b, ['vx', 'vy', 'v', 'vang']);
    if (sim.t === 0) { b.vx0 = 0; b.vy0 = 0; clearExpr(b, ['vx0', 'vy0', 'v0', 'v0ang']); initChanged(); }
    else if (b.st) { b.st.vx = 0; b.st.vy = 0; draw(); }
    return '已清除速度';
  }
  if (key === 'F') {
    b.F = 0; b.Fang = 0; b.Fmode = 'const'; b.Fexpr = ''; b.FangExpr = '0'; b._Ferr = '';
    clearExpr(b, ['F', 'Fang']);
    liveEdit();
    return '已清除外力';
  }
  return '';
}
function deleteSel() {
  if (selVec) {
    const b = byId(selVec.bodyId);
    const meta = vectorMeta(selVec.key);
    if (!b) { selVec = null; return; }
    if (!meta.editable) { toast('这是计算得到的力，不能删除'); return; }
    pushUndo('清除' + meta.name);
    const msg = clearVectorValue(b, selVec.key);
    buildPanel(); refreshLeft(); draw();
    toast(msg || '已清除');
    return;
  }
  const s = selected(); if (!s) return; pushUndo(`删除「${s.name}」`);
  const links = scene.objects.filter(o => isLink(o) && (o.a === s.id || o.b === s.id)).length;
  scene.objects = scene.objects.filter(o => o !== s && !(isLink(o) && (o.a === s.id || o.b === s.id)));
  selId = null; selVec = null; afterStructure(); buildPanel();
  toast(`已删除「${s.name}」${links ? `及 ${links} 个连接` : ''}`, { action: UNDO_ACTION });
}
function uniqueName(base) {
  const names = new Set(scene.objects.map(o => o.name));
  if (!names.has(base)) return base;
  for (let i = 2; ; i++) if (!names.has(`${base} ${i}`)) return `${base} ${i}`;
}
/** 克隆对象（连接件不单独复制）。dx/dy 为相对原位置的平移。 */
function cloneObject(src, dx, dy) {
  const c = JSON.parse(JSON.stringify(Object.assign({}, src, { st: null, _exprErr: null, _hold: null })));
  c.id = nextId++; c.name = uniqueName(String(src.name).replace(/ 副本( \d+)?$/, '') + ' 副本');
  if (isBody(c)) { c.x0 = clean(c.x0 + dx); c.y0 = clean(c.y0 + dy); }
  else if (c.kind === 'wall') { c.x1 = clean(c.x1 + dx); c.x2 = clean(c.x2 + dx); c.y1 = clean(c.y1 + dy); c.y2 = clean(c.y2 + dy); }
  else if (c.kind === 'ground') c.y = clean(c.y + dy);
  else { c.x = clean(c.x + dx); c.y = clean(c.y + dy); }
  return c;
}
function insertClone(src, dx, dy, label) {
  if (isLink(src)) { toast('连接件请用「添加」工具重新连接两端'); return; }
  pushUndo(label);
  const c = cloneObject(src, dx, dy);
  scene.objects.push(c);
  if (isBody(c)) { if (sim.t > 0) resetSim(); initState(c); }
  afterStructure(); select(c.id);
  toast(`已${label}「${c.name}」`, { action: UNDO_ACTION });
}
function duplicateSel() {
  const s = selected(); if (!s) return;
  const off = Math.max(scene.settings.snapStep || 0.1, 0.5);
  insertClone(s, off, s.kind === 'ground' ? off : -off, '复制');
}
let clipboard = null;
function copySel() {
  const s = selected(); if (!s) return;
  if (isLink(s)) { toast('连接件不能单独复制'); return; }
  clipboard = JSON.parse(JSON.stringify(Object.assign({}, s, { st: null })));
  toast(`已复制「${s.name}」· Ctrl+V 粘贴到鼠标处`);
}
function pasteClip() {
  if (!clipboard) { toast('剪贴板是空的'); return; }
  const c = clipboard;
  const ox = isBody(c) ? c.x0 : c.kind === 'wall' ? (c.x1 + c.x2) / 2 : c.kind === 'ground' ? 0 : c.x;
  const oy = isBody(c) ? c.y0 : c.kind === 'wall' ? (c.y1 + c.y2) / 2 : c.y;
  const p = mouseIn ? snapPt(mouseW) : [ox + 0.5, oy - 0.5];
  insertClone(c, p[0] - ox, p[1] - oy, '粘贴');
}

/* ---------------- 播放控制 ---------------- */
function updatePlayBtn() { const b = $('#bPlay'); if (b) b.innerHTML = `<svg class="ic"><use href="#i-${sim.running ? 'pause' : 'play'}"/></svg>${sim.running ? '暂停' : '播放'}`; }
function togglePlay() { ensureStates(); sim.running = !sim.running; updatePlayBtn(); lastTs = 0; draw(); updateReadouts(); }
function stepOnce() {
  sim.running = false; updatePlayBtn();
  if (isScrubbed()) restoreFrame(sim.frameIdx + 1);   // 回看中：先沿记录前进
  else advance(1 / 60 * scene.settings.speed);
  afterFrame();
}
function stepBack() {
  sim.running = false; updatePlayBtn();
  if (sim.frameIdx > 0) restoreFrame(sim.frameIdx - 1); else toast('已在记录的起点');
  afterFrame();
}
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
  updateTimeline();
  $('#tbadge').innerHTML = `t = <b>${fmt(sim.t, 3)}</b> s &nbsp;<span class="${sim.running ? 'run' : 'pause'}">${sim.running ? '● 运行中' : sim.t === 0 ? '○ 初始' : '‖ 暂停'}</span>${scene.settings.speed !== 1 ? ` &nbsp;${scene.settings.speed}×` : ''}`;
}
function fitView() {
  const pts = [];
  for (const o of scene.objects) {
    if (isBody(o)) { pts.push([o.st ? o.st.x : o.x0, o.st ? o.st.y : o.y0], [o.x0, o.y0]); }
    else if (o.kind === 'wall') pts.push([o.x1, o.y1], [o.x2, o.y2]);
    else if (o.kind === 'incline') pts.push(...inclinePts(o));
    else if ((o.kind === 'efield' || o.kind === 'bfield') && !o.all) pts.push([o.x, o.y], [o.x + o.w, o.y + o.h]);
    else if (o.kind === 'ground') pts.push([0, o.y]);
    else if (o.kind === 'pointCharge') pts.push([o.x, o.y]);
    else if (isLink(o) && o.b == null) {
      // 摆：把以悬点为圆心、可达半径内的范围都纳入视野
      const chain = scene.objects.filter(l => (l.kind === 'rod' || l.kind === 'rope') && l.b != null).reduce((a, l) => a + (l.L0 || 0), 0);
      const R = o.kind === 'spring' ? 0 : (o.L0 || 0) + chain;
      pts.push([o.ax - R, o.ay - R], [o.ax + R, o.ay + Math.min(R, 0.3)]);
    }
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
  const owner = o.slot && o.slot[0];
  const key = o.slot && o.slot[1];
  const formula = owner && owner.expr && owner.expr[key] != null && String(owner.expr[key]).trim() !== ''
    ? String(owner.expr[key]) : '';
  const r = el('div', { class: 'row' + (formula ? ' is-expr' : '') });
  const labEl = el('span', { class: 'lab' }, lab);
  r.appendChild(labEl);
  if (o.title) r.title = o.title;
  let pushed = false;
  let input;
  if (owner && formula) {
    input = el('input', { type: 'text', class: 'expr-input', spellcheck: 'false', autocomplete: 'off', title: FORCE_EXPR_HELP });
    input.placeholder = '如 小球2.vx';
    input.value = formula;
    input.addEventListener('focus', () => { pushed = false; });
    input.addEventListener('input', () => {
      if (!owner.expr) owner.expr = {};
      if (!pushed) { pushUndo(); pushed = true; }
      const text = input.value;
      if (!String(text).trim()) delete owner.expr[key];
      else owner.expr[key] = text;
      resolveExprs();
      const err = owner._exprErr && owner._exprErr[key];
      if (errEl) errEl.textContent = err || '';
      if (valEl) valEl.textContent = err ? '' : `= ${fmt(Number(get()), 3)} ${unit}`.trim();
      o.init ? initChanged() : liveEdit();
      if (o.after) o.after();
    });
  } else {
    input = el('input', { type: 'number', step: o.step || 'any' });
    if (o.min != null) input.min = o.min;
    input.value = fmtIn(get());
    input.addEventListener('focus', () => pushed = false);
    input.addEventListener('input', () => {
      let v = parseFloat(input.value); if (!isFinite(v)) return; if (o.min != null && v < o.min) return;
      if (!pushed) { pushUndo(); pushed = true; }
      if (owner && owner.expr) delete owner.expr[key];
      set(v); o.init ? initChanged() : liveEdit(); if (o.after) o.after();
    });
    input.addEventListener('change', () => { input.value = fmtIn(get()); });
    attachScrub(labEl, input, get, o);
  }
  input.addEventListener('keydown', e => { if (e.key === 'Enter') { e.preventDefault(); input.blur(); } });
  r.appendChild(input);
  if (owner) {
    const fx = el('button', {
      type: 'button',
      class: 'expr-toggle' + (formula ? ' on' : ''),
      title: formula ? '改回数字' : '改成公式，可引用其他物体',
    }, 'ƒ');
    fx.addEventListener('click', () => {
      pushUndo();
      if (!owner.expr) owner.expr = {};
      if (formula) delete owner.expr[key];
      else owner.expr[key] = fmtIn(get()) || '0';
      resolveExprs();
      buildPanel();
    });
    r.appendChild(fx);
  }
  r.appendChild(el('span', { class: 'unit' }, unit));
  parent.appendChild(r);
  let errEl = null;
  let valEl = null;
  if (formula) {
    valEl = el('div', { class: 'hint expr-val' }, `= ${fmt(Number(get()), 3)} ${unit}`.trim());
    errEl = el('div', { class: 'hint expr-err' }, (owner._exprErr && owner._exprErr[key]) || '');
    parent.appendChild(valEl);
    parent.appendChild(errEl);
  }
  binds.push({ el: input, get: () => (formula ? (owner.expr && owner.expr[key]) ?? '' : get()) });
  return input;
}
/**
 * 拖动标签改数值：左右拖动，Shift ×10、Alt ×0.1。
 * 步长取当前数值的数量级的 1/100，避免小量（如 1e-6 C）拖一下就飞掉。
 */
function attachScrub(labEl, input, get, o) {
  labEl.classList.add('scrub');
  labEl.title = (o.title ? o.title + '\n' : '') + '左右拖动可调节数值（Shift 加速，Alt 微调）';
  labEl.addEventListener('pointerdown', e => {
    if (e.button !== 0) return;
    e.preventDefault();
    const v0 = Number(get()) || 0;
    const fixedStep = o.step && o.step !== 'any' ? +o.step : 0;
    const mag = Math.abs(v0) > 1e-12 ? Math.pow(10, Math.floor(Math.log10(Math.abs(v0))) - 2) : 0.01;
    const base = fixedStep ? Math.min(fixedStep, Math.max(mag, fixedStep / 10)) : mag;
    const x0 = e.clientX;
    let moved = false;
    input.dispatchEvent(new Event('focus'));   // 让这次拖动成为独立的一步撤销
    try { labEl.setPointerCapture(e.pointerId); } catch { }
    document.body.classList.add('scrubbing');
    const move = ev => {
      const dx = ev.clientX - x0;
      if (!moved && Math.abs(dx) < 3) return;
      moved = true;
      const k = ev.shiftKey ? 10 : ev.altKey ? 0.1 : 1;
      let v = v0 + dx * base * k;
      if (o.min != null) v = Math.max(o.min, v);
      const dec = Math.max(0, -Math.floor(Math.log10(base * k)));
      v = +v.toFixed(Math.min(dec, 12));
      input.value = fmtIn(v);
      input.dispatchEvent(new Event('input'));
    };
    const up = () => {
      labEl.removeEventListener('pointermove', move);
      labEl.removeEventListener('pointerup', up);
      labEl.removeEventListener('pointercancel', up);
      document.body.classList.remove('scrubbing');
      if (!moved) input.focus(); else input.dispatchEvent(new Event('change'));
    };
    labEl.addEventListener('pointermove', move);
    labEl.addEventListener('pointerup', up);
    labEl.addEventListener('pointercancel', up);
  });
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
  const r = el('div', { class: 'row' + (o.className ? ' ' + o.className : '') });
  r.appendChild(el('span', { class: 'lab' }, lab));
  const s = el('select', { class: 'force-select' });
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
  const bar = el('div', { class: attrs.className || 'tabs', role: 'tablist' });
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
  if (isBody(o) && selVec && selVec.bodyId === o.id) return buildVectorInspector(P, o, selVec.key);

  const head = el('div', { class: 'insp-head' });
  const nm = el('input', { type: 'text', value: o.name, 'aria-label': '名称', title: '点击重命名', spellcheck: 'false' });
  let nmPushed = false;
  nm.addEventListener('focus', () => { nmPushed = false; });
  nm.addEventListener('input', () => { if (!nmPushed) { pushUndo('重命名'); nmPushed = true; } o.name = nm.value; refreshLeft(); draw(); markDirty(); });
  nm.addEventListener('keydown', e => { if (e.key === 'Enter' || e.key === 'Escape') { e.preventDefault(); nm.blur(); } });
  nm.addEventListener('blur', () => {
    const v = nm.value.trim();
    if (!v) { o.name = uniqueName(KIND_NAME[o.kind] || '对象'); nm.value = o.name; refreshLeft(); draw(); markDirty(); }
    else if (v !== nm.value) { o.name = v; nm.value = v; }
  });
  head.appendChild(nm);
  if (o.color != null) {
    const c = el('input', { type: 'color', value: o.color, title: '颜色' });
    let cPushed = false;
    c.addEventListener('focus', () => { cPushed = false; });
    c.addEventListener('input', () => { if (!cPushed) { pushUndo('改颜色'); cPushed = true; } o.color = c.value; refreshLeft(); draw(); markDirty(); });
    head.appendChild(c);
  }
  const acts = el('div', { class: 'insp-actions' });
  if (!isLink(o)) acts.appendChild(el('button', { type: 'button', title: '复制 (Ctrl+D)', 'aria-label': '复制', onclick: duplicateSel }, '<svg class="ic"><use href="#i-copy"/></svg>'));
  acts.appendChild(el('button', { type: 'button', class: 'danger', title: '删除 (Delete)', 'aria-label': '删除', onclick: deleteSel }, '<svg class="ic"><use href="#i-trash"/></svg>'));
  head.appendChild(acts);
  P.appendChild(head);
  const kindExtra = o.kind === 'bfield' ? (o.dir < 0 ? ' · 向里' : ' · 向外') : '';
  P.appendChild(el('div', { class: 'insp-meta' }, KIND_NAME[o.kind] + kindExtra));

  if (isBody(o)) buildBodyPanel(P, o);
  else buildNonBodyPanel(P, o);
}

function buildVectorInspector(P, b, key) {
  const meta = vectorMeta(key);
  const head = el('div', { class: 'insp-head' });
  head.appendChild(el('div', { class: 'vec-title' }, meta.name));
  P.appendChild(head);
  const metaRow = el('div', { class: 'insp-meta vec-parent' });
  metaRow.appendChild(document.createTextNode('属于 '));
  const jump = el('button', { type: 'button', class: 'linkish' }, b.name);
  jump.addEventListener('click', () => { selVec = null; select(b.id, { forcePanel: true }); refreshLeft(); });
  metaRow.appendChild(jump);
  P.appendChild(metaRow);

  if (key === 'v0' || key === 'v') {
    const live = key === 'v';
    const g = vectorScreen(b, key);
    const s = section(P, live ? '瞬时速度' : '初速度', 'hl');
    const getVx = () => live && b.st && sim.t > 0 ? b.st.vx : b.vx0;
    const getVy = () => live && b.st && sim.t > 0 ? b.st.vy : b.vy0;
    const setComp = (axis, v) => {
      if (live && b.st && sim.t > 0) b.st[axis] = v;
      else { if (axis === 'vx') b.vx0 = v; else b.vy0 = v; }
    };
    const mag = () => Math.hypot(getVx(), getVy());
    const ang = () => (getVx() || getVy()) ? Math.atan2(getVy(), getVx()) / DEG : 0;
    const initFlag = !(live && sim.t > 0);
    const kMag = live && sim.t > 0 ? 'v' : 'v0';
    const kAng = live && sim.t > 0 ? 'vang' : 'v0ang';
    const kVx = live && sim.t > 0 ? 'vx' : 'vx0';
    const kVy = live && sim.t > 0 ? 'vy' : 'vy0';
    numRow(s, '速率', mag, v => {
      const a = ang() * DEG;
      setComp('vx', clean(v * Math.cos(a)));
      setComp('vy', clean(v * Math.sin(a)));
    }, 'm/s', { min: 0, init: initFlag, slot: [b, kMag] });
    numRow(s, '方向', ang, a => {
      const m = mag();
      setComp('vx', clean(m * Math.cos(a * DEG)));
      setComp('vy', clean(m * Math.sin(a * DEG)));
    }, '°', { init: initFlag, slot: [b, kAng] });
    numRow(s, 'vx', getVx, v => setComp('vx', v), 'm/s', { init: initFlag, slot: [b, kVx] });
    numRow(s, 'vy', getVy, v => setComp('vy', v), 'm/s', { init: initFlag, slot: [b, kVy] });
    s.appendChild(el('div', { class: 'hint' }, '点 ƒ 写成公式，可引用其他物体，例如 小球2.vx。播放中公式会每一步重算，并压过积分结果。'));
    if (g) s.appendChild(el('div', { class: 'hint' }, `当前 ${fmt(g.mag, 3)} ${meta.unit}`));
  } else if (key === 'F') {
    const wrap = el('div', { class: 'force-tab' });
    buildAppliedForcePanel(wrap, b);
    P.appendChild(wrap);
  } else {
    const g = vectorScreen(b, key);
    const s = section(P, '计算力', 'hl');
    if (g && g.part) {
      readRow(s, [['大小', 'r_vec_mag'], ['方向', 'r_vec_ang'], ['Fx', 'r_vec_x'], ['Fy', 'r_vec_y']]);
      const set = () => {
        const p = g.part;
        const mag = Math.hypot(p.fx, p.fy);
        setTxt('r_vec_mag', `${fmt(mag, 3)} N`);
        setTxt('r_vec_ang', `${fmt(Math.atan2(p.fy, p.fx) / DEG, 1)}°`);
        setTxt('r_vec_x', `${fmt(p.fx, 3)} N`);
        setTxt('r_vec_y', `${fmt(p.fy, 3)} N`);
      };
      set();
    }
    s.appendChild(el('div', { class: 'hint' }, '由当前受力计算得到，不能单独删除。改物体或场景后会跟着变。'));
  }

  const act = section(P, '操作');
  const br = el('div', { class: 'btnrow' });
  if (meta.editable) btn(br, '删除矢量', deleteSel, 'danger');
  btn(br, '打开所属物体', () => { selVec = null; select(b.id, { forcePanel: true }); refreshLeft(); });
  act.appendChild(br);
}

function buildNonBodyPanel(P, o) {
  const panels = el('div', { class: 'tab-panels' });
  const pProps = panelBox('props', 'props');

  if (o.kind === 'pointCharge') buildPointChargePanel(pProps, o);
  else if (o.kind === 'efield') buildEPanel(pProps, o);
  else if (o.kind === 'bfield') buildBPanel(pProps, o);
  else if (o.kind === 'ground') {
    const s = section(pProps, '几何'); numRow(s, '高度 y', () => o.y, v => o.y = v, 'm', { slot: [o, 'y'] }); surfOpts(s, o);
  } else if (o.kind === 'wall') {
    const s = section(pProps, '端点');
    numRow(s, '起点 x₁', () => o.x1, v => o.x1 = v, 'm', { slot: [o, 'x1'] });
    numRow(s, '起点 y₁', () => o.y1, v => o.y1 = v, 'm', { slot: [o, 'y1'] });
    numRow(s, '终点 x₂', () => o.x2, v => o.x2 = v, 'm', { slot: [o, 'x2'] });
    numRow(s, '终点 y₂', () => o.y2, v => o.y2 = v, 'm', { slot: [o, 'y2'] });
    s.appendChild(el('div', { class: 'hint' }, '可拖动两端控制点。'));
    surfOpts(s, o);
  } else if (o.kind === 'incline') {
    const s = section(pProps, '几何');
    numRow(s, '底角 x', () => o.x, v => o.x = v, 'm', { slot: [o, 'x'] });
    numRow(s, '底角 y', () => o.y, v => o.y = v, 'm', { slot: [o, 'y'] });
    numRow(s, '底边 L', () => o.L, v => o.L = v, 'm', { min: 0.01, slot: [o, 'L'] });
    numRow(s, '倾角 θ', () => o.ang, v => o.ang = Math.min(85, v), '°', { min: 0, slot: [o, 'ang'] });
    const br = el('div', { class: 'btnrow' });
    [30, 37, 45, 53, 60].forEach(a => btn(br, a + '°', () => { pushUndo(); o.ang = a; liveEdit(); }));
    btn(br, o.dir > 0 ? '朝右' : '朝左', () => { pushUndo(); o.dir *= -1; liveEdit(); buildPanel(); });
    s.appendChild(br);
    s.appendChild(el('div', { class: 'hint' }, `斜面长 ${fmt(o.L / Math.cos(o.ang * DEG), 3)} m，高 ${fmt(o.L * Math.tan(o.ang * DEG), 3)} m`));
    surfOpts(s, o);
    const s2 = section(pProps, '快捷');
    btn(s2, '在顶端放置滑块', () => placeOnIncline(o));
  } else if (isLink(o)) buildLinkPanel(pProps, o);

  panels.appendChild(pProps);
  P.appendChild(panels);
  syncDesmosKeypadForAnalysis(false);
}

function buildAppliedForcePanel(P, b) {
  ensureForceFields(b);
  const af = appliedForceOn(b);
  const mode = b.Fmode || 'const';

  const vis = section(P, '画布编辑', 'hl');
  chkRow(vis, '显示受力箭头', () => !!scene.settings.showForce, v => {
    scene.settings.showForce = v;
    syncToolbar();
    liveEdit();
  });
  vis.appendChild(el('div', { class: 'hint' }, '开启后拖紫色箭头末端设置 F；Shift 按 15° 吸附。'));

  const live = section(P, '当前外力');
  const card = el('div', { class: 'force-live-card' });
  card.appendChild(el('div', { class: 'force-live-mag', id: 'r_force_mag' }, `${fmt(af.F, 3)} N`));
  card.appendChild(el('div', { class: 'force-live-ang', id: 'r_force_ang' }, `${fmt(af.ang, 1)}°`));
  card.appendChild(el('div', { class: 'force-live-mode', id: 'r_force_mode' }, mode === 'expr' ? '表达式' : mode === 'aim' ? '指向目标' : '恒定'));
  card.appendChild(el('div', { class: 'force-live-err', id: 'r_force_err' }, b._Ferr || ''));
  live.appendChild(card);

  const f = section(P, '外力设置');
  const modeBar = el('div', { class: 'force-mode-chips', role: 'group', 'aria-label': '外力模式' });
  [
    ['const', '恒定'],
    ['expr', '表达式'],
    ['aim', '指向目标'],
  ].forEach(([id, lab]) => {
    const c = el('button', {
      type: 'button',
      class: 'force-chip' + (mode === id ? ' active' : ''),
      'data-mode': id,
    }, lab);
    c.addEventListener('click', () => {
      if (b.Fmode === id) return;
      pushUndo();
      b.Fmode = id;
      b._Ferr = '';
      if (id === 'expr' && !String(b.Fexpr || '').trim()) b.Fexpr = String(b.F || 0);
      liveEdit();
      buildPanel();
    });
    modeBar.appendChild(c);
  });
  f.appendChild(modeBar);

  if (mode === 'const') {
    numRow(f, '大小 F', () => b.F, v => b.F = v, 'N', { min: 0, slot: [b, 'F'] });
    numRow(f, '方向角', () => b.Fang, v => b.Fang = v, '°', { slot: [b, 'Fang'] });
    f.appendChild(el('div', { class: 'hint' }, '点 ƒ 后可写 小球2.vx、小球2.F 这类引用。'));
    const br = el('div', { class: 'btnrow force-quick-angs' });
    [[0, '→ 0°'], [90, '↑ 90°'], [180, '← 180°'], [-90, '↓ −90°']].forEach(([a, t]) =>
      btn(br, t, () => { pushUndo(); b.Fang = a; liveEdit(); buildPanel(); }));
    f.appendChild(br);
  } else if (mode === 'expr') {
    textRow(f, '大小 F(t,…)', () => b.Fexpr, v => { b.Fexpr = v; }, {
      placeholder: '如 小球2.v*2 或 10*sin(2*pi*t)', title: FORCE_EXPR_HELP,
    });
    textRow(f, '方向 θ(°)', () => b.FangExpr, v => { b.FangExpr = v; }, {
      placeholder: '如 90 或 atan2(1,0)*180/pi', title: FORCE_EXPR_HELP,
    });
    f.appendChild(el('div', { class: 'hint force-expr-help' }, FORCE_EXPR_HELP));
    const ex = el('div', { class: 'btnrow force-expr-presets' });
    [
      ['10', '0', '恒定 10N'],
      ['10*sin(2*pi*t)', '0', '简谐 F'],
      ['5', '90*t', '旋转力'],
      ['m*g', '90', '等同重力'],
    ].forEach(([Fe, ae, lab]) => {
      btn(ex, lab, () => {
        pushUndo();
        b.Fexpr = Fe; b.FangExpr = ae;
        liveEdit(); buildPanel();
      });
    });
    f.appendChild(ex);
  } else {
    textRow(f, '大小表达式', () => b.Fexpr, v => { b.Fexpr = v; }, {
      placeholder: '留空则用下方恒定值', title: FORCE_EXPR_HELP,
    });
    numRow(f, '大小 F（备用）', () => b.F, v => b.F = v, 'N', { min: 0, slot: [b, 'F'] });
    const targets = [['', '世界坐标']].concat(
      scene.objects
        .filter(o => o.id !== b.id && (isBody(o) || o.kind === 'pointCharge'))
        .map(o => [String(o.id), o.name]),
    );
    selectRow(f, '指向', () => (b.FaimId == null ? '' : String(b.FaimId)), v => {
      b.FaimId = v === '' ? null : +v;
    }, targets, { className: 'force-aim-select' });
    if (b.FaimId == null) {
      numRow(f, '目标 x', () => b.FaimX, v => b.FaimX = v, 'm', { slot: [b, 'FaimX'] });
      numRow(f, '目标 y', () => b.FaimY, v => b.FaimY = v, 'm', { slot: [b, 'FaimY'] });
    } else {
      const aim = resolveAimPoint(b);
      f.appendChild(el('div', { class: 'hint' }, `当前目标：${aim.label}`));
    }
    f.appendChild(el('div', { class: 'hint' }, '力始终指向目标；大小可用表达式随时间/状态变化。'));
  }

  const act = section(P, '操作');
  const br = el('div', { class: 'btnrow' });
  btn(br, '清零外力', () => {
    pushUndo();
    b.F = 0; b.Fang = 0; b.Fmode = 'const'; b.Fexpr = ''; b.FangExpr = '0'; b._Ferr = '';
    clearExpr(b, ['F', 'Fang']);
    liveEdit(); buildPanel();
  }, 'danger');
  numRow(act, '箭头比例', () => scene.settings.FPx || 12, v => { scene.settings.FPx = v; liveEdit(); }, 'px/N', { min: 0.0001, slot: [scene.settings, 'FPx'] });
  act.appendChild(br);
}

let chartMode = 'v';
function buildBodyPanel(P, b) {
  const tabs = [
    ['init', '初态'],
    ['props', '属性'],
    ['force', '受力'],
    ['live', '实时'],
    ['analysis', '分析'],
    ['chart', '图像'],
  ];
  if (!tabs.some(([id]) => id === propsTab)) propsTab = 'init';
  P.appendChild(makeTabs(tabs, propsTab, id => { propsTab = id; buildPanel(); }, { className: 'tabs tabs-scroll' }));
  const panels = el('div', { class: 'tab-panels' });

  const pInit = panelBox('init', propsTab);
  {
    const s = section(pInit, '初始条件 t = 0', 'hl');
    numRow(s, '位置 x₀', () => b.x0, v => b.x0 = v, 'm', { init: true, slot: [b, 'x0'] });
    numRow(s, '位置 y₀', () => b.y0, v => b.y0 = v, 'm', { init: true, slot: [b, 'y0'] });
    const vmag = () => Math.hypot(b.vx0, b.vy0);
    const vang = () => (b.vx0 || b.vy0) ? Math.atan2(b.vy0, b.vx0) / DEG : (b._lastAng || 0);
    numRow(s, '速率 v₀', vmag, v => { const a = vang() * DEG; b.vx0 = clean(v * Math.cos(a)); b.vy0 = clean(v * Math.sin(a)); }, 'm/s', { init: true, min: 0, slot: [b, 'v0'] });
    numRow(s, '方向 θ', vang, a => { b._lastAng = a; const v = vmag(); b.vx0 = clean(v * Math.cos(a * DEG)); b.vy0 = clean(v * Math.sin(a * DEG)); }, '°', { init: true, title: '相对 +x，逆时针为正', slot: [b, 'v0ang'] });
    const br = el('div', { class: 'btnrow' });
    [[0, '0°'], [45, '45°'], [90, '90°'], [135, '135°'], [180, '180°'], [-90, '-90°']].forEach(([a, t]) =>
      btn(br, t, () => { pushUndo(); b._lastAng = a; const v = vmag() || 1; b.vx0 = clean(v * Math.cos(a * DEG)); b.vy0 = clean(v * Math.sin(a * DEG)); initChanged(); }));
    btn(br, '清零', () => { pushUndo(); b.vx0 = 0; b.vy0 = 0; initChanged(); }, 'danger');
    s.appendChild(br);
    numRow(s, '分量 vx₀', () => b.vx0, v => b.vx0 = v, 'm/s', { init: true, slot: [b, 'vx0'] });
    numRow(s, '分量 vy₀', () => b.vy0, v => b.vy0 = v, 'm/s', { init: true, slot: [b, 'vy0'] });
    s.appendChild(el('div', { class: 'hint' }, '每个数旁边的 ƒ 可改成公式，例如 vx₀ = 小球2.vx。拖箭头会取消该公式。'));
  }

  const pProps = panelBox('props', propsTab);
  {
    const p = section(pProps, '物理量');
    numRow(p, '质量 m', () => b.m, v => b.m = v, 'kg', { min: 1e-40, title: '如 9.1e-31', slot: [b, 'm'] });
    numRow(p, '电荷 q', () => b.q, v => b.q = v, 'C', { title: '如 -1.6e-19', slot: [b, 'q'] });
    if (b.kind === 'box') {
      numRow(p, '宽 w', () => b.w, v => b.w = v, 'm', { min: 0.001, slot: [b, 'w'] });
      numRow(p, '高 h', () => b.h, v => b.h = v, 'm', { min: 0.001, slot: [b, 'h'] });
      numRow(p, '倾角', () => b.ang0, v => b.ang0 = v, '°', { init: true, slot: [b, 'ang0'] });
    } else numRow(p, '半径 r', () => b.r, v => b.r = v, 'm', { min: 0.0001, slot: [b, 'r'] });
    numRow(p, '摩擦 μ', () => b.mu, v => b.mu = v, '', { step: 0.05, min: 0, slot: [b, 'mu'] });
    numRow(p, '恢复系数 e', () => b.e, v => b.e = Math.min(1, v), '', { step: 0.1, min: 0, slot: [b, 'e'] });
    const o = section(pProps, '行为');
    chkRow(o, '固定', () => b.fixed, v => b.fixed = v, { init: true });
    chkRow(o, '显示轨迹', () => b.trail, v => b.trail = v);
    chkRow(o, '参与碰撞', () => b.collide, v => b.collide = v);
    const tb = el('div', { class: 'btnrow' });
    btn(tb, '清除轨迹', () => { if (b.st) b.st.trail = [[b.st.x, b.st.y]]; draw(); });
    o.appendChild(tb);
    o.appendChild(el('div', { class: 'hint' }, '外力请到「受力」标签页设置。'));
  }

  const pForce = panelBox('force', propsTab);
  pForce.classList.add('force-tab');
  buildAppliedForcePanel(pForce, b);

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
    [['v', 'v–t 速率'], ['vx', 'vx–t'], ['vy', 'vy–t'], ['a', 'a–t 加速度'], ['x', 'x–t'], ['y', 'y–t'], ['ek', 'Ek–t 动能'], ['xy', 'y–x 轨迹'], ['px', 'vx–x 相图'], ['py', 'vy–y 相图']].forEach(([v, t]) => sel.appendChild(el('option', { value: v }, t)));
    sel.value = chartMode;
    sel.addEventListener('change', () => { chartMode = sel.value; drawChart(); });
    const g = section(pCh, '图像', '', sel);
    g.appendChild(el('canvas', { id: 'chart' }));
  }

  [pInit, pProps, pForce, pLive, pAn, pCh].forEach(x => panels.appendChild(x));
  P.appendChild(panels);
  ensureDesmosKeypadDock(document.getElementById('right'));
  syncDesmosKeypadForAnalysis(propsTab === 'analysis');
  updateReadouts();
}

function buildPointChargePanel(P, o) {
  const s = section(P, '点电荷');
  numRow(s, '位置 x', () => o.x, v => o.x = v, 'm', { slot: [o, 'x'] });
  numRow(s, '位置 y', () => o.y, v => o.y = v, 'm', { slot: [o, 'y'] });
  numRow(s, '电荷 q', () => o.q, v => { o.q = v; o.color = v >= 0 ? '#c8553d' : '#4a78b0'; }, 'C', { title: '如 1e-6；负号表示负电', slot: [o, 'q'] });
  numRow(s, '软化半径', () => o.softRadius, v => o.softRadius = v, 'm', { min: 1e-4, slot: [o, 'softRadius'] });
  const br = el('div', { class: 'btnrow' });
  btn(br, '+1 μC', () => { pushUndo(); o.q = 1e-6; o.color = '#c8553d'; liveEdit(); buildPanel(); });
  btn(br, '−1 μC', () => { pushUndo(); o.q = -1e-6; o.color = '#4a78b0'; liveEdit(); buildPanel(); });
  btn(br, '取反', () => { pushUndo(); o.q = -o.q; o.color = o.q >= 0 ? '#c8553d' : '#4a78b0'; liveEdit(); buildPanel(); });
  s.appendChild(br);
  s.appendChild(el('div', { class: 'hint' }, `力律：${scene.settings.coulombLaw === 'coulomb2d' ? '二维/线电荷截面 (1/r)' : '三维点电荷 (1/r²)'}。电场线由 FieldSystem 积分绘制。`));
  const f = fieldAt(o.x + (o.softRadius || 0.08) * 2, o.y);
  s.appendChild(el('div', { class: 'hint' }, `旁侧抽样 |E| ≈ ${fmt(Math.hypot(f.Ex, f.Ey))} V/m`));
}
function buildEPanel(P, o) {
  const s = section(P, '匀强电场');
  numRow(s, '场强 E', () => o.E, v => o.E = v, 'V/m', { min: 0, slot: [o, 'E'] });
  numRow(s, '方向角', () => o.ang, v => o.ang = v, '°', { slot: [o, 'ang'] });
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
  numRow(s, '磁感应强度 B', () => o.B, v => o.B = v, 'T', { min: 0, slot: [o, 'B'] });
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
    numRow(s, '左下角 x', () => o.x, v => o.x = v, 'm', { slot: [o, 'x'] });
    numRow(s, '左下角 y', () => o.y, v => o.y = v, 'm', { slot: [o, 'y'] });
    numRow(s, '宽度', () => o.w, v => o.w = v, 'm', { min: 0, slot: [o, 'w'] });
    numRow(s, '高度', () => o.h, v => o.h = v, 'm', { min: 0, slot: [o, 'h'] });
    s.appendChild(el('div', { class: 'hint' }, '可拖四角调整，拖内部移动。'));
  }
}
function buildLinkPanel(P, o) {
  const title = KIND_NAME[o.kind] || '约束';
  const s = section(P, title);
  if (o.kind === 'spring') {
    numRow(s, '原长 L₀', () => o.L0, v => o.L0 = v, 'm', { min: 0, slot: [o, 'L0'] });
    numRow(s, '劲度 k', () => o.k, v => o.k = v, 'N/m', { min: 0, slot: [o, 'k'] });
    s.appendChild(el('div', { class: 'hint' }, '理想弹簧：F = −k(L−L₀)，可拉可压。'));
  } else if (o.kind === 'rope') {
    numRow(s, '绳长 L', () => o.L0, v => o.L0 = v, 'm', { min: 0, slot: [o, 'L0'] });
    numRow(s, '张力刚度', () => o.k, v => o.k = v, 'N/m', { min: 0, slot: [o, 'k'] });
    numRow(s, '质量 m', () => o.m || 0, v => o.m = Math.max(0, v), 'kg', { min: 0, step: 0.01, slot: [o, 'm'] });
    s.appendChild(el('div', { class: 'hint' }, '轻绳只拉不压：L&lt;L₀ 时张力为 0；刚度宜较大（≥1000）。质量默认 0。'));
  } else if (o.kind === 'rod') {
    numRow(s, '杆长 L₀', () => o.L0, v => o.L0 = v, 'm', { min: 0, slot: [o, 'L0'] });
    numRow(s, '质量 m', () => o.m || 0, v => o.m = Math.max(0, v), 'kg', { min: 0, step: 0.01, slot: [o, 'm'] });
    s.appendChild(el('div', { class: 'hint' }, '轻杆长度恒定（刚性约束，可压可拉，自身不变形）。质量默认 0，自重均分到两端。'));
  }
  if (o.b == null) {
    numRow(s, '固定点 x', () => o.ax, v => o.ax = v, 'm', { slot: [o, 'ax'] });
    numRow(s, '固定点 y', () => o.ay, v => o.ay = v, 'm', { slot: [o, 'ay'] });
  }
  const A = byId(o.a), B = o.b != null ? byId(o.b) : null;
  s.appendChild(el('div', { class: 'hint' }, `连接：${A ? A.name : '?'} — ${B ? B.name : '固定点'}`));
  btn(s, '长度 = 当前距离', () => {
    const e = linkEnds(o);
    if (e) { pushUndo(); o.L0 = clean(Math.hypot(e.x2 - e.x1, e.y2 - e.y1)); liveEdit(); }
  });
}

function lessonCard(parent) {
  const info = scene.info && PRESETS[scene.info.key];
  if (!info) return null;
  const meta = PRESET_META[scene.info.key] || {};
  const card = el('div', { class: 'lesson' });
  card.appendChild(el('div', { class: 'l-kicker' }, `${meta.cat || '实验'} · 原理`));
  card.appendChild(el('div', { class: 'l-title' }, info.name));
  if (info.desc) card.appendChild(el('div', { class: 'l-desc' }, info.desc));
  if (info.formulas && info.formulas.length) {
    const fl = el('div', { class: 'l-formulas' });
    info.formulas.forEach(([f, note]) => fl.appendChild(el('div', { class: 'l-formula' }, `${f}${note ? `<span>${note}</span>` : ''}`)));
    card.appendChild(fl);
  }
  if (info.tips && info.tips.length) {
    const ul = el('ul', { class: 'l-tips' });
    info.tips.forEach(t => ul.appendChild(el('li', {}, t)));
    card.appendChild(ul);
  }
  parent.appendChild(card);
  return card;
}

const ENERGY_COLORS = { Ek: '#d97757', Ep: '#4a78b0', E: 'text', EW: '#5e8c4a' };
function buildEnergySection(parent) {
  const s = section(parent, '系统能量');
  const g = el('div', { class: 'energy-grid' });
  const cell = (id, k, color, cls = '') => {
    const c = el('div', { class: 'energy-cell ' + cls });
    c.appendChild(el('div', { class: 'ek' }, `${color ? `<i style="background:${color}"></i>` : ''}${k}`));
    c.appendChild(el('div', { class: 'ev', id }, '—'));
    g.appendChild(c);
  };
  cell('en_k', '动能 Eₖ', ENERGY_COLORS.Ek);
  cell('en_p', '势能 Eₚ', ENERGY_COLORS.Ep);
  cell('en_g', '重力势能');
  cell('en_s', '弹性势能');
  cell('en_e', '静电势能');
  cell('en_gr', '引力势能');
  cell('en_w', '电场 / 外力做功');
  cell('en_p_tot', '总动量 |p|');
  cell('en_tot', '机械能 E = Eₖ + Eₚ', 'var(--text)', 'total');
  s.appendChild(g);
  s.appendChild(el('div', { class: 'energy-drift', id: 'en_drift' }));
  s.appendChild(el('canvas', { id: 'energyChart', style: 'margin-top:12px' }));
  s.appendChild(el('div', { class: 'hint', style: 'margin-top:8px' },
    '<b style="color:#d97757">—</b> 动能 &nbsp;<b style="color:#4a78b0">—</b> 势能 &nbsp;<b>—</b> 机械能 &nbsp;<b style="color:#5e8c4a">‐‐</b> E − W（扣除非保守力做功）。只有保守力时，机械能曲线应保持水平。'));
  return s;
}
let lastEnergyTs = 0;
function updateEnergyReadout(force = false) {
  if (!document.getElementById('en_tot')) return;
  const now = performance.now();
  if (!force && sim.running && now - lastEnergyTs < 120) return;
  lastEnergyTs = now;
  const S = scene.settings, en = systemEnergy();
  const J = v => `${fmt(v, 4)} J`;
  setTxt('en_k', J(en.Ek)); setTxt('en_p', J(en.Ep));
  setTxt('en_g', S.gOn ? J(en.Ug) : '—');
  setTxt('en_s', J(en.Us));
  setTxt('en_e', J(en.Ue));
  setTxt('en_gr', S.gravity ? J(en.Ugr) : '—');
  setTxt('en_w', J(en.W));
  setTxt('en_p_tot', `${fmt(Math.hypot(en.px, en.py), 4)} kg·m/s`);
  setTxt('en_tot', J(en.E));
  const E0 = sim.E0 ?? en.E, base = Math.max(Math.abs(E0), 1e-9);
  const drift = (en.E - en.W) - E0;
  setTxt('en_drift', sim.t > 0
    ? `ΔE = ${fmt(en.E - E0, 4)} J · ΔE − W = ${fmt(drift, 4)} J（${fmt(drift / base * 100, 3)}%）`
    : '播放后记录能量随时间的变化。');
  drawEnergyChart();
}
function drawEnergyChart() {
  const c = document.getElementById('energyChart'); if (!c) return;
  const r = c.getBoundingClientRect(); if (!r.width) return;
  const d = window.devicePixelRatio || 1; c.width = r.width * d; c.height = r.height * d;
  const g = c.getContext('2d'); g.setTransform(d, 0, 0, d, 0, 0);
  const w = r.width, h = r.height, H_ = sim.eHist;
  g.fillStyle = C['--muted']; g.font = '10px ' + MONO_FONT;
  if (H_.length < 2) { g.textAlign = 'center'; g.fillText('播放后显示能量曲线', w / 2, h / 2); return; }
  const step = Math.max(1, Math.floor(H_.length / 800));
  let t0 = H_[0][0], t1 = H_[H_.length - 1][0], y0 = Infinity, y1 = -Infinity;
  for (let i = 0; i < H_.length; i += step) for (let k = 1; k <= 4; k++) { y0 = Math.min(y0, H_[i][k]); y1 = Math.max(y1, H_[i][k]); }
  y0 = Math.min(y0, 0); y1 = Math.max(y1, 0);
  if (y1 - y0 < 1e-12) { y1 += 1; y0 -= 1; } if (t1 - t0 < 1e-12) t1 = t0 + 1;
  const pad = (y1 - y0) * 0.08; y0 -= pad; y1 += pad;
  const L = 44, R = 8, T = 10, B = 18, X = v => L + (v - t0) / (t1 - t0) * (w - L - R), Y = v => h - B - (v - y0) / (y1 - y0) * (h - T - B);
  g.strokeStyle = C['--border']; g.lineWidth = 1; g.beginPath(); g.moveTo(L, T); g.lineTo(L, h - B); g.lineTo(w - R, h - B); g.stroke();
  g.setLineDash([3, 3]); g.beginPath(); g.moveTo(L, Y(0)); g.lineTo(w - R, Y(0)); g.stroke(); g.setLineDash([]);
  g.textAlign = 'right'; g.textBaseline = 'middle'; g.fillText(fmt(y1, 2), L - 4, T + 4); g.fillText(fmt(y0, 2), L - 4, h - B - 4);
  g.textAlign = 'center'; g.textBaseline = 'top'; g.fillText(fmt(t0, 2), L, h - B + 3); g.fillText(fmt(t1, 2) + ' s', w - R - 14, h - B + 3);
  const series = [[1, ENERGY_COLORS.Ek, 1.6, null], [2, ENERGY_COLORS.Ep, 1.6, null], [3, C['--text'], 2, null], [4, ENERGY_COLORS.EW, 1.4, [4, 3]]];
  for (const [k, col, lw, dash] of series) {
    g.strokeStyle = col; g.lineWidth = lw; g.setLineDash(dash || []); g.beginPath();
    for (let i = 0; i < H_.length; i += step) { const p = H_[i]; i ? g.lineTo(X(p[0]), Y(p[k])) : g.moveTo(X(p[0]), Y(p[k])); }
    const p = H_[H_.length - 1]; g.lineTo(X(p[0]), Y(p[k]));
    g.stroke();
  }
  g.setLineDash([]);
}

function buildScenePanel(P) {
  const S = scene.settings;
  if (!['overview', 'physics', 'display'].includes(propsTab)) propsTab = 'overview';
  const tabs = [['overview', '概览'], ['physics', '物理'], ['display', '显示']];
  const head = el('div', { class: 'insp-head' });
  head.appendChild(el('div', { class: 'insp-title' }, '场景'));
  P.appendChild(head);
  P.appendChild(el('div', { class: 'insp-meta' }, `${bodies().length} 个物体 · ${scene.objects.length - bodies().length} 个场 / 表面 / 约束`));
  P.appendChild(makeTabs(tabs, propsTab, id => { propsTab = id; buildPanel(); }));
  const panels = el('div', { class: 'tab-panels' });

  const pOv = panelBox('overview', propsTab);
  {
    if (!lessonCard(pOv)) {
      const s = section(pOv, '开始');
      s.appendChild(el('div', { class: 'hint' }, `从左侧「实验」载入一个经典模型，或在「添加」里自己搭建。选中物体可设置初态、外力，并查看运动分析。`));
      const br = el('div', { class: 'btnrow' });
      btn(br, '浏览实验库', () => { leftTab = 'presets'; leftChip = 'all'; document.querySelectorAll('#leftTabs button').forEach(b => b.classList.toggle('active', b.dataset.leftTab === 'presets')); if (window.matchMedia('(max-width: 820px)').matches) openMobile('presets'); else { $('#main').classList.remove('noleft'); syncPanelToggleButtons(); refreshLeft(); } });
      btn(br, '使用说明', () => $('#help').classList.add('show'));
      s.appendChild(br);
    }
    buildEnergySection(pOv);
  }

  const pPhy = panelBox('physics', propsTab);
  {
    const s = section(pPhy, '重力与阻力');
    chkRow(s, '均匀重力场', () => S.gOn, v => { S.gOn = v; syncToolbar(); });
    numRow(s, '重力加速度 g', () => S.g, v => { S.g = v; syncToolbar(); }, 'm/s²', { slot: [S, 'g'] });
    const br = el('div', { class: 'btnrow' });
    btn(br, 'g = 9.8', () => { S.g = 9.8; syncToolbar(); liveEdit(); });
    btn(br, 'g = 10', () => { S.g = 10; syncToolbar(); liveEdit(); });
    btn(br, '月球 1.62', () => { S.g = 1.62; syncToolbar(); liveEdit(); });
    s.appendChild(br);
    numRow(s, '线性阻力 b', () => S.drag, v => S.drag = v, 'N·s/m', { min: 0, slot: [S, 'drag'], title: 'F = −b v' });

    const gs = section(pPhy, '万有引力');
    chkRow(gs, '物体之间相互吸引', () => S.gravity, v => { S.gravity = v; }, { rebuild: true });
    numRow(gs, '引力常量 G', () => S.Gc, v => S.Gc = v, 'N·m²/kg²', { slot: [S, 'Gc'], title: '真实值 6.674e-11；教学场景常用 G = 1' });
    numRow(gs, '软化长度 ε', () => S.gravSoft, v => S.gravSoft = Math.max(0, v), 'm', { min: 0, slot: [S, 'gravSoft'] });
    gs.appendChild(el('div', { class: 'hint' }, 'F = G m₁m₂ / (r² + ε²)。ε 避免近距离发散；行星、双星、三体实验会自动开启。'));

    const es = section(pPhy, '静电');
    chkRow(es, '带电物体之间的库仑力', () => S.coulomb, v => { S.coulomb = v; markFieldsDirty(); });
    numRow(es, '静电力常量 k', () => S.kc, v => { S.kc = v; markFieldsDirty(); }, 'N·m²/C²', { slot: [S, 'kc'] });
    const lawRow = el('div', { class: 'btnrow' });
    btn(lawRow, '点电荷 1/r²', () => { S.coulombLaw = 'coulomb3d'; markFieldsDirty(); liveEdit(); buildPanel(); }, S.coulombLaw !== 'coulomb2d' ? 'active' : '');
    btn(lawRow, '线电荷 1/r', () => { S.coulombLaw = 'coulomb2d'; markFieldsDirty(); liveEdit(); buildPanel(); }, S.coulombLaw === 'coulomb2d' ? 'active' : '');
    es.appendChild(lawRow);
    chkRow(es, '显示电场线', () => S.showFieldLines, v => { S.showFieldLines = v; markFieldsDirty(); syncToolbar(); });

    const ns = section(pPhy, '数值积分');
    numRow(ns, '最大步长 Δt', () => S.maxDt, v => S.maxDt = v, 's', { min: 1e-9, slot: [S, 'maxDt'] });
    const nb = el('div', { class: 'btnrow' });
    [[0.002, '快速'], [0.001, '标准'], [0.0002, '精细']].forEach(([v, t]) => btn(nb, t, () => { S.maxDt = v; liveEdit(); buildPanel(); }, Math.abs(S.maxDt - v) < 1e-12 ? 'active' : ''));
    ns.appendChild(nb);
    ns.appendChild(el('div', { class: 'hint' }, '每帧拆成若干子步，用 Boris 蛙跳格式推进（二阶、辛），磁场中能量严格守恒；碰撞与摩擦用冲量法。步长越小越准，但更耗时。'));
  }

  const pDisp = panelBox('display', propsTab);
  {
    const d = section(pDisp, '矢量比例');
    numRow(d, '速度箭头', () => S.vPx, v => S.vPx = v, 'px/(m/s)', { min: 0.0001, slot: [S, 'vPx'] });
    numRow(d, '外力箭头', () => S.FPx || 12, v => S.FPx = v, 'px/N', { min: 0.0001, slot: [S, 'FPx'] });
    const g = section(pDisp, '画布');
    chkRow(g, '显示名称', () => S.showLabels, v => S.showLabels = v);
    chkRow(g, '吸附网格', () => S.snap, v => S.snap = v);
    numRow(g, '吸附步长', () => S.snapStep, v => S.snapStep = v, 'm', { min: 1e-6, slot: [S, 'snapStep'] });
    const view = el('div', { class: 'btnrow' });
    btn(view, '适配视图', fitView);
    btn(view, '回到原点', () => { scene.view.cx = 0; scene.view.cy = 0; draw(); });
    btn(view, '清除所有轨迹', () => { bodies().forEach(b => { if (b.st) b.st.trail = [[b.st.x, b.st.y]]; }); draw(); });
    g.appendChild(view);
  }

  [pOv, pPhy, pDisp].forEach(x => panels.appendChild(x));
  P.appendChild(panels);
  syncDesmosKeypadForAnalysis(false);
  requestAnimationFrame(() => updateEnergyReadout(true));
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
    { tool: 'spring', name: '弹簧', desc: '胡克定律 · 可拉可压' },
    { tool: 'rope', name: '轻绳', desc: '只拉不压 · 可设质量' },
    { tool: 'rod', name: '轻杆', desc: '刚性定长 · 可设质量' },
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
    if (leftChip === 'spring') leftChip = 'link';
    const chips = el('div', { class: 'chips' });
    [['all', '全部'], ['body', '物体'], ['field', '场'], ['surface', '表面'], ['link', '约束']].forEach(([id, lab]) => {
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
      ['约束', 'link', o => isLink(o)],
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
        const li = el('li', { class: (!selVec && o.id === selId) ? 'sel' : '' });
        const col = o.color || (o.kind === 'efield' ? '#b5762a' : o.kind === 'bfield' ? '#4f7faf' : isLink(o) ? (o.kind === 'rope' ? '#4f7faf' : o.kind === 'rod' ? '#4d6470' : '#3d8f86') : '#7c796f');
        li.appendChild(el('span', { class: 'dot', style: `background:${col}` }));
        li.appendChild(el('span', {}, o.name));
        const kindLabel = o.kind === 'bfield' ? (o.dir < 0 ? 'B⊗' : 'B⊙')
          : o.kind === 'efield' ? 'E'
          : isLink(o) ? (o.kind === 'rope' ? '绳' : o.kind === 'rod' ? '杆' : '簧')
          : KIND_NAME[o.kind];
        if (kindLabel && o.name !== kindLabel && !o.name.startsWith(KIND_NAME[o.kind] || '')) {
          li.appendChild(el('span', { class: 'ty' }, kindLabel));
        } else if (o.kind === 'bfield' || o.kind === 'efield' || isLink(o)) {
          li.appendChild(el('span', { class: 'ty' }, kindLabel));
        }
        li.dataset.id = String(o.id);
        li.addEventListener('click', () => { select(o.id); closeMobilePanels(); });
        li.addEventListener('contextmenu', (ev) => {
          ev.preventDefault();
          select(o.id);
          openContextMenu(ev.clientX, ev.clientY, o, 'list');
        });
        ul.appendChild(li);
        if (isBody(o) && (leftChip === 'all' || leftChip === 'body')) {
          const bodyMatch = matchQuery(o.name, q) || matchQuery(KIND_NAME[o.kind] || '', q);
          for (const v of listVectors(o)) {
            if (q && !bodyMatch && !matchQuery(v.name, q)) continue;
            const vi = el('li', { class: 'vec-item' + (selVec && selVec.bodyId === o.id && selVec.key === v.key ? ' sel' : '') });
            vi.dataset.vecBody = String(o.id);
            vi.dataset.vecKey = v.key;
            vi.appendChild(el('span', { class: 'dot', style: `background:${v.color}` }));
            vi.appendChild(el('span', {}, v.name));
            vi.appendChild(el('span', { class: 'ty' }, '矢量'));
            vi.addEventListener('click', (ev) => {
              ev.stopPropagation();
              selectVector(o.id, v.key);
              closeMobilePanels();
            });
            vi.addEventListener('contextmenu', (ev) => {
              ev.preventDefault();
              ev.stopPropagation();
              selectVector(o.id, v.key);
              openVectorMenu(ev.clientX, ev.clientY, o, v.key);
            });
            ul.appendChild(vi);
          }
        }
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
        const key = Object.keys(TOOL_KEYS).find(k => TOOL_KEYS[k] === it.tool);
        const b = el('button', { type: 'button', title: it.desc + (key ? `（快捷键 ${key}）` : '') }, `<div><b>${it.name}</b><div class="hint" style="margin:0">${it.desc}</div></div>${key ? `<kbd>${key}</kbd>` : ''}`);
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
    const cats = ['全部', ...PRESET_CATS];
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
      const P_ = PRESETS[key], name = P_.name;
      const blob = [name, meta.cat, ...(meta.tags || []), P_.desc || ''].join(' ');
      if (!matchQuery(blob, q)) continue;
      any = true;
      const item = el('div', { class: 'preset-item' + (scene.info && scene.info.key === key ? ' sel' : ''), role: 'button', tabindex: '0' });
      const top = el('div', { class: 'p-top' });
      top.appendChild(el('div', { class: 'p-name' }, `${name}${meta.fresh ? '<span class="p-new">新</span>' : ''}`));
      top.appendChild(el('div', { class: 'p-cat' }, meta.cat));
      item.appendChild(top);
      const short = (P_.desc || '').split('。')[0];
      if (short) item.appendChild(el('div', { class: 'p-desc' }, short.length > 46 ? short.slice(0, 45) + '…' : short + '。'));
      item.onclick = () => { loadPreset(key); closeMobilePanels(); };
      item.onkeydown = e => { if (e.key === 'Enter') item.onclick(); };
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

function setDockActive(dock) {
  mobileDock = dock;
  document.querySelectorAll('#mobile-dock button').forEach(b => b.classList.toggle('active', b.dataset.dock === dock));
}
function closeMobilePanels(resetDock = true) {
  $('#left')?.classList.remove('open');
  $('#right')?.classList.remove('open');
  $('#viewFlags')?.classList.remove('show');
  $('#panelScrim')?.classList.remove('show', 'view');
  if (resetDock) setDockActive('stage');
}
function openMobileView() {
  closeMobilePanels(false);
  $('#viewFlags')?.classList.add('show');
  $('#panelScrim')?.classList.add('show', 'view');
  setDockActive('view');
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
  if (!selected() && propsTab === 'overview') updateEnergyReadout(!sim.running);
  const b = selected(); if (!isBody(b) || !b.st) return;
  const st = b.st, S = scene.settings, v = Math.hypot(st.vx, st.vy), Ek = 0.5 * b.m * v * v, Ep = S.gOn ? b.m * S.g * st.y : 0;
  setTxt('r_pos', `(${fmt(st.x)}, ${fmt(st.y)}) m`); setTxt('r_vel', `(${fmt(st.vx)}, ${fmt(st.vy)}) m/s`);
  setTxt('r_v', `${fmt(v)} m/s`); setTxt('r_ang', v > 1e-9 ? `${fmt(Math.atan2(st.vy, st.vx) / DEG, 2)}°` : '—');
  setTxt('r_a', sim.t > 0 ? `${fmt(Math.hypot(st.ax, st.ay))} m/s²` : '—');
  setTxt('r_ek', `${fmt(Ek)} J`); setTxt('r_p', `${fmt(b.m * v)} kg·m/s`); setTxt('r_ep', S.gOn ? `${fmt(Ep)} J（以 y=0 为零势面）` : '—'); setTxt('r_e', `${fmt(Ek + Ep)} J`);
  // Throttle MathQuill remount while running — force only when paused / tab switch
  if (selVec && selVec.bodyId === b.id) {
    const g = vectorScreen(b, selVec.key);
    if (g) {
      setTxt('r_vec_mag', `${fmt(g.mag, 3)} ${vectorMeta(selVec.key).unit}`);
      setTxt('r_vec_ang', `${fmt(g.ang || 0, 1)}°`);
      if (g.part) {
        setTxt('r_vec_x', `${fmt(g.part.fx, 3)} N`);
        setTxt('r_vec_y', `${fmt(g.part.fy, 3)} N`);
      } else if (selVec.key === 'v') {
        setTxt('r_vec_x', `${fmt(b.st.vx, 3)} m/s`);
        setTxt('r_vec_y', `${fmt(b.st.vy, 3)} m/s`);
      }
    }
  }
  if (propsTab === 'force') {
    const af = appliedForceOn(b);
    const mode = b.Fmode || 'const';
    setTxt('r_force_mag', `${fmt(af.F, 3)} N`);
    setTxt('r_force_ang', `${fmt(af.ang, 1)}°`);
    setTxt('r_force_mode', mode === 'expr' ? '表达式' : mode === 'aim' ? '指向目标' : '恒定');
    setTxt('r_force_err', b._Ferr || '');
  }
  if (propsTab === 'analysis') renderMotionAnalysis(b, !sim.running);
  if (propsTab === 'chart' && (!sim.running || (performance.now() / 200 | 0) % 2 === 0)) drawChart();
}

let analysisRefMode = 'initial'; // 'initial' | 'current'

function collectSpringsFor(b) {
  const out = [];
  for (const s of scene.objects) {
    if (!isLink(s)) continue;
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
    out.push({
      kind: s.kind,
      k: s.k || 0,
      L0: s.L0,
      m: s.m || 0,
      rope: s.kind === 'rope',
      rod: s.kind === 'rod',
      anchor,
      otherBody,
    });
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
    ...gravityAsCentralCharge(b, { x, y }),
  };
}
/**
 * 万有引力 ↔ 库仑吸引的等价映射：k·q·Q = −G·m·M。
 * 只有一个固定的大质量物体、本物体不带电且无其它场时，借用有心力（开普勒）分析。
 */
function gravityAsCentralCharge(b, p) {
  const S = scene.settings;
  if (!S.gravity || b.q || b.fixed) return {};
  if (scene.objects.some(o => o.kind === 'efield' || o.kind === 'bfield' || o.kind === 'pointCharge')) return {};
  const others = bodies().filter(o => o !== b && o.m > 0);
  const fixed = others.filter(o => o.fixed);
  if (fixed.length !== 1) return {};
  const c = fixed[0];
  // 其余可动物体质量远小于中心体时才近似为单中心问题
  const rest = others.filter(o => !o.fixed).reduce((a, o) => a + o.m, 0);
  if (rest > 0.05 * c.m) return {};
  const k = scene.settings.kc || K_COULOMB;
  return {
    body: { id: b.id, name: b.name, kind: b.kind, m: b.m, q: 1, x: p.x, y: p.y,
      vx: analysisRefMode === 'initial' ? b.vx0 : b.st.vx, vy: analysisRefMode === 'initial' ? b.vy0 : b.st.vy, t: sim.t, fixed: false },
    pointCharges: [{ id: c.id, x: c.st ? c.st.x : c.x0, y: c.st ? c.st.y : c.y0, q: -(S.Gc || 0) * b.m * c.m / k, fixed: true }],
    coulombLaw: 'coulomb3d',
    field: { Ex: 0, Ey: 0, Bz: 0, phi: 0 },
    _gravityMapped: true,
  };
}
function gravityText(v) {
  if (typeof v === 'string') return v.replace(/固定点电荷/g, '固定中心天体').replace(/点电荷/g, '中心天体').replace(/库仑中心力/g, '万有引力 · 有心力').replace(/库仑力（非匀强）/g, '万有引力').replace(/库仑/g, '引力').replace(/电势能/g, '引力势能').replace(/k\s*q\s*Q/g, '−GMm');
  if (Array.isArray(v)) return v.map(gravityText);
  if (v && typeof v === 'object') { const o = {}; for (const k in v) o[k] = (k === 'latex' || k === 'sym' || k === 'num' || k === 'symbolic' || k === 'numeric' || k === 'regimeId') ? v[k] : gravityText(v[k]); return o; }
  return v;
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
  const actx = buildAnalysisContext(b);
  let report = analyzeMotion(actx);
  if (actx._gravityMapped) report = gravityText(report);
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
      <div data-mq="edit" data-latex="${attrLatex(analysisScratchLatex || 'E=')}" title="点击后用固定 Desmos 键盘输入"></div>
    </div>
    <div class="an-meta">
      <div><b>计入的力：</b>${report.forcesInModel.map(escapeHtml).join('、') || '无'}</div>
      <div><b>假定：</b>${report.assumptions.map(escapeHtml).join('； ')}</div>
      <div class="hint">${escapeHtml(report.validity)}</div>
    </div>`;
  mountMathQuill(host);
  syncDesmosKeypadForAnalysis(true);
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
  const hist = b.st.hist; const idx = { v: 5, vx: 3, vy: 4, x: 1, y: 2, ek: 6, a: 7 };
  const XY = { xy: [1, 2], px: [1, 3], py: [2, 4] }[chartMode];
  const spatial = !!XY;
  const pts = XY ? hist.map(p => [p[XY[0]], p[XY[1]]]) : hist.map(p => [p[0], p[idx[chartMode]] ?? 0]);
  g.fillStyle = C['--muted']; g.font = '10px ' + MONO_FONT;
  if (pts.length < 2) { g.fillText('播放后显示图像', 10, h / 2); return; }
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  pts.forEach(p => { x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]); y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]); });
  if (!spatial) { y0 = Math.min(y0, 0); y1 = Math.max(y1, 0); }
  if (y1 - y0 < 1e-12) { y1 += 1; y0 -= 1; } if (x1 - x0 < 1e-12) x1 = x0 + 1;
  const L = 38, R = 8, T = 8, B = 18, X = v => L + (v - x0) / (x1 - x0) * (w - L - R), Y = v => h - B - (v - y0) / (y1 - y0) * (h - T - B);
  g.strokeStyle = C['--border']; g.lineWidth = 1; g.beginPath(); g.moveTo(L, T); g.lineTo(L, h - B); g.lineTo(w - R, h - B); g.stroke();
  if (y0 < 0 && y1 > 0) { g.setLineDash([3, 3]); g.beginPath(); g.moveTo(L, Y(0)); g.lineTo(w - R, Y(0)); g.stroke(); g.setLineDash([]); }
  g.textAlign = 'right'; g.textBaseline = 'middle'; g.fillText(fmt(y1, 2), L - 3, T + 4); g.fillText(fmt(y0, 2), L - 3, h - B - 4);
  g.textAlign = 'center'; g.textBaseline = 'top'; g.fillText(fmt(x0, 2), L, h - B + 3); g.fillText(fmt(x1, 2) + (spatial ? ' m' : ' s'), w - R - 12, h - B + 3);
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
/** 轻提示。opts.action = { label, fn } 时显示一个可点的操作（例如「撤销」），停留更久。 */
function toast(msg, opts = {}) {
  const t = $('#toast');
  t.innerHTML = '';
  t.appendChild(el('span', {}, escapeHtml(msg)));
  t.classList.toggle('has-action', !!opts.action);
  if (opts.action) {
    const b = el('button', { type: 'button', class: 'toast-act' }, opts.action.label);
    b.addEventListener('click', ev => { ev.stopPropagation(); t.classList.remove('show'); opts.action.fn(); });
    t.appendChild(b);
  }
  t.classList.add('show');
  clearTimeout(toastT);
  toastT = setTimeout(() => t.classList.remove('show'), opts.action ? 5000 : 2200);
}
const UNDO_ACTION = { label: '撤销', fn: () => undo() };

/* =====================================================================
 *  预设场景
 * ===================================================================== */
function sceneOf(objects, settings = {}, view = null) { return { settings: Object.assign(defaultSettings(), settings), objects, view }; }
const PRESET_CATS = ['力学', '振动', '引力', '电磁'];
const PRESET_META = {
  welcome: { cat: '力学', tags: ['斜抛', '摩擦'] },
  projectile: { cat: '力学', tags: ['平抛'] },
  anglerange: { cat: '力学', tags: ['斜抛', '射程', '45°'], fresh: true },
  dragcompare: { cat: '力学', tags: ['空气阻力', '平方阻力'], fresh: true },
  vertical: { cat: '力学', tags: ['竖直上抛', '恢复系数'] },
  incline: { cat: '力学', tags: ['斜面', '摩擦'] },
  collision: { cat: '力学', tags: ['弹性碰撞', '动量守恒'] },
  inelastic: { cat: '力学', tags: ['完全非弹性', '动量守恒'], fresh: true },
  recoil: { cat: '力学', tags: ['反冲', '动量守恒', '弹簧'], fresh: true },
  newton: { cat: '力学', tags: ['牛顿摆', '碰撞'], fresh: true },
  vcircle: { cat: '力学', tags: ['竖直圆周', '临界速度', '轻绳'], fresh: true },
  springosc: { cat: '振动', tags: ['弹簧振子', '简谐'] },
  damped: { cat: '振动', tags: ['阻尼', '弹簧'], fresh: true },
  forced: { cat: '振动', tags: ['受迫振动', '共振'], fresh: true },
  coupled: { cat: '振动', tags: ['耦合振子', '拍'], fresh: true },
  pendulum: { cat: '振动', tags: ['单摆', '轻绳'] },
  rodpendulum: { cat: '振动', tags: ['单摆', '轻杆'] },
  pendlen: { cat: '振动', tags: ['单摆', '周期', '摆长'], fresh: true },
  doublepend: { cat: '振动', tags: ['双摆', '混沌', '轻杆'], fresh: true },
  kepler: { cat: '引力', tags: ['开普勒', '行星', '椭圆轨道'], fresh: true },
  cosmic: { cat: '引力', tags: ['宇宙速度', '卫星', '逃逸'], fresh: true },
  binary: { cat: '引力', tags: ['双星', '质心'], fresh: true },
  figure8: { cat: '引力', tags: ['三体', '8 字形'], fresh: true },
  selector: { cat: '电磁', tags: ['速度选择器'] },
  edeflect: { cat: '电磁', tags: ['电场偏转', '示波管'] },
  accel: { cat: '电磁', tags: ['加速', '偏转'] },
  bbound: { cat: '电磁', tags: ['有界磁场', '圆周'] },
  massspec: { cat: '电磁', tags: ['质谱仪', '同位素'], fresh: true },
  cyclotron: { cat: '电磁', tags: ['回旋加速器', '交变电场'], fresh: true },
  cycloid: { cat: '电磁', tags: ['摆线', '正交场'] },
  coulomb: { cat: '电磁', tags: ['库仑', '绕核'] },
  rutherford: { cat: '电磁', tags: ['卢瑟福', 'α 散射'], fresh: true },
  dipole: { cat: '电磁', tags: ['偶极', '场线'] },
  samecharge: { cat: '电磁', tags: ['同种电荷', '场线'], fresh: true },
};
const PRESETS = {
  welcome: { name: '斜抛与滑块',
    desc: '小球以 10 m/s、53° 斜抛并在地面弹跳；滑块在粗糙地面上匀减速直到停下。两种最常见的运动放在一起对照。',
    formulas: [['x = v₀cosθ · t,  y = v₀sinθ · t − ½gt²', '斜抛'], ['a = μg,  s = v₀² / 2μg', '滑块减速']],
    tips: ['选中小球，在「分析」页查看抛物线参数方程。', '把滑块的 μ 改成 0，观察它匀速滑出画面。'],
    make() {
      nextId = 1;
      return sceneOf([mkGround(0), mkBody('ball', { name: '小球', x0: -4, y0: 0.25, vx0: 6, vy0: 8, e: 0.5 }),
        mkBody('box', { name: '滑块', x0: -6, y0: 0.2, vx0: 5, vy0: 0, mu: 0.3, color: '#d08a3c' })], {}, { cx: 3, cy: 2.6, scale: 42 });
    } },
  projectile: { name: '平抛运动',
    desc: '小球从 5 m 高的平台以 3 m/s 水平抛出。水平方向匀速，竖直方向自由落体，两者互不影响。',
    formulas: [['t = √(2h / g)', '落地时间只由高度决定'], ['x = v₀ √(2h / g)', '水平射程']],
    tips: ['打开「显示 → 受力分析」，可以看到全程只受重力。', '改变 v₀，落地时间不变。'],
    make() {
      nextId = 1;
      return sceneOf([mkGround(0), mkWall(-3, 4.85, 0, 4.85, { name: '平台', smooth: true }),
        mkBody('ball', { name: '小球', x0: 0, y0: 5, r: 0.15, vx0: 3, vy0: 0, e: 0.5 })]);
    } },
  anglerange: { name: '抛射角与射程',
    desc: '五个小球以相同速率 10 m/s、不同仰角同时抛出。不计空气阻力时，45° 射程最远，互余的两个角度（如 30° 与 60°）射程相同。',
    formulas: [['R = v₀² sin2θ / g', '水平射程'], ['H = v₀² sin²θ / 2g', '最大高度']],
    tips: ['比较 30° 与 60° 两条轨迹的落点。', '在「物理」页加入线性阻力，最远射程的角度会小于 45°。'],
    make() {
      nextId = 1;
      const angs = [15, 30, 45, 60, 75], cols = ['#4a78b0', '#5e8c4a', '#d97757', '#8a6bbe', '#c2973e'];
      const balls = angs.map((a, i) => mkBody('ball', { name: `${a}°`, r: 0.12, x0: 0, y0: 0.12, vx0: +(10 * Math.cos(a * DEG)).toFixed(4), vy0: +(10 * Math.sin(a * DEG)).toFixed(4), e: 0, mu: 0.8, collide: false, color: cols[i] }));
      return sceneOf([mkGround(0), ...balls], { showAnalytic: false, vPx: 10 }, { cx: 5.2, cy: 2.6, scale: 62 });
    } },
  dragcompare: { name: '空气阻力下的抛体',
    desc: '两个小球以同样的初速度抛出。蓝色小球受到与速度平方成正比、方向与速度相反的阻力，轨迹不再对称：上升段较长、下落段更陡。',
    formulas: [['F阻 = c v²', '方向与 v 相反'], ['m dv/dt = mg − c|v|v', '无解析解，需数值积分']],
    tips: ['选中「有阻力」，在「受力」页可以看到公式 F = 0.02·v·v 与 θ = atan2(−vy, −vx)。', '把系数改大，观察轨迹趋近“竖直下落”。'],
    make() {
      nextId = 1;
      const a = mkBody('ball', { name: '无阻力', r: 0.15, x0: 0, y0: 0.15, vx0: 12, vy0: 12, e: 0, mu: 0.8, collide: false, color: '#d97757' });
      const b = mkBody('ball', { name: '有阻力', r: 0.15, x0: 0, y0: 0.15, vx0: 12, vy0: 12, e: 0, mu: 0.8, collide: false, color: '#4a78b0',
        Fmode: 'expr', Fexpr: '0.02*v*v', FangExpr: 'atan2(-vy,-vx)*180/pi' });
      return sceneOf([mkGround(0), a, b], { showAnalytic: false, vPx: 8 }, { cx: 13, cy: 4, scale: 34 });
    } },
  vertical: { name: '竖直上抛',
    desc: '小球以 10 m/s 竖直上抛，落地后按恢复系数 e = 0.6 反弹，每次反弹速度变为原来的 0.6 倍，最高点按 e² 衰减。',
    formulas: [['h = v₀² / 2g', '最大高度'], ['v′ = e·v,  h′ = e²·h', '碰撞反弹']],
    tips: ['在「图像」页选 v–t，可以看到斜率恒为 −g 的锯齿线。'],
    make() {
      nextId = 1;
      return sceneOf([mkGround(0), mkBody('ball', { name: '小球', x0: 0, y0: 0.2, r: 0.2, vx0: 0, vy0: 10, e: 0.6 })], {}, null);
    } },
  incline: { name: '斜面滑块',
    desc: '滑块从 37° 粗糙斜面顶端由静止释放。当 tanθ > μ 时下滑；受力图显示重力、支持力与摩擦力的合成。',
    formulas: [['a = g(sinθ − μcosθ)', '下滑加速度'], ['N = mg cosθ,  f = μN', '支持力与滑动摩擦']],
    tips: ['把 μ 调到 0.8（> tan37° ≈ 0.75），滑块将保持静止。'],
    make() {
      nextId = 1;
      const inc = mkIncline(-2, 0, { L: 4, ang: 37, dir: 1, smooth: false });
      const th = 37 * DEG, top = inclinePts(inc)[2], h = 0.4, along = 0.6;
      const b = mkBody('box', { name: '滑块', w: 0.6, h, m: 1, mu: 0.5, e: 0, ang0: 37,
        x0: +(top[0] - Math.sin(th) * h / 2 - Math.cos(th) * along).toFixed(3), y0: +(top[1] + Math.cos(th) * h / 2 - Math.sin(th) * along).toFixed(3) });
      return sceneOf([mkGround(0), inc, b], { showForce: true, g: 9.8 });
    } },
  collision: { name: '弹性碰撞',
    desc: '1 kg 的 A 以 4 m/s 撞向静止的 3 kg 的 B，恢复系数 e = 1。碰后 A 以 2 m/s 反弹，B 以 2 m/s 前进，动量与动能都守恒。',
    formulas: [['v₁′ = (m₁ − m₂)v₁ / (m₁ + m₂)', ''], ['v₂′ = 2m₁v₁ / (m₁ + m₂)', '']],
    tips: ['不选中任何物体，在「概览」里看系统总动量和动能保持不变。'],
    make() {
      nextId = 1;
      return sceneOf([mkGround(0, { smooth: true }),
        mkBody('ball', { name: 'A (1kg)', m: 1, r: 0.3, x0: -3, y0: 0.3, vx0: 4, e: 1, color: '#4a78b0' }),
        mkBody('ball', { name: 'B (3kg)', m: 3, r: 0.3, x0: 0, y0: 0.3, vx0: 0, e: 1, color: '#d97757' })]);
    } },
  inelastic: { name: '完全非弹性碰撞',
    desc: '恢复系数 e = 0：两球碰后以共同速度运动。动量守恒，但动能有一部分转化为内能。',
    formulas: [['v = m₁v₁ / (m₁ + m₂)', '共同速度'], ['ΔE = ½ · m₁m₂/(m₁+m₂) · v₁²', '损失的动能']],
    tips: ['在「概览」的能量曲线上，碰撞瞬间动能突然下降，总动量不变。'],
    make() {
      nextId = 1;
      return sceneOf([mkGround(0, { smooth: true }),
        mkBody('ball', { name: 'A (2kg)', m: 2, r: 0.3, x0: -4, y0: 0.3, vx0: 3, e: 0, color: '#4a78b0' }),
        mkBody('ball', { name: 'B (1kg)', m: 1, r: 0.3, x0: -0.5, y0: 0.3, vx0: 0, e: 0, color: '#d97757' })]);
    } },
  recoil: { name: '反冲：弹簧连接的两滑块',
    desc: '光滑水平面上，两滑块之间夹一根被压缩的弹簧，静止释放。系统所受合外力为零：任何时刻两者动量大小相等、方向相反，质心始终不动。',
    formulas: [['m₁v₁ + m₂v₂ = 0', '动量守恒'], ['½kΔx² = ½m₁v<sub>1,max</sub>² + ½m₂v<sub>2,max</sub>²', '弹簧恢复原长时速度最大']],
    tips: ['弹簧原长 2 m、初始长 0.8 m。由于弹簧仍连着两者，它们会来回振动。', '速度之比始终等于质量的反比 3 : 1。'],
    make() {
      nextId = 1;
      const a = mkBody('box', { name: '甲 (1kg)', w: 0.5, h: 0.5, m: 1, mu: 0, x0: -0.4, y0: 0.25, color: '#4a78b0' });
      const b = mkBody('box', { name: '乙 (3kg)', w: 0.7, h: 0.7, m: 3, mu: 0, x0: 0.4, y0: 0.35, color: '#d97757' });
      const sp = mkSpring(a.id, b.id, { k: 30, L0: 2 });
      return sceneOf([mkGround(0, { smooth: true }), a, b, sp], { vPx: 25 }, { cx: 0, cy: 1, scale: 65 });
    } },
  newton: { name: '牛顿摆',
    desc: '五个相同的钢球并排悬挂、彼此接触。拉起最左边的球释放，它把动量与动能“传递”给最右边的球，中间的球几乎不动。',
    formulas: [['m₁ = m₂, e = 1 ⇒ v₁′ = 0, v₂′ = v₁', '等质量弹性碰撞交换速度']],
    tips: ['这里用冲量法逐对处理碰撞，存在微小的数值偏差；把步长调到「精细」效果更好。'],
    make() {
      nextId = 1;
      const objs = [], r = 0.2, L = 2.4, top = 4;
      for (let i = 0; i < 5; i++) {
        const px = (i - 2) * 2 * r;
        let x0 = px, y0 = top - L;
        if (i === 0) { const a = 40 * DEG; x0 = px - L * Math.sin(a); y0 = top - L * Math.cos(a); }
        const b = mkBody('ball', { name: `球${i + 1}`, m: 1, r, e: 1, x0: +x0.toFixed(4), y0: +y0.toFixed(4), color: i === 0 ? '#d97757' : '#6b6a64' });
        objs.push(b, mkRod(b.id, null, { name: `杆${i + 1}`, ax: +px.toFixed(4), ay: top, L0: L }));
      }
      return sceneOf([mkWall(-1.8, top, 1.8, top, { name: '横梁' }), ...objs], { showLabels: false, showVel: false, maxDt: 0.0002 }, { cx: -0.4, cy: 2.6, scale: 110 });
    } },
  vcircle: { name: '竖直面内的圆周运动',
    desc: '两个用轻绳系住的小球在最低点获得水平速度。左侧 v₀ = √(5gL)，恰好能过最高点；右侧 v₀ = √(3gL)，在上半圈绳子松弛，小球改做斜抛。',
    formulas: [['v<sub>顶</sub> ≥ √(gL)', '绳模型过最高点的条件'], ['v₀ ≥ √(5gL)', '最低点所需速度'], ['T − mg = mv²/L', '最低点张力']],
    tips: ['打开受力分析，看张力 T 在最高点趋于零。', '把轻绳换成轻杆（可压），过最高点只需 v_top ≥ 0。'],
    make() {
      nextId = 1;
      const L = 1.5, g = 9.8;
      const a = mkBody('ball', { name: '√(5gL)', m: 0.5, r: 0.15, x0: -2.2, y0: 2 - L, vx0: +Math.sqrt(5 * g * L).toFixed(4), color: '#d97757' });
      const b = mkBody('ball', { name: '√(3gL)', m: 0.5, r: 0.15, x0: 2.2, y0: 2 - L, vx0: +Math.sqrt(3 * g * L).toFixed(4), color: '#4a78b0' });
      return sceneOf([a, mkRope(a.id, null, { name: '绳1', ax: -2.2, ay: 2, L0: L, k: 20000 }), b, mkRope(b.id, null, { name: '绳2', ax: 2.2, ay: 2, L0: L, k: 20000 })],
        { vPx: 10, maxDt: 0.0005 }, { cx: 0, cy: 2, scale: 80 });
    } },
  springosc: { name: '弹簧振子',
    desc: '光滑水平面上的弹簧振子，从偏离平衡位置 1 m 处静止释放，做简谐运动。周期只取决于 m 与 k，与振幅无关。',
    formulas: [['x = A cos(ωt),  ω = √(k/m)', ''], ['T = 2π√(m/k)', '周期']],
    tips: ['在「图像」页选 vx–x 相图，得到一个椭圆。', '「概览」里动能与弹性势能此消彼长，总和不变。'],
    make() {
      nextId = 1;
      const b = mkBody('box', { name: '振子', w: 0.5, h: 0.5, m: 1, mu: 0, x0: 1, y0: 0.25 });
      return sceneOf([mkGround(0, { smooth: true }), mkWall(-3, 0, -3, 1.5, { name: '墙' }), b,
        mkSpring(b.id, null, { ax: -3, ay: 0.25, k: 20, L0: 3 })]);
    } },
  damped: { name: '阻尼振动',
    desc: '在弹簧振子上加入线性阻力 F = −bv。振幅按指数规律衰减，机械能逐渐耗散；阻尼过大时不再振动（过阻尼）。',
    formulas: [['x = A e<sup>−βt</sup> cos(ω′t),  β = b / 2m', ''], ['ω′ = √(ω₀² − β²)', '阻尼使频率略降低']],
    tips: ['在「物理」页把阻力 b 改为 8.9（≈ 2√(km)），观察临界阻尼。', '能量曲线中 E − W（绿色虚线）保持水平，说明耗散全部来自阻力做功。'],
    make() {
      nextId = 1;
      const b = mkBody('box', { name: '振子', w: 0.5, h: 0.5, m: 1, mu: 0, x0: 1.2, y0: 0.25 });
      return sceneOf([mkGround(0, { smooth: true }), mkWall(-3, 0, -3, 1.5, { name: '墙' }), b,
        mkSpring(b.id, null, { ax: -3, ay: 0.25, k: 20, L0: 3 })], { drag: 0.6 });
    } },
  forced: { name: '受迫振动与共振',
    desc: '振子受到周期性驱动力 F = F₀ sin(ωt)，驱动频率恰好等于固有频率 ω₀ = √(k/m) ≈ 4.47 rad/s。弱阻尼下振幅持续增大——共振。',
    formulas: [['m ẍ + b ẋ + kx = F₀ sin ωt', ''], ['A = F₀ / √((k − mω²)² + (bω)²)', '稳态振幅']],
    tips: ['选中振子，在「受力」页把 4.47 改成 3 或 6，振幅明显变小。', '在「图像」页看 x–t 的包络线。'],
    make() {
      nextId = 1;
      const b = mkBody('box', { name: '振子', w: 0.5, h: 0.5, m: 1, mu: 0, x0: 0, y0: 0.25,
        Fmode: 'expr', Fexpr: 'sin(4.472*t)', FangExpr: '0' });
      return sceneOf([mkGround(0, { smooth: true }), mkWall(-3, 0, -3, 1.5, { name: '墙' }), b,
        mkSpring(b.id, null, { ax: -3, ay: 0.25, k: 20, L0: 3 })], { drag: 0.15, FPx: 30 }, { cx: 0, cy: 1, scale: 70 });
    } },
  coupled: { name: '耦合振子与拍',
    desc: '两个振子由一根弱弹簧相连。只拉开左侧振子释放，振动能量在两者之间来回转移，出现“拍”现象。',
    formulas: [['ω₁ = √(k/m),  ω₂ = √((k + 2k<sub>c</sub>)/m)', '两个简正模式'], ['T拍 = 2π / (ω₂ − ω₁)', '能量交换周期']],
    tips: ['把左右两个振子初始位移设为相同（同相）或相反（反相），分别只激发一种简正模式。'],
    make() {
      nextId = 1;
      const a = mkBody('box', { name: '左', w: 0.5, h: 0.5, m: 1, mu: 0, x0: -1.5, y0: 0.25, color: '#d97757' });
      const b = mkBody('box', { name: '右', w: 0.5, h: 0.5, m: 1, mu: 0, x0: 1.5, y0: 0.25, color: '#4a78b0' });
      return sceneOf([mkGround(0, { smooth: true }), mkWall(-4.5, 0, -4.5, 1.5, { name: '左墙' }), mkWall(4.5, 0, 4.5, 1.5, { name: '右墙' }), a, b,
        mkSpring(a.id, null, { name: '弹簧1', ax: -4.5, ay: 0.25, k: 20, L0: 2.5 }),
        mkSpring(a.id, b.id, { name: '耦合弹簧', k: 3, L0: 3 }),
        mkSpring(b.id, null, { name: '弹簧2', ax: 4.5, ay: 0.25, k: 20, L0: 3 })], {}, { cx: 0, cy: 1, scale: 70 });
    } },
  pendulum: { name: '单摆（轻绳）',
    desc: '摆长 2 m 的单摆从约 30° 处释放。小角度时近似简谐运动，周期与振幅、摆球质量无关。',
    formulas: [['T ≈ 2π√(L/g)', '小角度近似'], ['T − mg cosθ = mv²/L', '绳中张力']],
    tips: ['把初始角度加大到 60°，周期会比公式稍长。'],
    make() {
      nextId = 1;
      const b = mkBody('ball', { name: '摆球', m: 0.5, r: 0.15, x0: 1, y0: 2.268 });
      return sceneOf([b, mkRope(b.id, null, { name: '轻绳', ax: 0, ay: 4, L0: 2, k: 5000 })], {}, { cx: 0, cy: 2.5, scale: 90 });
    } },
  rodpendulum: { name: '单摆（轻杆）',
    desc: '用不可伸缩的轻杆代替轻绳：杆既能拉也能推，因此摆球可以完成整圈运动。',
    formulas: [['T ≈ 2π√(L/g)', '小角度近似'], ['v<sub>顶</sub> ≥ 0', '轻杆过最高点的条件']],
    tips: ['给摆球一个较大的初速度（> √(4gL)），它将做完整的圆周运动。'],
    make() {
      nextId = 1;
      const b = mkBody('ball', { name: '摆球', m: 0.5, r: 0.15, x0: 1, y0: 2.268 });
      return sceneOf([b, mkRod(b.id, null, { name: '轻杆', ax: 0, ay: 4, L0: 2, m: 0 })], {}, { cx: 0, cy: 2.5, scale: 90 });
    } },
  pendlen: { name: '摆长与周期',
    desc: '三个摆长分别为 0.5 m、1 m、2 m 的单摆同时从 15° 释放。摆长变为 4 倍，周期变为 2 倍。',
    formulas: [['T = 2π√(L/g)', ''], ['T₀.₅ : T₁ : T₂ = 1 : √2 : 2', '']],
    tips: ['数一数：最短的摆摆动两次时，最长的摆恰好摆动一次。'],
    make() {
      nextId = 1;
      const objs = [], Ls = [0.5, 1, 2], cols = ['#d97757', '#5e8c4a', '#4a78b0'], a = 15 * DEG;
      Ls.forEach((L, i) => {
        const px = (i - 1) * 1.4;
        const b = mkBody('ball', { name: `L=${L}`, m: 0.3, r: 0.1, x0: +(px + L * Math.sin(a)).toFixed(4), y0: +(3 - L * Math.cos(a)).toFixed(4), color: cols[i], collide: false });
        objs.push(b, mkRod(b.id, null, { name: `杆${i + 1}`, ax: px, ay: 3, L0: L }));
      });
      return sceneOf([mkWall(-2.2, 3, 2.2, 3, { name: '横梁' }), ...objs], { showVel: false }, { cx: 0, cy: 2, scale: 120 });
    } },
  doublepend: { name: '双摆与混沌',
    desc: '两节轻杆串联的双摆是最简单的混沌系统之一。两套双摆的初始角度只差 0.01°，十几秒后运动就完全不同。',
    formulas: [['|δ(t)| ≈ |δ₀| e<sup>λt</sup>', '初值的微小差别指数放大'], ['E = T + V 守恒', '但长期轨迹不可预测']],
    tips: ['打开轨迹，观察末端摆球画出的复杂曲线。', '在「概览」里能量依然守恒——混沌不等于耗散。'],
    make() {
      nextId = 1;
      const out = [];
      const mk = (dx, a1, color, tag) => {
        const L = 1.2, px = dx, py = 3.2;
        const ax = px + L * Math.sin(a1 * DEG), ay = py - L * Math.cos(a1 * DEG);
        const bx = ax + L * Math.sin(a1 * DEG), by = ay - L * Math.cos(a1 * DEG);
        const A = mkBody('ball', { name: `上${tag}`, m: 1, r: 0.09, x0: +ax.toFixed(6), y0: +ay.toFixed(6), color, collide: false, trail: false });
        const B = mkBody('ball', { name: `下${tag}`, m: 1, r: 0.11, x0: +bx.toFixed(6), y0: +by.toFixed(6), color, collide: false });
        out.push(A, B, mkRod(A.id, null, { name: `杆${tag}1`, ax: px, ay: py, L0: L }), mkRod(A.id, B.id, { name: `杆${tag}2`, L0: L }));
      };
      mk(0, 120, '#d97757', 'A');
      mk(0, 120.01, '#4a78b0', 'B');
      return sceneOf(out, { showLabels: false, showVel: false, maxDt: 0.0002, showAnalytic: false }, { cx: 0, cy: 2.4, scale: 110 });
    } },
  kepler: { name: '行星轨道与开普勒定律',
    desc: '固定的恒星（M = 1000）吸引一颗行星和一颗彗星（G = 1）。行星沿近圆轨道运动；彗星轨道偏心率很大，近日点附近速度快、远日点附近速度慢。',
    formulas: [['F = GMm / r²', '万有引力'], ['T² / a³ = 4π² / GM', '开普勒第三定律'], ['r × v = 常量', '开普勒第二定律：面积速度不变']],
    tips: ['选中彗星，在「分析」页查看椭圆轨道的偏心率与周期。', '「概览」里的引力势能与动能此消彼长。'],
    make() {
      nextId = 1;
      return sceneOf([
        mkBody('ball', { name: '恒星', m: 1000, r: 0.35, x0: 0, y0: 0, fixed: true, color: '#c2973e', collide: false }),
        mkBody('ball', { name: '行星', m: 1, r: 0.12, x0: 4, y0: 0, vx0: 0, vy0: 15.8, color: '#4a78b0', collide: false }),
        mkBody('ball', { name: '彗星', m: 0.05, r: 0.07, x0: -7.5, y0: 0, vx0: 0, vy0: -6, color: '#8a6bbe', collide: false }),
      ], { gOn: false, gravity: true, Gc: 1, gravSoft: 0.02, vPx: 6, maxDt: 0.0005 }, { cx: -1.5, cy: 0, scale: 48 });
    } },
  cosmic: { name: '宇宙速度',
    desc: '在离行星中心 1.5R 处，四颗卫星以不同切向速度发射：速度不足落回地面，等于第一宇宙速度做圆周运动，更大时为椭圆，超过 √2 倍则逃逸。',
    formulas: [['v₁ = √(GM / r)', '环绕速度'], ['v₂ = √(2GM / r) = √2 · v₁', '逃逸速度']],
    tips: ['本场景 GM = 500、r = 1.5，v₁ ≈ 18.26，v₂ ≈ 25.82。'],
    make() {
      nextId = 1;
      const r = 1.5, v1 = Math.sqrt(500 / r);
      // 四颗卫星分布在同一圆周的四个方位，沿逆时针切向发射
      const sats = [[0.8, '#c8553d', 90], [1, '#5e8c4a', 180], [1.2, '#4a78b0', 270], [1.45, '#8a6bbe', 0]].map(([f, c, a]) =>
        mkBody('ball', { name: `${f}·v₁`, m: 0.01, r: 0.06, x0: +(r * Math.cos(a * DEG)).toFixed(5), y0: +(r * Math.sin(a * DEG)).toFixed(5),
          vx0: +(-f * v1 * Math.sin(a * DEG)).toFixed(4), vy0: +(f * v1 * Math.cos(a * DEG)).toFixed(4), e: 0, mu: 1, color: c }));
      return sceneOf([mkBody('ball', { name: '行星', m: 500, r: 1, x0: 0, y0: 0, fixed: true, color: '#6a9bcc', e: 0, mu: 1 }), ...sats],
        { gOn: false, gravity: true, Gc: 1, gravSoft: 0.01, vPx: 3, maxDt: 0.0005 }, { cx: 0, cy: 0, scale: 40 });
    } },
  binary: { name: '双星系统',
    desc: '质量为 3 和 1 的两颗星在彼此的引力下，绕共同质心以相同角速度做圆周运动。轨道半径与质量成反比，系统总动量为零。',
    formulas: [['m₁r₁ = m₂r₂', '质心不动'], ['ω² = G(m₁ + m₂) / d³', ''], ['v₁ / v₂ = m₂ / m₁', '']],
    tips: ['不选中任何物体，「概览」里总动量始终为 0。'],
    make() {
      nextId = 1;
      const G = 20, m1 = 3, m2 = 1, d = 4, w = Math.sqrt(G * (m1 + m2) / d ** 3);
      const r1 = d * m2 / (m1 + m2), r2 = d * m1 / (m1 + m2);
      return sceneOf([
        mkBody('ball', { name: '主星 (3)', m: m1, r: 0.3, x0: -r1, y0: 0, vy0: +(-w * r1).toFixed(5), color: '#d97757', collide: false }),
        mkBody('ball', { name: '伴星 (1)', m: m2, r: 0.18, x0: r2, y0: 0, vy0: +(w * r2).toFixed(5), color: '#4a78b0', collide: false }),
      ], { gOn: false, gravity: true, Gc: G, gravSoft: 0.01, vPx: 30 }, { cx: 0, cy: 0, scale: 70 });
    } },
  figure8: { name: '三体问题：8 字形轨道',
    desc: '三个等质量天体在引力作用下沿同一条“8”字形曲线相互追逐（Chenciner–Montgomery 解）。这是三体问题中少见的稳定周期解之一。',
    formulas: [['m = 1,  G = 1,  T ≈ 6.326', ''], ['Σ mᵢvᵢ = 0', '质心静止']],
    tips: ['把任意一个天体的初速度稍作改动，轨道很快就会瓦解。'],
    make() {
      nextId = 1;
      const x1 = 0.97000436, y1 = -0.24308753, vx3 = -0.93240737, vy3 = -0.86473146;
      return sceneOf([
        mkBody('ball', { name: '天体 1', m: 1, r: 0.045, x0: x1, y0: y1, vx0: -vx3 / 2, vy0: -vy3 / 2, color: '#d97757', collide: false }),
        mkBody('ball', { name: '天体 2', m: 1, r: 0.045, x0: -x1, y0: -y1, vx0: -vx3 / 2, vy0: -vy3 / 2, color: '#4a78b0', collide: false }),
        mkBody('ball', { name: '天体 3', m: 1, r: 0.045, x0: 0, y0: 0, vx0: vx3, vy0: vy3, color: '#5e8c4a', collide: false }),
      ], { gOn: false, gravity: true, Gc: 1, gravSoft: 0, vPx: 60, maxDt: 0.0002, showLabels: false, showAnalytic: false }, { cx: 0, cy: 0, scale: 300 });
    } },
  selector: { name: '速度选择器',
    desc: '正交的电场与磁场中，只有速度 v = E/B 的粒子所受电场力与洛伦兹力平衡，沿直线通过；更快的向一侧偏，更慢的向另一侧偏。',
    formulas: [['qE = qvB  ⇒  v = E / B', ''], ['与粒子的电量、质量无关', '']],
    tips: ['把三颗粒子的电荷改成负值，结论仍然成立。'],
    make() {
      nextId = 1;
      return sceneOf([
        mkE({ name: '电场 E↓', E: 5, ang: -90, x: 0, y: -2, w: 10, h: 4 }), mkB({ name: '磁场 ⊗', B: 1, dir: -1, x: 0, y: -2, w: 10, h: 4 }),
        mkWall(0, 2, 10, 2, { name: '上极板(−)' }), mkWall(0, -2, 10, -2, { name: '下极板(+)' }),
        mkBody('particle', { name: 'v=5 (=E/B)', x0: -2, y0: 0, vx0: 5, color: '#5e8c4a' }),
        mkBody('particle', { name: 'v=8', x0: -2, y0: 0, vx0: 8, color: '#c8553d' }),
        mkBody('particle', { name: 'v=3', x0: -2, y0: 0, vx0: 3, color: '#4a78b0' })], { gOn: false, showLabels: false });
    } },
  edeflect: { name: '电场偏转',
    desc: '带电粒子垂直射入平行板间的匀强电场，做类平抛运动；离开电场后沿直线打到荧光屏上。示波管就是利用这一原理。',
    formulas: [['y = qEL² / (2mv₀²)', '出电场时的偏移'], ['tanθ = qEL / (mv₀²)', '偏转角']],
    tips: ['延长出射速度的反向延长线，它恰好经过板长的中点。'],
    make() {
      nextId = 1;
      return sceneOf([mkE({ name: '偏转电场', E: 1, ang: -90, x: 0, y: -1, w: 4, h: 2 }),
        mkWall(0, 1, 4, 1, { name: '上极板' }), mkWall(0, -1, 4, -1, { name: '下极板' }), mkWall(8, -3, 8, 3, { name: '荧光屏', e: 0 }),
        mkBody('particle', { name: '粒子', x0: -1, y0: 0, vx0: 4, m: 0.01, q: 0.01 })], { gOn: false });
    } },
  accel: { name: '加速后偏转',
    desc: '粒子先经电场加速，再进入匀强磁场做匀速圆周运动。加速电压决定速度，速度决定圆周半径。',
    formulas: [['qU = ½mv²', '电场加速'], ['r = mv / qB', '磁场偏转']],
    tips: [],
    make() {
      nextId = 1;
      return sceneOf([mkE({ name: '加速电场', E: 2, ang: 0, x: -4, y: -1, w: 3, h: 2 }),
        mkB({ name: '偏转磁场 ⊗', B: 1, dir: -1, x: 0, y: -1, w: 10, h: 9 }),
        mkBody('particle', { name: '粒子 q/m=1', m: 1, q: 1, r: 0.1, x0: -3.9, y0: 0 })], { gOn: false });
    } },
  bbound: { name: '有界磁场',
    desc: '粒子从左边界射入匀强磁场，做半径 r = mv/qB 的匀速圆周运动。斜射入的粒子出射时与边界的夹角等于入射角。',
    formulas: [['r = mv / qB', ''], ['T = 2πm / qB', '与速度无关']],
    tips: [],
    make() {
      nextId = 1;
      return sceneOf([mkB({ name: '磁场 ⊗', B: 1, dir: -1, x: 0, y: -3, w: 6, h: 9 }),
        mkBody('particle', { name: '粒子 q/m=1', m: 1, q: 1, r: 0.1, x0: -2, y0: 0, vx0: 2 }),
        mkBody('particle', { name: '斜射入', m: 1, q: 1, r: 0.1, x0: -2, y0: -2, vx0: 1.732, vy0: 1, color: '#8a6bbe' })], { gOn: false });
    } },
  massspec: { name: '质谱仪',
    desc: '电量相同、质量不同的同位素离子经同一电压加速，进入磁场后做半圆运动并打在底片上。质量越大，半径越大，于是按质量分开。',
    formulas: [['qU = ½mv²', ''], ['r = (1/B)√(2mU/q)', 'r ∝ √m'], ['m = qB²r² / 2U', '由落点反推质量']],
    tips: ['这里 U = 4 V、B = 1 T，三种离子的半径为 2.83、4、5.66 m。'],
    make() {
      nextId = 1;
      const ions = [[1, '#d97757'], [2, '#5e8c4a'], [4, '#4a78b0']].map(([m, c], i) =>
        mkBody('particle', { name: `m = ${m}`, m, q: 1, r: 0.09, x0: 0, y0: -1.95, color: c, collide: false, e: 0, mu: 1 }));
      return sceneOf([
        mkE({ name: '加速电场', E: 2, ang: 90, x: -0.5, y: -2, w: 1, h: 2 }),
        mkB({ name: '偏转磁场 ⊗', B: 1, dir: -1, x: -12.5, y: 0, w: 13.1, h: 6.5 }),
        mkWall(-12.5, 0, -0.6, 0, { name: '底片', e: 0, smooth: false }),
        ...ions], { gOn: false, vPx: 25, speed: 2 }, { cx: -5.5, cy: 2, scale: 50 });
    } },
  cyclotron: { name: '回旋加速器',
    desc: '两个 D 形盒中有匀强磁场，缝隙中的电场方向随时间交替反转，频率等于粒子的回旋频率 qB/2πm。粒子每经过一次缝隙就被加速一次，半径逐圈增大。',
    formulas: [['T = 2πm / qB', '回旋周期与速度无关'], ['E<sub>k,max</sub> = q²B²R² / 2m', '最大动能由 D 形盒半径决定']],
    tips: ['缝隙电场的方向角写成公式 90 − 90·sign(cos t)，即周期 2π 的方波。', '选中电场，把公式里的 t 改成 1.2*t，频率失配后加速很快失效。'],
    make() {
      nextId = 1;
      const E = mkE({ name: '缝隙电场', E: 6, ang: 0, x: -0.05, y: -6, w: 0.1, h: 12 });
      E.expr = { ang: '90-90*sign(cos(t))' };
      return sceneOf([E,
        mkB({ name: '左 D 形盒', B: 1, dir: -1, x: -6, y: -6, w: 5.95, h: 12 }),
        mkB({ name: '右 D 形盒', B: 1, dir: -1, x: 0.05, y: -6, w: 5.95, h: 12 }),
        mkBody('particle', { name: '质子 q/m=1', m: 1, q: 1, r: 0.08, x0: 0, y0: 0, color: '#d97757' })],
        { gOn: false, speed: 5, showLabels: false, vPx: 12, showAnalytic: false }, { cx: 0, cy: 0, scale: 48 });
    } },
  cycloid: { name: '正交电磁场摆线',
    desc: '静止的带电粒子放在正交的匀强电场与磁场中，轨迹是一条摆线：可看作以 E/B 漂移的参考系中的匀速圆周运动。',
    formulas: [['v<sub>漂</sub> = E / B', '漂移速度'], ['x = (E/ωB)(ωt − sinωt)', ''], ['y = −(E/ωB)(1 − cosωt)', 'ω = qB/m']],
    tips: [],
    make() {
      nextId = 1;
      return sceneOf([mkE({ name: '电场 E↓', E: 2, ang: -90, all: true }), mkB({ name: '磁场 ⊗', B: 1, dir: -1, all: true }),
        mkBody('particle', { name: '粒子 q/m=1', m: 1, q: 1, r: 0.1, x0: 0, y0: 0 })], { gOn: false }, { cx: 8, cy: -2, scale: 45 });
    } },
  coulomb: { name: '库仑绕核',
    desc: '负电荷在固定正电荷的库仑引力作用下做圆周运动，与行星绕日在数学上完全相同（都是平方反比力）。',
    formulas: [['k|qQ| / r² = mv² / r', ''], ['v = √(k|qQ| / mr)', '']],
    tips: ['稍微改变初速度，轨道变成椭圆。'],
    make() {
      nextId = 1;
      return sceneOf([
        mkBody('ball', { name: '+Q 固定', fixed: true, q: 1e-4, m: 1, r: 0.2, x0: 0, y0: 0, color: '#c8553d' }),
        mkBody('ball', { name: '−q', q: -1e-5, m: 1, r: 0.1, x0: 2, y0: 0, vx0: 0, vy0: 2.12, color: '#4a78b0' }),
      ], { gOn: false, coulomb: true, showFieldLines: true, showAnalytic: true }, { cx: 0, cy: 0, scale: 80 });
    } },
  rutherford: { name: '卢瑟福 α 粒子散射',
    desc: '一束 α 粒子以相同速度、不同瞄准距离 b 射向固定的原子核。b 越小偏转越大，正对核的粒子几乎被原路弹回。',
    formulas: [['tan(θ/2) = kqQ / (mv²b)', '散射角'], ['d = 2kqQ / mv²', '正碰时的最近距离']],
    tips: ['这里 kqQ ≈ 0.9 J·m，v = 2 m/s：b = 0.3 m 对应 θ ≈ 74°。'],
    make() {
      nextId = 1;
      const bs = [0, 0.15, 0.3, 0.6, 1, 1.6, 2.5];
      const cols = ['#c8553d', '#d97757', '#c2973e', '#5e8c4a', '#3d8f86', '#4a78b0', '#8a6bbe'];
      const ps = bs.map((b, i) => mkBody('particle', { name: `b=${b}`, m: 1, q: 1e-5, r: 0.06, x0: -6, y0: b, vx0: 2, color: cols[i], collide: false }));
      return sceneOf([mkPointCharge(0, 0, { name: '原子核', q: 1e-5, color: '#c8553d', softRadius: 0.1 }), ...ps],
        { gOn: false, showFieldLines: false, showLabels: false, vPx: 25 }, { cx: 0, cy: 0.6, scale: 60 });
    } },
  dipole: { name: '电偶极子场线',
    desc: '一对等量异种电荷的电场线从正电荷出发、终止于负电荷。试探电荷沿着复杂的曲线运动——注意它并不沿电场线运动。',
    formulas: [['E = Σ kqᵢ r̂ᵢ / rᵢ²', '叠加原理']],
    tips: ['拖动任一电荷，场线实时重绘。'],
    make() {
      nextId = 1;
      return sceneOf([
        mkPointCharge(-1, 0, { name: '+q', q: 1e-6, color: '#c8553d' }),
        mkPointCharge(1, 0, { name: '−q', q: -1e-6, color: '#4a78b0' }),
        mkBody('particle', { name: '试探电荷', m: 0.01, q: 1e-8, r: 0.06, x0: 0, y0: 1.2, vx0: 0.8 }),
      ], { gOn: false, showFieldLines: true, coulomb: false }, { cx: 0, cy: 0, scale: 80 });
    } },
  samecharge: { name: '等量同种电荷',
    desc: '两个等量正电荷的电场线互相“排斥”，两电荷连线中点的场强为零；中垂线上的试探电荷在中点两侧往复运动。',
    formulas: [['E中点 = 0', ''], ['中垂线上 E 先增后减', '']],
    tips: ['把试探电荷改为负电荷，并放在中垂线上，观察它在中点附近振动。'],
    make() {
      nextId = 1;
      return sceneOf([
        mkPointCharge(-1.2, 0, { name: '+q₁', q: 1e-6, color: '#c8553d' }),
        mkPointCharge(1.2, 0, { name: '+q₂', q: 1e-6, color: '#c8553d' }),
        mkBody('particle', { name: '试探电荷 (−)', m: 0.01, q: -1e-8, r: 0.06, x0: 0, y0: 1.6, color: '#4a78b0' }),
      ], { gOn: false, showFieldLines: true, coulomb: false }, { cx: 0, cy: 0, scale: 80 });
    } },
};
function loadPreset(key) {
  const p = PRESETS[key]; if (!p) return;
  pushUndo(`载入「${p.name}」`);
  const sc = p.make(); sc.objects.forEach(o => { if (isBody(o)) o.st = null; });
  sc.info = { key };
  propsTab = 'overview';
  loadScene(sc, false);
  if (!sc.view || W < 640) fitView();
  toast(`已载入「${p.name}」· 空格播放`, { action: UNDO_ACTION });
}

/* =====================================================================
 *  保存 / 载入
 * ===================================================================== */
function serialize() {
  return JSON.stringify({ app: 'physim', version: 1, settings: scene.settings, view: scene.view, info: scene.info || null,
    objects: scene.objects.map(o => {
      const c = Object.assign({}, o);
      delete c.st; delete c._exprErr; delete c._hold; delete c._Ferr; delete c._grab; delete c._N;
      return c;
    }) });
}
function normalizeObject(raw) {
  const o = Object.assign({}, raw, { st: null });
  // 旧版 spring.rope → 独立 rope 种类
  if (o.kind === 'spring' && o.rope) {
    o.kind = 'rope';
    o.name = o.name && o.name !== '弹簧' ? o.name : '轻绳';
    if (o.m == null) o.m = 0;
    delete o.rope;
  }
  if (o.kind === 'spring') {
    if (o.m == null) o.m = 0;
    delete o.rope;
  }
  if (o.kind === 'rope' || o.kind === 'rod') {
    if (o.m == null) o.m = 0;
    if (o.kind === 'rope' && (o.k == null || o.k <= 0)) o.k = 5000;
  }
  return o;
}
function loadScene(data, keepView) {
  if (!data || !Array.isArray(data.objects)) throw new Error('无效的场景文件');
  const oldView = scene.view;
  scene = {
    settings: Object.assign(defaultSettings(), data.settings || {}),
    objects: data.objects.map(o => {
      const c = normalizeObject(o);
      if (isBody(c)) ensureForceFields(c);
      return c;
    }),
    view: keepView ? oldView : Object.assign({ cx: 0, cy: 2, scale: 60 }, data.view || {}),
    info: data.info && PRESETS[data.info.key] ? { key: data.info.key } : null,
  };
  nextId = scene.objects.reduce((m, o) => Math.max(m, o.id || 0), 0) + 1;
  if (selId != null && !byId(selId)) selId = null;
  if (selVec && !byId(selVec.bodyId)) selVec = null;
  if (!keepView) { selId = null; selVec = null; }
  if (leftChip === 'spring') leftChip = 'link';
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
  const tl = $('#tlRange');
  if (tl) {
    tl.addEventListener('input', () => {
      if (sim.running) { sim.running = false; updatePlayBtn(); }
      restoreFrame(+tl.value);
    });
  }
  $('#tlBack')?.addEventListener('click', stepBack);
  $('#tlFwd')?.addEventListener('click', stepOnce);
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

  // display popover
  const viewBtn = $('#bView'), viewPop = $('#viewFlags');
  viewBtn?.addEventListener('click', e => {
    e.preventDefault(); e.stopPropagation();
    moreMenu?.classList.remove('open');
    const open = !viewPop.classList.contains('show');
    viewPop.classList.toggle('show', open);
    viewBtn.classList.toggle('active', open);
  });
  moreBtn?.addEventListener('click', () => { viewPop?.classList.remove('show'); viewBtn?.classList.remove('active'); });
  document.addEventListener('click', e => {
    if (!viewPop?.classList.contains('show')) return;
    if (viewPop.contains(e.target) || viewBtn?.contains(e.target) || e.target.closest?.('#mobile-dock')) return;
    if (window.matchMedia('(max-width: 820px)').matches) return; // mobile sheet closes via scrim
    viewPop.classList.remove('show'); viewBtn?.classList.remove('active');
  });

  // zoom controls
  const zoomBy = f => {
    const v = scene.view; v.scale = Math.min(1e6, Math.max(0.5, v.scale * f));
    markFieldsDirty(); draw(); markDirty();
  };
  $('#bZoomIn')?.addEventListener('click', () => zoomBy(1.25));
  $('#bZoomOut')?.addEventListener('click', () => zoomBy(0.8));
  $('#bFit')?.addEventListener('click', () => fitView());
  $('#bShot')?.addEventListener('click', () => {
    moreMenu?.classList.remove('open');
    try {
      const a = el('a', { href: cv.toDataURL('image/png'), download: `物理实验_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.png` });
      document.body.appendChild(a); a.click(); a.remove();
      toast('已保存截图');
    } catch (err) { toast('截图失败：' + err.message); }
  });

  $('#bUndo').onclick = undo;
  $('#bRedo') && ($('#bRedo').onclick = redo);
  syncHistoryButtons();
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
    rd.onload = () => { try { const data = JSON.parse(rd.result); pushUndo('导入场景'); loadScene(data, false); toast(`已导入 ${f.name}`, { action: UNDO_ACTION }); } catch (err) { if (undoStack.at(-1)?.label === '导入场景') cancelLastUndo(); toast('导入失败：文件不是有效的场景 JSON'); } };
    rd.readAsText(f); e.target.value = '';
  };
  $('#bClear').onclick = () => {
    if (!scene.objects.length) { toast('场景已经是空的'); moreMenu?.classList.remove('open'); return; }
    pushUndo('清空场景');
    const s = scene.settings;
    loadScene({ objects: [], settings: s, view: scene.view }, true);
    toast('已清空场景', { action: UNDO_ACTION });
    moreMenu?.classList.remove('open');
  };
  $('#bTheme').onclick = () => {
    document.body.classList.toggle('dark');
    try { localStorage.setItem('physim.theme', document.body.classList.contains('dark') ? 'dark' : 'light'); } catch (e) {}
    syncThemeMeta(); readColors(); markFieldsDirty(); draw(); drawChart(); updateEnergyReadout(true);
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
      if (dock === 'view') {
        if (mobileDock === 'view' && $('#viewFlags')?.classList.contains('show')) closeMobilePanels();
        else openMobileView();
        return;
      }
      setDockActive(dock);
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
 *  右键菜单
 * ===================================================================== */
let ctxMenuEl = null;
let ctxTarget = null;

function ensureContextMenu() {
  if (ctxMenuEl) return ctxMenuEl;
  ctxMenuEl = document.createElement('div');
  ctxMenuEl.id = 'ctxMenu';
  ctxMenuEl.className = 'ctx-menu';
  ctxMenuEl.hidden = true;
  document.body.appendChild(ctxMenuEl);
  document.addEventListener('pointerdown', (e) => {
    if (!ctxMenuEl || ctxMenuEl.hidden) return;
    if (ctxMenuEl.contains(e.target)) return;
    hideContextMenu();
  }, true);
  window.addEventListener('blur', hideContextMenu);
  window.addEventListener('resize', hideContextMenu);
  return ctxMenuEl;
}

function hideContextMenu() {
  if (!ctxMenuEl) return;
  ctxMenuEl.hidden = true;
  ctxMenuEl.innerHTML = '';
  ctxTarget = null;
}

function ctxItem(label, fn, opts = {}) {
  const b = document.createElement('button');
  b.type = 'button';
  b.textContent = label;
  if (opts.kbd) { const k = document.createElement('span'); k.className = 'ck'; k.textContent = opts.kbd; b.appendChild(k); }
  if (opts.danger) b.classList.add('danger');
  if (opts.disabled) b.disabled = true;
  b.addEventListener('click', (e) => {
    e.preventDefault();
    e.stopPropagation();
    hideContextMenu();
    fn();
  });
  return b;
}
function ctxSep() {
  const d = document.createElement('div');
  d.className = 'sep';
  return d;
}

function openVectorMenu(clientX, clientY, body, key) {
  const menu = ensureContextMenu();
  const meta = vectorMeta(key);
  menu.innerHTML = '';
  menu.appendChild(ctxItem(`选中「${meta.name}」`, () => selectVector(body.id, key)));
  menu.appendChild(ctxItem('在检视栏打开', () => {
    selectVector(body.id, key);
    $('#right')?.classList.add('open');
    $('#main')?.classList.remove('noright');
    syncPanelToggleButtons?.();
  }));
  menu.appendChild(ctxItem(`转到「${body.name}」`, () => { selVec = null; select(body.id, { forcePanel: true }); refreshLeft(); }));
  if (meta.editable) {
    menu.appendChild(ctxSep());
    menu.appendChild(ctxItem('删除', () => { selectVector(body.id, key); deleteSel(); }, { danger: true }));
  }
  placeContextMenu(menu, clientX, clientY);
}
function openContextMenu(clientX, clientY, target, where = 'canvas') {
  const menu = ensureContextMenu();
  ctxTarget = target;
  menu.innerHTML = '';

  if (target) {
    menu.appendChild(ctxItem(`选中「${target.name}」`, () => select(target.id)));
    menu.appendChild(ctxItem('在检视栏打开', () => {
      select(target.id);
      $('#right')?.classList.add('open');
      $('#main')?.classList.remove('noright');
      syncPanelToggleButtons?.();
    }));
    menu.appendChild(ctxSep());
    if (!isLink(target)) {
      menu.appendChild(ctxItem('复制一份', () => { select(target.id); duplicateSel(); }, { kbd: 'Ctrl+D' }));
      menu.appendChild(ctxItem('拷贝', () => { select(target.id); copySel(); }, { kbd: 'Ctrl+C' }));
    }
    menu.appendChild(ctxItem('删除', () => { select(target.id); deleteSel(); }, { danger: true, kbd: 'Del' }));
    if (isBody(target)) {
      menu.appendChild(ctxSep());
      menu.appendChild(ctxItem(target.fixed ? '取消固定' : '固定不动', () => {
        pushUndo();
        target.fixed = !target.fixed;
        liveEdit();
        buildPanel();
        toast(target.fixed ? '已固定' : '已取消固定');
      }));
      menu.appendChild(ctxItem('清除轨迹', () => {
        if (target.st) { target.st.trail = [[target.st.x, target.st.y]]; draw(); }
      }));
      menu.appendChild(ctxItem('转到分析页', () => {
        select(target.id);
        propsTab = 'analysis';
        buildPanel();
      }));
    }
    if (isLink(target)) {
      menu.appendChild(ctxSep());
      menu.appendChild(ctxItem('长度 = 当前距离', () => {
        const e = linkEnds(target);
        if (e) {
          pushUndo();
          target.L0 = clean(Math.hypot(e.x2 - e.x1, e.y2 - e.y1));
          liveEdit();
          buildPanel();
        }
      }));
    }
    if (target.kind === 'pointCharge') {
      menu.appendChild(ctxSep());
      menu.appendChild(ctxItem('电荷取反', () => {
        pushUndo();
        target.q = -target.q;
        target.color = target.q >= 0 ? '#c8553d' : '#4a78b0';
        liveEdit();
        buildPanel();
      }));
    }
  } else if (where === 'canvas') {
    const at = snapPt(s2w(clientX - cv.getBoundingClientRect().left, clientY - cv.getBoundingClientRect().top));
    const placeHere = kind => () => {
      pushUndo('添加' + KIND_NAME[kind]); if (sim.t > 0) resetSim();
      const b = kind === 'pointCharge' ? mkPointCharge(at[0], at[1]) : mkBody(kind, { x0: at[0], y0: at[1] });
      scene.objects.push(b); if (isBody(b)) initState(b); afterStructure(); select(b.id);
    };
    menu.appendChild(ctxItem('在此放置小球', placeHere('ball'), { kbd: '1' }));
    menu.appendChild(ctxItem('在此放置带电粒子', placeHere('particle'), { kbd: '2' }));
    menu.appendChild(ctxItem('在此放置滑块', placeHere('box'), { kbd: '3' }));
    menu.appendChild(ctxItem('在此放置点电荷', placeHere('pointCharge')));
    if (clipboard) menu.appendChild(ctxItem('粘贴到此处', () => { mouseW = at; mouseIn = true; pasteClip(); }, { kbd: 'Ctrl+V' }));
    menu.appendChild(ctxSep());
    menu.appendChild(ctxItem('适配视图', () => fitView(), { kbd: 'F' }));
    if (selId != null) menu.appendChild(ctxItem('取消选择', () => select(null), { kbd: 'Esc' }));
  } else {
    hideContextMenu();
    return;
  }

  placeContextMenu(menu, clientX, clientY);
}
function placeContextMenu(menu, clientX, clientY) {
  menu.hidden = false;
  const pad = 8;
  const mw = menu.offsetWidth || 200;
  const mh = menu.offsetHeight || 120;
  let x = clientX, y = clientY;
  if (x + mw > window.innerWidth - pad) x = window.innerWidth - mw - pad;
  if (y + mh > window.innerHeight - pad) y = window.innerHeight - mh - pad;
  menu.style.left = `${Math.max(pad, x)}px`;
  menu.style.top = `${Math.max(pad, y)}px`;
}

/* =====================================================================
 *  启动（仅由 main.ts 调用一次；禁止在模块底再自动 boot）
 * ===================================================================== */
function syncThemeMeta() {
  const m = document.querySelector('meta[name="theme-color"]');
  if (m) m.setAttribute('content', document.body.classList.contains('dark') ? '#252422' : '#faf9f5');
}
function bootOnce() {
  try {
    const th = localStorage.getItem('physim.theme');
    if (th === 'dark' || (!th && window.matchMedia?.('(prefers-color-scheme: dark)').matches)) document.body.classList.add('dark');
  } catch (e) { }
  syncThemeMeta();
  readColors(); initToolbar();
  new ResizeObserver(resize).observe($('#stage'));
  resize();
  let loaded = false;
  try {
    const s = localStorage.getItem(STORE_KEY);
    if (s) { loadScene(JSON.parse(s), false); loaded = true; }
  } catch (e) { loaded = false; }
  if (!loaded) { const sc = PRESETS.welcome.make(); sc.info = { key: 'welcome' }; loadScene(sc, false); }
  setTool('select');
  syncPanelToggleButtons();
  ensureDesmosKeypadDock(document.getElementById('right'));
  syncDesmosKeypadForAnalysis(false);
  toast(loaded ? '已恢复上次场景' : '选中物体后拖动绿色箭头设置初速度，空格播放');
  try { if (!localStorage.getItem('physim.seenHelp')) { $('#help').classList.add('show'); localStorage.setItem('physim.seenHelp', '1'); } } catch (e) { }
  window.__physim = {
    scene: () => scene, sim, advance, loadPreset, select, resetSim, togglePlay, fieldSys,
    presetKeys: () => Object.keys(PRESETS), energy: systemEnergy,
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
