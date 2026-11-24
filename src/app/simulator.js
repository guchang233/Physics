import { analyzeMotion, sampleAnalyticPath } from '../analysis/motionAnalysis.ts';
import { collectContacts, solveVelocities, solvePositions, rotationalKE, inertia as rigidInertia, canRotate } from '../engine/rigid.js';
import { mountMathQuill } from '../ui/mathquill.ts';
import { syncDesmosKeypadForAnalysis, ensureDesmosKeypadDock, setDesmosKeypadVisible } from '../ui/mathKeyboard.ts';
import { validateVarName, findUnknownNames, DISPLAY_LOCALS } from '../engine/forceExpr.ts';
import { OVERLAY_TYPES, OVERLAY_CATS, isVecType, overlayDefaults, normalizeOverlay, overlaysFromLegacy, LEGACY_DISPLAY_KEYS, recommendedOverlays } from './display/overlayTypes.js';
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
  // vPx / FPx：没有对应显示项时，初速度 / 外力编辑手柄的箭头比例（px 每单位）
  return { gOn: true, g: 9.8, drag: 0, coulomb: false, kc: K_COULOMB, maxDt: 0.001, vPx: 20, FPx: 12, snap: true, snapStep: 0.1,
    showGrid: true, coulombLaw: 'coulomb3d', contactBackend: 'native', speed: 1,
    /* 物体间万有引力（教学用缩放常量） */
    gravity: false, Gc: 1, gravSoft: 0.05 };
}
/** 旧版的全局显示开关默认值：仅用于迁移旧文档、生成模板的“推荐显示” */
const LEGACY_DISPLAY_DEFAULTS = { showLabels: true, showTrail: true, showVel: true, showForce: false, showAnalytic: true, showFieldLines: true, showPredict: true, predictT: 3, strobe: 0 };
let scene = { settings: defaultSettings(), objects: [], groups: [], vars: [], overlays: [], view: { cx: 0, cy: 2, scale: 60 } };

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
    fixed: false, trail: true, collide: true, rotate: true, omega0: 0 };
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
/* =====================================================================
 *  属性规范（Schema）：每类元素的每个属性只在这里定义一次
 *  —— 名称、物理符号、单位、合法范围、跨字段约束、说明。
 *  检视栏、输入校验、导入修复都由它生成，保证全局一致。
 * ===================================================================== */
const num = (key, label, sym, unit, o = {}) => Object.assign({ type: 'num', key, label, sym, unit }, o);
const bool = (key, label, o = {}) => Object.assign({ type: 'bool', key, label }, o);
const BODY_COMMON = [
  num('m', '质量', 'm', 'kg', { group: '物理量', min: 0, minEx: true, def: 1, title: '必须为正。电子 9.11e-31 kg' }),
  num('q', '电荷量', 'q', 'C', { group: '物理量', def: 0, title: '可正可负。元电荷 1.6e-19 C' }),
  num('mu', '动摩擦因数', 'μ', '', { group: '接触', min: 0, def: 0, step: 0.05, title: '与非光滑表面接触时生效：f = μN' }),
  num('mus', '静摩擦因数', 'μₛ', '', { group: '接触', min: 0, def: null, step: 0.05,
    title: '最大静摩擦 fₘ = μₛN。留空等于 μ', check: (o, v) => v < (o.mu || 0) - 1e-12 ? `不能小于动摩擦因数 μ = ${o.mu}` : '' }),
  num('e', '恢复系数', 'e', '', { group: '接触', min: 0, max: 1, def: 0.6, step: 0.1, title: '碰后与碰前法向相对速度之比：1 弹性，0 完全非弹性' }),
  bool('rotate', '允许转动', { group: '转动', init: true, rebuild: true, def: true, title: '关闭后只平动（姿态保持初始倾角）' }),
  num('omega0', '初始角速度', 'ω₀', 'rad/s', { group: '转动', def: 0, init: true, when: o => o.rotate !== false, title: '逆时针为正' }),
  bool('fixed', '固定（不参与运动）', { group: '行为', init: true }),
  bool('collide', '参与物体间碰撞', { group: '行为' }),
];
const SURF_COMMON = [
  bool('smooth', '光滑（无摩擦）', { group: '接触' }),
  num('e', '恢复系数', 'e', '', { group: '接触', min: 0, max: 1, def: 0.6, step: 0.1, title: '与物体碰撞时取两者中较小的 e' }),
];
const REGION = [
  bool('all', '充满全空间', { group: '区域', rebuild: true }),
  num('x', '左下角', 'x', 'm', { group: '区域', when: o => !o.all, def: 0 }),
  num('y', '左下角', 'y', 'm', { group: '区域', when: o => !o.all, def: 0 }),
  num('w', '宽度', 'w', 'm', { group: '区域', when: o => !o.all, min: 0, minEx: true, def: 4 }),
  num('h', '高度', 'h', 'm', { group: '区域', when: o => !o.all, min: 0, minEx: true, def: 4 }),
];
const SCHEMA = {
  ball: [...BODY_COMMON.slice(0, 2), num('r', '半径', 'r', 'm', { group: '几何', min: 0, minEx: true, def: 0.25, title: '碰撞与接触用的半径；质点模型下不影响受力' }), ...BODY_COMMON.slice(2)],
  particle: [...BODY_COMMON.slice(0, 2), num('r', '半径', 'r', 'm', { group: '几何', min: 0, minEx: true, def: 0.08 }), ...BODY_COMMON.slice(2).filter(d => d.group !== '转动')],
  box: [...BODY_COMMON.slice(0, 2),
    num('w', '宽', 'w', 'm', { group: '几何', min: 0, minEx: true, def: 0.6 }),
    num('h', '高', 'h', 'm', { group: '几何', min: 0, minEx: true, def: 0.4 }),
    num('ang0', '初始倾角', 'θ₀', '°', { group: '几何', def: 0, init: true, title: '矩形的初始姿态，逆时针为正' }),
    ...BODY_COMMON.slice(2)],
  ground: [num('y', '高度', 'y', 'm', { group: '几何', def: 0 }), ...SURF_COMMON],
  wall: [
    num('x1', '起点', 'x₁', 'm', { group: '几何', def: 0 }), num('y1', '起点', 'y₁', 'm', { group: '几何', def: 0 }),
    num('x2', '终点', 'x₂', 'm', { group: '几何', def: 0 }), num('y2', '终点', 'y₂', 'm', { group: '几何', def: 2 }),
    ...SURF_COMMON],
  incline: [
    num('x', '底角顶点', 'x', 'm', { group: '几何', def: 0 }), num('y', '底角顶点', 'y', 'm', { group: '几何', def: 0 }),
    num('L', '底边长', 'L', 'm', { group: '几何', min: 0, minEx: true, def: 4 }),
    num('ang', '倾角', 'θ', '°', { group: '几何', min: 0, minEx: true, max: 85, def: 37, step: 1 }),
    ...SURF_COMMON],
  efield: [
    num('E', '场强', 'E', 'V/m', { group: '场', min: 0, def: 1 }),
    num('ang', '方向角', 'φ', '°', { group: '场', def: -90, title: '从 +x 轴逆时针量起；−90° 竖直向下' }),
    ...REGION],
  bfield: [num('B', '磁感应强度', 'B', 'T', { group: '场', min: 0, def: 1 }), ...REGION],
  pointCharge: [
    num('q', '电荷量', 'Q', 'C', { group: '源', def: 1e-6 }),
    num('x', '位置', 'x', 'm', { group: '源', def: 0 }), num('y', '位置', 'y', 'm', { group: '源', def: 0 }),
    num('softRadius', '软化半径', 'ε', 'm', { group: '源', min: 0, minEx: true, def: 0.08, title: 'E = kQ·r / (r² + ε²)^{3/2}，避免 r→0 时发散' }),
  ],
  spring: [
    num('L0', '原长', 'L₀', 'm', { group: '参数', min: 0, def: 1 }),
    num('k', '劲度系数', 'k', 'N/m', { group: '参数', min: 0, minEx: true, def: 20 }),
    num('m', '自身质量', 'm', 'kg', { group: '参数', min: 0, def: 0, title: '自重平分到两端；通常取 0（轻弹簧）' }),
  ],
  rope: [
    num('L0', '绳长', 'L', 'm', { group: '参数', min: 0, minEx: true, def: 1 }),
    num('k', '张力刚度', 'k', 'N/m', { group: '参数', min: 0, minEx: true, def: 5000, title: '轻绳按“只拉不压的极硬弹簧”建模；刚度越大越接近不可伸长' }),
    num('m', '自身质量', 'm', 'kg', { group: '参数', min: 0, def: 0 }),
  ],
  rod: [
    num('L0', '杆长', 'L', 'm', { group: '参数', min: 0, minEx: true, def: 1, title: '刚性约束（SHAKE），长度严格不变，可拉可压' }),
    num('m', '自身质量', 'm', 'kg', { group: '参数', min: 0, def: 0 }),
  ],
};
const LINK_ANCHOR = [num('ax', '固定点', 'x', 'm', { group: '固定点', def: 0 }), num('ay', '固定点', 'y', 'm', { group: '固定点', def: 0 })];
function schemaOf(o) {
  const list = SCHEMA[o.kind] || [];
  return isLink(o) && o.b == null ? [...list, ...LINK_ANCHOR] : list;
}
/** 把导入/旧版数据修复到合法范围：缺失或非法的数值回到默认值，越界的夹到边界 */
function sanitizeBySchema(o) {
  const fixes = [];
  for (const d of schemaOf(o)) {
    if (d.type === 'bool') { if (o[d.key] != null) o[d.key] = !!o[d.key]; else if (d.def != null) o[d.key] = d.def; continue; }
    let v = o[d.key];
    if (v == null) { if (d.def != null) o[d.key] = d.def; continue; }
    v = Number(v);
    if (!Number.isFinite(v)) { o[d.key] = d.def ?? 0; fixes.push(d.label); continue; }
    if (d.min != null && (d.minEx ? v <= d.min : v < d.min)) { o[d.key] = d.def != null && d.def > d.min ? d.def : d.min; fixes.push(d.label); continue; }
    if (d.max != null && v > d.max) { o[d.key] = d.max; fixes.push(d.label); continue; }
    o[d.key] = v;
  }
  return fixes;
}


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
let sheet = 'lab';
const fieldSys = new FieldSystem2({ law: 'coulomb3d', k: K_COULOMB, softening: 0.02 });
let contactWorld = new StubContactWorld();
let fieldsDirty = true;


function initState(b) {
  b.st = { x: b.x0, y: b.y0, vx: b.vx0, vy: b.vy0, ang: (b.ang0 || 0) * DEG, omega: canRotate(b) ? (b.omega0 || 0) : 0, trail: [[b.x0, b.y0, sim.t, b.vx0, b.vy0]], hist: [], forces: [],
    contactN: [0, 0], contactF: [0, 0], cN: 0, ax: 0, ay: 0, pvx: b.vx0, pvy: b.vy0 };
  pushHist(b);
}
function resetSim(msg) {
  sim.t = 0; sim.running = false; sim.accum = 0;
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
    enabled: ovActive('fieldLines'),
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
  if (moved && ovActive('fieldLines')) {
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

/* =====================================================================
 *  全局变量（Desmos 式）
 *  · 写成纯数字的变量自动成为滑块（最小 / 最大 / 步长可改，可 ▶ 往返播放）
 *  · 也可以写表达式：b = 2*a，ω = 2*pi/T，甚至 c = sin(t)
 *  · 任何带 ƒ 的公式框里都能直接用变量名；依赖自动排序，循环引用会报错
 * ===================================================================== */
let nextVarId = 1;
const NUM_LITERAL = /^\s*-?(\d+\.?\d*|\.\d+)(e[+-]?\d+)?\s*$/i;
function isSliderVar(v) { return NUM_LITERAL.test(String(v.expr ?? '')); }
function mkVar(name, expr = '1', o = {}) {
  const n = +expr;
  const span = Number.isFinite(n) && n !== 0 ? Math.pow(10, Math.ceil(Math.log10(Math.abs(n) * 2))) : 10;
  return Object.assign({ id: nextVarId++, name, expr: String(expr), min: Number.isFinite(n) && n < 0 ? -span : 0, max: span, step: span / 100, anim: false, period: 4, _isVar: true, value: NaN, _dir: 1 }, o);
}
function serializeVars() { return (scene.vars || []).map(v => ({ id: v.id, name: v.name, expr: v.expr, min: v.min, max: v.max, step: v.step, period: v.period, ...(v.anim ? { anim: true } : {}) })); }
function loadVars(list) {
  scene.vars = (Array.isArray(list) ? list : []).filter(v => v && typeof v.name === 'string').map(v => ({
    id: +v.id || 0, name: v.name, expr: String(v.expr ?? '0'), min: Number.isFinite(+v.min) ? +v.min : 0, max: Number.isFinite(+v.max) ? +v.max : 10,
    step: +v.step > 0 ? +v.step : 0.1, period: +v.period > 0 ? +v.period : 4, anim: !!v.anim, _isVar: true, value: NaN, _dir: 1,
  }));
  nextVarId = scene.vars.reduce((m, v) => Math.max(m, v.id), 0) + 1;
}
function nextVarName() {
  const used = new Set(varNames());
  for (const c of 'abcdkABCDKαβγ') if (!used.has(c)) return c;
  for (let i = 1; ; i++) if (!used.has('a' + i)) return 'a' + i;
}
/** 变量改名时，把所有公式里对它的引用一起改掉 */
function renameVarRefs(oldName, newName) {
  const esc = oldName.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
  const re = new RegExp(`(^|[^A-Za-z0-9_\\u0391-\\u03A9\\u03B1-\\u03C9.])${esc}(?![A-Za-z0-9_\\u0391-\\u03A9\\u03B1-\\u03C9.(])`, 'g');
  const fix = s => typeof s === 'string' ? s.replace(re, `$1${newName}`) : s;
  let n = 0;
  const touch = (obj, k) => { const a = obj[k]; const b = fix(a); if (a !== b) { obj[k] = b; n++; } };
  for (const o of scene.objects) {
    if (o.expr) for (const k of Object.keys(o.expr)) touch(o.expr, k);
    touch(o, 'Fexpr'); touch(o, 'FangExpr');
  }
  if (scene.settings.expr) for (const k of Object.keys(scene.settings.expr)) touch(scene.settings.expr, k);
  for (const v of scene.vars) touch(v, 'expr');
  for (const ov of scene.overlays || []) for (const k of ['ex', 'ey', 'expr']) touch(ov, k);
  return n;
}
/** 变量变化后：运行中实时生效；暂停时回到 t = 0（初始条件可能依赖变量），预测线随之更新 */
function onVarsChanged(light = false) {
  resolveExprs();
  if (sim.running) liveEdit();
  else if (sim.t > 0 && !light) initChanged();
  else { syncInitialLive(null); markFieldsDirty(); }
  markDirty(); requestDraw();
  updateVarRows();
}
function addVariable(name, expr, opts = {}) {
  pushUndo(name ? `新建变量 ${name}` : '新建变量');
  if (!scene.vars) scene.vars = [];
  const v = mkVar(name || nextVarName(), expr ?? '1');
  scene.vars.push(v);
  onVarsChanged(true);
  if (opts.focus === false) { if (leftTab === 'vars') refreshLeft(); else flashLeftTab('vars'); return v; }
  if (leftTab !== 'vars') setLeftTab('vars'); else refreshLeft();
  setTimeout(() => { const r = document.querySelector(`.vrow[data-var="${v.id}"] .v-expr`); if (r) { r.focus(); r.select(); } }, 0);
  return v;
}
/**
 * 公式输入框下方的辅助行（Desmos 式）：
 *  · 公式里出现了还不存在的名字（如 v0、θ）→ 给出「＋ 创建变量 v0」按钮，点一下就建好（初值 1，可拖滑块）
 *  · 「⌖ 引用物体」：在画布上点选一个物体，把「名称.」插到光标处
 */
function exprAssist(parent, input, o = {}) {
  const row = el('div', { class: 'expr-assist' });
  const update = () => {
    row.innerHTML = '';
    const unknown = findUnknownNames(input.value, exprNames(), varNames(), o.locals || []);
    for (const n of unknown.slice(0, 4)) {
      const b = el('button', { type: 'button', class: 'mk-var', title: `新建全局变量 ${n}（初值 1，可在「变量」面板拖滑块）` }, `＋ 创建变量 <b>${escapeHtml(n)}</b>`);
      b.addEventListener('mousedown', e => e.preventDefault());
      b.onclick = () => {
        addVariable(n, '1', { focus: false });
        input.dispatchEvent(new Event('input'));
        toast(`已创建变量 ${n} = 1，在「变量」面板里可以拖滑块调节`, { action: { label: '去调节', fn: () => setLeftTab('vars') } });
      };
      row.appendChild(b);
    }
    const ref = el('button', { type: 'button', class: 'pick-ref', title: '在画布上点选一个物体，插入对它的引用（如 小球2.vx）' }, '⌖ 引用物体');
    ref.addEventListener('mousedown', e => e.preventDefault());
    ref.onclick = () => {
      const a = input.selectionStart ?? input.value.length, z = input.selectionEnd ?? a;
      startPick({
        prompt: '点选要在公式里引用的物体', from: o.from, accept: x => !!x.name && (o.acceptRef ? o.acceptRef(x) : true),
        onPick: x => {
          const ins = `${x.name}.`;
          input.value = input.value.slice(0, a) + ins + input.value.slice(z);
          input.dispatchEvent(new Event('input'));
          input.focus(); input.setSelectionRange(a + ins.length, a + ins.length);
          toast(`已插入「${x.name}.」，接着写要引用的量，如 x、vx、v、m、F`);
        },
      });
    };
    row.appendChild(ref);
  };
  input.addEventListener('input', update);
  update();
  parent.appendChild(row);
  return row;
}
function deleteVariable(v) {
  pushUndo(`删除变量 ${v.name}`);
  scene.vars = scene.vars.filter(x => x !== v);
  onVarsChanged(); refreshLeft();
  toast(`已删除变量 ${v.name}；引用它的公式会显示错误`, { action: UNDO_ACTION });
}
function fmtVarValue(v) { return Number.isFinite(v.value) ? fmt(v.value, 6) : '—'; }
function updateVarRows() {
  for (const v of scene.vars || []) {
    const r = document.querySelector(`.vrow[data-var="${v.id}"]`); if (!r) continue;
    const err = v._exprErr && v._exprErr.value;
    const out = r.querySelector('.v-out'); if (out) { out.textContent = err ? err : isSliderVar(v) ? '' : `= ${fmtVarValue(v)}`; out.classList.toggle('err', !!err); }
    const sl = r.querySelector('.v-slider'); if (sl && document.activeElement !== sl && Number.isFinite(v.value)) sl.value = String(v.value);
    const ex = r.querySelector('.v-expr'); if (ex && document.activeElement !== ex && ex.value !== v.expr) ex.value = v.expr;
    const pl = r.querySelector('.v-play'); if (pl) pl.classList.toggle('on', !!v.anim);
  }
}
function renderVars(host) {
  const wrap = el('div', { class: 'vars' });
  wrap.appendChild(el('div', { class: 'vars-hint' }, '在任何 <i>ƒ</i> 公式里直接写变量名即可引用，例如质量写 <code>m₀</code>、初速度写 <code>v0*cos(θ)</code>。写成数字的变量会变成滑块。'));
  const list = scene.vars || [];
  if (!list.length) wrap.appendChild(el('div', { class: 'empty-hint' }, '还没有变量。'));
  for (const v of list) {
    const r = el('div', { class: 'vrow', 'data-var': String(v.id) });
    const top = el('div', { class: 'v-top' });
    const nm = el('input', { type: 'text', class: 'v-name', value: v.name, spellcheck: 'false', 'aria-label': '变量名' });
    const ex = el('input', { type: 'text', class: 'v-expr', value: v.expr, spellcheck: 'false', 'aria-label': `${v.name} 的值或表达式` });
    const del = el('button', { type: 'button', class: 'v-del', title: '删除变量', 'aria-label': '删除变量' }, '×');
    top.appendChild(nm); top.appendChild(el('span', { class: 'v-eq' }, '=')); top.appendChild(ex); top.appendChild(del);
    r.appendChild(top);
    const out = el('div', { class: 'v-out' });
    r.appendChild(out);
    const nmErr = el('div', { class: 'field-err' }); r.appendChild(nmErr);
    nm.addEventListener('change', () => {
      const nn = nm.value.trim();
      if (nn === v.name) return;
      const err = validateVarName(nn, varNames().filter(x => x !== v.name));
      if (err) { nmErr.textContent = err; nm.value = v.name; setTimeout(() => { nmErr.textContent = ''; }, 2600); return; }
      pushUndo('重命名变量');
      const old = v.name; v.name = nn;
      const n = renameVarRefs(old, nn);
      onVarsChanged(true); refreshLeft(); buildPanel();
      if (n) toast(`已重命名为 ${nn}，同步更新了 ${n} 处公式`);
    });
    nm.addEventListener('keydown', e => { if (e.key === 'Enter') nm.blur(); });
    let exPushed = false;
    ex.addEventListener('focus', () => { exPushed = false; });
    ex.addEventListener('input', () => {
      if (!exPushed) { pushUndo(`修改变量 ${v.name}`); exPushed = true; }
      const wasSlider = isSliderVar(v);
      v.expr = ex.value; delete v._raw;
      if (isSliderVar(v)) { const n = +v.expr; if (n < v.min) v.min = n; if (n > v.max) v.max = n; }
      onVarsChanged();
      if (wasSlider !== isSliderVar(v)) { refreshLeft(); setTimeout(() => { const e2 = document.querySelector(`.vrow[data-var="${v.id}"] .v-expr`); if (e2) { e2.focus(); const L = e2.value.length; e2.setSelectionRange(L, L); } }, 0); }
    });
    ex.addEventListener('keydown', e => { if (e.key === 'Enter') ex.blur(); });
    del.onclick = () => deleteVariable(v);
    if (isSliderVar(v)) {
      const sr = el('div', { class: 'v-slide' });
      const play = el('button', { type: 'button', class: 'v-play' + (v.anim ? ' on' : ''), title: '往返播放', 'aria-label': '往返播放' }, '<svg viewBox="0 0 16 16"><path d="M5 3v10l8-5z" fill="currentColor"/></svg>');
      const mn = el('input', { type: 'number', class: 'v-lim', value: fmtIn(v.min), title: '最小值', 'aria-label': '最小值' });
      const sl = el('input', { type: 'range', class: 'v-slider', min: String(v.min), max: String(v.max), step: String(v.step), value: String(+v.expr), 'aria-label': `${v.name} 滑块` });
      const mx = el('input', { type: 'number', class: 'v-lim', value: fmtIn(v.max), title: '最大值', 'aria-label': '最大值' });
      sr.appendChild(play); sr.appendChild(sl);
      r.appendChild(sr);
      const st = el('div', { class: 'v-steprow' });
      const stp = el('input', { type: 'number', class: 'v-lim', value: fmtIn(v.step), min: '0', title: '步长', 'aria-label': '步长' });
      const per = el('input', { type: 'number', class: 'v-lim', value: fmtIn(v.period), min: '0.1', title: '播放一个来回的时长（秒）', 'aria-label': '播放周期' });
      st.appendChild(mn); st.appendChild(el('span', {}, '≤ ' + escapeHtml(v.name) + ' ≤')); st.appendChild(mx);
      st.appendChild(el('span', { class: 'v-gap' }, '步长')); st.appendChild(stp);
      st.appendChild(el('span', { class: 'v-gap', title: '▶ 往返播放一个来回的时长' }, '周期')); st.appendChild(per); st.appendChild(el('span', {}, 's'));
      r.appendChild(st);
      let slPushed = false;
      sl.addEventListener('pointerdown', () => { slPushed = false; });
      sl.addEventListener('input', () => {
        if (!slPushed) { pushUndo(`调节 ${v.name}`); slPushed = true; }
        v.expr = String(+(+sl.value).toPrecision(10)); ex.value = v.expr; delete v._raw;
        onVarsChanged();
      });
      const lim = (inp, key, check) => inp.addEventListener('change', () => {
        const n = parseFloat(inp.value);
        const err = !Number.isFinite(n) ? '请输入数字' : check(n);
        if (err) { nmErr.textContent = err; inp.value = fmtIn(v[key]); setTimeout(() => { nmErr.textContent = ''; }, 2600); return; }
        pushUndo('修改滑块范围'); v[key] = n;
        sl.min = String(v.min); sl.max = String(v.max); sl.step = String(v.step);
        markDirty();
      });
      lim(mn, 'min', n => n >= v.max ? '最小值必须小于最大值' : '');
      lim(mx, 'max', n => n <= v.min ? '最大值必须大于最小值' : '');
      lim(stp, 'step', n => n <= 0 ? '步长必须大于 0' : '');
      lim(per, 'period', n => n <= 0 ? '周期必须大于 0' : '');
      play.onclick = () => { v.anim = !v.anim; play.classList.toggle('on', v.anim); markDirty(); if (v.anim) lastVarAnim = 0; };
    }
    wrap.appendChild(r);
  }
  const add = el('button', { type: 'button', class: 'v-add' }, '＋ 新建变量');
  add.onclick = () => addVariable();
  wrap.appendChild(add);
  host.appendChild(wrap);
  updateVarRows();
  const cnt = $('#layerCount'); if (cnt) cnt.textContent = list.length ? `${list.length} 个变量` : '';
}
/** 滑块往返播放：每帧推进，按步长取整 */
let lastVarAnim = 0;
function animateVars(ts) {
  const list = (scene.vars || []).filter(v => v.anim && isSliderVar(v));
  if (!list.length || !docOpen) { lastVarAnim = 0; return; }
  const dt = lastVarAnim ? Math.min(0.1, (ts - lastVarAnim) / 1000) : 0;
  lastVarAnim = ts;
  if (!dt) return;
  for (const v of list) {
    const span = v.max - v.min; if (!(span > 0)) continue;
    let x = (Number.isFinite(v._raw) ? v._raw : +v.expr) + v._dir * 2 * span / v.period * dt;
    if (x >= v.max) { x = v.max; v._dir = -1; } else if (x <= v.min) { x = v.min; v._dir = 1; }
    const q = v.step > 0 ? Math.round((x - v.min) / v.step) * v.step + v.min : x;
    v._raw = x;
    v.expr = String(+q.toPrecision(10));
  }
  onVarsChanged(true);
}

function varNames() { return (scene.vars || []).map(v => v.name); }
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
function hasExpr(owner, key) { return !!(owner && owner.expr && owner.expr[key] != null && String(owner.expr[key]).trim() !== ''); }
function readQuantity(id, field) {
  if (id === -1) { const v = (scene.vars || []).find(x => x.name === field); return v && Number.isFinite(v.value) ? v.value : NaN; }
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
    case 'omega': return o.st ? (o.st.omega || 0) : (o.omega0 || 0);
    case 'omega0': return o.omega0 || 0;
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
  if (owner._isVar) { owner.value = value; return; }
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
  // 全局变量：槽位 id 为 “-1:名字”，与表达式里的 __ref(-1,"名字") 依赖对应
  for (const v of scene.vars || []) {
    const src = String(v.expr ?? '').trim();
    if (!src) { v._exprErr = { value: '表达式为空' }; continue; }
    slots.push({ owner: v, key: 'value', src, id: `-1:${v.name}` });
  }
  return slots;
}
function resolveExprs() {
  if (resolveExprs.busy) return;
  resolveExprs.busy = true;
  try {
    for (const b of bodies()) b._hold = null;
    const slots = collectExprSlots();
    const bySlot = new Map(slots.map(s => [s.id, s]));
    const names = exprNames(), vars = varNames();
    const nodes = slots.map(s => {
      const refs = rewriteExpr(s.src, names, vars).refs || [];
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
      const r = evalForceExpr(s.src, ctx, { names, vars, ref });
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
    if (b._grab) { const g = b._grab; st.x = g.x; st.y = g.y; st.vx = g.vx; st.vy = g.vy; st.omega = 0; continue; }
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
    st.ang += (st.omega || 0) * h;
  }
  // 轻杆：刚性长度约束（SHAKE，在接触前满足约束）
  shakeRods(h, true);
  // 接触（engine/rigid.js）：收集穿透点 → 顺序冲量（含转动、静/动摩擦、两点块求解）→ 位置修正
  const bl = bodies().filter(b => b.collide);
  const contacts = collectContacts(movers, bl, segs);
  if (contacts.length) {
    const pairs = contacts.some(c => c.a);
    solveVelocities(contacts, h, pairs ? 16 : 8);
    for (const c of contacts) {
      const nx = c.nx * c.jn / h, ny = c.ny * c.jn / h, fx = c.tx * c.jt / h, fy = c.ty * c.jt / h;
      const B = c.b.st; B.contactN[0] += nx; B.contactN[1] += ny; B.contactF[0] += fx; B.contactF[1] += fy;
      if (c.a) { const A = c.a.st; A.contactN[0] -= nx; A.contactN[1] -= ny; A.contactF[0] -= fx; A.contactF[1] -= fy; }
    }
    solvePositions(() => collectContacts(movers, bl, segs), 3);
  }
  // 接触可能破坏杆长，再做一次 SHAKE
  shakeRods(h, false);
  reapplyHolds();
  // 轨迹：按模拟时间等间隔采样（与缩放无关），速度方向突变（碰撞）时额外采样
  if (!sim.predicting) {
    const tn = sim.t + h;
    for (const b of movers) {
      const tr = b.st.trail;
      if (sampleInto(tr, b.st, tn) && tr.length > TRAIL_MAX) { const n = tr.length - TRAIL_MAX; tr.splice(0, n); b.st.trailDrop = (b.st.trailDrop || 0) + n; }
    }
  }
}
/* ---------------- 轨迹采样 ---------------- */
const TRAIL_DT = 1 / 240;      // 采样间隔（模拟时间）
const TRAIL_MAX = 24000;       // 每个物体最多保留的样本数（100 s @ 240 Hz）
const KINK_COS = Math.cos(10 * DEG);
/** 样本格式：[x, y, t, vx, vy] */
function trailSample(st, t) { return [st.x, st.y, t, st.vx, st.vy]; }
function resetTrail(b) { if (b.st) b.st.trail = [trailSample(b.st, sim.t)]; }
function sampleInto(arr, st, t) {
  const last = arr[arr.length - 1];
  if (!last) { arr.push(trailSample(st, t)); return true; }
  let take = t - last[2] >= TRAIL_DT - 1e-12;
  if (!take) {
    const v1 = Math.hypot(last[3], last[4]), v2 = Math.hypot(st.vx, st.vy);
    if (v1 > 1e-6 && v2 > 1e-6 && (last[3] * st.vx + last[4] * st.vy) / (v1 * v2) < KINK_COS) take = true;
    else if (Math.abs(v2 - v1) > 0.2 * Math.max(v1, v2, 1e-6) && Math.abs(v2 - v1) > 0.05) take = true;
  }
  if (take) arr.push(trailSample(st, t));
  return take;
}
/** 在样本序列上按时间插值：返回 [x, y, t, vx, vy] 或 null */
function curveAt(arr, t) {
  if (!arr || !arr.length || t < arr[0][2] - 1e-9 || t > arr[arr.length - 1][2] + 1e-9) return null;
  let lo = 0, hi = arr.length - 1;
  while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (arr[mid][2] <= t) lo = mid; else hi = mid; }
  const a = arr[lo], b = arr[hi], span = b[2] - a[2];
  const u = span > 1e-12 ? Math.min(1, Math.max(0, (t - a[2]) / span)) : 0;
  return [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, t, a[3] + (b[3] - a[3]) * u, a[4] + (b[4] - a[4]) * u];
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
    Ek += 0.5 * b.m * v2 + rotationalKE(b); px += b.m * (p.vx || 0); py += b.m * (p.vy || 0); M += b.m;
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
    b: bodies().filter(b => b.st).map(b => { const st = b.st; return [b.id, st.x, st.y, st.vx, st.vy, st.ang, st.trail.length + (st.trailDrop || 0), st.hist.length + (st.histDrop || 0), st.ax || 0, st.ay || 0, st.omega || 0]; }) });
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
    st.x = r[1]; st.y = r[2]; st.vx = r[3]; st.vy = r[4]; st.ang = r[5]; st.ax = r[8]; st.ay = r[9]; st.omega = r[10] || 0;
    st.trail.length = Math.max(1, Math.min(st.trail.length, r[6] - (st.trailDrop || 0)));
    st.hist.length = Math.max(1, Math.min(st.hist.length, r[7] - (st.histDrop || 0)));
    st.contactN = [0, 0]; st.contactF = [0, 0];
  }
  sim.frameIdx = i; sim.accum = 0;
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
/**
 * 固定步长推进：子步长度恒为 Δt（= 设置中的积分步长），与帧率无关。
 * 帧时间累加到 sim.accum，凑满整步才积分，余量留到下一帧。
 * 因此结果可复现，数值预测与实际运行走在同一个时间网格上。
 */
function advance(dtFrame) {
  ensureStates();
  commitScrub();
  const h = Math.max(1e-9, scene.settings.maxDt);
  sim.accum += dtFrame;
  let n = Math.floor((sim.accum + 1e-9 * h) / h);
  if (n > 20000) { n = 20000; sim.accum = n * h; }   // 跑不动时丢弃积压，避免卡死
  if (n < 1) return;
  sim.accum = Math.max(0, sim.accum - n * h);
  dtFrame = n * h;
  const segs = getSegments();
  for (const b of bodies()) { b.st.contactN = [0, 0]; b.st.contactF = [0, 0]; b.st.pvx = b.st.vx; b.st.pvy = b.st.vy; }
  // 子步内推进 sim.t，使随时间变化的力（受迫振动、交变电场）在每个子步取到正确的 t
  const t0 = sim.t;
  for (let i = 0; i < n; i++) {
    sim.t = t0 + i * h;
    syncBodyEmitterPositions();
    substep(h, segs);
  }
  sim.t = t0 + dtFrame;
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
/**
 * 数值显示：最多 d 位小数、最多 6 位有效数字；很小或很大的数用 a×10ⁿ 记法。
 * 负号用真正的减号 “−”，与公式排版一致。
 */
const SUP = { '-': '⁻', 0: '⁰', 1: '¹', 2: '²', 3: '³', 4: '⁴', 5: '⁵', 6: '⁶', 7: '⁷', 8: '⁸', 9: '⁹' };
function fmt(v, d = 3) {
  if (v == null || !isFinite(v)) return '—';
  const a = Math.abs(v);
  if (a < 1e-12) return '0';
  const sign = v < 0 ? '−' : '';
  if (a < 1e-3 || a >= 1e5) {
    let [m, e] = a.toExponential(Math.max(1, Math.min(d, 3))).split('e');
    m = String(+m);
    const n = String(+e).split('').map(c => SUP[c]).join('');
    return `${sign}${m}×10${n}`;
  }
  const digits = Math.max(0, Math.min(d, 6 - Math.max(1, Math.floor(Math.log10(a)) + 1)));
  return sign + String(+a.toFixed(digits));
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
  const [x, y, w, h] = fieldScreenRect(f), sel = isSel(f);
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
  const sel = false, col = C['--surface'];
  ctx.save(); ctx.lineCap = 'round';
  if (o.kind === 'ground') {
    const y = w2s(0, o.y)[1];
    ctx.strokeStyle = col; ctx.lineWidth = sel ? 3 : 2; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
    hatchLine(0, y, W, y, -1, col);
    if (o.smooth) label('光滑', 8, y + 14, C['--muted'], 'left', false);
  } else if (o.kind === 'wall') {
    const [a, b] = w2s(o.x1, o.y1), [c, d] = w2s(o.x2, o.y2);
    ctx.strokeStyle = col; ctx.lineWidth = sel ? 4 : 3; ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
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
  const [x1, y1] = w2s(e.x1, e.y1), [x2, y2] = w2s(e.x2, e.y2), sel = false;
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
    if (canRotate(b) && r > 5) {
      // 姿态标记：一条半径，转动时随之旋转
      ctx.save(); ctx.strokeStyle = 'rgba(255,255,255,.75)'; ctx.lineWidth = Math.max(1.2, r * 0.08); ctx.lineCap = 'round';
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + Math.cos(ang) * r * 0.82, y - Math.sin(ang) * r * 0.82); ctx.stroke(); ctx.restore();
    }
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
  const st = b.st, [x, y] = w2s(st.x, st.y), S = scene.settings;
  if (b.hidden) return;
  drawBodyShape(b, x, y, st.ang, false);
  if (b.fixed) { ctx.save(); ctx.fillStyle = C['--text']; ctx.beginPath(); ctx.arc(x, y, 2.5, 0, 7); ctx.fill(); ctx.restore(); }
}
/* =====================================================================
 *  显示项（overlays）：画布上的一切附加显示都来自 scene.overlays
 *  · 默认一条都没有；在「显示」面板或物体属性里逐项添加
 *  · 每条可设对象（全部 / 选中 / 某个物体）、颜色、比例、是否标数值
 *  · 编辑手柄（拖初速度、拖外力）与显示无关：只在选中、暂停时出现，且总画在物体当前位置
 * ===================================================================== */
let ovSeq = 1;
function nextOverlayId() { return ovSeq++; }
function overlays() { return scene.overlays || (scene.overlays = []); }
function serializeOverlays() {
  return overlays().map(o => { const c = {}; for (const k of Object.keys(o)) if (k[0] !== '_') c[k] = o[k]; return c; });
}
function loadOverlays(list) {
  ovSeq = 1;
  const ids = new Set();
  scene.overlays = (list || []).map(r => normalizeOverlay(r, () => 0)).filter(Boolean);
  ovSeq = scene.overlays.reduce((m, o) => Math.max(m, o.id), 0) + 1;
  for (const o of scene.overlays) { if (!o.id || ids.has(o.id)) o.id = ovSeq++; ids.add(o.id); }
}
/** 有没有依赖“当前选择”的显示项（选择变化时需要重画） */
function ovSelDependent() { return overlays().some(o => o.on && o.target === 'sel'); }
function ovShownBodies() { return bodies().filter(b => !b.hidden && !(groupOf(b) && groupOf(b).hidden) && b.st); }
function ovApplies(ov, b) {
  if (!ov.on || OVERLAY_TYPES[ov.type].noTarget) return false;
  return ov.target === 'all' || (ov.target === 'sel' ? selSet.has(b.id) : ov.target === b.id);
}
function ovTargets(ov) { return ov.on ? ovShownBodies().filter(b => ovApplies(ov, b)) : []; }
function ovActive(type) { return overlays().some(o => o.on && o.type === type); }
function ovColor(ov, b) { return ov.color || OVERLAY_TYPES[ov.type].color || (b && b.color) || C['--muted']; }
function maxPredictT() { let T = 0; for (const o of overlays()) if (o.on && o.type === 'predict') T = Math.max(T, o.T || 3); return T; }
function ovTargetText(ov) {
  if (OVERLAY_TYPES[ov.type].noTarget) return '整个场景';
  if (ov.target === 'all') return '全部物体';
  if (ov.target === 'sel') return '选中的物体';
  const b = byId(ov.target); return b ? b.name : '（已删除的物体）';
}

/* ---------- 瞬时动力学：加速度、接触力、杆的约束力 ----------
 * 运行中直接用上一帧的结果；暂停时用同一个引擎从当前状态试走一个子步再还原，
 * 所以 t = 0 时也能显示加速度、支持力和摩擦力，且与实际运行完全一致。 */
let dynCache = { key: '', map: null };
let frameKey = null;
function sceneKey() { return frameKey ?? (frameKey = predictionKey()); }
function probeDynamics() {
  ensureStates();
  const out = new Map(), rodN = new Map();
  const h = Math.max(1e-9, scene.settings.maxDt);
  const saved = saveSimState();
  sim.predicting = true;
  try {
    for (const b of bodies()) { b.st.contactN = [0, 0]; b.st.contactF = [0, 0]; b.st.pvx = b.st.vx; b.st.pvy = b.st.vy; }
    syncBodyEmitterPositions();
    substep(h, getSegments());
    for (const b of bodies()) {
      const st = b.st, fx = b.fixed || b._grab;
      out.set(b.id, { ax: fx ? 0 : (st.vx - st.pvx) / h, ay: fx ? 0 : (st.vy - st.pvy) / h, N: st.contactN.slice(), f: st.contactF.slice() });
    }
    for (const s of scene.objects) if (s.kind === 'rod') rodN.set(s.id, s._N || 0);
  } finally {
    sim.predicting = false;
    restoreSimState(saved);
  }
  out.rodN = rodN;
  return out;
}
function dyn(b) {
  if (sim.running || drag && drag.type === 'grab') {
    const st = b.st;
    return { ax: st.ax || 0, ay: st.ay || 0, N: st.contactN, f: st.contactF, rodN: s => s._N || 0 };
  }
  const key = sceneKey();
  if (dynCache.key !== key || !dynCache.map) dynCache = { key, map: probeDynamics() };
  const d = dynCache.map.get(b.id) || { ax: 0, ay: 0, N: [0, 0], f: [0, 0] };
  const rm = dynCache.map.rodN;
  return Object.assign({ rodN: s => rm.get(s.id) || 0 }, d);
}
/** 物体受到的每一个力（受力分析） */
function forceParts(b) {
  const d = dyn(b), st = b.st;
  const parts = computeForces([b], true).get(b).parts.slice();
  for (const s of scene.objects) {
    if (s.kind !== 'rod' || (s.a !== b.id && s.b !== b.id)) continue;
    const e = linkEnds(s); if (!e) continue;
    const N = d.rodN(s); if (Math.abs(N) < 1e-9) continue;
    const dx = e.x2 - e.x1, dy = e.y2 - e.y1, L = Math.hypot(dx, dy) || 1e-9, sign = s.a === b.id ? 1 : -1;
    parts.push({ fx: sign * N * dx / L, fy: sign * N * dy / L, label: 'N杆', color: '#4d6470' });
  }
  if (b.q) {
    const Bz = fieldAt(st.x, st.y).Bz;
    if (Bz) parts.push({ fx: b.q * st.vy * Bz, fy: -b.q * st.vx * Bz, label: 'F洛', color: '#4a78b0' });
  }
  if (Math.hypot(d.N[0], d.N[1]) > 1e-6) parts.push({ fx: d.N[0], fy: d.N[1], label: 'N', color: '#3f8fb0' });
  if (Math.hypot(d.f[0], d.f[1]) > 1e-6) parts.push({ fx: d.f[0], fy: d.f[1], label: 'f', color: '#9a6a2a' });
  return parts.filter(p => Math.hypot(p.fx, p.fy) > 1e-9);
}
/** 自定义显示公式的求值语境：本物体的 t,x,y,vx,vy,v,m,q,g 之外，再加 ax,ay,a,omega,ang,Ek,Ep,px,py,p,Fx,Fy,F */
function displayCtx(b) {
  const c = forceEvalCtx(b), d = dyn(b), st = b.st, S = scene.settings;
  const Fx = b.m * d.ax, Fy = b.m * d.ay;
  return Object.assign(c, {
    ax: d.ax, ay: d.ay, a: Math.hypot(d.ax, d.ay), omega: st.omega || 0, ang: (st.ang || 0) / DEG,
    Ek: 0.5 * b.m * (st.vx * st.vx + st.vy * st.vy) + rotationalKE(b), Ep: S.gOn ? b.m * S.g * st.y : 0,
    px: b.m * st.vx, py: b.m * st.vy, p: b.m * Math.hypot(st.vx, st.vy), Fx, Fy, F: Math.hypot(Fx, Fy),
  });
}
function evalDisplay(src, b) {
  return evalForceExpr(src, displayCtx(b), { names: exprNames(), vars: varNames(), ref: readQuantity, locals: DISPLAY_LOCALS });
}
/** 一条矢量显示项在某物体上的值 [x, y]；出错返回 { err } */
function vecValue(ov, b) {
  const st = b.st;
  switch (ov.type) {
    case 'v': return [st.vx, st.vy];
    case 'p': return [b.m * st.vx, b.m * st.vy];
    case 'a': { if (b.fixed) return [0, 0]; const d = dyn(b); return [d.ax, d.ay]; }
    case 'Fnet': { if (b.fixed) return [0, 0]; const d = dyn(b); return [b.m * d.ax, b.m * d.ay]; }
    case 'F': { ensureForceFields(b); const af = appliedForceOn(b); return [af.fx, af.fy]; }
    case 'vec': {
      const x = evalDisplay(ov.ex, b), y = evalDisplay(ov.ey, b);
      if (!x.ok) return { err: `x 分量：${x.error}` };
      if (!y.ok) return { err: `y 分量：${y.error}` };
      return [x.value, y.value];
    }
  }
  return [0, 0];
}
/** 自动比例：让最长的箭头约 80 px；变化不大时保持不变，避免运行中箭头比例来回跳 */
function vecScale(ov, maxMag) {
  if (ov.scale > 0) return ov.scale;
  if (!(maxMag > 1e-12)) return ov._auto || 1;
  const target = 80 / maxMag, cur = ov._auto;
  if (!cur || target / cur > 1.25 || target / cur < 0.4) ov._auto = target;
  return ov._auto;
}
function vecText(sym, mag, unit, ang) {
  return `${sym ? sym + ' = ' : ''}${fmt(mag, 3)}${unit ? ' ' + unit : ''}${ang != null && mag > 1e-12 ? `  ∠${fmt(ang, 1)}°` : ''}`;
}
function drawVec(x, y, vx, vy, k, color, text, comps, width = 2.2) {
  const hx = x + vx * k, hy = y - vy * k, L = Math.hypot(hx - x, hy - y);
  if (L < 1.5) return;
  if (comps && Math.abs(vx * k) > 2 && Math.abs(vy * k) > 2) {
    ctx.save(); ctx.strokeStyle = color; ctx.globalAlpha = 0.55; ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(hx, y); ctx.lineTo(hx, hy); ctx.moveTo(x, y); ctx.lineTo(x, hy); ctx.lineTo(hx, hy); ctx.stroke(); ctx.restore();
  }
  arrow(x, y, hx, hy, color, width, 9);
  if (text) {
    const ux = (hx - x) / L, uy = (hy - y) / L;
    label(text, hx + ux * 8 + (ux >= 0 ? 4 : -4), hy + uy * 8, color, ux >= -0.2 ? 'left' : 'right', true);
  }
}
/** 画所有矢量类显示项。返回图例条目 */
function drawVectorOverlays(legend) {
  for (const ov of overlays()) {
    if (!ov.on || !isVecType(ov.type)) continue;
    const T = OVERLAY_TYPES[ov.type], targets = ovTargets(ov);
    if (!targets.length) continue;
    if (ov.type === 'forces') {
      const all = targets.map(b => [b, forceParts(b)]);
      let mx = 0; for (const [, ps] of all) for (const p of ps) mx = Math.max(mx, Math.hypot(p.fx, p.fy));
      const k = vecScale(ov, mx);
      for (const [b, ps] of all) {
        const [x, y] = w2s(b.st.x, b.st.y);
        for (const p of ps) drawVec(x, y, p.fx, p.fy, k, ov.color || p.color, ov.value ? `${p.label} ${fmt(Math.hypot(p.fx, p.fy), 3)} N` : p.label, ov.comps, 2);
      }
      legend.push({ kind: 'vec', color: ov.color || C['--muted'], text: `受力 · ${scaleText(k, 'N')}` });
      continue;
    }
    const vals = targets.map(b => [b, vecValue(ov, b)]);
    let mx = 0, err = null;
    for (const [, v] of vals) { if (v.err) { err = err || v.err; continue; } mx = Math.max(mx, Math.hypot(v[0], v[1])); }
    ov._err = err;
    const k = vecScale(ov, mx);
    const sym = ov.type === 'vec' ? (ov.label || 'u') : T.sym, unit = ov.type === 'vec' ? ov.unit : T.unit;
    for (const [b, v] of vals) {
      if (v.err) continue;
      const [x, y] = w2s(b.st.x, b.st.y), mag = Math.hypot(v[0], v[1]);
      drawVec(x, y, v[0], v[1], k, ovColor(ov, b), ov.value ? vecText(sym, mag, unit, Math.atan2(v[1], v[0]) / DEG) : '', ov.comps);
    }
    legend.push({ kind: 'vec', color: ovColor(ov, targets[0]), text: `${ov.type === 'vec' ? sym : T.name} · ${scaleText(k, unit)}` });
  }
}
function scaleText(k, unit) {
  // 图例里给出“多少单位画成 50 px”，取整到 1/2/5
  const per = niceStep(50 / k);
  return `${fmt(per * k, 2)} px = ${fmt(per, 3)}${unit ? ' ' + unit : ''}`;
}

/* ---------- 标注 ---------- */
function drawLabelOverlays() {
  const stack = new Map();
  for (const ov of overlays()) {
    if (!ov.on) continue;
    if (ov.type === 'names') {
      for (const b of ovTargets(ov)) { const [x, y] = w2s(b.st.x, b.st.y); label(b.name, x, y + bodyScreenR(b) + 13, ov.color || C['--muted'], 'center', false); }
      if (ov.target === 'all') for (const o of scene.objects) if (o.kind === 'pointCharge' && !o.hidden) { const [x, y] = w2s(o.x, o.y); label(`${o.name} (${o.q >= 0 ? '+' : ''}${fmt(o.q, 2)} C)`, x, y + Math.max(8, (o.softRadius || 0.08) * scene.view.scale) + 12, C['--muted'], 'center', false); }
    } else if (ov.type === 'value') {
      ov._err = null;
      for (const b of ovTargets(ov)) {
        const r = evalDisplay(ov.expr, b);
        if (!r.ok) ov._err = r.error;
        const i = stack.get(b.id) || 0; stack.set(b.id, i + 1);
        const [x, y] = w2s(b.st.x, b.st.y), R = bodyScreenR(b);
        const txt = r.ok ? `${ov.label || ov.expr} = ${fmt(r.value, ov.digits || 4)}${ov.unit ? ' ' + ov.unit : ''}` : `${ov.label || ov.expr}：${r.error}`;
        label(txt, x + R + 8, y - R - 6 - i * 19, r.ok ? (ov.color || C['--text']) : C['--danger'] || '#c0392b', 'left', true);
      }
    }
  }
}
function drawGhostOverlays() {
  if (sim.t <= 0) return;
  for (const ov of overlays()) {
    if (!ov.on || ov.type !== 'ghost') continue;
    for (const b of ovTargets(ov)) { const [x, y] = w2s(b.x0, b.y0); drawBodyShape(b, x, y, (b.ang0 || 0) * DEG, true); }
  }
}

/* ---------- 编辑手柄：初速度（仅 t = 0）· 外力（恒力） ---------- */
let hoverHandle = null;
function handleBody() {
  const s = selected();
  if (tool !== 'select' || sim.running || selSet.size !== 1 || !isBody(s) || s.locked || s.hidden || !s.st) return null;
  if (groupOf(s) && (groupOf(s).locked || groupOf(s).hidden)) return null;
  return s;
}
function handleScale(b, key) {
  const type = key === 'v0' ? 'v' : 'F';
  const ov = overlays().find(o => o.on && o.type === type && ovApplies(o, b));
  if (ov) return ov.scale > 0 ? ov.scale : (ov._auto || (key === 'v0' ? 20 : 12));
  return key === 'v0' ? (scene.settings.vPx || 20) : (scene.settings.FPx || 12);
}
function editHandles() {
  const b = handleBody(); if (!b) return [];
  const out = [], [x, y] = w2s(b.st.x, b.st.y);
  if (sim.t === 0 && !b.fixed) {
    const k = handleScale(b, 'v0'), vx = b.vx0 || 0, vy = b.vy0 || 0;
    let hx = x + vx * k, hy = y - vy * k;
    const zero = Math.hypot(hx - x, hy - y) < 6;
    if (zero) { hx = x + bodyScreenR(b) + 16; hy = y; }
    out.push({ key: 'v0', b, k, x, y, hx, hy, zero, color: '#4a8a4f', text: zero ? '拖动设置初速度 v₀' : vecText('v₀', Math.hypot(vx, vy), 'm/s', Math.atan2(vy, vx) / DEG) });
  }
  ensureForceFields(b);
  if (b.Fmode === 'const' && b.F > 1e-12 && !hasExpr(b, 'F') && !hasExpr(b, 'Fang')) {
    const k = handleScale(b, 'F'), fx = b.F * Math.cos(b.Fang * DEG), fy = b.F * Math.sin(b.Fang * DEG);
    out.push({ key: 'F', b, k, x, y, hx: x + fx * k, hy: y - fy * k, zero: false, color: '#8a6bbe', text: vecText('F', b.F, 'N', b.Fang) });
  }
  return out;
}
function hitHandle(sx, sy) {
  let best = null, bestD = pointerCoarse ? 24 : 11;
  for (const h of editHandles()) { const d = Math.hypot(sx - h.hx, sy - h.hy); if (d < bestD) { best = h; bestD = d; } }
  return best;
}
function drawEditHandles() {
  const b = handleBody();
  if (b && b.Fmode === 'aim') {
    const aim = resolveAimPoint(b), [x, y] = w2s(b.st.x, b.st.y), [ax, ay] = w2s(aim.x, aim.y);
    ctx.save(); ctx.strokeStyle = 'rgba(138,107,190,.55)'; ctx.setLineDash([3, 3]); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(ax, ay); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = '#8a6bbe'; ctx.beginPath(); ctx.arc(ax, ay, 3.5, 0, 7); ctx.fill(); ctx.restore();
  }
  for (const h of editHandles()) {
    const active = (drag && drag.type === h.key && drag.b === h.b) || (hoverHandle && hoverHandle.key === h.key && hoverHandle.b === h.b);
    if (h.zero) { ctx.save(); ctx.globalAlpha = 0.7; arrow(h.x, h.y, h.hx, h.hy, h.color, 1.6, 8, [4, 3]); ctx.restore(); }
    else arrow(h.x, h.y, h.hx, h.hy, h.color, active ? 3 : 2.4, 11);
    handleDot(h.hx, h.hy, h.color, active ? 7.5 : 6);
    if (active || !h.zero) label(h.text, h.hx + 12, h.hy - 12, h.color, 'left', true);
  }
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
  ctx.restore();
}
/* =====================================================================
 *  数值预测：用同一个引擎从当前状态向前积分，结束后完整还原状态
 * ===================================================================== */
const PRED_MAX_STEPS = 20000;
const ST_KEYS = ['x', 'y', 'vx', 'vy', 'ang', 'omega', 'ax', 'ay', 'pvx', 'pvy', '_ox', '_oy', 'cN'];
function saveSimState() {
  return {
    t: sim.t, wE: sim.wE, wExt: sim.wExt,
    bodies: bodies().filter(b => b.st).map(b => {
      const o = {}; for (const k of ST_KEYS) o[k] = b.st[k];
      o.contactN = b.st.contactN.slice(); o.contactF = b.st.contactF.slice();
      return [b, o];
    }),
    links: scene.objects.filter(isLink).map(l => [l, l._N]),
  };
}
function restoreSimState(S) {
  sim.t = S.t; sim.wE = S.wE; sim.wExt = S.wExt;
  for (const [b, o] of S.bodies) if (b.st) Object.assign(b.st, o);
  for (const [l, N] of S.links) l._N = N;
  resolveExprs();          // 公式驱动的参数（外力、交变场）回到当前时刻的值
  fieldsDirty = true;
  syncBodyEmitterPositions();
}
let predCache = { key: '', paths: null, ms: 0, T: 0 };
function predictionKey() {
  const st = bodies().map(b => b.st ? `${b.st.x},${b.st.y},${b.st.vx},${b.st.vy}` : '').join('|');
  const S = scene.settings;
  return JSON.stringify(scene.objects, (k, v) => (k === 'st' || k[0] === '_') ? undefined : v)
    + `|${(scene.vars || []).map(v => v.name + '=' + v.value).join(',')}|${sim.t}|${maxPredictT()}|${S.maxDt}|${S.gOn}|${S.g}|${S.drag}|${S.gravity}|${S.Gc}|${S.coulomb}|${S.coulombLaw}|${st}`;
}
function computePrediction(Tfix) {
  const S = scene.settings, T = Math.max(0.05, Tfix || maxPredictT() || 3);
  ensureStates();
  const movers = bodies().filter(b => !b.fixed && b.st);
  const out = new Map();
  if (!movers.length) return out;
  const t0 = performance.now();
  const saved = saveSimState();
  sim.predicting = true;
  try {
    for (const b of movers) out.set(b.id, [trailSample(b.st, sim.t)]);
    const segs = getSegments();
    // 与实际运行相同的步长 Δt，保证预测与实际逐步一致；步数过多时截短预测时长
    const h = Math.max(1e-9, S.maxDt);
    let n = Math.max(1, Math.min(PRED_MAX_STEPS, Math.round(T / h)));
    // 交互预算：按上次实测的单步耗时把总耗时控制在 ~35 ms 内（拖动时保持跟手），图例会显示实际预测时长
    if (predCache.msPerStep) {
      const per01 = Math.max(1, Math.round(0.1 / h));   // 预算截短时按 0.1 s 取整，图例上显示整齐的时长
      // 拖动中保持跟手（35 ms）；静止时只在参数变化后算一次，可以放宽到 150 ms
      const budget = Math.max(200, Math.floor((drag ? 35 : 150) / predCache.msPerStep));
      if (budget < n) n = Math.max(per01, Math.floor(budget / per01) * per01);
    }
    const tStart = sim.t;
    predCache.steps = n;
    for (let i = 0; i < n; i++) {
      sim.t = tStart + i * h;
      syncBodyEmitterPositions();
      substep(h, segs);
      const tn = tStart + (i + 1) * h;
      for (const b of movers) {
        const st = b.st;
        if (!Number.isFinite(st.x) || !Number.isFinite(st.y)) continue;
        const arr = out.get(b.id);
        if (i === n - 1) arr.push(trailSample(st, tn)); else sampleInto(arr, st, tn);
      }
    }
  } finally {
    sim.predicting = false;
    restoreSimState(saved);
  }
  predCache.ms = performance.now() - t0;
  const per = predCache.ms / Math.max(1, predCache.steps || 1);
  predCache.msPerStep = predCache.msPerStep ? predCache.msPerStep * 0.5 + per * 0.5 : per;
  predCache.T = (predCache.steps || 0) * Math.max(1e-9, S.maxDt);
  return out;
}
function getPrediction() {
  if (!maxPredictT() || sim.running || (drag && drag.type === 'grab')) return null;
  const key = predictionKey();
  if (key !== predCache.key) { predCache.key = key; predCache.paths = computePrediction(); }
  return predCache.paths;
}

/* =====================================================================
 *  曲线绘制：已走轨迹（实线）· 数值预测（虚线）· 解析解（点线）· 频闪点
 * ===================================================================== */
function strokePath(arr, extra) {
  ctx.beginPath();
  let started = false;
  for (const p of arr) { const [a, c] = w2s(p[0], p[1]); if (started) ctx.lineTo(a, c); else { ctx.moveTo(a, c); started = true; } }
  if (extra) { const [a, c] = w2s(extra[0], extra[1]); ctx.lineTo(a, c); }
  ctx.stroke();
}
/** 频闪点：在 t = k·Δt 处插值画点；屏幕间距过小时跳过，保证可读 */
function drawStrobe(arr, color, hollow, dt) {
  if (!(dt > 0) || !arr || arr.length < 2) return;
  const tA = arr[0][2], tB = arr[arr.length - 1][2];
  let k = Math.ceil((tA - 1e-9) / dt);
  if ((tB - tA) / dt > 4000) return;
  ctx.save(); ctx.fillStyle = hollow ? C['--canvas'] : color; ctx.strokeStyle = color; ctx.lineWidth = 1.3;
  let px = -1e9, py = -1e9, j = 0;
  for (; k * dt <= tB + 1e-9; k++) {
    const t = k * dt;
    while (j < arr.length - 2 && arr[j + 1][2] < t) j++;
    const a = arr[j], b = arr[j + 1] || a, span = b[2] - a[2];
    const u = span > 1e-12 ? Math.min(1, Math.max(0, (t - a[2]) / span)) : 0;
    const [sx, sy] = w2s(a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u);
    if (Math.hypot(sx - px, sy - py) < 5) continue;
    px = sx; py = sy;
    ctx.beginPath(); ctx.arc(sx, sy, 2.4, 0, 7);
    if (hollow) { ctx.fill(); ctx.stroke(); } else ctx.fill();
  }
  ctx.restore();
}
function liveTrail(b) { return b.st.trail.concat([trailSample(b.st, sim.t)]); }
/** 预测曲线（截到该显示项的时长 T） */
function predFor(b, T) {
  const paths = getPrediction(); if (!paths) return null;
  const arr = paths.get(b.id); if (!arr || arr.length < 2) return null;
  if (!(T > 0)) return arr;
  const tEnd = sim.t + T + 1e-9;
  if (arr[arr.length - 1][2] <= tEnd) return arr;
  const out = []; for (const p of arr) { if (p[2] > tEnd) break; out.push(p); }
  const q = curveAt(arr, tEnd - 1e-9); if (q) out.push(q);
  return out.length > 1 ? out : null;
}
function predictOverlayOf(b) { return overlays().find(o => o.on && o.type === 'predict' && ovApplies(o, b)); }
function analyticPoints(b) {
  const key = sim.running ? `run|${(performance.now() / 120) | 0}` : sceneKey();
  const c = b._an;
  if (c && c.key === key) return c.pts;
  let pts = null;
  try {
    const sampled = sampleAnalyticPath(buildAnalysisContext(b), {});
    if (sampled && sampled.points.length > 1) {
      // 闭式解不含碰撞：在第一次碰到表面处截断
      const segs = getSegments(), rr = bodyRadius(b);
      let cut = sampled.points.length;
      for (let i = 1; i < sampled.points.length && cut === sampled.points.length; i++) {
        const q = sampled.points[i];
        for (const sg of segs) {
          const cp = sg.ground ? null : closestOnSeg(q.x, q.y, sg);
          const d = sg.ground ? q.y - sg.y1 : Math.hypot(q.x - cp[0], q.y - cp[1]);
          if (d < rr - 1e-6) { cut = i + 1; break; }
        }
      }
      pts = sampled.points.slice(0, cut);
    }
  } catch { pts = null; }
  b._an = { key, pts };
  return pts;
}
/** 曲线类显示项：已走轨迹（实线）· 预测（虚线）· 解析（点线）· 频闪点 */
function drawCurveOverlays(legend) {
  const lineStyle = (color, alpha, w, dash) => { ctx.strokeStyle = color; ctx.globalAlpha = alpha; ctx.lineWidth = w; ctx.lineJoin = 'round'; ctx.lineCap = 'round'; ctx.setLineDash(dash || []); };
  for (const ov of overlays()) {
    if (!ov.on) continue;
    if (ov.type === 'trail') {
      const ts = ovTargets(ov).filter(b => b.st.trail.length >= 1);
      for (const b of ts) { ctx.save(); lineStyle(ovColor(ov, b), 0.8, 1.8); strokePath(b.st.trail, [b.st.x, b.st.y]); ctx.restore(); }
      if (ts.length) legend.push({ kind: 'solid', text: '已走轨迹' });
    } else if (ov.type === 'predict') {
      if (sim.running || (drag && drag.type === 'grab')) continue;
      let shown = 0;
      for (const b of ovTargets(ov)) {
        if (b.fixed) continue;
        const arr = predFor(b, ov.T); if (!arr) continue;
        shown++;
        ctx.save(); lineStyle(ovColor(ov, b), 0.6, 1.6, [6, 5]); strokePath(arr); ctx.restore();
        if (selSet.has(b.id)) {
          const end = arr[arr.length - 1], [ex, ey] = w2s(end[0], end[1]);
          ctx.save(); ctx.fillStyle = ovColor(ov, b); ctx.globalAlpha = 0.8; ctx.beginPath(); ctx.arc(ex, ey, 2.5, 0, 7); ctx.fill(); ctx.restore();
          label(`t = ${fmt(end[2], 3)} s`, ex + 8, ey - 10, C['--muted'], 'left', false);
        }
      }
      if (shown) legend.push({ kind: 'dash', text: `预测 ${fmt(Math.min(ov.T, predCache.T || ov.T), 3)} s` });
    } else if (ov.type === 'analytic') {
      let shown = 0;
      for (const b of ovTargets(ov)) {
        if (b.fixed) continue;
        const pts = analyticPoints(b); if (!pts) continue;
        shown++;
        ctx.save(); lineStyle(ov.color || C['--accent'], 0.9, 2, [1.5, 4.5]);
        ctx.beginPath(); pts.forEach((p, i) => { const [sx, sy] = w2s(p.x, p.y); if (i) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy); }); ctx.stroke(); ctx.restore();
      }
      if (shown) legend.push({ kind: 'dot', text: '解析轨迹' });
    } else if (ov.type === 'strobe') {
      const ts = ovTargets(ov).filter(b => !b.fixed);
      for (const b of ts) {
        drawStrobe(liveTrail(b), b.color, false, ov.dt);
        const po = !sim.running && predictOverlayOf(b);
        if (po) drawStrobe(predFor(b, po.T), b.color, true, ov.dt);
      }
      if (ts.length) legend.push({ kind: 'strobe', text: `频闪 Δt = ${fmt(ov.dt, 3)} s` });
    }
  }
}

/* ---------------- 轨迹追踪（悬停读数）与标记点 ---------------- */
let traceHit = null;
/** 找光标附近的曲线点：只在画出来的已走轨迹 / 预测曲线上找。返回插值后的精确点。 */
function findTrace(sx, sy, maxD = 7) {
  let best = null, bestD = maxD;
  const consider = (b, arr, src) => {
    if (!arr) return;
    for (let i = 0; i + 1 < arr.length; i++) {
      const [ax, ay] = w2s(arr[i][0], arr[i][1]), [bx, by] = w2s(arr[i + 1][0], arr[i + 1][1]);
      const dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy;
      let u = L2 > 0 ? ((sx - ax) * dx + (sy - ay) * dy) / L2 : 0;
      u = Math.max(0, Math.min(1, u));
      const d = Math.hypot(sx - (ax + dx * u), sy - (ay + dy * u));
      if (d < bestD) {
        bestD = d;
        const a = arr[i], c = arr[i + 1];
        best = { b, src, t: a[2] + (c[2] - a[2]) * u, x: a[0] + (c[0] - a[0]) * u, y: a[1] + (c[1] - a[1]) * u, vx: a[3] + (c[3] - a[3]) * u, vy: a[4] + (c[4] - a[4]) * u };
      }
    }
  };
  for (const ov of overlays()) {
    if (!ov.on || (ov.type !== 'trail' && ov.type !== 'predict')) continue;
    for (const b of ovTargets(ov)) {
      if (ov.type === 'trail') consider(b, liveTrail(b), 'trail');
      else if (!sim.running) consider(b, predFor(b, ov.T), 'pred');
    }
  }
  return best;
}
function curveFor(b, src) {
  if (src === 'trail') return b.st ? liveTrail(b) : null;
  if (sim.running) return null;
  const po = predictOverlayOf(b);
  return po ? predFor(b, po.T) : null;
}
/**
 * 标记点只记 (物体, 时刻 t)。位置每次重新求值：t ≤ 当前时刻取已走轨迹，t > 当前时刻取数值预测。
 * 由于预测与实际运行逐步一致，同一个标记点从“预测”变成“已走”时位置不跳变。
 */
let pins = [];
function pinPos(p) {
  const b = byId(p.bodyId); if (!b) return null;
  const past = p.t <= sim.t + 1e-9;
  const arr = curveFor(b, past ? 'trail' : 'pred');
  const q = arr ? curveAt(arr, p.t) : null;
  if (q) q.src = past ? 'trail' : 'pred';
  return q;
}
function hitPin(sx, sy) {
  for (let i = pins.length - 1; i >= 0; i--) {
    const q = pinPos(pins[i]); if (!q) continue;
    const [x, y] = w2s(q[0], q[1]);
    if (Math.hypot(sx - x, sy - y) < 8) return i;
  }
  return -1;
}
function addPin(h) {
  const t = +h.t.toFixed(4);
  if (pins.some(p => p.bodyId === h.b.id && Math.abs(p.t - t) < 1e-3)) return;
  pins.push({ bodyId: h.b.id, t });
  toast(`已标记 ${h.b.name} 在 t = ${fmt(t, 3)} s 的位置（再点一次可移除）`);
  if (selId === h.b.id) buildPanel();
}
function pinInfoLines(b, q, src) {
  const v = Math.hypot(q[3], q[4]);
  return [
    `${b.name} · ${src === 'trail' ? '已走轨迹' : '数值预测'}`,
    `t = ${fmt(q[2], 4)} s`,
    `(x, y) = (${fmt(q[0], 4)}, ${fmt(q[1], 4)}) m`,
    `v = ${fmt(v, 4)} m/s${v > 1e-9 ? `  ∠ ${fmt(Math.atan2(q[4], q[3]) / DEG, 2)}°` : ''}`,
  ];
}
/** 画一张读数卡片（画布内，自动避开边缘） */
function infoCard(lines, x, y, color) {
  ctx.save();
  ctx.font = '12px ' + MONO_FONT;
  const pad = 8, lh = 17;
  const w = Math.max(...lines.map((l, i) => { ctx.font = (i === 0 ? '600 12px ' + UI_FONT : '12px ' + MONO_FONT); return ctx.measureText(l).width; })) + pad * 2;
  const h = lines.length * lh + pad * 2 - 4;
  let bx = x + 14, by = y - h - 10;
  if (bx + w > W - 6) bx = x - w - 14;
  if (by < 6) by = y + 14;
  ctx.fillStyle = C['--panel']; ctx.strokeStyle = C['--border']; ctx.lineWidth = 1;
  ctx.shadowColor = 'rgba(20,20,19,.12)'; ctx.shadowBlur = 12; ctx.shadowOffsetY = 2;
  ctx.beginPath(); ctx.roundRect ? ctx.roundRect(bx, by, w, h, 8) : ctx.rect(bx, by, w, h); ctx.fill();
  ctx.shadowColor = 'transparent'; ctx.stroke();
  ctx.fillStyle = color; ctx.fillRect(bx, by + 8, 3, h - 16);
  lines.forEach((l, i) => {
    ctx.font = i === 0 ? '600 12px ' + UI_FONT : '12px ' + MONO_FONT;
    ctx.fillStyle = i === 0 ? C['--text'] : C['--muted'];
    ctx.textBaseline = 'top';
    ctx.fillText(l, bx + pad + 2, by + pad + i * lh - 2);
  });
  ctx.restore();
}
function drawPins() {
  for (const p of pins) {
    const b = byId(p.bodyId); if (!b || b.hidden) continue;
    const q = pinPos(p); if (!q) continue;
    const [x, y] = w2s(q[0], q[1]);
    ctx.save(); ctx.lineWidth = 2;
    if (q.src === 'pred') { ctx.fillStyle = C['--canvas']; ctx.strokeStyle = b.color; } else { ctx.fillStyle = b.color; ctx.strokeStyle = C['--canvas']; }
    ctx.beginPath(); ctx.arc(x, y, 5, 0, 7); ctx.fill(); ctx.stroke(); ctx.restore();
    label(`t=${fmt(q[2], 3)}s (${fmt(q[0], 3)}, ${fmt(q[1], 3)})`, x + 9, y - 11, C['--text'], 'left', true);
  }
}
function drawTraceHover() {
  const h = traceHit; if (!h || drag) return;
  const [x, y] = w2s(h.x, h.y);
  ctx.save(); ctx.fillStyle = C['--panel']; ctx.strokeStyle = h.b.color; ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.arc(x, y, 5, 0, 7); ctx.fill(); ctx.stroke(); ctx.restore();
  infoCard(pinInfoLines(h.b, [h.x, h.y, h.t, h.vx, h.vy], h.src).concat(['点击标记此点']), x, y, h.b.color);
}
/** 图例：只列出当前确实画在画布上的显示项 */
function drawLegend(items) {
  if (W < 640 || !items.length) return;
  ctx.save(); ctx.font = '11.5px ' + UI_FONT; ctx.textBaseline = 'middle';
  let x = 16, y = (tool !== 'select' && TOOL_HINT[tool]) ? 58 : 26;
  for (const it of items) {
    const w = 24 + ctx.measureText(it.text).width + 18;
    if (x + w > W - 16 && x > 16) { x = 16; y += 20; }
    const col = it.color || C['--muted'];
    ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 1.6;
    ctx.setLineDash(it.kind === 'dash' ? [5, 4] : it.kind === 'dot' ? [1.5, 3.5] : []);
    if (it.kind === 'strobe') { ctx.beginPath(); ctx.arc(x + 4, y, 2.4, 0, 7); ctx.fill(); ctx.beginPath(); ctx.arc(x + 14, y, 2.4, 0, 7); ctx.fill(); }
    else if (it.kind === 'vec') { ctx.restore(); arrow(x, y, x + 18, y, col, 1.6, 6); ctx.save(); ctx.font = '11.5px ' + UI_FONT; ctx.textBaseline = 'middle'; }
    else { ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + 18, y); ctx.stroke(); }
    ctx.setLineDash([]); ctx.fillStyle = C['--muted'];
    ctx.fillText(it.text, x + 24, y);
    x += w;
  }
  ctx.restore();
}

let pending = null; // 正在创建的矩形/线段/弹簧
/** 悬停反馈：被指到的对象描一圈半透明强调色 */
function drawHover() {
  const o = hoverObj;
  if (!o || (isSel(o) && tool === 'select') || !scene.objects.includes(o)) return;
  outlineObject(o, 'hover');
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
  frameKey = null;
  if (!drag) resolveExprs();
  ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
  ctx.fillStyle = C['--canvas']; ctx.fillRect(0, 0, W, H);
  if (scene.settings.showGrid) drawGrid();
  const objs = scene.objects;
  ensureFields();
  if (ovActive('fieldLines')) drawElectricFieldLines();
  // 绘制顺序 = 图层顺序（数组越靠后越在上）；场与点电荷作为背景层先画，其余按图层顺序
  const shown = objs.filter(o => !o.hidden && !(groupOf(o) && groupOf(o).hidden));
  shown.filter(o => o.kind === 'efield' || o.kind === 'bfield').forEach(drawField);
  shown.filter(o => o.kind === 'pointCharge').forEach(drawPointCharge);
  shown.filter(o => ['ground', 'wall', 'incline'].includes(o.kind) || isLink(o)).forEach(o => isLink(o) ? drawLink(o) : drawSurface(o));
  ensureStates();
  const legend = [];
  drawGhostOverlays();
  drawCurveOverlays(legend);
  shown.filter(isBody).forEach(drawBody);
  drawVectorOverlays(legend);
  drawLabelOverlays();
  drawEditHandles();
  drawPins();
  drawSelection();
  drawHover();
  drawMarquee();
  drawGhost();
  drawTraceHover();
  drawPickMode();
  drawLegend(legend);
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
let tool = 'select', selId = null, drag = null, mouseW = [0, 0];
let pointerCoarse = false;
let mouseIn = false, hoverObj = null, snapOff = false;
const touchPoints = new Map();
let pinch = null;
function selected() { return selId != null ? byId(selId) : null; }
function syncListSelection() {
  renderLayerFoot();
  document.querySelectorAll('.lrow[data-id]').forEach(r => {
    const id = +r.dataset.id;
    r.classList.toggle('sel', selSet.has(id));
    r.classList.toggle('primary', id === selId && selSet.size > 1);
  });
}
function select(id, opts = {}) {
  const prev = selId;
  const prevSize = selSet.size;
  if (id == null) selSet = new Set();
  else {
    const o = byId(id);
    const ids = o && opts.group ? expandForClick(o, opts.deep) : [id];
    if (opts.toggle) {
      if (ids.every(i => selSet.has(i))) ids.forEach(i => selSet.delete(i)); else ids.forEach(i => selSet.add(i));
      id = selSet.has(id) ? id : ([...selSet].at(-1) ?? null);
    } else selSet = new Set(ids);
  }
  if (prevSize !== selSet.size) opts.forcePanel = true;
  selId = id;
  if (!selSet.size) delete document.body.dataset.more;
  updateOptContext();
  syncListSelection();
  if (ovSelDependent()) requestDraw();
  if (prev !== id && !opts.deferPanel) buildPanel();
  else if (!opts.deferPanel && opts.forcePanel) buildPanel();
  draw();
}
/* =====================================================================
 *  选择体系：selSet 为当前选中的对象集合，selId 为主对象（最后点选的那个）。
 *  规则（与 Adobe 系软件一致）：
 *   · 单击选中；Shift / Ctrl 单击加选或减选；空白处拖出选框
 *   · 编组内的对象：单击选中整组，双击进入组内只选该对象
 *   · 锁定的对象在画布上不可选、不可动，只能在图层面板里选中
 * ===================================================================== */
let selSet = new Set();
function isSel(o) { return !!o && selSet.has(o.id); }
function selection() {
  const list = [...selSet].map(byId).filter(Boolean);
  if (list.length !== selSet.size) { selSet = new Set(list.map(o => o.id)); if (selId != null && !selSet.has(selId)) selId = list.length ? list.at(-1).id : null; }
  return list;
}
function groupOf(o) { return o && o.group != null ? (scene.groups || []).find(g => g.id === o.group) : null; }
function groupMembers(gid) { return scene.objects.filter(o => o.group === gid); }
function canvasPickable(o) { return o && !o.hidden && !o.locked && !(groupOf(o) && (groupOf(o).locked || groupOf(o).hidden)); }
/** 单击命中的对象应扩展成哪些 id（编组 → 整组） */
function expandForClick(o, deep) {
  const g = groupOf(o);
  if (!g || deep) return [o.id];
  return groupMembers(g.id).filter(m => !m.locked).map(m => m.id);
}
function setSelection(ids, primary, opts = {}) {
  selSet = new Set(ids.filter(id => byId(id)));
  selId = selSet.size ? (primary != null && selSet.has(primary) ? primary : [...selSet].at(-1)) : null;
  if (!selSet.size) delete document.body.dataset.more;
  syncListSelection();
  if (!opts.deferPanel) buildPanel();
  requestDraw();
  updateOptContext();
}
/** 所有对象（含隐藏、锁定之外）中可以“全选”的 */
function selectAll() { setSelection(scene.objects.filter(canvasPickable).map(o => o.id)); }

/* ---------------- 包围盒（世界坐标）：统一用于选框、对齐、选中框 ---------------- */
/** initial = true：物体按初始位置（x₀, y₀, θ₀）计算——对齐、分布编辑的正是初始位置 */
function objBounds(o, initial = false) {
  if (isBody(o)) {
    const st = !initial && o.st ? o.st : { x: o.x0, y: o.y0, ang: (o.ang0 || 0) * DEG };
    if (o.kind === 'box') {
      const c = Math.cos(st.ang || 0), s = Math.sin(st.ang || 0), hw = o.w / 2, hh = o.h / 2;
      const ex = Math.abs(hw * c) + Math.abs(hh * s), ey = Math.abs(hw * s) + Math.abs(hh * c);
      return [st.x - ex, st.y - ey, st.x + ex, st.y + ey];
    }
    return [st.x - o.r, st.y - o.r, st.x + o.r, st.y + o.r];
  }
  if (o.kind === 'wall') return [Math.min(o.x1, o.x2), Math.min(o.y1, o.y2), Math.max(o.x1, o.x2), Math.max(o.y1, o.y2)];
  if (o.kind === 'incline') { const p = inclinePts(o); return [Math.min(...p.map(q => q[0])), Math.min(...p.map(q => q[1])), Math.max(...p.map(q => q[0])), Math.max(...p.map(q => q[1]))]; }
  if (o.kind === 'efield' || o.kind === 'bfield') return o.all ? null : [o.x, o.y, o.x + o.w, o.y + o.h];
  if (o.kind === 'pointCharge') { const r = o.softRadius || 0.08; return [o.x - r, o.y - r, o.x + r, o.y + r]; }
  if (o.kind === 'ground') return null;
  if (isLink(o)) { const e = linkEnds(o); return e ? [Math.min(e.x1, e.x2), Math.min(e.y1, e.y2), Math.max(e.x1, e.x2), Math.max(e.y1, e.y2)] : null; }
  return null;
}
function screenRectOf(bb) { const [a, b] = w2s(bb[0], bb[3]), [c, d] = w2s(bb[2], bb[1]); return [a, b, c, d]; }
/** 选框命中：与选框相交即选中（地面看水平线是否穿过选框） */
function hitsMarquee(o, r) {
  if (o.kind === 'ground') { const y = w2s(0, o.y)[1]; return y >= r[1] && y <= r[3]; }
  const bb = objBounds(o); if (!bb) return false;
  const [a, b, c, d] = screenRectOf(bb);
  return a <= r[2] && c >= r[0] && b <= r[3] && d >= r[1];
}

/* ---------------- 统一的选中/悬停描边 ---------------- */
function traceShape(o) {
  const sc = scene.view.scale;
  ctx.beginPath();
  if (isBody(o) && o.st) {
    const [x, y] = w2s(o.st.x, o.st.y);
    if (o.kind === 'box') {
      const c = Math.cos(o.st.ang || 0), s = Math.sin(o.st.ang || 0), hw = o.w / 2 * sc + 3, hh = o.h / 2 * sc + 3;
      const pts = [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]].map(([lx, ly]) => [x + lx * c - ly * s, y - (lx * s + ly * c)]);
      ctx.moveTo(...pts[0]); pts.slice(1).forEach(p => ctx.lineTo(...p)); ctx.closePath();
    } else ctx.arc(x, y, bodyScreenR(o) + 3, 0, Math.PI * 2);
  } else if (o.kind === 'pointCharge') { const [x, y] = w2s(o.x, o.y); ctx.arc(x, y, Math.max(8, (o.softRadius || 0.08) * sc) + 3, 0, Math.PI * 2); }
  else if (o.kind === 'wall') { const [a, b] = w2s(o.x1, o.y1), [c, d] = w2s(o.x2, o.y2); ctx.moveTo(a, b); ctx.lineTo(c, d); }
  else if (isLink(o)) { const e = linkEnds(o); if (e) { const [a, b] = w2s(e.x1, e.y1), [c, d] = w2s(e.x2, e.y2); ctx.moveTo(a, b); ctx.lineTo(c, d); } }
  else if (o.kind === 'ground') { const y = w2s(0, o.y)[1]; ctx.moveTo(0, y); ctx.lineTo(W, y); }
  else if (o.kind === 'incline') { const p = inclinePts(o).map(q => w2s(q[0], q[1])); ctx.moveTo(...p[0]); ctx.lineTo(...p[1]); ctx.lineTo(...p[2]); ctx.closePath(); }
  else if ((o.kind === 'efield' || o.kind === 'bfield') && !o.all) { const [x, y, w, h] = fieldScreenRect(o); ctx.rect(x, y, w, h); }
}
const LINEAR_KINDS = ['wall', 'ground', 'spring', 'rope', 'rod'];
function outlineObject(o, mode) {
  if (!o || o.hidden || (o.kind === 'efield' || o.kind === 'bfield') && o.all) return;
  ctx.save(); ctx.strokeStyle = C['--accent']; ctx.lineCap = 'round'; ctx.lineJoin = 'round';
  traceShape(o);
  const linear = LINEAR_KINDS.includes(o.kind);
  if (mode === 'hover') { ctx.globalAlpha = 0.35; ctx.lineWidth = linear ? 8 : 4; ctx.stroke(); }
  else { ctx.globalAlpha = linear ? 0.35 : 1; ctx.lineWidth = linear ? 7 : 1.6; ctx.stroke(); }
  ctx.restore();
}
/** 选中对象：统一描边 + 外接框 + 名称标签；单选时显示尺寸，并画该类型的编辑手柄 */
function drawSelection() {
  const list = selection().filter(o => !o.hidden);
  if (!list.length) return;
  for (const o of list) outlineObject(o, 'sel');
  let box = null;
  for (const o of list) {
    const bb = objBounds(o); if (!bb) continue;
    const r = screenRectOf(bb);
    box = box ? [Math.min(box[0], r[0]), Math.min(box[1], r[1]), Math.max(box[2], r[2]), Math.max(box[3], r[3])] : r;
  }
  if (box && (list.length > 1 || groupOf(list[0]))) {
    const pad = 6, [a, b, c, d] = [box[0] - pad, box[1] - pad, box[2] + pad, box[3] + pad];
    ctx.save(); ctx.strokeStyle = C['--accent']; ctx.lineWidth = 1; ctx.setLineDash([4, 3]);
    ctx.strokeRect(Math.round(a) + .5, Math.round(b) + .5, Math.round(c - a), Math.round(d - b)); ctx.restore();
    const g = groupOf(list[0]), allGroup = g && list.length === groupMembers(g.id).length && list.every(o => o.group === g.id);
    selChip(allGroup ? `▣ ${g.name} · ${list.length} 个对象` : `${list.length} 个对象`, a, b);
  } else if (list.length === 1) {
    const o = list[0];
    if (box) {
      const dims = dimText(o);
      if (dims) label(dims, (box[0] + box[2]) / 2, box[3] + (o.kind === 'box' || o.kind === 'ball' || o.kind === 'particle' ? 28 : 14), C['--accent'], 'center', true);
    }
    // 单选时的编辑手柄
    if (o.kind === 'wall') { const [p, q] = w2s(o.x1, o.y1), [u, v] = w2s(o.x2, o.y2); handleDot(p, q, C['--accent']); handleDot(u, v, C['--accent']); }
    if ((o.kind === 'efield' || o.kind === 'bfield') && !o.all) for (const [hx, hy] of rectHandles(o)) handleDot(hx, hy, C['--accent']);
  }
}
function dimText(o) {
  if (o.kind === 'box') return `${fmt(o.w)} × ${fmt(o.h)} m`;
  if (o.kind === 'ball' || o.kind === 'particle') return `r = ${fmt(o.r)} m`;
  if (o.kind === 'wall') return `${fmt(Math.hypot(o.x2 - o.x1, o.y2 - o.y1))} m`;
  if (o.kind === 'efield' || o.kind === 'bfield') return o.all ? '' : `${fmt(o.w)} × ${fmt(o.h)} m`;
  if (o.kind === 'incline') return `θ = ${fmt(o.ang)}° · L = ${fmt(o.L)} m`;
  return '';
}
function selChip(text, x, y) {
  ctx.save(); ctx.font = '600 11px ' + UI_FONT;
  const w = ctx.measureText(text).width + 12;
  const bx = Math.max(2, Math.min(W - w - 2, x)), by = Math.max(2, y - 20);
  ctx.fillStyle = C['--accent']; ctx.beginPath(); ctx.roundRect ? ctx.roundRect(bx, by, w, 17, 4) : ctx.rect(bx, by, w, 17); ctx.fill();
  ctx.fillStyle = '#fff'; ctx.textBaseline = 'middle'; ctx.fillText(text, bx + 6, by + 9);
  ctx.restore();
}
function drawMarquee() {
  if (!drag || drag.type !== 'marquee' || !drag.active) return;
  const [a, b, c, d] = drag.rect;
  ctx.save(); ctx.fillStyle = C['--accent-soft']; ctx.strokeStyle = C['--accent']; ctx.lineWidth = 1; ctx.setLineDash([4, 3]);
  ctx.fillRect(a, b, c - a, d - b); ctx.strokeRect(a + .5, b + .5, c - a, d - b); ctx.restore();
}

/* ---------------- 选择集合上的操作 ---------------- */
/** 记录选中对象的位置字段，供拖动时“从起点 + 位移”重算，避免累计误差 */
const POS_KEYS = ['x0', 'y0', 'x', 'y', 'x1', 'y1', 'x2', 'y2', 'ax', 'ay'];
function posSnapshot(list) { return list.map(o => [o, Object.fromEntries(POS_KEYS.filter(k => o[k] != null).map(k => [k, o[k]]))]); }
function restorePos(snap) { for (const [o, p] of snap) Object.assign(o, p); }
/** 连接件随两端物体移动，不单独平移；两端都在选择里的连接件不重复移动固定点 */
function movableSelection() {
  const list = selection().filter(o => !o.locked);
  return list.filter(o => !(isLink(o) && o.b != null));
}
function deleteSelection() {
  const list = selection();
  if (!list.length) return;
  if (list.length === 1) { deleteSel(); return; }
  pushUndo(`删除 ${list.length} 个对象`);
  const ids = new Set(list.map(o => o.id));
  const before = scene.objects.length;
  scene.objects = scene.objects.filter(o => !ids.has(o.id) && !(isLink(o) && (ids.has(o.a) || ids.has(o.b))));
  pruneGroups();
  selSet.clear(); selId = null;
  afterStructure(); buildPanel();
  toast(`已删除 ${before - scene.objects.length} 个对象`, { action: UNDO_ACTION });
}
function pruneGroups() {
  if (!scene.groups) scene.groups = [];
  scene.groups = scene.groups.filter(g => scene.objects.filter(o => o.group === g.id).length >= 2);
  const live = new Set(scene.groups.map(g => g.id));
  for (const o of scene.objects) if (o.group != null && !live.has(o.group)) delete o.group;
}
let nextGroupId = 1;
function groupSelection() {
  const list = selection().filter(o => !isLink(o) || o.b == null || true);
  if (list.length < 2) { toast('至少选中两个对象才能编组'); return; }
  pushUndo('编组');
  if (!scene.groups) scene.groups = [];
  const id = nextGroupId++;
  const n = scene.groups.length + 1;
  scene.groups.push({ id, name: uniqueGroupName(`编组 ${n}`) });
  for (const o of list) o.group = id;
  pruneGroups();
  // 编组内对象在 z 序上连续：移到组内最上层成员的位置
  const ids = new Set(list.map(o => o.id)), top = Math.max(...list.map(o => scene.objects.indexOf(o)));
  const rest = scene.objects.filter(o => !ids.has(o.id)), insertAt = rest.filter(o => scene.objects.indexOf(o) < top).length;
  rest.splice(insertAt, 0, ...scene.objects.filter(o => ids.has(o.id)));
  scene.objects = rest;
  afterStructure(); setSelection(list.map(o => o.id), selId);
  toast(`已编组 ${list.length} 个对象`, { action: UNDO_ACTION });
}
function uniqueGroupName(base) { const names = new Set((scene.groups || []).map(g => g.name)); let n = base, i = 2; while (names.has(n)) n = base.replace(/\d+$/, '') + (i++); return n; }
function ungroupSelection() {
  const gids = new Set(selection().map(o => o.group).filter(g => g != null));
  if (!gids.size) { toast('选中的对象没有编组'); return; }
  pushUndo('取消编组');
  for (const o of scene.objects) if (gids.has(o.group)) delete o.group;
  pruneGroups(); afterStructure(); buildPanel();
  toast(`已取消 ${gids.size} 个编组`, { action: UNDO_ACTION });
}
function setLocked(list, v) {
  if (!list.length) return;
  pushUndo(v ? '锁定' : '解锁');
  for (const o of list) { if (v) o.locked = true; else delete o.locked; }
  if (v) setSelection([]); else { refreshLeft(); buildPanel(); }
  markDirty(); requestDraw();
  toast(v ? `已锁定 ${list.length} 个对象（在「对象」里解锁）` : `已解锁 ${list.length} 个对象`, { action: UNDO_ACTION });
}
function setHidden(list, v) {
  if (!list.length) return;
  pushUndo(v ? '隐藏' : '显示');
  for (const o of list) { if (v) o.hidden = true; else delete o.hidden; }
  if (v) setSelection([]); else refreshLeft();
  markDirty(); requestDraw();
  toast(v ? `已隐藏 ${list.length} 个对象（仍参与物理计算）` : `已显示 ${list.length} 个对象`, { action: UNDO_ACTION });
}
/** z 序：scene.objects 越靠后越在上层。how = front | forward | backward | back */
function arrangeSelection(how) {
  const list = selection(); if (!list.length) return;
  pushUndo({ front: '置于顶层', forward: '上移一层', backward: '下移一层', back: '置于底层' }[how]);
  const ids = new Set(list.map(o => o.id)), arr = scene.objects;
  if (how === 'front' || how === 'back') {
    const moving = arr.filter(o => ids.has(o.id)), rest = arr.filter(o => !ids.has(o.id));
    scene.objects = how === 'front' ? [...rest, ...moving] : [...moving, ...rest];
  } else if (how === 'forward') {
    for (let i = arr.length - 2; i >= 0; i--) if (ids.has(arr[i].id) && !ids.has(arr[i + 1].id)) [arr[i], arr[i + 1]] = [arr[i + 1], arr[i]];
  } else {
    for (let i = 1; i < arr.length; i++) if (ids.has(arr[i].id) && !ids.has(arr[i - 1].id)) [arr[i], arr[i - 1]] = [arr[i - 1], arr[i]];
  }
  afterStructure();
}
/** 对齐 / 分布：按包围盒计算，平移对象本身 */
function alignSelection(mode) {
  const list = movableSelection().filter(o => objBounds(o, true));
  if (list.length < (mode.startsWith('dist') ? 3 : 2)) { toast(mode.startsWith('dist') ? '分布需要至少 3 个对象' : '对齐需要至少 2 个对象'); return; }
  pushUndo({ left: '左对齐', hcenter: '水平居中对齐', right: '右对齐', top: '顶对齐', vcenter: '垂直居中对齐', bottom: '底对齐', disth: '水平分布', distv: '垂直分布' }[mode]);
  const bb = list.map(o => objBounds(o, true));
  const minX = Math.min(...bb.map(b => b[0])), maxX = Math.max(...bb.map(b => b[2])), minY = Math.min(...bb.map(b => b[1])), maxY = Math.max(...bb.map(b => b[3]));
  if (mode === 'disth' || mode === 'distv') {
    const ax = mode === 'disth' ? 0 : 1;
    const order = list.map((o, i) => [o, bb[i]]).sort((p, q) => (p[1][ax] + p[1][ax + 2]) - (q[1][ax] + q[1][ax + 2]));
    const c0 = (order[0][1][ax] + order[0][1][ax + 2]) / 2, c1 = (order.at(-1)[1][ax] + order.at(-1)[1][ax + 2]) / 2;
    order.forEach(([o, b], i) => { const target = c0 + (c1 - c0) * i / (order.length - 1), cur = (b[ax] + b[ax + 2]) / 2; translateObject(o, ax ? 0 : target - cur, ax ? target - cur : 0); });
  } else {
    list.forEach((o, i) => {
      const b = bb[i];
      const dx = mode === 'left' ? minX - b[0] : mode === 'right' ? maxX - b[2] : mode === 'hcenter' ? (minX + maxX) / 2 - (b[0] + b[2]) / 2 : 0;
      const dy = mode === 'bottom' ? minY - b[1] : mode === 'top' ? maxY - b[3] : mode === 'vcenter' ? (minY + maxY) / 2 - (b[1] + b[3]) / 2 : 0;
      translateObject(o, clean(dx), clean(dy));
    });
  }
  if (list.some(isBody)) initChanged(); else liveEdit();
  buildPanel();
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
/* ---------------- 拾取模式：在画布上点选对象来填写字段（与创建轻杆时点物体一样） ----------------
 * startPick({ prompt, from, accept(o), allowPoint, onPick(o), onPoint([x, y]) })
 * 可选对象高亮，其他对象变暗不可点；Esc 或右键取消。 */
let pickMode = null;
function startPick(opts) {
  if (tool !== 'select') setTool('select');
  pickMode = Object.assign({ accept: o => isBody(o) }, opts);
  document.body.classList.add('picking');
  setSheet('canvas');
  const hint = $('#toolhint');
  if (hint) hint.innerHTML = `${escapeHtml(opts.prompt || '在画布上点选一个对象')} <span class="th-k">${opts.allowPoint ? '点空白处取坐标 · ' : ''}Esc 取消</span>`;
  cv.style.cursor = 'crosshair';
  closeMobilePanels(false);
  requestDraw();
}
function endPick(msg) {
  if (!pickMode) return;
  pickMode = null;
  document.body.classList.remove('picking');
  const hint = $('#toolhint'); if (hint) hint.innerHTML = '';
  cv.style.cursor = 'default';
  if (msg) toast(msg);
  requestDraw();
}
function pickHit(sx, sy) {
  const o = hitBody(sx, sy) || hitOther(sx, sy);
  return o && pickMode.accept(o) ? o : null;
}
/** 拾取模式下的画布反馈：可选对象描边，从来源物体拉一条虚线到光标 */
function drawPickMode() {
  if (!pickMode) return;
  ctx.save();
  for (const o of scene.objects) if (!o.hidden && pickMode.accept(o)) outlineObject(o, 'hover');
  const from = pickMode.from && byId(pickMode.from);
  if (from && from.st && mouseIn) {
    const [a, b] = w2s(from.st.x, from.st.y);
    const tgt = hoverObj && pickMode.accept(hoverObj) ? (hoverObj.st ? [hoverObj.st.x, hoverObj.st.y] : [hoverObj.x ?? mouseW[0], hoverObj.y ?? mouseW[1]]) : snapPt(mouseW);
    const [c, d] = w2s(tgt[0], tgt[1]);
    ctx.strokeStyle = C['--accent']; ctx.setLineDash([5, 4]); ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
  }
  ctx.restore();
  if (mouseIn) {
    const [mx, my] = w2s(mouseW[0], mouseW[1]);
    const o = hoverObj && pickMode.accept(hoverObj) ? hoverObj : null;
    label(o ? `→ ${o.name}` : pickMode.allowPoint ? `(${fmt(snapPt(mouseW)[0], 3)}, ${fmt(snapPt(mouseW)[1], 3)})` : '点选一个对象', mx + 14, my + 16, C['--accent'], 'left', true);
  }
}
function setTool(t) {
  if (pickMode) endPick();
  tool = t; pending = null;
  document.querySelectorAll('[data-tool]').forEach(b => b.classList.toggle('active', b.dataset.tool === t));
  const hint = $('#toolhint');
  if (hint) hint.innerHTML = TOOL_HINT[t] ? `${escapeHtml(TOOL_HINT[t])}${t !== 'select' && t !== 'pan' ? ' <span class="th-k">Alt 关闭吸附 · Esc 取消</span>' : ''}` : '';
  hoverObj = null;
  updateOptContext();
  cv.style.cursor = t === 'pan' ? 'grab' : t === 'select' ? 'default' : 'crosshair';
  if (!navLock && t !== 'select') rememberNav('tool');
  updateArmBar();
  if (typeof syncGeom === 'function') syncGeom();
  if (leftTab === 'add') refreshLeft();
  draw();
}
function snap(v) { const S = scene.settings; return S.snap && !snapOff ? Math.round(v / S.snapStep) * S.snapStep : v; }
function snapPt(p) { return [+snap(p[0]).toFixed(6), +snap(p[1]).toFixed(6)]; }
function evPos(e) { const r = cv.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; }

function hitBody(sx, sy) {
  const list = bodies();
  for (let i = list.length - 1; i >= 0; i--) {
    const b = list[i]; if (!canvasPickable(b) || !b.st) continue;
    const [x, y] = w2s(b.st.x, b.st.y);
    if (b.kind === 'box') {
      const a = b.st.ang, dx = sx - x, dy = -(sy - y), c = Math.cos(a), s = Math.sin(a);
      const lx = dx * c + dy * s, ly = -dx * s + dy * c, sc = scene.view.scale;
      if (Math.abs(lx) <= Math.max(b.w * sc, 6) / 2 + 3 && Math.abs(ly) <= Math.max(b.h * sc, 6) / 2 + 3) return b;
    } else if (Math.hypot(sx - x, sy - y) <= bodyScreenR(b) + (pointerCoarse ? 12 : 4)) return b;
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
  const objs = scene.objects.filter(canvasPickable);
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
/** 双指开始时丢掉未完成的单指操作，不提交、不创建。 */
function revertLiveDrag() {
  if (!drag) { pending = null; return; }
  const d = drag;
  drag = null;
  pending = null;
  if (d.type === 'grab' && d.b && d.b._grab) {
    if (d.b.st) { d.b.st.x = d.b._grab.x; d.b.st.y = d.b._grab.y; }
    d.b._grab = null;
    requestDraw();
    return;
  }
  if (d.type === 'pan' || d.type === 'create') {
    cv.style.cursor = 'default';
    requestDraw();
    return;
  }
  if (d.pushed && undoStack.length) {
    const it = undoStack.pop();
    editSeq = Math.max(0, editSeq - 1);
    restoreSnapshot(it.s);
    syncHistoryButtons();
  }
}
function abortDragForPinch() { revertLiveDrag(); }
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

const TOUCH_SLOP = 12;
const LONG_PRESS_MS = 480;
let touchSession = null;
let navLock = false;
const POINT_TOOLS = ['ball', 'particle', 'box', 'pointCharge', 'ground', 'incline'];

function rememberNav(kind) {
  if (navLock) return;
  try {
    if (history.state && history.state.physim === kind) return;
    history.pushState({ physim: kind }, '');
  } catch { /* 非安全上下文 */ }
}
function syncGeom() {
  const geom = window.__geom || {};
  document.querySelectorAll('#geombar [data-geom]').forEach(b => {
    const g = b.dataset.geom;
    const on = g === 'select' ? tool === 'select' : (geom[g] || []).some(([id]) => id === tool);
    b.classList.toggle('on', on);
  });
}
function openToolPop(group, anchor) {
  const pop = $('#toolPop'); if (!pop) return;
  const geom = window.__geom || {};
  pop.innerHTML = '';
  (geom[group] || []).forEach(([id, name]) => {
    const b = el('button', { type: 'button', 'data-tool': id }, name);
    if (tool === id) b.classList.add('on');
    b.onclick = () => { setTool(id); pop.hidden = true; syncGeom(); document.body.dataset.list = 'open'; };
    pop.appendChild(b);
  });
  pop.hidden = false;
  const r = anchor && anchor.getBoundingClientRect ? anchor.getBoundingClientRect() : { left: 8, bottom: 80 };
  pop.style.left = Math.max(8, Math.min(r.left, innerWidth - 240)) + 'px';
  pop.style.top = Math.max(8, r.bottom + 6) + 'px';
}
function openStyle(o, anchor) {
  const pop = $('#stylePop'); if (!pop || !o) return;
  pop.innerHTML = '';
  ['#c8553d', '#d97757', '#c9a227', '#3d8f86', '#4a78b0', '#4d6470', '#141413'].forEach(c => {
    const b = el('button', { type: 'button', class: 'swatch-btn', title: c, style: `--c:${c}` });
    b.onclick = () => { pushUndo('改颜色'); o.color = c; refreshLeft(); buildPanel(); requestDraw(); markDirty(); pop.hidden = true; };
    pop.appendChild(b);
  });
  const hide = el('button', { type: 'button' }, o.hidden ? '显示' : '隐藏');
  hide.onclick = () => { setHidden([o], !o.hidden); pop.hidden = true; };
  pop.appendChild(hide);
  pop.hidden = false;
  if (anchor && anchor.getBoundingClientRect) {
    const r = anchor.getBoundingClientRect();
    pop.style.left = Math.max(8, Math.min(r.left, innerWidth - 220)) + 'px';
    pop.style.top = Math.min(innerHeight - 80, r.bottom + 6) + 'px';
  }
}
function updateContextBar() {
  const bar = $('#ctxBar'); if (!bar) return;
  const o = selSet.size === 1 ? selected() : null;
  const multi = selSet.size > 1;
  bar.hidden = tool !== 'select' || !(o || multi);
  const name = $('#ctxName');
  if (name) name.textContent = o ? o.name : (multi ? `${selSet.size} 个对象` : '');
  const hide = $('#ctxHide');
  if (hide) hide.textContent = o && o.hidden ? '显示' : '隐藏';
}
function updateArmBar() {
  const bar = $('#armBar'); if (!bar) return;
  const on = !!(pickMode || pending || (tool && tool !== 'select'));
  bar.hidden = !on;
  const label = $('#armLabel');
  if (!label) return;
  if (pickMode) label.textContent = pickMode.prompt || '在画布上点选';
  else if (pending && LINK_KINDS.includes(String(pending.type || tool))) label.textContent = '再点另一端，或取消';
  else if (tool && tool !== 'select') label.textContent = `点按放置${KIND_NAME[tool] || tool}。拖动则只平移，不会创建`;
  else label.textContent = '';
}
function cancelArm() {
  pending = null;
  if (touchSession) { clearTimeout(touchSession.timer); touchSession = null; }
  if (pickMode) endPick('已取消');
  if (tool !== 'select') setTool('select');
  updateArmBar();
  requestDraw();
}
function requestCancel() {
  if (!navLock && history.state && history.state.physim === 'tool') { history.back(); return; }
  cancelArm();
}
function bootHistory() {
  try { history.replaceState({ physim: 'root' }, ''); } catch { /* ignore */ }
  window.addEventListener('popstate', () => {
    if (navLock) return;
    navLock = true;
    try {
      const pop = id => { const n = document.getElementById(id); if (n && !n.hidden) { n.hidden = true; return true; } return false; };
      if (pop('stylePop') || pop('toolPop') || pop('overflowMenu')) return;
      if (document.body.dataset.more === 'open') { delete document.body.dataset.more; return; }
      if (pickMode || pending || tool !== 'select') cancelArm();
      else if (sheet === 'lab' || sheet === 'analysis') { $('#home')?.classList.remove('show'); setSheet('canvas'); }
      else if (document.body.dataset.list === 'closed') document.body.dataset.list = 'open';
      else if (selSet.size) setSelection([]);
    } finally { navLock = false; updateArmBar(); updateContextBar(); }
  });
}
function applyKeyboardInset(px) {
  const n = Math.max(0, Number(px) || 0);
  document.documentElement.style.setProperty('--kb', n + 'px');
  document.body.classList.toggle('kb-open', n > 40);
  if (n > 40) {
    const ae = document.activeElement;
    if (ae && ae.scrollIntoView) ae.scrollIntoView({ block: 'center', inline: 'nearest' });
  }
}
function bindKeyboardInset() {
  const vv = window.visualViewport;
  const fromViewport = () => {
    if (!vv) return;
    applyKeyboardInset(Math.max(0, window.innerHeight - vv.height - vv.offsetTop));
  };
  vv?.addEventListener('resize', fromViewport);
  vv?.addEventListener('scroll', fromViewport);
  document.addEventListener('focusin', (e) => {
    const t = e.target;
    if (!t || !t.closest) return;
    if (t.closest('.mq-editable-field, .mq-textarea')) {
      syncDesmosKeypadForAnalysis(true);
      setDesmosKeypadVisible(true);
    }
    if (t.matches && t.matches('input, textarea, select') && window.matchMedia('(pointer: coarse)').matches) {
      const inset = window.visualViewport ? Math.max(0, window.innerHeight - window.visualViewport.height - window.visualViewport.offsetTop) : 0;
      if (inset < 40) applyKeyboardInset(Math.round(window.innerHeight * 0.38));
      setTimeout(() => t.scrollIntoView({ block: 'center', inline: 'nearest' }), 60);
    }
  });
  document.addEventListener('focusout', (e) => {
    const t = e.target;
    if (t && t.closest && t.closest('.mq-editable-field, .mq-textarea')) {
      setTimeout(() => {
        const ae = document.activeElement;
        if (ae && ae.closest && ae.closest('.mq-editable-field, #desmosKeypadDock, #desmosKeypadShow')) return;
        setDesmosKeypadVisible(false);
      }, 140);
    }
    setTimeout(() => {
      const ae = document.activeElement;
      if (ae && ae.matches && ae.matches('input, textarea, select')) return;
      if (window.visualViewport) {
        const inset = Math.max(0, window.innerHeight - window.visualViewport.height - window.visualViewport.offsetTop);
        if (inset < 40) applyKeyboardInset(0);
      } else applyKeyboardInset(0);
    }, 80);
  });
}
function placePointAt(kind, sp) {
  if (['ball', 'particle', 'box'].includes(kind)) {
    pushUndo('添加' + KIND_NAME[kind]); if (sim.t > 0) resetSim('添加物体，已重置到 t=0');
    const b = mkBody(kind, { x0: sp[0], y0: sp[1] }); scene.objects.push(b); initState(b);
    afterStructure(); select(b.id); return;
  }
  if (kind === 'pointCharge') {
    pushUndo('添加点电荷'); if (sim.t > 0) resetSim('添加点电荷，已重置');
    const q = 1e-6;
    const o = mkPointCharge(sp[0], sp[1], { q, color: q >= 0 ? '#c8553d' : '#4a78b0' });
    scene.objects.push(o); afterStructure(); select(o.id); return;
  }
  if (kind === 'ground') { pushUndo('添加地面'); const g = mkGround(sp[1]); scene.objects.push(g); afterStructure(); select(g.id); return; }
  if (kind === 'incline') { pushUndo('添加斜面'); const o = mkIncline(sp[0], sp[1]); scene.objects.push(o); afterStructure(); select(o.id); }
}
function releaseToolHistory() {
  navLock = true;
  if (tool !== 'select') setTool('select');
  navLock = false;
  if (history.state && history.state.physim === 'tool') {
    try { history.replaceState({ physim: 'root' }, ''); } catch { /* ignore */ }
  }
  updateArmBar();
}
function commitTouchTap(s) {
  const sp = snapPt([s.wx, s.wy]);
  const sx = s.sx, sy = s.sy;
  hideContextMenu();
  if (pickMode) {
    const pm = pickMode, o = pickHit(sx, sy);
    if (o) { endPick(); pm.onPick(o); }
    else if (pm.allowPoint && !(hitBody(sx, sy) || hitOther(sx, sy))) { endPick(); pm.onPoint(sp); }
    else toast(pm.rejectMsg || '这个对象不能选，请点高亮的对象');
    updateArmBar();
    return;
  }
  if (POINT_TOOLS.includes(tool)) {
    placePointAt(tool, sp);
    updateArmBar();
    return;
  }
  if (tool === 'wall' || tool === 'efield' || tool === 'bfield_in' || tool === 'bfield_out') {
    pending = tool === 'wall'
      ? { type: 'line', x1: sp[0], y1: sp[1], x2: sp[0], y2: sp[1], sx, sy }
      : { type: 'rect', x1: sp[0], y1: sp[1], x2: sp[0], y2: sp[1], sx, sy };
    drag = { type: 'create' };
    finishDrag({ shiftKey: false, keepArm: true, pointerId: s.id });
    updateArmBar();
    return;
  }
  if (LINK_KINDS.includes(tool)) {
    const hb = hitBody(sx, sy);
    const label = KIND_NAME[tool] || tool;
    if (!pending) {
      if (hb) { pending = { type: tool, a: hb.id }; updateArmBar(); draw(); }
      else toast(`请先点击一个物体作为${label}的一端`);
      return;
    }
    if (hb && hb.id === pending.a) { toast('不能连接到自身：请点另一个物体，或点空白处作为固定点'); return; }
    pushUndo('添加' + label);
    const A = byId(pending.a);
    const L0 = hb && hb.id !== pending.a
      ? +Math.hypot(hb.st.x - A.st.x, hb.st.y - A.st.y).toFixed(3)
      : +Math.hypot(sp[0] - A.st.x, sp[1] - A.st.y).toFixed(3);
    const ends = hb && hb.id !== pending.a
      ? { a: pending.a, b: hb.id, L0 }
      : { a: pending.a, b: null, ax: sp[0], ay: sp[1], L0 };
    let link;
    if (tool === 'rope') link = mkRope(ends.a, ends.b, ends);
    else if (tool === 'rod') link = mkRod(ends.a, ends.b, ends);
    else link = mkSpring(ends.a, ends.b, ends);
    scene.objects.push(link); pending = null; afterStructure(); select(link.id);
    updateArmBar();
    return;
  }
  const hb = hitBody(sx, sy);
  if (hb && sim.running && !hb.fixed) { select(hb.id, { forcePanel: true }); return; }
  if (!hb) {
    const pi = hitPin(sx, sy);
    if (pi >= 0) { const p = pins[pi]; pins.splice(pi, 1); toast('已移除标记点'); if (selId === p.bodyId) buildPanel(); requestDraw(); return; }
    const th = findTrace(sx, sy);
    if (th) { addPin(th); requestDraw(); return; }
  }
  const ho = hb || hitOther(sx, sy);
  if (ho) select(ho.id, { group: true, forcePanel: true });
  else select(null);
}
function onTouchLongPress() {
  const s = touchSession;
  if (!s || s.mode !== 'undecided' || tool !== 'select' || s.hitId == null) return;
  s.longFired = true;
  s.mode = 'consumed';
  select(s.hitId, { group: true, toggle: true, forcePanel: true });
  toast(selSet.has(s.hitId) ? '已加入选择' : '已从选择中去掉');
}
function promoteTouch(s) {
  clearTimeout(s.timer);
  const sx = s.sx, sy = s.sy;
  const wp = [s.wx, s.wy];
  const sp = snapPt(wp);
  if (pickMode || LINK_KINDS.includes(tool) || (POINT_TOOLS.includes(tool) && s.hitId == null && !s.handle)) {
    drag = { type: 'pan', sx, sy, cx: scene.view.cx, cy: scene.view.cy, click: false };
    s.mode = 'pan';
    return;
  }
  if (tool === 'wall' || tool === 'efield' || tool === 'bfield_in' || tool === 'bfield_out') {
    pending = tool === 'wall'
      ? { type: 'line', x1: sp[0], y1: sp[1], x2: sp[0], y2: sp[1], sx, sy }
      : { type: 'rect', x1: sp[0], y1: sp[1], x2: sp[0], y2: sp[1], sx, sy };
    drag = { type: 'create' };
    s.mode = 'drag';
    return;
  }
  if (s.handle) {
    const hh = s.handle;
    pushUndo(hh.key === 'v0' ? '调整初速度' : '调整外力');
    drag = { type: hh.key, b: hh.b, k: hh.k, moved: false, pushed: true };
    s.mode = 'drag';
    return;
  }
  const hb = s.hitId != null ? byId(s.hitId) : null;
  if (hb && isBody(hb) && sim.running && !hb.fixed) {
    select(hb.id, { deferPanel: true });
    hb._grab = { x: hb.st.x, y: hb.st.y, vx: 0, vy: 0, t: performance.now() };
    drag = { type: 'grab', b: hb, ox: wp[0] - hb.st.x, oy: wp[1] - hb.st.y };
    s.mode = 'drag';
    return;
  }
  if (hb && canvasPickable(hb)) {
    if (!isSel(hb)) select(hb.id, { group: true, deferPanel: true });
    const movers = movableSelection();
    if (movers.length === 1 && isBody(movers[0])) {
      const b = movers[0];
      pushUndo(`移动「${b.name}」`);
      drag = { type: 'body', b, ox: wp[0] - b.st.x, oy: wp[1] - b.st.y, moved: false, sx, sy, wasT: sim.t, pushed: true };
      s.mode = 'drag';
      return;
    }
    if (movers.length) {
      pushUndo(movers.length > 1 ? `移动 ${movers.length} 个对象` : `移动「${hb.name}」`);
      drag = { type: 'moveSel', snap: posSnapshot(movers), start: sp, moved: false, sx, sy, wasT: sim.t, hasBody: movers.some(isBody), pushed: true };
      s.mode = 'drag';
      return;
    }
  }
  drag = { type: 'pan', sx, sy, cx: scene.view.cx, cy: scene.view.cy, click: false };
  s.mode = 'pan';
}
function beginPinch() {
  if (touchSession) { clearTimeout(touchSession.timer); touchSession.mode = 'pinch'; touchSession = null; }
  revertLiveDrag();
  startPinchFromTouches();
}
function touchPointerDown(e) {
  pointerCoarse = true;
  const [sx, sy] = evPos(e);
  if (pinch || touchPoints.size >= 2) return;
  touchPoints.set(e.pointerId, { x: sx, y: sy });
  try { cv.setPointerCapture(e.pointerId); } catch { /* ignore */ }
  if (touchPoints.size === 2) { beginPinch(); return; }
  const wp = s2w(sx, sy);
  mouseW = wp;
  const hit = hitBody(sx, sy) || hitOther(sx, sy);
  touchSession = {
    id: e.pointerId, sx, sy, wx: wp[0], wy: wp[1],
    hitId: hit ? hit.id : null,
    handle: tool === 'select' && !pickMode ? hitHandle(sx, sy) : null,
    mode: 'undecided', longFired: false,
    timer: setTimeout(onTouchLongPress, LONG_PRESS_MS),
  };
}

cv.addEventListener('contextmenu', e => {
  e.preventDefault();
  if (pickMode) { endPick('已取消点选'); return; }
  const [sx, sy] = evPos(e);
  const hb = hitBody(sx, sy);
  const ho = hb || hitOther(sx, sy);
  if (ho) select(ho.id);
  openContextMenu(e.clientX, e.clientY, ho || null, 'canvas');
});
cv.addEventListener('pointerdown', e => {
  // 右键不 capture，交给 contextmenu；中键/左键才拖拽
  if (e.button === 2) return;
  e.preventDefault();
  if (e.pointerType === 'touch') { touchPointerDown(e); return; }
  pointerCoarse = false;
  const [sx, sy] = evPos(e);
  try { cv.setPointerCapture(e.pointerId); } catch { /* ignore */ }
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
  if (pickMode) {
    const pm = pickMode, o = pickHit(sx, sy);
    if (o) { endPick(); pm.onPick(o); }
    else if (pm.allowPoint && !(hitBody(sx, sy) || hitOther(sx, sy))) { endPick(); pm.onPoint(sp); }
    else toast(pm.rejectMsg || '这个对象不能选，请点高亮的对象');
    return;
  }
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
  const hh = tool === 'select' ? hitHandle(sx, sy) : null;
  if (hh) {
    pushUndo(hh.key === 'v0' ? '调整初速度' : '调整外力');
    drag = { type: hh.key, b: hh.b, k: hh.k, moved: false };
    requestDraw();
    return;
  }
  const additive = e.shiftKey || e.ctrlKey || e.metaKey;
  // 单选时的类型手柄（区域四角、挡板端点）
  if (cur && selSet.size === 1 && !cur.locked && (cur.kind === 'efield' || cur.kind === 'bfield') && !cur.all) {
    const hs = rectHandles(cur);
    for (let i = 0; i < 4; i++) if (Math.hypot(sx - hs[i][0], sy - hs[i][1]) < 9) { pushUndo('调整区域'); drag = { type: 'fieldCorner', o: cur, i, moved: false }; return; }
  }
  if (cur && selSet.size === 1 && !cur.locked && cur.kind === 'wall') {
    const [a, b] = w2s(cur.x1, cur.y1), [c, d] = w2s(cur.x2, cur.y2);
    if (Math.hypot(sx - a, sy - b) < 9) { pushUndo('调整端点'); drag = { type: 'wallEnd', o: cur, end: 1, moved: false }; return; }
    if (Math.hypot(sx - c, sy - d) < 9) { pushUndo('调整端点'); drag = { type: 'wallEnd', o: cur, end: 2, moved: false }; return; }
  }
  const hb = hitBody(sx, sy);
  if (hb && sim.running && !hb.fixed && !additive) {
    // 运行中：直接抓住物体，松手时按鼠标速度甩出（不改初始条件）
    select(hb.id, { deferPanel: true });
    hb._grab = { x: hb.st.x, y: hb.st.y, vx: 0, vy: 0, t: performance.now() };
    drag = { type: 'grab', b: hb, ox: wp[0] - hb.st.x, oy: wp[1] - hb.st.y };
    cv.style.cursor = 'grabbing';
    return;
  }
  if (!hb && !additive) {
    const pi = hitPin(sx, sy);
    if (pi >= 0) { const p = pins[pi]; pins.splice(pi, 1); toast('已移除标记点'); if (selId === p.bodyId) buildPanel(); requestDraw(); drag = null; return; }
    const th = findTrace(sx, sy);
    if (th) { addPin(th); traceHit = null; requestDraw(); return; }
  }
  const ho = hb || hitOther(sx, sy);
  if (ho) {
    if (additive) { select(ho.id, { group: true, toggle: true }); return; }          // Shift/Ctrl：加选或减选
    if (!isSel(ho)) select(ho.id, { group: true, deferPanel: true });               // 编组：整组选中
    else if (selId !== ho.id) { selId = ho.id; }
    const movers = movableSelection();
    if (!movers.length) return;
    if (movers.length === 1 && isBody(movers[0])) {
      const b = movers[0];
      pushUndo(`移动「${b.name}」`);
      drag = { type: 'body', b, ox: wp[0] - b.st.x, oy: wp[1] - b.st.y, moved: false, sx, sy, wasT: sim.t };
      return;
    }
    pushUndo(movers.length > 1 ? `移动 ${movers.length} 个对象` : `移动「${ho.name}」`);
    drag = { type: 'moveSel', snap: posSnapshot(movers), start: sp, moved: false, sx, sy, wasT: sim.t, hasBody: movers.some(isBody) };
    return;
  }
  if (e.pointerType === 'touch') { drag = { type: 'pan', sx, sy, cx: scene.view.cx, cy: scene.view.cy, click: true }; return; }
  // 空白处：拖出选框（Shift 在原选择上追加）
  drag = { type: 'marquee', sx, sy, rect: [sx, sy, sx, sy], add: additive, base: additive ? new Set(selSet) : new Set(), active: false };
});
let lastDragMovedAt = 0;
cv.addEventListener('dblclick', e => {
  if (tool !== 'select') return;
  if (performance.now() - lastDragMovedAt < 120) return;   // 第二下其实是一次拖动（dblclick 紧随其 pointerup 触发），不算双击
  const [sx, sy] = evPos(e);
  const o = hitBody(sx, sy) || hitOther(sx, sy);
  if (o && groupOf(o)) { select(o.id, { group: true, deep: true, forcePanel: true }); toast(`已进入「${groupOf(o).name}」，只选中「${o.name}」`); }
});
cv.addEventListener('pointermove', e => {
  const [sx, sy] = evPos(e);
  if (e.pointerType === 'touch') {
    if (touchPoints.has(e.pointerId)) touchPoints.set(e.pointerId, { x: sx, y: sy });
    if (touchPoints.size >= 2) {
      if (!pinch) beginPinch();
      if (pinch) { e.preventDefault(); updatePinch(); }
      return;
    }
    const s = touchSession;
    if (!s || s.id !== e.pointerId || s.mode === 'consumed' || s.mode === 'pinch') return;
    if (s.mode === 'undecided') {
      if (Math.hypot(sx - s.sx, sy - s.sy) < TOUCH_SLOP) return;
      promoteTouch(s);
    }
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
  if (drag.type === 'v0') {
    const b = drag.b, [ox, oy] = w2s(b.st.x, b.st.y);
    let vx = (sx - ox) / drag.k, vy = -(sy - oy) / drag.k;
    if (e.shiftKey) { const v = Math.round(Math.hypot(vx, vy)), a = Math.round(Math.atan2(vy, vx) / DEG / 15) * 15 * DEG; vx = v * Math.cos(a); vy = v * Math.sin(a); }
    else { const q = niceStep(8 / drag.k) || 0.1; vx = Math.round(vx / q) * q; vy = Math.round(vy / q) * q; }
    vx = clean(vx); vy = clean(vy);
    drag.moved = true;
    clearExpr(b, ['vx0', 'vy0', 'v0', 'v0ang']);
    b.vx0 = vx; b.vy0 = vy;
    syncInitialLive(b);
    return;
  }
  if (drag.type === 'F') {
    const b = drag.b, [x, y] = w2s(b.st.x, b.st.y);
    let fx = (sx - x) / drag.k, fy = -(sy - y) / drag.k;
    if (e.shiftKey) {
      const mag = Math.round(Math.hypot(fx, fy) * 10) / 10;
      const a = Math.round(Math.atan2(fy, fx) / DEG / 15) * 15 * DEG;
      fx = mag * Math.cos(a); fy = mag * Math.sin(a);
    } else { const q = niceStep(8 / drag.k) || 0.01; fx = Math.round(fx / q) * q; fy = Math.round(fy / q) * q; }
    drag.moved = true;
    clearExpr(b, ['F', 'Fang']);
    b.Fmode = 'const';
    b.F = clean(Math.hypot(fx, fy));
    b.Fang = clean(Math.atan2(fy, fx) / DEG);
    b._Ferr = '';
    liveEdit(); return;
  }
  if (drag.type === 'marquee') {
    if (!drag.active && Math.hypot(sx - drag.sx, sy - drag.sy) < 4) return;
    drag.active = true;
    drag.rect = [Math.min(drag.sx, sx), Math.min(drag.sy, sy), Math.max(drag.sx, sx), Math.max(drag.sy, sy)];
    const ids = new Set(drag.base);
    for (const o of scene.objects) {
      if (!canvasPickable(o) || !hitsMarquee(o, drag.rect)) continue;
      for (const id of expandForClick(o, false)) ids.add(id);
    }
    setSelection([...ids], null, { deferPanel: true });
    return;
  }
  if (drag.type === 'moveSel') {
    if (!drag.moved && Math.hypot(sx - drag.sx, sy - drag.sy) < 3) return;
    drag.moved = true;
    restorePos(drag.snap);
    const dx = clean(sp[0] - drag.start[0]), dy = clean(sp[1] - drag.start[1]);
    for (const [o] of drag.snap) translateObject(o, dx, dy);
    if (drag.hasBody) syncInitialLive(null); else liveEditLight();
    return;
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
  if (d.moved || d.active) lastDragMovedAt = performance.now();
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
  if (d.type === 'marquee') {
    if (!d.active) { if (!d.add) select(null); return; }
    buildPanel(); refreshLeft(); requestDraw();
    return;
  }
  if (d.type === 'moveSel') {
    if (!d.moved) { cancelLastUndo(); buildPanel(); refreshLeft(); return; }
    if (d.hasBody && d.wasT > 0) toast('已修改初始位置，回到 t = 0');
    if (d.hasBody) initChanged(); else { markFieldsDirty(); markDirty(); draw(); }
    buildPanel(); refreshLeft();
    return;
  }
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
    if (!(e && (e.shiftKey || e.keepArm))) setTool('select');
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
      if (touchSession) { clearTimeout(touchSession.timer); touchSession = null; }
      drag = null;
      pending = null;
      cv.style.cursor = 'default';
      markDirty();
      requestDraw();
      return;
    }
    if (touchSession && touchSession.id === e.pointerId) {
      clearTimeout(touchSession.timer);
      const s = touchSession;
      const mode = s.mode;
      touchSession = null;
      if (e.type === 'pointercancel') { revertLiveDrag(); return; }
      if (mode === 'undecided') { commitTouchTap(s); return; }
      if (mode === 'consumed' || mode === 'pinch') return;
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
cv.addEventListener('pointerleave', () => { mouseIn = false; hoverObj = null; traceHit = null; requestDraw(); });
function hoverCursor(sx, sy) {
  if (pickMode) {
    const o = pickHit(sx, sy);
    cv.style.cursor = o ? 'pointer' : pickMode.allowPoint ? 'crosshair' : 'not-allowed';
    hoverObj = o; requestDraw(); return;
  }
  let h = null;
  let tr = null;
  if (tool === 'select') {
    const hh = hitHandle(sx, sy);
    const hk = hh ? hh.key + hh.b.id : '', ok = hoverHandle ? hoverHandle.key + hoverHandle.b.id : '';
    hoverHandle = hh; if (hk !== ok) requestDraw();
    if (hh) { cv.style.cursor = 'grab'; h = null; }
    else {
      h = hitBody(sx, sy);
      if (!h) { tr = hitPin(sx, sy) >= 0 ? null : findTrace(sx, sy); }
      if (!h && !tr) h = hitOther(sx, sy);
      cv.style.cursor = tr || hitPin(sx, sy) >= 0 ? 'pointer' : h ? (sim.running && isBody(h) && !h.fixed ? 'grab' : 'move') : 'default';
    }
  }
  const trKey = tr ? `${tr.b.id}|${tr.src}|${tr.t}` : '', oldKey = traceHit ? `${traceHit.b.id}|${traceHit.src}|${traceHit.t}` : '';
  traceHit = tr;
  if (trKey !== oldKey) requestDraw(); else if (LINK_KINDS.includes(tool)) {
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
      b.st.x = b.x0; b.st.y = b.y0; b.st.vx = b.vx0; b.st.vy = b.vy0; b.st.ang = (b.ang0 || 0) * DEG; b.st.omega = canRotate(b) ? (b.omega0 || 0) : 0;
      b.st.trail = [[b.x0, b.y0, 0, b.vx0, b.vy0]]; b.st.hist = []; pushHist(b);
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
  const list = movableSelection(); if (!list.length) return false;
  const now = performance.now();
  if (now - lastNudge > 700) pushUndo(list.length > 1 ? `移动 ${list.length} 个对象` : `移动「${list[0].name}」`);   // 连续按键合并为一步撤销
  lastNudge = now;
  list.forEach(o => translateObject(o, dx, dy));
  if (list.some(isBody)) initChanged(); else liveEdit();
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
  if (modalOpen()) return;
  if (isRange && (e.key.startsWith('Arrow') || e.key === 'Home' || e.key === 'End')) return; // 滑块自身处理方向键
  if (isRange && e.key === ' ') ae.blur();
  if (e.key === 'Escape') {
    if (openMenuIdx >= 0) { closeMenus(); return; }
    if ($('#help').classList.contains('show')) { $('#help').classList.remove('show'); return; }
    $('#viewFlags')?.classList.remove('show'); $('#bView')?.classList.remove('active');
    hideContextMenu();
    closeMobilePanels();
    if (homeShown() && docOpen) { hideHome(); return; }
    if (pickMode) { endPick('已取消点选'); return; }
    if (pending) { pending = null; draw(); }
    if (tool !== 'select') setTool('select');
    else if (selSet.size) setSelection([]);
    return;
  }
  if (e.key.startsWith('Arrow') && !(e.ctrlKey || e.metaKey || e.altKey)) {
    if (!docOpen || homeShown() || !selSet.size) return;
    e.preventDefault();
    const st = (scene.settings.snapStep || 0.1) * (e.shiftKey ? 10 : 1), k = e.key;
    nudgeSel(k === 'ArrowLeft' ? -st : k === 'ArrowRight' ? st : 0, k === 'ArrowUp' ? st : k === 'ArrowDown' ? -st : 0);
    return;
  }
  if (e.repeat && e.code === 'Space') { e.preventDefault(); return; }
  handleShortcut(e);
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
  bumpEdit();
  undoStack.push({ s: serialize(), label, redo: redoStack.splice(0) });
  if (undoStack.length > 120) undoStack.shift();
  syncHistoryButtons();
}
/** 操作最终没有产生改动（例如点了一下没拖动）：撤回刚压入的记录，并恢复重做栈。 */
function cancelLastUndo() {
  const it = undoStack.pop();
  if (it) editSeq--;
  if (it && it.redo) redoStack.push(...it.redo);
  syncHistoryButtons();
}
function restoreSnapshot(json) {
  const keepSel = selId, keepSet = [...selSet];
  loadScene(JSON.parse(json), true);
  const alive = keepSet.filter(id => byId(id));
  if (alive.length) {
    selSet = new Set(alive); selId = byId(keepSel) ? keepSel : alive.at(-1);
    syncListSelection(); buildPanel(); draw(); updateOptContext();
  }
}
function undo() {
  const it = undoStack.pop();
  if (!it) { toast('没有可撤销的操作'); return; }
  redoStack.push({ s: serialize(), label: it.label });
  restoreSnapshot(it.s); bumpEdit();
  toast(it.label ? `已撤销：${it.label}` : '已撤销');
  syncHistoryButtons();
}
function redo() {
  const it = redoStack.pop();
  if (!it) { toast('没有可重做的操作'); return; }
  undoStack.push({ s: serialize(), label: it.label });
  restoreSnapshot(it.s); bumpEdit();
  toast(it.label ? `已重做：${it.label}` : '已重做');
  syncHistoryButtons();
}
function syncHistoryButtons() {
  const u = $('#bUndo'), r = $('#bRedo');
  if (u) { u.disabled = !undoStack.length; u.title = undoStack.length ? `撤销${undoStack.at(-1).label ? '：' + undoStack.at(-1).label : ''} (Ctrl+Z)` : '没有可撤销的操作'; }
  if (r) { r.disabled = !redoStack.length; r.title = redoStack.length ? `重做${redoStack.at(-1).label ? '：' + redoStack.at(-1).label : ''} (Ctrl+Shift+Z)` : '没有可重做的操作'; }
  syncCommands();
}

/* ---------------- 复制/删除 ---------------- */
function deleteSel() {
  const s = selected(); if (!s) return; pushUndo(`删除「${s.name}」`);
  const links = scene.objects.filter(o => isLink(o) && (o.a === s.id || o.b === s.id)).length;
  scene.objects = scene.objects.filter(o => o !== s && !(isLink(o) && (o.a === s.id || o.b === s.id)));
  selId = null; selSet = new Set(); afterStructure(); buildPanel(); updateOptContext();
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
/**
 * 克隆一组对象：连接件只有两端都在这组里（或一端固定点）时才一起复制，并把端点 id 重映射到副本；
 * 编组关系在副本中保留为新的编组。
 */
function cloneSet(srcs, dx, dy) {
  const ids = new Set(srcs.map(o => o.id)), idMap = new Map(), gMap = new Map(), out = [];
  for (const o of srcs) {
    if (isLink(o)) continue;
    const c = cloneObject(o, dx, dy);
    idMap.set(o.id, c.id);
    if (o.group != null) { if (!gMap.has(o.group)) gMap.set(o.group, nextGroupId++); c.group = gMap.get(o.group); }
    out.push(c);
  }
  // 连接件：选中的，或两端都被复制的
  const links = scene.objects.filter(l => isLink(l) && idMap.has(l.a) && (l.b == null || idMap.has(l.b)));
  for (const l of [...new Set([...srcs.filter(isLink), ...links])]) {
    if (!idMap.has(l.a) || (l.b != null && !idMap.has(l.b))) continue;
    const c = JSON.parse(JSON.stringify(Object.assign({}, l, { _N: 0 })));
    c.id = nextId++; c.name = uniqueName(l.name + ' 副本'); c.a = idMap.get(l.a); if (l.b != null) c.b = idMap.get(l.b);
    if (l.b == null) { c.ax = clean(c.ax + dx); c.ay = clean(c.ay + dy); }
    delete c.group;
    out.push(c);
  }
  if (gMap.size) {
    if (!scene.groups) scene.groups = [];
    for (const [old, nid] of gMap) { const g = (scene.groups || []).find(x => x.id === old); scene.groups.push({ id: nid, name: uniqueGroupName((g ? g.name : '编组') + ' 副本') }); }
  }
  return out;
}
function insertClones(srcs, dx, dy, label) {
  if (!srcs.length) return;
  if (srcs.every(isLink)) { toast('连接件请与它连接的物体一起复制'); return; }
  pushUndo(label);
  const made = cloneSet(srcs, dx, dy);
  scene.objects.push(...made);
  pruneGroups();
  if (made.some(isBody)) { if (sim.t > 0) resetSim(); made.filter(isBody).forEach(initState); }
  afterStructure();
  setSelection(made.filter(o => !isLink(o)).map(o => o.id));
  toast(made.length === 1 ? `已${label}「${made[0].name}」` : `已${label} ${made.length} 个对象`, { action: UNDO_ACTION });
}
function insertClone(src, dx, dy, label) { insertClones([src], dx, dy, label); }
function duplicateSel() {
  const list = selection().filter(o => !o.locked);
  if (!list.length) return;
  const off = Math.max(scene.settings.snapStep || 0.1, 0.5);
  insertClones(list, off, list.length === 1 && list[0].kind === 'ground' ? off : -off, '复制');
}
let clipboard = null;
function copySel(cut = false) {
  const list = selection();
  if (!list.length) return;
  const ids = new Set(list.map(o => o.id));
  const links = scene.objects.filter(l => isLink(l) && ids.has(l.a) && (l.b == null || ids.has(l.b)));
  clipboard = JSON.parse(JSON.stringify([...new Set([...list, ...links])].map(o => Object.assign({}, o, { st: null, _exprErr: null, _hold: null, _grab: null }))));
  if (cut) {
    pushUndo('剪切');
    const kill = new Set(list.map(o => o.id));
    scene.objects = scene.objects.filter(o => !kill.has(o.id) && !(isLink(o) && (kill.has(o.a) || kill.has(o.b))));
    pruneGroups(); selSet.clear(); selId = null; afterStructure(); buildPanel();
    toast(`已剪切 ${list.length} 个对象`, { action: UNDO_ACTION });
  } else toast(list.length === 1 ? `已拷贝「${list[0].name}」· Ctrl+V 粘贴到鼠标处` : `已拷贝 ${list.length} 个对象`);
}
function pasteClip() {
  if (!clipboard || !clipboard.length) { toast('剪贴板是空的'); return; }
  // 以剪贴板内容的包围盒中心对齐到鼠标位置（不在画布上时偏移 0.5 m）
  const objs = clipboard.map(o => JSON.parse(JSON.stringify(o)));
  const bbs = objs.filter(o => !isLink(o)).map(o => { const t = Object.assign({}, o, { st: null }); return objBounds(t); }).filter(Boolean);
  let cx = 0, cy = 0;
  if (bbs.length) { cx = (Math.min(...bbs.map(b => b[0])) + Math.max(...bbs.map(b => b[2]))) / 2; cy = (Math.min(...bbs.map(b => b[1])) + Math.max(...bbs.map(b => b[3]))) / 2; }
  const p = mouseIn ? snapPt(mouseW) : [cx + 0.5, cy - 0.5];
  insertClonesFrom(objs, clean(p[0] - cx), clean(p[1] - cy));
}
/** 从剪贴板数据（可能已不在场景中）克隆 */
function insertClonesFrom(pool, dx, dy) {
  const idMap = new Map(), out = [];
  pushUndo('粘贴');
  for (const o of pool) { if (isLink(o)) continue; const c = cloneObject(o, dx, dy); delete c.group; idMap.set(o.id, c.id); out.push(c); }
  for (const l of pool) {
    if (!isLink(l) || !idMap.has(l.a) || (l.b != null && !idMap.has(l.b))) continue;
    const c = JSON.parse(JSON.stringify(l)); c.id = nextId++; c.name = uniqueName(l.name); c.a = idMap.get(l.a); if (l.b != null) c.b = idMap.get(l.b);
    if (l.b == null) { c.ax = clean(c.ax + dx); c.ay = clean(c.ay + dy); }
    delete c.group; out.push(c);
  }
  scene.objects.push(...out);
  if (out.some(isBody)) { if (sim.t > 0) resetSim(); out.filter(isBody).forEach(initState); }
  afterStructure();
  setSelection(out.filter(o => !isLink(o)).map(o => o.id));
  toast(`已粘贴 ${out.length} 个对象`, { action: UNDO_ACTION });
}

/* ---------------- 播放控制 ---------------- */
function updatePlayBtn() {
  const b = $('#bPlay'); if (!b) return;
  const label = sim.running ? '暂停' : '播放';
  b.innerHTML = `<svg class="ic"><use href="#i-${sim.running ? 'pause' : 'play'}"/></svg><span class="tb-label">${label}</span>`;
  b.setAttribute('aria-label', label);
  b.classList.toggle('primary', !sim.running);
}
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
  animateVars(ts);
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
  const zp = $('#zoomPct'); if (zp) { const t = `${fmt(scene.view.scale, 3)} px/m`; if (zp.textContent !== t) zp.textContent = t; }
  const el = $('#tbadge'); if (!el) return;
  const state = sim.running ? '运行' : sim.t === 0 ? '初始' : '暂停';
  const speed = scene.settings.speed !== 1 ? ` ${scene.settings.speed}×` : '';
  const t = `${fmt(sim.t, 2)} s · ${state}${speed}`;
  if (el.textContent !== t) el.textContent = t;
}
function fitView(only) {
  const pts = [];
  for (const o of (only && only.length ? only : scene.objects)) {
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
    input = el('input', { type: 'number', step: o.step || 'any', inputmode: 'decimal' });
    if (o.min != null) input.min = o.min;
    if (o.max != null) input.max = o.max;
    input.value = fmtIn(get());
    input.addEventListener('focus', () => pushed = false);
    const showErr = msg => {
      r.classList.toggle('invalid', !!msg);
      fieldErr.textContent = msg || '';
    };
    input.addEventListener('input', () => {
      const raw = input.value.trim();
      const v = parseFloat(raw);
      if (!raw || !isFinite(v)) { showErr(raw ? '请输入一个数' : ''); return; }   // 还在输入中
      const msg = checkNumber(v, o);
      if (msg) { showErr(msg); return; }
      showErr('');
      if (!pushed) { pushUndo(); pushed = true; }
      if (owner && owner.expr) delete owner.expr[key];
      set(v); o.init ? initChanged() : liveEdit(); if (o.after) o.after();
    });
    input.addEventListener('change', () => {
      if (r.classList.contains('invalid')) {
        const why = fieldErr.textContent;
        input.value = fmtIn(get());
        showErr(`${why}，已恢复为 ${fmtIn(get())}`);
        setTimeout(() => { if (document.activeElement !== input) showErr(''); }, 2600);
      } else input.value = fmtIn(get());
    });
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
  const fieldErr = el('div', { class: 'field-err' });
  parent.appendChild(fieldErr);
  let errEl = null;
  let valEl = null;
  if (formula) {
    valEl = el('div', { class: 'hint expr-val' }, `= ${fmt(Number(get()), 3)} ${unit}`.trim());
    errEl = el('div', { class: 'hint expr-err' }, (owner._exprErr && owner._exprErr[key]) || '');
    parent.appendChild(valEl);
    parent.appendChild(errEl);
    exprAssist(parent, input, { from: isBody(owner) ? owner.id : null });
  }
  binds.push({ el: input, get: () => (formula ? (owner.expr && owner.expr[key]) ?? '' : get()) });
  return input;
}
/** 数值合法性：min/max（可开区间）+ 自定义跨字段校验，返回错误文案或空串 */
function checkNumber(v, o) {
  if (!isFinite(v)) return '请输入一个数';
  if (o.min != null && (o.minEx ? v <= o.min : v < o.min)) return o.minEx ? `必须大于 ${o.min}` : `不能小于 ${o.min}`;
  if (o.max != null && (o.maxEx ? v >= o.max : v > o.max)) return o.maxEx ? `必须小于 ${o.max}` : `不能大于 ${o.max}`;
  return o.validate ? (o.validate(v) || '') : '';
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
      if (o.min != null) v = Math.max(o.minEx ? o.min + base * k : o.min, v);
      if (o.max != null) v = Math.min(o.maxEx ? o.max - base * k : o.max, v);
      if (o.validate && o.validate(v)) return;
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
  if (o.assist !== false) exprAssist(parent, i, o.assist || {});
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
  if (o.pick) {
    const pk = el('button', { type: 'button', class: 'pick-btn', title: '在画布上点选' }, '⌖');
    pk.onclick = () => startPick(Object.assign({}, o.pick, {
      onPick: obj => { pushUndo(); set(String(obj.id)); liveEdit(); buildPanel(); toast(`已选择「${obj.name}」`); },
    }));
    r.appendChild(pk);
  }
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
  derivedLive = [];
  const P = $('#props');
  if (P) {
    P.innerHTML = '';
    P.classList.remove('is-locked');
    const o = selected();
    if (!docOpen || !o) {
      P.appendChild(el('div', { class: 'empty-hint' }, !docOpen ? '新建或打开实验后，点选物体即可编辑参数。' : '未选中物体。在画布上点选，或打开「对象」。'));
    } else if (selSet.size > 1) buildMultiPanel(P);
    else buildSinglePanel(P, o);
  }
  buildAnalysisPane();
  buildEnvPane();
  document.body.classList.toggle('has-sel', !!(docOpen && selSet.size));
  syncCommands();
  buildQuick();
  updateContextBar();
}

/** 列表下方的紧凑参数：不打开完整检查器也能改 m、q、位置和初速度。 */
function openMore() {
  if (!selSet.size) return;
  document.body.dataset.more = 'open';
  document.body.dataset.list = 'open';
}
function buildQuick() {
  const host = $('#quick');
  if (!host) return;
  const list = selection();
  host.innerHTML = '';
  if (!docOpen || !list.length) { host.hidden = true; return; }
  host.hidden = false;
  const head = el('div', { class: 'quick-head' });
  if (list.length === 1) {
    const o = list[0];
    const nm = el('input', { type: 'text', value: o.name, 'aria-label': '名称', spellcheck: 'false' });
    let pushed = false;
    nm.addEventListener('focus', () => { pushed = false; });
    nm.addEventListener('input', () => {
      if (!pushed) { pushUndo('重命名'); pushed = true; }
      o.name = nm.value; refreshLeft(); draw(); markDirty();
      const ctx = $('#ctxName'); if (ctx) ctx.textContent = o.name;
    });
    nm.addEventListener('keydown', e => { if (e.key === 'Enter' || e.key === 'Escape') { e.preventDefault(); nm.blur(); } });
    nm.addEventListener('blur', () => {
      const v = nm.value.trim();
      if (!v) { o.name = uniqueName(KIND_NAME[o.kind] || '对象'); nm.value = o.name; refreshLeft(); draw(); markDirty(); }
    });
    head.appendChild(nm);
  } else head.appendChild(el('span', { class: 'quick-count' }, `${list.length} 个对象`));
  head.appendChild(el('button', { type: 'button', onclick: openMore }, '更多'));
  host.appendChild(head);
  if (list.length !== 1) return;
  const o = list[0];
  const grid = el('div', { class: 'quick-grid' });
  if (isBody(o)) {
    for (const d of schemaOf(o).filter(d => d.key === 'm' || d.key === 'q')) propRow(grid, o, d);
    numRow(grid, '位置 x₀', () => o.x0, v => o.x0 = v, 'm', { init: true, slot: [o, 'x0'] });
    numRow(grid, '位置 y₀', () => o.y0, v => o.y0 = v, 'm', { init: true, slot: [o, 'y0'] });
    const vmag = () => Math.hypot(o.vx0 || 0, o.vy0 || 0);
    const vang = () => (o.vx0 || o.vy0) ? Math.atan2(o.vy0, o.vx0) / DEG : (o._lastAng || 0);
    numRow(grid, '速率 v₀', vmag, v => { const a = vang() * DEG; o.vx0 = clean(v * Math.cos(a)); o.vy0 = clean(v * Math.sin(a * DEG)); }, 'm/s', { init: true, min: 0, slot: [o, 'v0'] });
    numRow(grid, '方向 θ', vang, a => { o._lastAng = a; const v = vmag(); o.vx0 = clean(v * Math.cos(a * DEG)); o.vy0 = clean(v * Math.sin(a * DEG)); }, '°', { init: true, slot: [o, 'v0ang'] });
  } else {
    for (const d of schemaOf(o).filter(d => d.type !== 'bool' && (!d.when || d.when(o))).slice(0, 4)) propRow(grid, o, d);
  }
  host.appendChild(grid);
}

function buildSinglePanel(P, o) {
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
  const g = groupOf(o);
  P.appendChild(el('div', { class: 'insp-meta' }, KIND_NAME[o.kind] + kindExtra + (g ? ` · 属于「${escapeHtml(g.name)}」` : '')));
  if (o.locked) {
    P.classList.add('is-locked');
    const bn = el('div', { class: 'lock-banner' }, '<span>已锁定：画布上不可选择或移动，属性只读。</span>');
    bn.appendChild(el('button', { type: 'button', onclick: () => setLocked([o], false) }, '解锁'));
    P.appendChild(bn);
  }

  if (isBody(o)) buildBodyPanel(P, o);
  else buildNonBodyPanel(P, o);
}

/** 多选：共同属性（取值不同时显示“混合”，修改会应用到全部）、排列与对齐 */
function buildMultiPanel(P) {
  const list = selection();
  const head = el('div', { class: 'insp-head' });
  const g = groupOf(list[0]);
  const whole = g && list.every(o => o.group === g.id) && list.length === groupMembers(g.id).length;
  if (whole) {
    const nm = el('input', { type: 'text', value: g.name, 'aria-label': '编组名称', title: '编组名称' });
    nm.addEventListener('change', () => { pushUndo('重命名编组'); g.name = nm.value.trim() || g.name; nm.value = g.name; refreshLeft(); requestDraw(); markDirty(); });
    nm.addEventListener('keydown', e => { if (e.key === 'Enter') nm.blur(); });
    head.appendChild(nm);
  } else head.appendChild(el('div', { class: 'insp-title' }, `${list.length} 个对象`));
  const acts = el('div', { class: 'insp-actions' });
  acts.appendChild(el('button', { type: 'button', title: '复制 (Ctrl+D)', 'aria-label': '复制', onclick: duplicateSel }, '<svg class="ic"><use href="#i-copy"/></svg>'));
  acts.appendChild(el('button', { type: 'button', class: 'danger', title: '删除 (Delete)', 'aria-label': '删除', onclick: deleteSelection }, '<svg class="ic"><use href="#i-trash"/></svg>'));
  head.appendChild(acts);
  P.appendChild(head);
  const counts = {};
  list.forEach(o => { counts[KIND_NAME[o.kind]] = (counts[KIND_NAME[o.kind]] || 0) + 1; });
  P.appendChild(el('div', { class: 'insp-meta' }, (whole ? `编组 · ${list.length} 个对象 · ` : '') + Object.entries(counts).map(([k, n]) => `${n} ${k}`).join(' · ')));
  const panels = el('div', { class: 'tab-panels' });
  const pp = panelBox('props', 'props');

  // 共同属性：每个对象的 schema 里都有、且单位一致的属性
  const defsOf = o => schemaOf(o).filter(d => !d.when || d.when(o));
  const common = defsOf(list[0]).filter(d => list.every(o => defsOf(o).some(e => e.key === d.key && e.type === d.type && e.unit === d.unit)));
  const editable = list.filter(o => !o.locked);
  if (common.length) {
    const groups = [...new Set(common.map(d => d.group))];
    for (const gname of groups) {
      const sec = section(pp, `${gname}（共同）`);
      for (const d of common.filter(x => x.group === gname)) {
        if (['x', 'y', 'x0', 'y0', 'x1', 'y1', 'x2', 'y2', 'ax', 'ay'].includes(d.key)) continue;   // 位置各不相同，用对齐/移动
        const vals = list.map(o => d.key === 'mus' ? (o.mus ?? o.mu ?? 0) : (o[d.key] ?? d.def));
        const same = vals.every(v => v === vals[0]);
        if (d.type === 'bool') {
          const c = chkRow(sec, d.label + (same ? '' : '（混合）'), () => list.every(o => !!(o[d.key] ?? d.def)), v => { editable.forEach(o => { o[d.key] = v; }); }, { init: d.init });
          if (!same) c.indeterminate = true;
          continue;
        }
        const inp = numRow(sec, `${d.label} <i class="sym">${d.sym}</i>`, () => (list.every(o => (d.key === 'mus' ? (o.mus ?? o.mu ?? 0) : o[d.key]) === vals[0]) ? vals[0] : NaN),
          v => { editable.forEach(o => { o[d.key] = v; }); }, d.unit, {
            min: d.min, minEx: d.minEx, max: d.max, step: d.step, init: d.init, title: d.title,
            validate: v => { for (const o of editable) { const m = d.check ? d.check(o, v) : ''; if (m) return `「${o.name}」：${m}`; } return ''; },
            after: () => { if (list.some(o => FIELD_SOURCE_KINDS.includes(o.kind))) markFieldsDirty(); refreshLeft(); },
          });
        if (!same) inp.placeholder = '混合';
      }
    }
  } else {
    const s0 = section(pp, '属性');
    s0.appendChild(el('div', { class: 'hint' }, '所选对象类型不同，没有共同属性。在图层面板里单独选中某个对象即可编辑它。'));
  }

  const ar = section(pp, '排列');
  const b1 = el('div', { class: 'btnrow' });
  btn(b1, whole ? '取消编组' : '编组', whole ? ungroupSelection : groupSelection, '', whole ? 'Ctrl+Shift+G' : 'Ctrl+G');
  btn(b1, '置于顶层', () => arrangeSelection('front')); btn(b1, '置于底层', () => arrangeSelection('back'));
  btn(b1, '锁定', () => setLocked(list, true)); btn(b1, '隐藏', () => setHidden(list, true));
  ar.appendChild(b1);
  const al = section(pp, '对齐与分布');
  const grid = el('div', { class: 'align-grid' });
  [['left', '左对齐', 'M3 2v12M6 5h7M6 11h4'], ['hcenter', '水平居中', 'M8 2v12M4 5h8M5.5 11h5'], ['right', '右对齐', 'M13 2v12M3 5h7M6 11h4'],
    ['top', '顶对齐', 'M2 3h12M5 6v7M11 6v4'], ['vcenter', '垂直居中', 'M2 8h12M5 4v8M11 5.5v5'], ['bottom', '底对齐', 'M2 13h12M5 3v7M11 6v4'],
    ['disth', '水平分布', 'M2 3v10M14 3v10M6 5v6M10 5v6'], ['distv', '垂直分布', 'M3 2h10M3 14h10M5 6h6M5 10h6']].forEach(([m, t, d]) => {
    const bt = el('button', { type: 'button', title: t, 'aria-label': t }, `<svg viewBox="0 0 16 16" width="16" height="16"><path d="${d}" stroke="currentColor" stroke-width="1.5" fill="none" stroke-linecap="round"/></svg><span>${t}</span>`);
    bt.onclick = () => alignSelection(m);
    grid.appendChild(bt);
  });
  al.appendChild(grid);
  al.appendChild(el('div', { class: 'hint' }, '按各对象的包围盒对齐；物体移动的是初始位置。分布需要至少 3 个对象。'));
  panels.appendChild(pp); P.appendChild(panels);
  syncDesmosKeypadForAnalysis(false);
}


/* ---------------- 由 Schema 生成的属性面板 ---------------- */
const FIELD_SOURCE_KINDS = ['efield', 'bfield', 'pointCharge'];
function propRow(parent, o, d) {
  if (d.type === 'bool') {
    return chkRow(parent, d.label, () => !!(o[d.key] ?? d.def), v => { o[d.key] = v; if (FIELD_SOURCE_KINDS.includes(o.kind)) markFieldsDirty(); },
      { init: d.init, rebuild: d.rebuild });
  }
  const get = d.key === 'mus' ? () => (o.mus ?? o.mu ?? 0) : () => o[d.key];
  return numRow(parent, `${d.label} <i class="sym">${d.sym}</i>`, get, v => { o[d.key] = v; }, d.unit, {
    min: d.min, minEx: d.minEx, max: d.max, step: d.step, title: d.title, init: d.init, slot: [o, d.key],
    validate: d.check ? v => d.check(o, v) : null,
    after: () => { if (d.after) d.after(o); if (FIELD_SOURCE_KINDS.includes(o.kind)) markFieldsDirty(); updateDerived(true); },
  });
}
function schemaSections(P, o, extras = {}, skipKeys = []) {
  const skip = new Set(skipKeys);
  const defs = schemaOf(o).filter(d => !skip.has(d.key) && (!d.when || d.when(o)));
  const groups = [...new Set(defs.map(d => d.group))];
  for (const g of groups) {
    const items = defs.filter(x => x.group === g);
    if (!items.length && !extras[g]) continue;
    const sec = section(P, g);
    for (const d of items) propRow(sec, o, d);
    if (extras[g]) extras[g](sec);
  }
}

/* ---------------- 派生量（只读，随状态实时更新） ---------------- */
let derivedLive = [];
function bodyPos(b) { return b.st || { x: b.x0, y: b.y0, vx: b.vx0 || 0, vy: b.vy0 || 0 }; }
function derivedOf(o) {
  const S = scene.settings, g = S.gOn ? S.g : 0, out = [];
  const add = (k, f) => out.push([k, f]);
  const kc = S.kc || K_COULOMB;
  if (isBody(o)) {
    if (g) add('重力 G = mg', () => `${fmt(o.m * g)} N`);
    if (o.q) add('比荷 q/m', () => `${fmt(o.q / o.m)} C/kg`);
    if (canRotate(o)) add(o.kind === 'box' ? '转动惯量 m(w²+h²)/12' : '转动惯量 2mr²/5（实心球）', () => `${fmt(rigidInertia(o))} kg·m²`);
    if (canRotate(o)) add('角速度 ω', () => `${fmt(o.st ? o.st.omega || 0 : o.omega0 || 0)} rad/s`);
    if (canRotate(o)) add('转动动能 ½Iω²', () => `${fmt(rotationalKE(o))} J`);
    if (o.kind === 'box') add('临界滑动倾角 arctan μₛ', () => `${fmt(Math.atan(o.mus ?? o.mu ?? 0) / DEG, 3)}°`);
    if (o.kind === 'box' && canRotate(o)) add('临界倾倒倾角 arctan(w/h)', () => `${fmt(Math.atan(o.w / o.h) / DEG, 3)}°`);
    if (o.q) for (const f of scene.objects) {
      if (f.kind === 'bfield' && f.B) {
        add(`「${f.name}」中周期 T = 2πm/|qB|`, () => `${fmt(2 * Math.PI * o.m / Math.abs(o.q * f.B))} s`);
        add(`「${f.name}」中半径 r = mv/|qB|`, () => { const p = bodyPos(o); return `${fmt(o.m * Math.hypot(p.vx, p.vy) / Math.abs(o.q * f.B))} m`; });
      }
      if (f.kind === 'efield' && f.E) add(`「${f.name}」中加速度 qE/m`, () => `${fmt(Math.abs(o.q * f.E / o.m))} m/s²`);
    }
  } else if (o.kind === 'wall') {
    add('长度', () => `${fmt(Math.hypot(o.x2 - o.x1, o.y2 - o.y1))} m`);
    add('与 +x 轴夹角', () => `${fmt(Math.atan2(o.y2 - o.y1, o.x2 - o.x1) / DEG, 3)}°`);
  } else if (o.kind === 'incline') {
    add('斜边长 L / cosθ', () => `${fmt(o.L / Math.cos(o.ang * DEG))} m`);
    add('高 L·tanθ', () => `${fmt(o.L * Math.tan(o.ang * DEG))} m`);
    add('临界摩擦因数 tanθ', () => fmt(Math.tan(o.ang * DEG), 4));
    if (g) add('光滑时下滑加速度 g·sinθ', () => `${fmt(g * Math.sin(o.ang * DEG))} m/s²`);
  } else if (o.kind === 'efield') {
    add('分量 (Eₓ, E_y)', () => `(${fmt(o.E * Math.cos(o.ang * DEG))}, ${fmt(o.E * Math.sin(o.ang * DEG))}) V/m`);
    if (!o.all) add('沿场方向的宽度 d', () => `${fmt(Math.abs(o.w * Math.cos(o.ang * DEG)) + Math.abs(o.h * Math.sin(o.ang * DEG)))} m`);
    if (!o.all) add('两侧电势差 U = Ed', () => `${fmt(o.E * (Math.abs(o.w * Math.cos(o.ang * DEG)) + Math.abs(o.h * Math.sin(o.ang * DEG))))} V`);
  } else if (o.kind === 'bfield') {
    add('方向', () => o.dir < 0 ? '垂直纸面向里 ⊗' : '垂直纸面向外 ⊙');
    for (const b of bodies()) if (b.q) add(`「${b.name}」回旋周期 2πm/|qB|`, () => `${fmt(2 * Math.PI * b.m / Math.abs(b.q * o.B))} s`);
  } else if (o.kind === 'pointCharge') {
    add('1 m 处场强 k|Q|/r²', () => `${fmt(kc * Math.abs(o.q))} V/m`);
    add('1 m 处电势 kQ/r', () => `${fmt(kc * o.q)} V`);
  } else if (isLink(o)) {
    const len = () => { const e = linkEnds(o); return e ? Math.hypot(e.x2 - e.x1, e.y2 - e.y1) : NaN; };
    add('当前长度', () => `${fmt(len())} m`);
    if (o.kind === 'spring') {
      add('形变量 ΔL = L − L₀', () => `${fmt(len() - o.L0)} m`);
      add('弹力 |kΔL|', () => `${fmt(Math.abs(o.k * (len() - o.L0)))} N（${len() >= o.L0 ? '拉伸' : '压缩'}）`);
      add('弹性势能 ½kΔL²', () => `${fmt(0.5 * o.k * (len() - o.L0) ** 2)} J`);
      const A = byId(o.a);
      if (o.b == null && A && !A.fixed) add(`「${A.name}」固有周期 2π√(m/k)`, () => `${fmt(2 * Math.PI * Math.sqrt(A.m / o.k))} s`);
    } else if (o.kind === 'rope') {
      add('状态', () => len() > o.L0 + 1e-9 ? '张紧' : '松弛');
      add('张力 T', () => `${fmt(Math.max(0, o.k * (len() - o.L0)))} N`);
    } else {
      add('轴向力', () => { const N = o._N || 0; return `${fmt(Math.abs(N))} N（${N >= 0 ? '拉' : '压'}）`; });
    }
  }
  return out;
}
function derivedSection(P, o) {
  const rows = derivedOf(o);
  if (!rows.length) return;
  const s = section(P, '派生量');
  const g = el('div', { class: 'readout derived' });
  rows.forEach(([k, f], i) => {
    g.appendChild(el('span', { class: 'k' }, k));
    const v = el('span', { id: 'dv_' + i }, '—');
    g.appendChild(v);
    derivedLive.push([v, f]);
  });
  s.appendChild(g);
  s.appendChild(el('div', { class: 'hint', style: 'margin-top:8px' }, '由当前参数与状态计算，只读。'));
  queueMicrotask(() => updateDerived(true));
}
let lastDerivedTs = 0;
function updateDerived(force = false) {
  if (!derivedLive.length) return;
  const now = performance.now();
  if (!force && now - lastDerivedTs < 100) return;
  lastDerivedTs = now;
  for (const [node, f] of derivedLive) {
    let v; try { v = f(); } catch { v = '—'; }
    node.textContent = v;
  }
}
const MODEL_NOTE = {
  ball: '刚性圆盘/实心球：平动 + 转动（I = 2mr²/5）。接触点处的摩擦产生力矩，可以滚动；关闭“允许转动”则只滑动。',
  particle: '带电质点：受电场力 qE 与洛伦兹力 qv×B（Boris 推进，磁场中不做功）。默认不与其他物体碰撞。',
  box: '矩形刚体：平动 + 转动（I = m(w²+h²)/12）。以四个角点与表面、其他物体接触，可以倾倒、翻滚；物体间按有向矩形（分离轴定理）求接触。',
  ground: '水平半空间：下方为实体，物体不会穿过。',
  wall: '有限长线段（挡板、极板、荧光屏）。两侧都可接触。',
  incline: '三角形楔块，三条边都是接触面。滑块落在斜面上会自动贴合斜面。',
  efield: '匀强电场：区域内 E 处处相同，区域外为零。方向角可写成随时间变化的公式（交变电场）。',
  bfield: '匀强磁场，方向垂直纸面。区域内对带电体施加 qv×B，区域外为零。',
  pointCharge: '固定点电荷，按库仑定律（可选 1/r² 或 1/r）产生电场，并绘制电场线。',
  spring: '理想弹簧：F = −k(L − L₀)，可拉可压。',
  rope: '轻绳：只拉不压，L ≤ 绳长时张力为 0。',
  rod: '轻杆：长度严格不变（SHAKE 约束），可拉可压，能提供支持力。',
};
function placeOnIncline(inc) {
  pushUndo('在斜面上放置滑块');
  if (sim.t > 0) resetSim();
  const th = inc.ang * DEG, d = inc.dir || 1, top = inclinePts(inc)[2], h = 0.4, along = Math.min(0.6, inc.L / Math.cos(th) * 0.2);
  const b = mkBody('box', { w: 0.6, h, m: 1, mu: 0.3, e: 0, ang0: d * inc.ang,
    x0: clean(top[0] - d * (Math.sin(th) * h / 2 + Math.cos(th) * along)), y0: clean(top[1] + Math.cos(th) * h / 2 - Math.sin(th) * along) });
  scene.objects.push(b); initState(b); afterStructure(); select(b.id);
  toast(`已在「${inc.name}」顶端放置滑块`, { action: UNDO_ACTION });
}
function buildNonBodyPanel(P, o) {
  const panels = el('div', { class: 'tab-panels' });
  const pp = panelBox('props', 'props');
  const extras = {};
  if (o.kind === 'incline') extras['几何'] = sec => {
    const br = el('div', { class: 'btnrow' });
    [30, 37, 45, 53, 60].forEach(a => btn(br, a + '°', () => { pushUndo('改倾角'); o.ang = a; liveEdit(); buildPanel(); }, Math.abs(o.ang - a) < 1e-9 ? 'active' : ''));
    btn(br, o.dir > 0 ? '朝向：右 ⇄' : '朝向：左 ⇄', () => { pushUndo('翻转斜面'); o.dir *= -1; liveEdit(); buildPanel(); });
    sec.appendChild(br);
  };
  if (o.kind === 'efield') extras['场'] = sec => {
    const br = el('div', { class: 'btnrow' });
    [[90, '↑ 上'], [-90, '↓ 下'], [180, '← 左'], [0, '→ 右']].forEach(([a, t]) => btn(br, t, () => { pushUndo('改电场方向'); o.ang = a; markFieldsDirty(); liveEdit(); buildPanel(); }, Math.abs(((o.ang - a) % 360 + 360) % 360) < 1e-9 ? 'active' : ''));
    sec.appendChild(br);
  };
  if (o.kind === 'bfield') extras['场'] = sec => {
    const br = el('div', { class: 'btnrow' });
    btn(br, '向里 ⊗', () => { pushUndo('改磁场方向'); o.dir = -1; markFieldsDirty(); liveEdit(); buildPanel(); }, o.dir < 0 ? 'active' : '');
    btn(br, '向外 ⊙', () => { pushUndo('改磁场方向'); o.dir = 1; markFieldsDirty(); liveEdit(); buildPanel(); }, o.dir > 0 ? 'active' : '');
    sec.appendChild(br);
  };
  if (o.kind === 'pointCharge') extras['源'] = sec => {
    const set = q => { pushUndo('改电荷量'); o.q = q; o.color = q >= 0 ? '#c8553d' : '#4a78b0'; markFieldsDirty(); liveEdit(); buildPanel(); };
    const br = el('div', { class: 'btnrow' });
    btn(br, '+1 μC', () => set(1e-6)); btn(br, '−1 μC', () => set(-1e-6)); btn(br, '取反', () => set(-o.q));
    sec.appendChild(br);
  };
  if (isLink(o)) extras['参数'] = sec => {
    const A = byId(o.a), B = o.b != null ? byId(o.b) : null;
    sec.appendChild(el('div', { class: 'hint' }, `连接：${escapeHtml(A ? A.name : '?')} — ${escapeHtml(B ? B.name : '固定点')}`));
    const br = el('div', { class: 'btnrow' });
    btn(br, `${o.kind === 'spring' ? '原长' : '长度'} = 当前距离`, () => {
      const e = linkEnds(o);
      if (e) { pushUndo('设为当前距离'); o.L0 = clean(Math.hypot(e.x2 - e.x1, e.y2 - e.y1)); liveEdit(); buildPanel(); }
    });
    sec.appendChild(br);
  };
  if (o.kind === 'wall') extras['几何'] = sec => sec.appendChild(el('div', { class: 'hint' }, '选中后可拖动两端的控制点。'));
  if ((o.kind === 'efield' || o.kind === 'bfield')) extras['区域'] = sec => { if (!o.all) sec.appendChild(el('div', { class: 'hint' }, '可拖动四角调整大小，拖动内部移动。')); };
  schemaSections(pp, o, extras);
  if (o.kind === 'incline') { const s2 = section(pp, '快捷'); btn(s2, '在顶端放置滑块', () => placeOnIncline(o)); }
  if (o.kind === 'efield') {
    const s2 = section(pp, '由电压推算');
    const r1 = el('div', { class: 'row' }); r1.appendChild(el('span', { class: 'lab' }, 'U，d'));
    const iu = el('input', { type: 'number', value: '100', title: '电压 U (V)', 'aria-label': '电压 U' });
    const id_ = el('input', { type: 'number', value: '1', title: '间距 d (m)', 'aria-label': '间距 d' });
    r1.appendChild(iu); r1.appendChild(el('span', { class: 'unit' }, 'V')); r1.appendChild(id_); r1.appendChild(el('span', { class: 'unit' }, 'm')); s2.appendChild(r1);
    btn(s2, '设 E = U / d', () => { const u = +iu.value, dd = +id_.value; if (!(dd > 0) || !isFinite(u)) { toast('d 必须大于 0'); return; } pushUndo('由电压设场强'); o.E = Math.abs(u / dd); markFieldsDirty(); liveEdit(); buildPanel(); });
  }
  derivedSection(pp, o);
  if (MODEL_NOTE[o.kind]) { const m = section(pp, '物理模型'); m.appendChild(el('div', { class: 'hint' }, MODEL_NOTE[o.kind])); }
  panels.appendChild(pp);
  P.appendChild(panels);
  syncDesmosKeypadForAnalysis(false);
}

function buildAppliedForcePanel(P, b) {
  ensureForceFields(b);
  const af = appliedForceOn(b);
  const mode = b.Fmode || 'const';

  const vis = section(P, '画布');
  vis.appendChild(el('div', { class: 'hint' }, '恒力模式下，暂停时拖动物体上紫色箭头的末端即可设置 F（Shift 按 15° 吸附）。'));
  ovQuickToggles(vis, b, ['F', 'forces', 'Fnet']);

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
    }, targets, { className: 'force-aim-select', pick: {
      prompt: `点选「${b.name}」的外力要指向的物体，或点空白处指向一个固定点`, from: b.id, allowPoint: true,
      accept: o => o.id !== b.id && (isBody(o) || o.kind === 'pointCharge'),
      onPoint: p => { pushUndo('外力指向一点'); b.FaimId = null; b.FaimX = p[0]; b.FaimY = p[1]; clearExpr(b, ['FaimX', 'FaimY']); liveEdit(); buildPanel(); toast(`外力指向 (${fmt(p[0], 3)}, ${fmt(p[1], 3)})`); },
    } });
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
  act.appendChild(br);
}

let chartMode = 'v';
function buildBodyPanel(P, b) {
  const form = el('div', { class: 'prop-form' });

  const pInit = panelBox('init', 'init');
  {
    const s = section(pInit, '关键参数', 'hl');
    for (const d of schemaOf(b).filter(d => d.key === 'm' || d.key === 'q')) propRow(s, b, d);
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
    s.appendChild(el('div', { class: 'hint' }, '每个数旁边的 ƒ 可改成公式，例如 vx₀ = v0*cos(θ)；写了不存在的名字会提示一键创建变量。t = 0 时选中物体，拖绿色箭头末端也能设置 v₀（会取消公式）。'));
    const dsec = section(pInit, '在画布上显示');
    ovQuickToggles(dsec, b, ['v', 'a', 'forces', 'Fnet', 'p', 'trail', 'predict', 'strobe', 'analytic', 'ghost', 'names']);
    const dr = el('div', { class: 'btnrow' });
    btn(dr, '＋ 数值标签…', () => { const ov = addOverlay('value', b.id); if (ov) setLeftTab('display'); });
    btn(dr, '＋ 自定义矢量…', () => { const ov = addOverlay('vec', b.id); if (ov) setLeftTab('display'); });
    dsec.appendChild(dr);
  }

  const pProps = panelBox('props', 'props');
  {
    schemaSections(pProps, b, {
      '行为': sec => {
        const tb = el('div', { class: 'btnrow' });
        btn(tb, '清除轨迹', () => { resetTrail(b); draw(); });
        sec.appendChild(tb);
      },
    }, ['m', 'q']);
    derivedSection(pProps, b);
    const m = section(pProps, '物理模型'); m.appendChild(el('div', { class: 'hint' }, MODEL_NOTE[b.kind] || ''));
  }

  const pForce = panelBox('force', 'force');
  pForce.classList.add('force-tab');
  buildAppliedForcePanel(pForce, b);

  const pLive = panelBox('live', 'live');
  {
    const r = section(pLive, '瞬时状态');
    readRow(r, [['位置', 'r_pos'], ['速度', 'r_vel'], ['速率', 'r_v'], ['方向', 'r_ang'], ['加速度', 'r_a'], ['动能', 'r_ek'], ['角速度', 'r_w'], ['动量', 'r_p'], ['重力势能', 'r_ep'], ['机械能', 'r_e']]);
    const ps = section(pLive, '轨迹标记点');
    const mine = pins.filter(p => p.bodyId === b.id).sort((u, v) => u.t - v.t);
    if (!mine.length) ps.appendChild(el('div', { class: 'hint' }, '把鼠标移到轨迹或预测曲线上会显示该时刻的读数，点击即可钉住一个标记点。'));
    else {
      const tb = el('table', { class: 'pin-table' });
      tb.innerHTML = '<thead><tr><th>t / s</th><th>x / m</th><th>y / m</th><th>v / (m/s)</th><th></th></tr></thead>';
      const body = el('tbody');
      for (const p of mine) {
        const tr = el('tr', { title: p.t <= sim.t + 1e-9 ? '已走轨迹' : '数值预测（尚未走到）' });
        const cells = [0, 1, 2, 3].map(() => el('td'));
        cells.forEach(c => tr.appendChild(c));
        const x = el('button', { type: 'button', class: 'pin-del', title: '移除', 'aria-label': '移除' }, '×');
        x.onclick = () => { pins.splice(pins.indexOf(p), 1); buildPanel(); requestDraw(); };
        const td = el('td'); td.appendChild(x); tr.appendChild(td);
        if (p.t > sim.t + 1e-9) tr.classList.add('pred');
        body.appendChild(tr);
        derivedLive.push([cells[0], () => { const q = pinPos(p); if (!q) { cells.slice(1).forEach(c => c.textContent = '—'); return fmt(p.t, 4); } cells[1].textContent = fmt(q[0], 4); cells[2].textContent = fmt(q[1], 4); cells[3].textContent = fmt(Math.hypot(q[3], q[4]), 4); return fmt(p.t, 4) + (p.t > sim.t + 1e-9 ? '*' : ''); }]);
      }
      tb.appendChild(body); ps.appendChild(tb);
      if (mine.some(p => p.t > sim.t + 1e-9)) ps.appendChild(el('div', { class: 'hint' }, '* 尚未走到的时刻，取自数值预测，随参数修改自动更新。'));
      const br = el('div', { class: 'btnrow' });
      btn(br, '清除全部标记', () => { pins = pins.filter(p => p.bodyId !== b.id); buildPanel(); requestDraw(); });
      ps.appendChild(br);
      updateDerived(true);
    }
  }

  const pArrange = panelBox('arrange', 'arrange');
  {
    const ar = section(pArrange, '排列');
    const b1 = el('div', { class: 'btnrow' });
    btn(b1, '置于顶层', () => arrangeSelection('front'));
    btn(b1, '上移一层', () => arrangeSelection('forward'));
    btn(b1, '下移一层', () => arrangeSelection('backward'));
    btn(b1, '置于底层', () => arrangeSelection('back'));
    btn(b1, '锁定', () => setLocked([b], true));
    btn(b1, '隐藏', () => setHidden([b], true));
    ar.appendChild(b1);
  }

  [pInit, pProps, pForce, pLive, pArrange].forEach(x => form.appendChild(x));
  P.appendChild(form);
  syncDesmosKeypadForAnalysis(document.body.dataset.sheet === 'analysis');
  updateReadouts();
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
  const rec = scene.info.recommend || [];
  if (rec.length) {
    const missing = recommendedOverlays(rec).filter(r => !overlays().some(o => o.on && o.type === r.type));
    const br = el('div', { class: 'btnrow l-actions' });
    if (missing.length) btn(br, `启用推荐显示（${missing.map(r => OVERLAY_TYPES[r.type].name).join('、')}）`, applyRecommended, 'primary');
    else br.appendChild(el('span', { class: 'hint' }, '推荐显示已启用。'));
    btn(br, '显示面板', () => setLeftTab('display'));
    card.appendChild(br);
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

let analysisSig = '';
function chartBlock(parent) {
  const sel = el('select', { id: 'chartSel' });
  [['v', 'v–t 速率'], ['vx', 'vx–t'], ['vy', 'vy–t'], ['a', 'a–t 加速度'], ['x', 'x–t'], ['y', 'y–t'], ['ek', 'Ek–t 动能'], ['xy', 'y–x 轨迹'], ['px', 'vx–x 相图'], ['py', 'vy–y 相图']].forEach(([v, t]) => sel.appendChild(el('option', { value: v }, t)));
  sel.value = chartMode;
  sel.addEventListener('change', () => { chartMode = sel.value; drawChart(); });
  const g = section(parent, '图像', '', sel);
  g.appendChild(el('canvas', { id: 'chart' }));
}
function buildAnalysisPane() {
  const host = $('#analysisBody'); if (!host) return;
  const b = selected();
  const sig = [docOpen ? 1 : 0, selId ?? '', selSet.size, (scene.info && scene.info.key) || '', overlays().filter(o => o.on).length, (scene.info && scene.info.recommend || []).length].join(':');
  if (sig === analysisSig && host.childElementCount) return;
  analysisSig = sig;
  host.innerHTML = '';
  if (isBody(b) && selSet.size === 1) {
    const th = section(host, '运动分析');
    th.appendChild(el('div', { id: 'r_theory', class: 'analysis-panel' }));
    chartBlock(host);
  } else {
    const s = section(host, '运动分析');
    s.appendChild(el('div', { class: 'hint' }, selSet.size > 1 ? '多选时请只保留一个物体，再查看它的参数方程与图像。' : '选中一个物体后，这里给出运动类型、参数方程，以及 v–t、相图等图像。'));
  }
  if (!lessonCard(host)) {
    const s = section(host, '实验原理');
    s.appendChild(el('div', { class: 'hint' }, '从「实验」载入模板后，这里显示原理、公式和可以尝试的改动。'));
    const br = el('div', { class: 'btnrow' });
    btn(br, '打开实验库', () => showHome('templates'));
    btn(br, '使用说明', () => $('#help').classList.add('show'));
    s.appendChild(br);
  }
  buildEnergySection(host);
  requestAnimationFrame(() => {
    updateEnergyReadout(true);
    const cur = selected();
    if (isBody(cur) && selSet.size === 1) renderMotionAnalysis(cur, true);
    drawChart();
  });
}
function buildEnvPane() {
  const P = $('#envBody'); if (!P) return;
  P.innerHTML = '';
  if (!docOpen) {
    P.appendChild(el('p', { class: 'env-note' }, '新建或打开文档后，可以在这里改重力、引力、库仑力和积分步长。'));
    return;
  }
  const S = scene.settings;
  const pPhy = el('div', { class: 'env-form' });
  pPhy.appendChild(el('h3', { class: 'flat-sec-title' }, '物理环境'));
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
    { const fl = overlays().find(o => o.type === 'fieldLines');
      const r = el('label', { class: 'chk', style: 'margin:4px 10px 4px 0' }), i = el('input', { type: 'checkbox' });
      i.checked = !!(fl && fl.on);
      i.onchange = () => { if (fl) { pushUndo(); fl.on = i.checked; onOverlaysChanged(true); } else addOverlay('fieldLines'); };
      r.appendChild(i); r.appendChild(document.createTextNode('显示电场线')); es.appendChild(r); }

    const ns = section(pPhy, '数值积分');
    numRow(ns, '最大步长 Δt', () => S.maxDt, v => S.maxDt = v, 's', { min: 1e-9, slot: [S, 'maxDt'] });
    const nb = el('div', { class: 'btnrow' });
    [[0.002, '快速'], [0.001, '标准'], [0.0002, '精细']].forEach(([v, t]) => btn(nb, t, () => { S.maxDt = v; liveEdit(); buildPanel(); }, Math.abs(S.maxDt - v) < 1e-12 ? 'active' : ''));
    ns.appendChild(nb);
    ns.appendChild(el('div', { class: 'hint' }, '每帧拆成若干子步，用 Boris 蛙跳格式推进（二阶、辛），磁场中能量严格守恒；碰撞与摩擦用冲量法。步长越小越准，但更耗时。'));
  }

  {
    const dv = section(pPhy, '显示项');
    const n = overlays().length;
    dv.appendChild(el('div', { class: 'hint' }, n ? `当前文档有 ${n} 条显示项（${overlays().filter(o => o.on).length} 条开启）。` : '画布上默认什么都不显示。速度、加速度、受力、轨迹、预测、频闪、数值标签都在「对象 → 显示」里逐项添加。'));
    const br = el('div', { class: 'btnrow' });
    btn(br, '打开显示项', () => setLeftTab('display'), 'primary');
    if (scene.info && scene.info.recommend && scene.info.recommend.length) btn(br, '启用推荐显示', applyRecommended);
    dv.appendChild(br);
    const d = section(pPhy, '编辑手柄比例');
    d.appendChild(el('div', { class: 'hint' }, '没有对应显示项时，拖初速度 / 外力用的箭头比例；有显示项时与显示项一致。'));
    numRow(d, '初速度', () => S.vPx, v => S.vPx = v, 'px/(m/s)', { min: 0.0001, slot: [S, 'vPx'] });
    numRow(d, '外力', () => S.FPx || 12, v => S.FPx = v, 'px/N', { min: 0.0001, slot: [S, 'FPx'] });
    const g = section(pPhy, '画布');
    chkRow(g, '网格', () => S.showGrid, v => S.showGrid = v);
    chkRow(g, '吸附网格', () => S.snap, v => S.snap = v);
    numRow(g, '吸附步长', () => S.snapStep, v => S.snapStep = v, 'm', { min: 1e-6, slot: [S, 'snapStep'] });
    const view = el('div', { class: 'btnrow' });
    btn(view, '适配视图', fitView);
    btn(view, '回到原点', () => { scene.view.cx = 0; scene.view.cy = 0; draw(); });
    btn(view, '清除所有轨迹', () => { bodies().forEach(resetTrail); draw(); });
    g.appendChild(view);
  }
  P.appendChild(pPhy);
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

function objColor(o) {
  return o.color || (o.kind === 'efield' ? '#b5762a' : o.kind === 'bfield' ? '#4f7faf' : o.kind === 'rope' ? '#4f7faf' : o.kind === 'rod' ? '#4d6470' : o.kind === 'spring' ? '#3d8f86' : '#7c796f');
}
/** 列表第二行：该对象最关键的参数，带单位 */
function summaryOf(o) {
  const v0 = o => Math.hypot(o.vx0 || 0, o.vy0 || 0);
  switch (o.kind) {
    case 'ball': case 'particle': case 'box':
      return [`m = ${fmt(o.m)} kg`, o.q ? `q = ${fmt(o.q)} C` : '', o.fixed ? '固定' : `v₀ = ${fmt(v0(o))} m/s`].filter(Boolean).join(' · ');
    case 'ground': return `y = ${fmt(o.y)} m${o.smooth ? ' · 光滑' : ''}`;
    case 'wall': return `长 ${fmt(Math.hypot(o.x2 - o.x1, o.y2 - o.y1))} m${o.smooth ? ' · 光滑' : ''}`;
    case 'incline': return `θ = ${fmt(o.ang)}° · 底边 ${fmt(o.L)} m${o.smooth ? ' · 光滑' : ''}`;
    case 'efield': return `E = ${fmt(o.E)} V/m · ${fmt(o.ang)}°${o.all ? ' · 全空间' : ''}${o.expr && Object.keys(o.expr).length ? ' · 含公式' : ''}`;
    case 'bfield': return `B = ${fmt(o.B)} T ${o.dir < 0 ? '⊗' : '⊙'}${o.all ? ' · 全空间' : ''}`;
    case 'pointCharge': return `Q = ${fmt(o.q)} C`;
    case 'spring': return `k = ${fmt(o.k)} N/m · L₀ = ${fmt(o.L0)} m`;
    case 'rope': return `L = ${fmt(o.L0)} m${o.m ? ` · m = ${fmt(o.m)} kg` : ''}`;
    case 'rod': return `L = ${fmt(o.L0)} m${o.m ? ` · m = ${fmt(o.m)} kg` : ''}`;
  }
  return '';
}
function matchQuery(text, q) {
  if (!q) return true;
  return String(text).toLowerCase().includes(q.trim().toLowerCase());
}

/* =====================================================================
 *  图层面板：编组 ▸ 对象 ▸（连接件、矢量）
 *  · 顺序即 z 序：列表越靠上越在前（scene.objects 越靠后）
 *  · 连接件挂在它连接的第一个物体下面，随物体移动
 *  · 眼睛 = 隐藏（仍参与物理计算）；锁 = 画布上不可选、不可动
 * ===================================================================== */
const ICON = {
  chev: '<svg viewBox="0 0 12 12"><path d="M4.5 2.5 8 6l-3.5 3.5" stroke="currentColor" stroke-width="1.5" fill="none" stroke-linecap="round" stroke-linejoin="round"/></svg>',
  eye: '<svg viewBox="0 0 16 16"><path d="M1.5 8S4 3.5 8 3.5 14.5 8 14.5 8 12 12.5 8 12.5 1.5 8 1.5 8z" stroke="currentColor" stroke-width="1.3" fill="none"/><circle cx="8" cy="8" r="2" fill="currentColor"/></svg>',
  eyeOff: '<svg viewBox="0 0 16 16"><path d="M2 2l12 12M6.3 4c.5-.3 1.1-.5 1.7-.5 4 0 6.5 4.5 6.5 4.5s-.7 1.3-2 2.5M9.8 11.9c-.6.4-1.2.6-1.8.6-4 0-6.5-4.5-6.5-4.5s.8-1.4 2.2-2.6" stroke="currentColor" stroke-width="1.3" fill="none" stroke-linecap="round"/></svg>',
  lock: '<svg viewBox="0 0 16 16"><rect x="3.5" y="7" width="9" height="6.5" rx="1.3" stroke="currentColor" stroke-width="1.3" fill="none"/><path d="M5.5 7V5a2.5 2.5 0 0 1 5 0v2" stroke="currentColor" stroke-width="1.3" fill="none"/></svg>',
  unlock: '<svg viewBox="0 0 16 16"><rect x="3.5" y="7" width="9" height="6.5" rx="1.3" stroke="currentColor" stroke-width="1.3" fill="none"/><path d="M5.5 7V5a2.5 2.5 0 0 1 4.8-1" stroke="currentColor" stroke-width="1.3" fill="none"/></svg>',
};
const layerOpen = new Set();          // 展开的物体 id（显示连接件 / 矢量）
let layerDrag = null;
function attachedTo(l) { return isLink(l) ? byId(l.a) : null; }
function isChildLink(o) { const a = attachedTo(o); return !!(a && isBody(a)); }
function swatchClass(o) { return isBody(o) ? (o.kind === 'box' ? 'sq' : '') : (isLink(o) || o.kind === 'wall' || o.kind === 'ground') ? 'ln' : 'sq'; }
function layerMatches(o, q) { return !q || matchQuery(o.name, q) || matchQuery(KIND_NAME[o.kind] || '', q) || matchQuery(summaryOf(o), q); }

function buildLayerTree(q) {
  const objs = scene.objects, nodes = [], seen = new Set();
  const objNode = o => {
    const kids = isBody(o) ? objs.filter(l => isLink(l) && l.a === o.id).reverse() : [];
    return { type: 'obj', o, kids };
  };
  for (let i = objs.length - 1; i >= 0; i--) {
    const o = objs[i];
    if (isChildLink(o)) continue;
    const g = groupOf(o);
    if (g) {
      if (seen.has(g.id)) continue; seen.add(g.id);
      const members = objs.filter(m => m.group === g.id && !isChildLink(m)).reverse().map(objNode);
      nodes.push({ type: 'group', g, kids: members });
    } else nodes.push(objNode(o));
  }
  if (!q) return nodes;
  const keep = n => n.type === 'group' ? (n.kids = n.kids.filter(keep)).length > 0 || matchQuery(n.g.name, q)
    : layerMatches(n.o, q) || n.kids.some(k => layerMatches(k, q));
  return nodes.filter(keep);
}

function layerRow(opts) {
  const { o, g, ind, hasKids, open, vec } = opts;
  const r = el('div', { class: 'lrow' + (g ? ' group' : ''), style: `--ind:${ind}` });
  const tw = el('button', { type: 'button', class: 'tw' + (open ? ' open' : ''), tabindex: '-1', 'aria-label': open ? '折叠' : '展开' }, hasKids ? ICON.chev : '');
  if (!hasKids) tw.style.visibility = 'hidden';
  r.appendChild(tw);
  if (vec) {
    r.appendChild(el('span')); r.appendChild(el('span'));
    r.appendChild(el('span', { class: 'lsw ln', style: `--c:${vec.color}` }));
    const m = el('div', { class: 'lmain' }); m.appendChild(el('div', { class: 'lname' }, escapeHtml(vec.name))); r.appendChild(m);
    r.appendChild(el('span', { class: 'lkind' }, '矢量'));
    return { r, tw };
  }
  const target = g || o;
  const hidden = !!target.hidden, locked = !!target.locked;
  const eye = el('button', { type: 'button', class: 'eye' + (hidden ? ' off' : ''), title: hidden ? '显示' : '隐藏（仍参与物理计算）', 'aria-label': hidden ? '显示' : '隐藏', tabindex: '-1' }, hidden ? ICON.eyeOff : ICON.eye);
  const lk = el('button', { type: 'button', class: 'lock' + (locked ? ' on' : ''), title: locked ? '解锁' : '锁定', 'aria-label': locked ? '解锁' : '锁定', tabindex: '-1' }, locked ? ICON.lock : ICON.unlock);
  r.appendChild(eye); r.appendChild(lk);
  r.appendChild(el('span', { class: 'lsw ' + (g ? 'grp' : swatchClass(o)), style: g ? '' : `--c:${objColor(o)}` }));
  const m = el('div', { class: 'lmain' });
  m.appendChild(el('div', { class: 'lname' }, escapeHtml(g ? g.name : o.name)));
  m.appendChild(el('div', { class: 'lsum' }, escapeHtml(g ? `${groupMembers(g.id).length} 个对象` : summaryOf(o))));
  r.appendChild(m);
  r.appendChild(el('span', { class: 'lkind' }, g ? '编组' : (KIND_NAME[o.kind] || '')));
  const dimmed = hidden || (o && groupOf(o) && groupOf(o).hidden);
  if (dimmed) r.classList.add('dim');
  return { r, tw, eye, lk, m };
}

function renderLayers(host) {
  const q = leftQuery;
  const wrap = el('div', { class: 'layers', role: 'tree' });
  const tree = buildLayerTree(q);
  const cnt = $('#layerCount'); if (cnt) cnt.textContent = scene.objects.length ? `${scene.objects.length} 个对象${(scene.groups || []).length ? ` · ${scene.groups.length} 个编组` : ''}` : '';
  if (!tree.length) {
    wrap.appendChild(el('div', { class: 'empty-hint' }, scene.objects.length ? '没有匹配的图层。' : '文档是空的。到「添加」里放置物体、表面、场与约束。'));
    host.appendChild(wrap); renderLayerFoot(); return;
  }
  const addObj = (n, ind) => {
    const o = n.o;
    const open = layerOpen.has(o.id) || (q && n.kids.some(k => layerMatches(k, q)));
    const { r, tw, eye, lk, m } = layerRow({ o, ind, hasKids: n.kids.length > 0, open });
    r.dataset.id = String(o.id);
    if (selSet.has(o.id)) r.classList.add('sel');
    if (selSet.size > 1 && o.id === selId) r.classList.add('primary');
    tw.onclick = ev => { ev.stopPropagation(); open ? layerOpen.delete(o.id) : layerOpen.add(o.id); refreshLeft(); };
    eye.onclick = ev => { ev.stopPropagation(); setHidden([o], !o.hidden); };
    lk.onclick = ev => { ev.stopPropagation(); setLocked([o], !o.locked); };
    r.onclick = ev => {
      const additive = ev.shiftKey || ev.ctrlKey || ev.metaKey;
      select(o.id, { toggle: additive, forcePanel: true });
    };
    const sw = r.querySelector('.lsw');
    if (sw) sw.onclick = ev => { ev.stopPropagation(); select(o.id, { forcePanel: true }); openStyle(o, sw); };
    m.ondblclick = ev => { ev.stopPropagation(); inlineRename(m, o.name, v => { pushUndo('重命名'); o.name = v; buildPanel(); requestDraw(); markDirty(); }); };
    r.oncontextmenu = ev => { ev.preventDefault(); if (!selSet.has(o.id)) select(o.id); openContextMenu(ev.clientX, ev.clientY, o, 'list'); };
    if (!isChildLink(o)) makeDraggable(r, { id: o.id });
    wrap.appendChild(r);
    if (open) {
      for (const k of n.kids) addObj({ o: k, kids: [] }, ind + 1);
    }
  };
  for (const n of tree) {
    if (n.type === 'group') {
      const g = n.g, open = !g.collapsed || !!q;
      const { r, tw, eye, lk, m } = layerRow({ g, ind: 0, hasKids: true, open });
      r.dataset.group = String(g.id);
      const members = groupMembers(g.id);
      if (members.length && members.every(x => selSet.has(x.id))) r.classList.add('sel');
      tw.onclick = ev => { ev.stopPropagation(); g.collapsed = !g.collapsed; if (!g.collapsed) delete g.collapsed; refreshLeft(); markDirty(); };
      eye.onclick = ev => { ev.stopPropagation(); pushUndo(g.hidden ? '显示编组' : '隐藏编组'); g.hidden = !g.hidden; if (!g.hidden) delete g.hidden; refreshLeft(); requestDraw(); markDirty(); };
      lk.onclick = ev => { ev.stopPropagation(); pushUndo(g.locked ? '解锁编组' : '锁定编组'); g.locked = !g.locked; if (!g.locked) delete g.locked; if (g.locked) setSelection([]); refreshLeft(); markDirty(); };
      r.onclick = ev => {
        const ids = members.map(x => x.id);
        if (ev.shiftKey || ev.ctrlKey || ev.metaKey) { const all = ids.every(i => selSet.has(i)); ids.forEach(i => all ? selSet.delete(i) : selSet.add(i)); setSelection([...selSet]); }
        else setSelection(ids);
      };
      m.ondblclick = ev => { ev.stopPropagation(); inlineRename(m, g.name, v => { pushUndo('重命名编组'); g.name = v; buildPanel(); requestDraw(); markDirty(); }); };
      makeDraggable(r, { group: g.id });
      wrap.appendChild(r);
      if (open) for (const k of n.kids) addObj(k, 1);
    } else addObj(n, 0);
  }
  host.appendChild(wrap);
  renderLayerFoot();
}
function renderLayerFoot() {
  const f = $('#layerFoot'); if (!f) return;
  f.innerHTML = '';
  const n = selSet.size;
  const b = (t, fn, title, dis) => { const x = el('button', { type: 'button', title: title || t }, t); x.onclick = fn; if (dis) x.disabled = true; f.appendChild(x); };
  b('编组', groupSelection, '编组 (Ctrl+G)', n < 2);
  b('取消编组', ungroupSelection, '取消编组 (Ctrl+Shift+G)', !selection().some(o => o.group != null));
  f.appendChild(el('span', { class: 'lf-sp' }));
  b('删除', () => deleteSelection(), '删除 (Delete)', !n);
}
function inlineRename(host, value, commit) {
  const name = host.querySelector('.lname');
  const inp = el('input', { type: 'text', class: 'rename', value });
  name.replaceWith(inp); inp.focus(); inp.select();
  let done = false;
  const finish = ok => { if (done) return; done = true; const v = inp.value.trim(); if (ok && v && v !== value) commit(v); refreshLeft(); };
  inp.addEventListener('keydown', e => { if (e.key === 'Enter') finish(true); else if (e.key === 'Escape') finish(false); e.stopPropagation(); });
  inp.addEventListener('blur', () => finish(true));
  inp.addEventListener('click', e => e.stopPropagation());
}
/* 拖动排序：放在某行上方/下方 = 调整前后；放在编组行中部 = 移入该组；放在顶层对象旁 = 移出编组 */
function makeDraggable(r, ref) {
  r.draggable = true;
  r.addEventListener('dragstart', e => {
    let ids;
    if (ref.group != null) ids = groupMembers(ref.group).map(o => o.id);
    else ids = selSet.has(ref.id) ? selection().filter(o => !isChildLink(o)).map(o => o.id) : [ref.id];
    layerDrag = { ids, group: ref.group };
    e.dataTransfer.effectAllowed = 'move';
    try { e.dataTransfer.setData('text/plain', 'layer'); } catch { }
  });
  r.addEventListener('dragend', () => { layerDrag = null; document.querySelectorAll('.lrow').forEach(x => x.classList.remove('drop-above', 'drop-below', 'drop-into')); });
  r.addEventListener('dragover', e => {
    if (!layerDrag) return;
    e.preventDefault();
    const rc = r.getBoundingClientRect(), y = (e.clientY - rc.top) / rc.height;
    const isGroup = ref.group != null && layerDrag.group == null;
    const pos = isGroup && y > 0.3 && y < 0.7 ? 'into' : y < 0.5 ? 'above' : 'below';
    r.classList.toggle('drop-above', pos === 'above'); r.classList.toggle('drop-below', pos === 'below'); r.classList.toggle('drop-into', pos === 'into');
    r.dataset.drop = pos;
  });
  r.addEventListener('dragleave', () => r.classList.remove('drop-above', 'drop-below', 'drop-into'));
  r.addEventListener('drop', e => {
    e.preventDefault();
    if (!layerDrag) return;
    const pos = r.dataset.drop || 'above';
    moveLayers(layerDrag.ids, ref, pos, layerDrag.group);
    layerDrag = null;
  });
}
function moveLayers(ids, ref, pos, draggedGroup) {
  const idset = new Set(ids);
  if (ref.id != null && idset.has(ref.id)) return;
  if (ref.group != null && draggedGroup === ref.group) return;
  pushUndo('调整图层顺序');
  const moving = scene.objects.filter(o => idset.has(o.id));
  const rest = scene.objects.filter(o => !idset.has(o.id));
  let anchorIdx, targetGroup = undefined;
  if (ref.group != null) {
    const mem = rest.filter(o => o.group === ref.group).map(o => rest.indexOf(o));
    if (pos === 'into') { anchorIdx = Math.max(...mem) + 1; targetGroup = ref.group; }
    else { anchorIdx = pos === 'above' ? Math.max(...mem) + 1 : Math.min(...mem); targetGroup = null; }
  } else {
    const t = byId(ref.id), k = rest.indexOf(t);
    anchorIdx = pos === 'above' ? k + 1 : k;
    targetGroup = t.group != null ? t.group : null;
  }
  if (draggedGroup == null) for (const o of moving) { if (targetGroup == null) delete o.group; else o.group = targetGroup; }
  rest.splice(anchorIdx, 0, ...moving);
  scene.objects = rest;
  pruneGroups();
  afterStructure(); buildPanel();
}

/* =====================================================================
 *  「显示」面板：显示项列表 · 添加 · 每项的设置
 * ===================================================================== */
const ovOpen = new Set();
function onOverlaysChanged(structural) {
  if (overlays().some(o => o.type === 'fieldLines')) markFieldsDirty();
  markDirty(); requestDraw();
  if (structural) { if (leftTab === 'display') refreshLeft(); buildPanel(); }
  updateDisplayCount(); syncViewQuick();
}
function addOverlay(type, target, extra) {
  const T = OVERLAY_TYPES[type]; if (!T) return null;
  pushUndo(`添加显示：${T.name}`);
  const ov = Object.assign(overlayDefaults(type), { id: nextOverlayId() }, extra || {});
  if (!T.noTarget && target != null) ov.target = target;
  overlays().push(ov);
  ovOpen.add(ov.id);
  onOverlaysChanged(true);
  return ov;
}
function removeOverlay(ov) {
  pushUndo(`移除显示：${OVERLAY_TYPES[ov.type].name}`);
  scene.overlays = overlays().filter(o => o !== ov);
  onOverlaysChanged(true);
}
function applyRecommended() {
  const rec = scene.info && scene.info.recommend;
  if (!rec || !rec.length) return;
  pushUndo('启用推荐显示');
  for (const r of recommendedOverlays(rec)) {
    if (overlays().some(o => o.type === r.type && o.target === r.target)) continue;
    overlays().push(Object.assign(r, { id: nextOverlayId() }));
  }
  onOverlaysChanged(true);
  toast('已启用本模板的推荐显示，可在「显示」面板里逐项调整', { action: UNDO_ACTION });
}
function updateDisplayCount() {
  const n = overlays().filter(o => o.on).length;
  const t = document.querySelector('#leftTabs [data-left-tab="display"] .lt-n'); if (t) t.textContent = n ? String(n) : '';
  const v = document.querySelector('#leftTabs [data-left-tab="vars"] .lt-n'); if (v) v.textContent = (scene.vars || []).length ? String(scene.vars.length) : '';
}
function ovSummary(ov) {
  const T = OVERLAY_TYPES[ov.type], bits = [ovTargetText(ov)];
  if (T.cat === 'vec') bits.push(ov.scale > 0 ? `${fmt(ov.scale, 3)} px/${(ov.type === 'vec' ? ov.unit : T.unit) || '单位'}` : '自动比例');
  if (ov.type === 'predict') bits.push(`${fmt(ov.T, 3)} s`);
  if (ov.type === 'strobe') bits.push(`Δt = ${fmt(ov.dt, 3)} s`);
  if (ov.type === 'value') bits.push(ov.expr);
  if (ov.type === 'vec') bits.push(`(${ov.ex}, ${ov.ey})`);
  return bits.join(' · ');
}
function ovSwatch(ov) {
  const T = OVERLAY_TYPES[ov.type], col = ov.color || T.color || '';
  const sw = el('span', { class: 'ov-sw ov-' + T.cat });
  if (col) sw.style.setProperty('--c', col);
  return sw;
}
/** 显示项的“对象”选择：全部 / 选中 / 某个物体（可在画布上点选） */
function ovTargetRow(parent, ov) {
  const opts = [['all', '全部物体'], ['sel', '选中的物体（跟随选择）']].concat(bodies().map(b => [String(b.id), b.name]));
  if (typeof ov.target === 'number' && !byId(ov.target)) opts.push([String(ov.target), '（已删除的物体）']);
  const r = el('div', { class: 'row' });
  r.appendChild(el('span', { class: 'lab' }, '对象'));
  const s = el('select', { class: 'force-select' });
  opts.forEach(([v, t]) => s.appendChild(el('option', { value: v }, escapeHtml(t))));
  s.value = String(ov.target);
  s.onchange = () => { pushUndo('修改显示对象'); ov.target = s.value === 'all' || s.value === 'sel' ? s.value : +s.value; onOverlaysChanged(true); };
  r.appendChild(s);
  const pk = el('button', { type: 'button', class: 'pick-btn', title: '在画布上点选物体' }, '⌖');
  pk.onclick = () => startPick({ prompt: `点选要显示「${OVERLAY_TYPES[ov.type].name}」的物体`, accept: o => isBody(o),
    onPick: o => { pushUndo('修改显示对象'); ov.target = o.id; onOverlaysChanged(true); toast(`「${OVERLAY_TYPES[ov.type].name}」现在只显示在「${o.name}」上`); } });
  r.appendChild(pk);
  parent.appendChild(r);
}
function ovNum(parent, lab, ov, key, unit, o = {}) {
  return numRow(parent, lab, () => ov[key], v => { ov[key] = v; }, unit, Object.assign({ after: () => { markDirty(); requestDraw(); updateOvSummary(ov); } }, o));
}
function ovText(parent, lab, ov, key, o = {}) {
  return textRow(parent, lab, () => ov[key], v => { ov[key] = v; requestDraw(); updateOvSummary(ov); }, Object.assign({ assist: { locals: DISPLAY_LOCALS } }, o));
}
function ovCheck(parent, lab, ov, key) {
  const r = el('label', { class: 'chk', style: 'margin:4px 12px 4px 0' }), i = el('input', { type: 'checkbox' });
  i.checked = !!ov[key];
  i.onchange = () => { pushUndo(); ov[key] = i.checked; markDirty(); requestDraw(); };
  r.appendChild(i); r.appendChild(document.createTextNode(lab)); parent.appendChild(r);
}
function ovColorRow(parent, ov) {
  const T = OVERLAY_TYPES[ov.type];
  const r = el('div', { class: 'row' });
  r.appendChild(el('span', { class: 'lab' }, '颜色'));
  const c = el('input', { type: 'color', value: ov.color || T.color || '#6b6a64' });
  let pushed = false;
  c.addEventListener('focus', () => { pushed = false; });
  c.oninput = () => { if (!pushed) { pushUndo('修改显示颜色'); pushed = true; } ov.color = c.value; markDirty(); requestDraw(); const sw = document.querySelector(`.ovrow[data-ov="${ov.id}"] .ov-sw`); if (sw) sw.style.setProperty('--c', c.value); };
  r.appendChild(c);
  const reset = el('button', { type: 'button', class: 'mini' }, T.color ? '默认' : '随物体');
  reset.onclick = () => { pushUndo(); ov.color = ''; onOverlaysChanged(true); };
  r.appendChild(reset);
  parent.appendChild(r);
}
function updateOvSummary(ov) {
  const s = document.querySelector(`.ovrow[data-ov="${ov.id}"] .ov-sub`); if (s) s.textContent = ovSummary(ov);
}
function ovOptions(host, ov) {
  const T = OVERLAY_TYPES[ov.type];
  const box = el('div', { class: 'ov-opts' });
  if (!T.noTarget) ovTargetRow(box, ov);
  if (T.cat === 'vec') {
    if (ov.type === 'vec') {
      ovText(box, '名称', ov, 'label', { placeholder: '如 u', assist: false });
      ovText(box, 'x 分量', ov, 'ex', { placeholder: '如 vx 或 m*ax' });
      ovText(box, 'y 分量', ov, 'ey', { placeholder: '如 vy 或 m*ay' });
      ovText(box, '单位', ov, 'unit', { placeholder: '如 m/s', assist: false });
    }
    const sr = el('div', { class: 'row' });
    sr.appendChild(el('span', { class: 'lab' }, '比例'));
    const auto = el('label', { class: 'chk', style: 'margin:0 8px 0 0' }), ai = el('input', { type: 'checkbox' });
    ai.checked = !(ov.scale > 0);
    auto.appendChild(ai); auto.appendChild(document.createTextNode('自动'));
    sr.appendChild(auto);
    const si = el('input', { type: 'number', min: '0', step: 'any', value: ov.scale > 0 ? fmtIn(ov.scale) : fmtIn(+(ov._auto || 0).toPrecision(3)) });
    si.disabled = ai.checked;
    const unit = (ov.type === 'vec' ? ov.unit : T.unit) || '单位';
    sr.appendChild(si); sr.appendChild(el('span', { class: 'unit' }, `px / ${unit}`));
    ai.onchange = () => { pushUndo('修改箭头比例'); ov.scale = ai.checked ? 0 : +(ov._auto || T.scale || 20).toPrecision(3); si.disabled = ai.checked; si.value = fmtIn(ov.scale || ov._auto || 0); markDirty(); requestDraw(); updateOvSummary(ov); };
    let pushed = false;
    si.addEventListener('focus', () => { pushed = false; });
    si.oninput = () => { const v = parseFloat(si.value); if (!(v > 0)) return; if (!pushed) { pushUndo('修改箭头比例'); pushed = true; } ov.scale = v; markDirty(); requestDraw(); updateOvSummary(ov); };
    box.appendChild(sr);
    const ck = el('div', { class: 'chkrow' });
    ovCheck(ck, '标出数值', ov, 'value');
    ovCheck(ck, '画出 x、y 分量', ov, 'comps');
    box.appendChild(ck);
    ovColorRow(box, ov);
  }
  if (ov.type === 'predict') {
    ovNum(box, '预测时长 T', ov, 'T', 's', { min: 0.05, max: 120, title: '从当前状态向前积分的时长；越长越耗时' });
    ovColorRow(box, ov);
  }
  if (ov.type === 'trail' || ov.type === 'analytic') ovColorRow(box, ov);
  if (ov.type === 'strobe') {
    ovNum(box, '间隔 Δt', ov, 'dt', 's', { min: 0.001, title: '课本频闪照片常用 0.1 s' });
    const sb = el('div', { class: 'btnrow' });
    [0.02, 0.05, 0.1, 0.2, 0.5].forEach(v => btn(sb, `${v} s`, () => { pushUndo(); ov.dt = v; onOverlaysChanged(true); }, Math.abs(ov.dt - v) < 1e-12 ? 'active' : ''));
    box.appendChild(sb);
    box.appendChild(el('div', { class: 'hint' }, '实心点画在已走轨迹上；若同一物体还开着预测轨迹，未来的频闪点画成空心。'));
  }
  if (ov.type === 'value') {
    ovText(box, '标签', ov, 'label', { placeholder: '如 Ek', assist: false });
    ovText(box, '公式', ov, 'expr', { placeholder: '如 0.5*m*v^2 或 Ek + Ep' });
    ovText(box, '单位', ov, 'unit', { placeholder: '如 J', assist: false });
    ovNum(box, '有效数字', ov, 'digits', '位', { min: 1, max: 8, step: 1 });
    const pr = el('div', { class: 'btnrow' });
    [['速率', 'v', 'v', 'm/s'], ['动能', 'Ek', 'Ek', 'J'], ['机械能', 'E', 'Ek + Ep', 'J'], ['加速度', 'a', 'a', 'm/s²'], ['高度', 'y', 'y', 'm'], ['合力', 'F合', 'F', 'N']]
      .forEach(([t, lab, ex, u]) => btn(pr, t, () => { pushUndo(); Object.assign(ov, { label: lab, expr: ex, unit: u }); onOverlaysChanged(true); }));
    box.appendChild(pr);
  }
  if (ov.type === 'vec' || ov.type === 'value') {
    box.appendChild(el('div', { class: 'hint' }, '可用本物体的量：t x y vx vy v ax ay a m q g omega ang Ek Ep px py p Fx Fy F；也可以写全局变量和 小球2.vx 这类引用。'));
    const err = el('div', { class: 'hint expr-err', 'data-ov-err': String(ov.id) }, ov._err || '');
    box.appendChild(err);
  }
  if (ov.type === 'fieldLines') box.appendChild(el('div', { class: 'hint' }, '电场线由电场区域与点电荷生成；开启「带电物体之间的库仑力」后，带电物体也会产生电场线。'));
  if (T.desc && T.cat !== 'field') box.appendChild(el('div', { class: 'hint' }, T.desc));
  host.appendChild(box);
}
function renderDisplay(host) {
  const wrap = el('div', { class: 'ovpanel' });
  const list = overlays();
  const rec = scene.info && scene.info.recommend && scene.info.recommend.length ? scene.info.recommend : null;
  if (!list.length) {
    wrap.appendChild(el('div', { class: 'empty-hint' }, '画布上默认不显示任何附加信息。<br>需要什么，就在下面逐项添加：速度、加速度、受力、轨迹、预测、频闪、数值标签……'));
  }
  if (rec && !rec.every(r => list.some(o => o.type === (r.type || r) ))) {
    const card = el('div', { class: 'ov-rec' });
    const names = recommendedOverlays(rec).map(r => OVERLAY_TYPES[r.type].name);
    card.appendChild(el('div', {}, `<b>本模板推荐</b>：${escapeHtml(names.join('、'))}`));
    btn(card, '一键启用', applyRecommended, 'primary');
    wrap.appendChild(card);
  }
  for (const ov of list) {
    const T = OVERLAY_TYPES[ov.type];
    const r = el('div', { class: 'ovrow' + (ov.on ? '' : ' off') + (ovOpen.has(ov.id) ? ' open' : ''), 'data-ov': String(ov.id) });
    const head = el('div', { class: 'ov-head' });
    const on = el('input', { type: 'checkbox', class: 'ov-on', title: ov.on ? '隐藏' : '显示', 'aria-label': `显示${T.name}` });
    on.checked = !!ov.on;
    on.onclick = e => e.stopPropagation();
    on.onchange = () => { pushUndo(ov.on ? `隐藏${T.name}` : `显示${T.name}`); ov.on = on.checked; r.classList.toggle('off', !ov.on); onOverlaysChanged(false); };
    head.appendChild(on);
    head.appendChild(ovSwatch(ov));
    const txt = el('div', { class: 'ov-txt' });
    txt.appendChild(el('div', { class: 'ov-name' }, escapeHtml(ov.type === 'vec' ? `${T.name} ${ov.label || ''}` : ov.type === 'value' ? `${T.name} ${ov.label || ''}` : T.name)));
    txt.appendChild(el('div', { class: 'ov-sub' }, escapeHtml(ovSummary(ov))));
    head.appendChild(txt);
    const del = el('button', { type: 'button', class: 'v-del', title: '移除此显示项', 'aria-label': '移除' }, '×');
    del.onclick = e => { e.stopPropagation(); removeOverlay(ov); toast(`已移除「${T.name}」`, { action: UNDO_ACTION }); };
    head.appendChild(del);
    head.onclick = () => { ovOpen.has(ov.id) ? ovOpen.delete(ov.id) : ovOpen.add(ov.id); refreshLeft(); };
    r.appendChild(head);
    if (ovOpen.has(ov.id)) ovOptions(r, ov);
    wrap.appendChild(r);
  }
  const add = el('button', { type: 'button', class: 'v-add', id: 'bAddOverlay' }, '＋ 添加显示项');
  add.onclick = e => openOverlayMenu(add, sel => addOverlay(sel, 'all'));
  wrap.appendChild(add);
  host.appendChild(wrap);
  const cnt = $('#layerCount'); if (cnt) cnt.textContent = list.length ? `${list.filter(o => o.on).length} / ${list.length} 项` : '';
}
/** “添加显示项”菜单：按类别分组，每项带说明 */
function openOverlayMenu(anchor, onChoose) {
  const menu = ensureContextMenu();
  menu.innerHTML = '';
  menu.classList.add('ov-menu');
  for (const [cat, title] of OVERLAY_CATS) {
    menu.appendChild(el('div', { class: 'ctx-head' }, title));
    for (const [type, T] of Object.entries(OVERLAY_TYPES)) {
      if (T.cat !== cat) continue;
      const it = ctxItem(T.name, () => { menu.classList.remove('ov-menu'); onChoose(type); });
      it.title = T.desc || '';
      it.appendChild(el('span', { class: 'ctx-desc' }, escapeHtml(T.desc || '')));
      menu.appendChild(it);
    }
  }
  const r = anchor.getBoundingClientRect();
  placeContextMenu(menu, r.left, r.bottom + 4);
}
/**
 * 物体属性里的“显示”开关：直接勾选这个物体要显示什么。
 * 勾上 = 新建一条只作用于该物体的显示项；若已被「全部物体」类的显示项覆盖，则提示去面板里改。
 */
function ovQuickToggles(parent, b, types) {
  const row = el('div', { class: 'ov-quick' });
  for (const type of types) {
    const T = OVERLAY_TYPES[type];
    const own = overlays().filter(o => o.type === type && o.target === b.id);
    const shared = overlays().find(o => o.on && o.type === type && o.target !== b.id && ovApplies(o, b));
    const on = own.some(o => o.on) || !!shared;
    const c = el('button', { type: 'button', class: 'ov-chip' + (on ? ' on' : '') + (shared && !own.some(o => o.on) ? ' shared' : ''), title: shared ? `由「显示」面板中的“${ovTargetText(shared)}”项控制` : T.desc || '' }, escapeHtml(T.name));
    c.onclick = () => {
      if (shared && !own.some(o => o.on)) { ovOpen.add(shared.id); setLeftTab('display'); toast(`「${T.name}」由“${ovTargetText(shared)}”显示项控制，在「显示」面板里调整`); return; }
      if (own.length) {
        pushUndo(on ? `隐藏「${b.name}」的${T.name}` : `显示「${b.name}」的${T.name}`);
        if (on) scene.overlays = overlays().filter(o => !own.includes(o)); else own.forEach(o => { o.on = true; });
        onOverlaysChanged(true);
      } else addOverlay(type, b.id);
    };
    row.appendChild(c);
  }
  parent.appendChild(row);
}

/* ---------------- 视图弹出框 / 视图菜单里的快捷开关：作用于“全部物体”的显示项 ---------------- */
const VIEW_QUICK = ['names', 'trail', 'predict', 'strobe', 'v', 'a', 'forces', 'analytic', 'fieldLines'];
function globalOverlay(type) { return overlays().find(o => o.type === type && (o.target === 'all' || OVERLAY_TYPES[type].noTarget)); }
function globalOverlayOn(type) { const o = globalOverlay(type); return !!(o && o.on); }
function toggleGlobalOverlay(type, on) {
  const o = globalOverlay(type), want = on ?? !(o && o.on);
  if (o) { pushUndo(`${want ? '显示' : '隐藏'}${OVERLAY_TYPES[type].name}`); o.on = want; onOverlaysChanged(true); }
  else if (want) addOverlay(type, 'all');
  syncViewQuick();
}
function syncViewQuick() {
  const host = $('#viewQuick'); if (!host) return;
  host.innerHTML = '';
  for (const type of VIEW_QUICK) {
    const r = el('label', { class: 'chk', title: OVERLAY_TYPES[type].desc || '' }), i = el('input', { type: 'checkbox' });
    i.checked = globalOverlayOn(type);
    i.onchange = () => toggleGlobalOverlay(type, i.checked);
    r.appendChild(i); r.appendChild(document.createTextNode(OVERLAY_TYPES[type].name)); host.appendChild(r);
  }
}

/* ---------------- 左栏：图层 · 变量 · 显示 ---------------- */
const LEFT_TABS = { objects: '图层', vars: '变量', display: '显示', add: '添加' };
function setSheet(name) {
  const next = name || 'canvas';
  if ((next === 'lab' || next === 'analysis') && next !== sheet && (tool !== 'select' || pending || pickMode)) {
    const prevLock = navLock;
    navLock = true;
    cancelArm();
    navLock = prevLock;
  }
  if (next === 'objects' || next === 'add') {
    document.body.dataset.list = 'open';
    if (next === 'objects') {
      if (leftTab === 'add') setLeftMode('objects');
      refreshLeft();
    }
    sheet = 'canvas';
  } else sheet = next;
  document.body.dataset.sheet = sheet;
  const ov = $('#overlay');
  if (ov) ov.hidden = !(sheet === 'lab' || sheet === 'analysis');
  const title = $('#ovTitle');
  if (title) title.textContent = sheet === 'analysis' ? '分析' : '实验库';
  if (sheet === 'analysis') { analysisSig = ''; buildAnalysisPane(); }
  if (!navLock && (sheet === 'lab' || sheet === 'analysis')) rememberNav('sheet');
  syncDesmosKeypadForAnalysis(sheet === 'analysis');
  updateArmBar();
  updateContextBar();
  requestAnimationFrame(() => { resize(); requestDraw(); if (sheet === 'analysis') { updateEnergyReadout(true); drawChart(); } });
}
function setLeftTab(mode) {
  if (mode === 'add') { setSheet('add'); return; }
  setLeftMode(mode);
  if (sheet !== 'objects') setSheet('objects');
  else refreshLeft();
}
function flashLeftTab(mode) {
  const b = document.querySelector(`#leftTabs [data-left-tab="${mode}"]`);
  updateDisplayCount();
  if (!b) return;
  b.classList.remove('flash'); void b.offsetWidth; b.classList.add('flash');
}

function renderAddInto(host, q) {
  const grid = el('div', { class: 'add-grid' });
  for (const block of ADD_CATALOG) {
    const visibleItems = block.items.filter(it => matchQuery(it.name + it.desc + block.cat, q));
    if (!visibleItems.length) continue;
    grid.appendChild(el('div', { class: 'add-cat' }, block.cat));
    for (const it of visibleItems) {
      const key = Object.keys(TOOL_KEYS).find(k => TOOL_KEYS[k] === it.tool);
      const b = el('button', { type: 'button', title: it.desc + (key ? `（快捷键 ${key}）` : '') }, `<div><b>${it.name}</b><div class="hint" style="margin:0">${it.desc}</div></div>${key ? `<kbd>${key}</kbd>` : ''}`);
      b.dataset.tool = it.tool;
      b.onclick = () => { setTool(it.tool); setSheet('canvas'); toast(pointerCoarse || window.matchMedia('(pointer: coarse)').matches ? '点按画布放置。拖动只平移，不会创建' : '在画布上点击或拖拽以放置'); };
      if (tool === it.tool) b.classList.add('active');
      grid.appendChild(b);
    }
  }
  if (!grid.childNodes.length) grid.appendChild(el('div', { class: 'empty-hint' }, '无匹配的对象。'));
  host.appendChild(grid);
}
function refreshLeft() {
  updateDisplayCount();
  if (sheet === 'add') {
    const add = $('#addBody');
    if (add) { add.innerHTML = ''; renderAddInto(add, ($('#qAdd') && $('#qAdd').value) || leftQuery); }
    return;
  }
  const host = $('#leftBody');
  if (!host) return;
  host.innerHTML = '';
  const q = leftQuery;
  if (leftTab === 'vars') { renderVars(host); return; }
  if (leftTab === 'display') { renderDisplay(host); return; }
  renderLayers(host);
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
  $('#viewFlags')?.classList.remove('show');
  $('#panelScrim')?.classList.remove('show', 'view');
  // 窄屏上 Escape 收起盖住画布的功能页；宽屏底栏是主导航，不因此跳回画布。
  if (resetDock && sheet !== 'canvas' && window.matchMedia('(max-width: 799px)').matches) setSheet('canvas');
}
function openMobileView() { setLeftTab('display'); }

function setLeftMode(mode) {
  leftTab = mode;
  document.body.dataset.left = mode;
  const t = $('#leftTitle'); if (t) t.textContent = mode === 'objects' ? '对象' : (LEFT_TABS[mode] || '对象');
  const f = $('#layerFoot'); if (f) f.style.display = mode === 'objects' ? '' : 'none';
  const sw = document.querySelector('#left .search-wrap'); if (sw) sw.style.display = mode === 'objects' || mode === 'vars' ? '' : 'none';
  document.querySelectorAll('#leftTabs button').forEach(b => b.classList.toggle('active', b.dataset.leftTab === mode));
  const cnt = $('#layerCount'); if (cnt && mode !== 'objects') cnt.textContent = '';
}
function openMobile(which) {
  if (which === 'add') setSheet('add');
  else if (which === 'display') setLeftTab('display');
  else if (which === 'vars') setLeftTab('vars');
  else if (which === 'objects') setSheet('objects');
  else if (which === 'presets') showHome('templates');
  else if (which === 'inspect' || which === 'stage') setSheet('canvas');
}

/* ---------------- 实时读数 ---------------- */
function setTxt(id, t) { const e = document.getElementById(id); if (e) e.innerHTML = t; }
function updateReadouts() {
  updateStatus();
  if (document.getElementById('en_tot')) updateEnergyReadout(!sim.running);
  updateDerived(!sim.running);
  const b = selected();
  if (document.body.dataset.sheet === 'analysis') {
    if (isBody(b) && b.st && document.getElementById('r_theory')) renderMotionAnalysis(b, !sim.running);
    if (document.getElementById('chart') && (!sim.running || (performance.now() / 200 | 0) % 2 === 0)) drawChart();
  }
  if (!isBody(b) || !b.st) return;
  const st = b.st, S = scene.settings, v = Math.hypot(st.vx, st.vy), Ek = 0.5 * b.m * v * v, Ep = S.gOn ? b.m * S.g * st.y : 0;
  setTxt('r_pos', `(${fmt(st.x)}, ${fmt(st.y)}) m`); setTxt('r_vel', `(${fmt(st.vx)}, ${fmt(st.vy)}) m/s`);
  setTxt('r_v', `${fmt(v)} m/s`); setTxt('r_ang', v > 1e-9 ? `${fmt(Math.atan2(st.vy, st.vx) / DEG, 2)}°` : '—');
  setTxt('r_a', sim.t > 0 ? `${fmt(Math.hypot(st.ax, st.ay))} m/s²` : '—');
  setTxt('r_ek', `${fmt(Ek + rotationalKE(b))} J${canRotate(b) && st.omega ? `（含转动 ${fmt(rotationalKE(b))} J）` : ''}`);
  setTxt('r_w', canRotate(b) ? `${fmt(st.omega || 0)} rad/s · θ = ${fmt((st.ang || 0) / DEG, 2)}°` : '—（不转动）'); setTxt('r_p', `${fmt(b.m * v)} kg·m/s`); setTxt('r_ep', S.gOn ? `${fmt(Ep)} J（以 y=0 为零势面）` : '—'); setTxt('r_e', `${fmt(Ek + Ep)} J`);
  if (document.getElementById('r_force_mag')) {
    const af = appliedForceOn(b);
    const mode = b.Fmode || 'const';
    setTxt('r_force_mag', `${fmt(af.F, 3)} N`);
    setTxt('r_force_ang', `${fmt(af.ang, 1)}°`);
    setTxt('r_force_mode', mode === 'expr' ? '表达式' : mode === 'aim' ? '指向目标' : '恒定');
    setTxt('r_force_err', b._Ferr || '');
  }
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
  syncDesmosKeypadForAnalysis(document.body.dataset.sheet === 'analysis');
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
  const status = $('#status'); if (status) status.innerHTML = parts.join('');
  updateStatusMouse();
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
/** 模板场景：显示一律默认关闭；模板里写的显示偏好变成「推荐显示」，可在教学卡片里一键启用 */
function sceneOf(objects, settings = {}, view = null) {
  const recommend = overlaysFromLegacy(Object.assign({}, LEGACY_DISPLAY_DEFAULTS, settings));
  const S = Object.assign(defaultSettings(), settings);
  for (const k of LEGACY_DISPLAY_KEYS) delete S[k];
  return { version: 4, settings: S, objects, view, overlays: [], recommend };
}
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
  rolling: { cat: '力学', tags: ['滚动', '转动惯量', '斜面'], fresh: true },
  topple: { cat: '力学', tags: ['倾倒', '力矩', '斜面'], fresh: true },
  dominoes: { cat: '力学', tags: ['多米诺', '转动', '碰撞'], fresh: true },
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
        x0: +(top[0] - Math.sin(th) * h / 2 - Math.cos(th) * along).toFixed(9), y0: +(top[1] + Math.cos(th) * h / 2 - Math.sin(th) * along).toFixed(9) });
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
  rolling: { name: '滚动与滑动',
    desc: '三个物体从同一斜面同时释放：光滑斜面上的小球只滑不转；粗糙斜面上的实心球纯滚动，部分重力势能变成转动动能，所以更慢；滑块在 μ = 0.2 时边滑边减少加速度。',
    formulas: [['a = g sinθ', '光滑：只平动'], ['a = g sinθ / (1 + I/mr²) = 5/7 · g sinθ', '实心球纯滚动'], ['v = ωr', '纯滚动约束']],
    tips: ['选中滚动的小球，在「属性 → 派生量」里看转动动能。', '把它的 μ 调到 0.05，会变成“又滚又滑”。'],
    make() {
      nextId = 1;
      const th = 20 * DEG, L = 9;
      const mkInc = (y, smooth, name) => mkIncline(-4.5, y, { name, L, ang: 20, dir: -1, smooth, e: 0 });
      const top = (y) => [-4.5 - L, y + L * Math.tan(th)];
      const out = [];
      const rows = [[9, true, '光滑斜面', 'ball', '滑动的球', '#4a78b0', 0], [4.5, false, '粗糙斜面', 'ball', '滚动的球', '#d97757', 1], [0, false, '粗糙斜面 2', 'box', '滑块 μ=0.2', '#5e8c4a', 0.2]];
      for (const [y, smooth, iname, kind, bname, color, mu] of rows) {
        out.push(mkInc(y, smooth, iname));
        const [tx, ty] = top(y), r = 0.3, along = 0.6;
        const nx = Math.sin(th), ny = Math.cos(th);  // 斜面朝左上，法向
        const cx = tx + Math.cos(th) * along, cy = ty - Math.sin(th) * along;
        if (kind === 'ball') out.push(mkBody('ball', { name: bname, r, mu, mus: mu, e: 0, color, x0: +(cx + nx * r).toFixed(6), y0: +(cy + ny * r).toFixed(6), collide: false }));
        else out.push(mkBody('box', { name: bname, w: 0.6, h: 0.4, mu, mus: mu, e: 0, color, ang0: -20, x0: +(cx + Math.cos(th) * 0.3 + nx * 0.2).toFixed(6), y0: +(cy - Math.sin(th) * 0.3 + ny * 0.2).toFixed(6), collide: false }));
      }
      return sceneOf([mkGround(0), ...out], { showAnalytic: false, vPx: 8 }, { cx: -9, cy: 6.4, scale: 40 });
    } },
  topple: { name: '倾倒还是滑动',
    desc: '两个质量相同的木块放在 30° 粗糙斜面上（μₛ = 1.2，不会滑动）。重力作用线若落在底面之外就会倾倒：判据是 tanθ > w/h。高瘦的木块倾倒，矮胖的保持静止。',
    formulas: [['tanθ > w / h', '倾倒判据（不滑动时）'], ['tanθ > μₛ', '滑动判据'], ['I = m(w² + h²)/12', '矩形绕质心的转动惯量']],
    tips: ['把斜面倾角改为 20°（tan20° = 0.36 < 0.4），高瘦木块也不倒了。', '打开「显示 → 受力分析」，观察支持力的作用点向下坡角移动。'],
    make() {
      nextId = 1;
      const th = 30 * DEG, inc = mkIncline(-3, 0, { L: 6, ang: 30, dir: 1, smooth: false, e: 0 });
      const place = (s, w, h, name, color) => {
        const bx = -3 + s * Math.cos(th), by = s * Math.sin(th), nx = -Math.sin(th), ny = Math.cos(th);
        return mkBody('box', { name, w, h, m: 1, mu: 1.2, mus: 1.2, e: 0, color, ang0: 30, x0: +(bx + nx * h / 2).toFixed(6), y0: +(by + ny * h / 2).toFixed(6) });
      };
      return sceneOf([mkGround(0), inc, place(2.2, 0.4, 1.2, '高瘦 w/h=0.33', '#d97757'), place(4.8, 0.8, 0.5, '矮胖 w/h=1.6', '#4a78b0')],
        { showAnalytic: false }, { cx: 0.5, cy: 1.8, scale: 80 });
    } },
  dominoes: { name: '多米诺骨牌',
    desc: '一排竖立的骨牌，第一块以绕右下角转动的初速度推出（ω = −1.6 rad/s，v = ω×r），每一块倒下时撞击下一块，转动一路传下去。这是平动、转动与碰撞耦合的典型例子。',
    formulas: [['倾倒条件：重心越过支点', ''], ['ΔE = mg·Δh_重心', '每块骨牌释放的势能']],
    tips: ['调整骨牌间距（选中后改 x₀），间距约为高度的一半时传播最快。', '把最后一块换成很重的木块，看能否推倒它。'],
    make() {
      nextId = 1;
      const out = [mkGround(0)];
      for (let i = 0; i < 9; i++) out.push(mkBody('box', { name: `骨牌${i + 1}`, w: 0.16, h: 1, m: 0.3, mu: 0.5, mus: 0.6, e: 0.1, color: i === 0 ? '#d97757' : '#6b6a64', x0: +(i * 0.55).toFixed(2), y0: 0.5, ...(i === 0 ? { omega0: -1.6, vx0: 0.8, vy0: 0.128 } : {}) }));   // 第一块：绕右下角支点转动的相容初速度 v = ω×r
      return sceneOf(out, { showAnalytic: false, showVel: false, showLabels: false, maxDt: 0.0005 }, { cx: 2.4, cy: 0.9, scale: 115 });
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
  if (!docOpen) { docOpen = true; doc = { id: null, title: p.name, savedSeq: editSeq + 1 }; hideHome(); }
  pushUndo(`载入「${p.name}」`);
  const sc = p.make(); sc.objects.forEach(o => { if (isBody(o)) o.st = null; });
  sc.info = { key, recommend: sc.recommend || [] };
  propsTab = 'overview';
  loadScene(sc, false);
  if (!sc.view || W < 640) fitView();
  toast(`已载入「${p.name}」· 空格播放`, { action: UNDO_ACTION });
  syncDocTitle();
}

/* =====================================================================
 *  保存 / 载入
 * ===================================================================== */
function serialize() {
  return JSON.stringify({ app: 'physim', version: 4, settings: scene.settings, view: scene.view, info: scene.info || null, groups: scene.groups || [], vars: serializeVars(), overlays: serializeOverlays(),
    objects: scene.objects.map(o => {
      const c = Object.assign({}, o);
      delete c.st; delete c._exprErr; delete c._hold; delete c._Ferr; delete c._grab; delete c._N;
      return c;
    }) });
}
function normalizeObject(raw) {
  const o = Object.assign({}, raw, { st: null });
  delete o._grab; delete o._hold; delete o._exprErr;
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
  const fixes = sanitizeBySchema(o);
  if (fixes.length) normalizeObject.fixed.push(`${o.name || KIND_NAME[o.kind]}：${[...new Set(fixes)].join('、')}`);
  return o;
}
normalizeObject.fixed = [];
function loadScene(data, keepView) {
  if (!data || !Array.isArray(data.objects)) throw new Error('无效的场景文件');
  normalizeObject.fixed = [];
  const oldView = scene.view;
  scene = {
    settings: Object.assign(defaultSettings(), data.settings || {}),
    objects: data.objects.map(o => {
      const c = normalizeObject(o);
      if (isBody(c)) ensureForceFields(c);
      return c;
    }),
    view: keepView ? oldView : Object.assign({ cx: 0, cy: 2, scale: 60 }, data.view || {}),
    info: data.info && PRESETS[data.info.key] ? { key: data.info.key, recommend: Array.isArray(data.info.recommend) ? data.info.recommend : [] } : null,
    groups: Array.isArray(data.groups) ? data.groups.map(g => ({ id: +g.id, name: String(g.name || '编组'), ...(g.hidden ? { hidden: true } : {}), ...(g.locked ? { locked: true } : {}), ...(g.collapsed ? { collapsed: true } : {}) })) : [],
  };
  nextGroupId = scene.groups.reduce((m, g) => Math.max(m, g.id), 0) + 1;
  pruneGroups();
  loadVars(data.vars);
  loadOverlays(Array.isArray(data.overlays) ? data.overlays : overlaysFromLegacy(Object.assign({}, LEGACY_DISPLAY_DEFAULTS, data.settings || {})));
  for (const k of LEGACY_DISPLAY_KEYS) delete scene.settings[k];
  nextId = scene.objects.reduce((m, o) => Math.max(m, o.id || 0), 0) + 1;
  if (selId != null && !byId(selId)) selId = null;
  if (!keepView) { selId = null; pins = []; selSet = new Set(); }
  selSet = new Set([...selSet].filter(id => byId(id)));
  pins = pins.filter(p => scene.objects.some(o => o.id === p.bodyId));
  predCache.key = '';
  if (leftChip === 'spring') leftChip = 'link';
  resetSim(); markFieldsDirty(); syncToolbar(); buildList(); buildPanel(); draw(); markDirty();
  if (normalizeObject.fixed.length) setTimeout(() => toast(`已修正 ${normalizeObject.fixed.length} 处不合法的参数：${normalizeObject.fixed.slice(0, 2).join('；')}${normalizeObject.fixed.length > 2 ? '…' : ''}`), 50);
}
let saveT = 0;
function markDirty() { clearTimeout(saveT); saveT = setTimeout(() => { if (!docOpen) return; try { localStorage.setItem(STORE_KEY, serialize()); } catch (e) { } }, 400); }

/* =====================================================================
 *  工具栏绑定
 * ===================================================================== */
function syncToolbar() {
  const S = scene.settings;
  if ($('#chkG')) $('#chkG').checked = S.gOn;
  if ($('#numG')) $('#numG').value = S.g;
  if ($('#chkGrid')) $('#chkGrid').checked = S.showGrid;
  syncViewQuick();
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
  if ($('#selSpeed')) $('#selSpeed').onchange = e => { scene.settings.speed = +e.target.value; markDirty(); draw(); updateBadge(); };
  if ($('#chkG')) $('#chkG').onchange = e => { scene.settings.gOn = e.target.checked; liveEdit(); };
  if ($('#numG')) $('#numG').oninput = e => { const v = parseFloat(e.target.value); if (isFinite(v)) { scene.settings.g = v; liveEdit(); } };
  const flag = (id, k) => { const node = $(id); if (node) node.onchange = e => { scene.settings[k] = e.target.checked; markDirty(); draw(); }; };
  flag('#chkGrid', 'showGrid');
  document.querySelectorAll('#leftTabs [data-left-tab]').forEach(b => b.addEventListener('click', () => setLeftTab(b.dataset.leftTab)));

  const q = $('#qLeft');
  if (q) q.addEventListener('input', () => { leftQuery = q.value; refreshLeft(); });
  q?.addEventListener('keydown', e => { if (e.key === 'Escape') { q.value = ''; leftQuery = ''; refreshLeft(); q.blur(); } });
  const qAdd = $('#qAdd');
  if (qAdd) qAdd.addEventListener('input', () => { if (sheet === 'add') setSheet('add'); });

  $('#bUndo').onclick = undo;
  $('#bRedo') && ($('#bRedo').onclick = redo);
  syncHistoryButtons();
  $('#fileIn').onchange = e => { const f = e.target.files[0]; if (f) importFile(f); e.target.value = ''; };
  $('#docTitle')?.addEventListener('click', renameDocument);
  $('#zoomPct')?.addEventListener('click', () => fitView());
  $('#bHelpClose')?.addEventListener('click', () => $('#help').classList.remove('show'));
  $('#help')?.addEventListener('click', e => { if (e.target.id === 'help') e.target.classList.remove('show'); });

  const GEOM = {
    body: [['ball', '小球'], ['particle', '带电粒子'], ['box', '滑块']],
    surface: [['ground', '地面'], ['wall', '挡板'], ['incline', '斜面']],
    field: [['efield', '匀强电场'], ['bfield_in', '磁场向里'], ['bfield_out', '磁场向外'], ['pointCharge', '点电荷']],
    link: [['spring', '弹簧'], ['rope', '轻绳'], ['rod', '轻杆']],
  };
  window.__geom = GEOM;
  document.querySelectorAll('#geombar [data-geom]').forEach(b => {
    b.addEventListener('click', () => {
      const g = b.dataset.geom;
      $('#overflowMenu') && ($('#overflowMenu').hidden = true);
      if (g === 'select') { setTool('select'); const p = $('#toolPop'); if (p) p.hidden = true; syncGeom(); return; }
      const armed = (GEOM[g] || []).some(([id]) => id === tool);
      if (armed) { setTool('select'); const p = $('#toolPop'); if (p) p.hidden = true; syncGeom(); return; }
      openToolPop(g, b);
    });
  });
  $('#bAdd')?.addEventListener('click', () => openToolPop('body', $('#bAdd')));
  $('#bKey')?.addEventListener('click', () => {
    const on = document.body.classList.toggle('key-on');
    syncDesmosKeypadForAnalysis(on || sheet === 'analysis');
    if (on) setDesmosKeypadVisible(true); else setDesmosKeypadVisible(false);
  });
  $('#bCollapse')?.addEventListener('click', () => { document.body.dataset.list = 'closed'; resize(); });
  $('#bListExpand')?.addEventListener('click', () => { document.body.dataset.list = 'open'; resize(); });
  $('#bOverflow')?.addEventListener('click', () => { const m = $('#overflowMenu'); if (m) m.hidden = !m.hidden; });
  $('#bHome')?.addEventListener('click', () => { $('#overflowMenu') && ($('#overflowMenu').hidden = true); showHome('home'); });
  $('#bOpenAnalysis')?.addEventListener('click', () => { $('#overflowMenu') && ($('#overflowMenu').hidden = true); setSheet('analysis'); });
  $('#bGraphSettings')?.addEventListener('click', () => { $('#overflowMenu') && ($('#overflowMenu').hidden = true); showHome('home'); requestAnimationFrame(() => $('#envBody')?.scrollIntoView({ block: 'start' })); });
  $('#bHelpMenu')?.addEventListener('click', () => { $('#overflowMenu') && ($('#overflowMenu').hidden = true); $('#help')?.classList.add('show'); });
  $('#bOverlayClose')?.addEventListener('click', () => { $('#home')?.classList.remove('show'); setSheet('canvas'); });
  $('#ctxDelete')?.addEventListener('click', () => deleteSelection());
  $('#ctxHide')?.addEventListener('click', () => { const o = selected(); if (o) setHidden([o], !o.hidden); });
  $('#ctxMore')?.addEventListener('click', () => openMore());
  $('#bMoreClose')?.addEventListener('click', () => { delete document.body.dataset.more; });
  $('#ctxColor')?.addEventListener('click', (e) => { const o = selected(); if (o) openStyle(o, e.currentTarget); });
  $('#panelScrim')?.addEventListener('click', () => closeMobilePanels());
  $('#armCancel')?.addEventListener('click', () => requestCancel());
  document.addEventListener('pointerdown', (e) => {
    for (const id of ['overflowMenu', 'toolPop', 'stylePop']) {
      const n = document.getElementById(id);
      if (!n || n.hidden) continue;
      if (n.contains(e.target) || e.target.closest?.('#bOverflow, #geombar, #bAdd, #ctxColor, .lsw')) continue;
      n.hidden = true;
    }
  }, true);

  document.addEventListener('keydown', e => { if (e.key === ' ' && e.target === document.body) e.preventDefault(); });
  document.addEventListener('gesturestart', e => e.preventDefault());
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
  ctxMenuEl.classList.remove('ov-menu');
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

function openContextMenu(clientX, clientY, target, where = 'canvas') {
  const menu = ensureContextMenu();
  ctxTarget = target;
  menu.innerHTML = '';

  if (target) {
    menu.appendChild(ctxItem(`选中「${target.name}」`, () => select(target.id)));
    menu.appendChild(ctxItem('编辑属性', () => {
      select(target.id);
      setSheet('canvas');
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
        resetTrail(target); draw();
      }));
      menu.appendChild(ctxItem('转到分析页', () => {
        select(target.id);
        setSheet('analysis');
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
/* =====================================================================
 *  文档：本地文档库、新建 / 打开 / 保存 / 另存为 / 关闭、未保存状态
 * ===================================================================== */
const DOCS_KEY = 'physim.docs.v1', CUR_KEY = 'physim.current.v1';
const docKey = id => 'physim.doc.' + id;
let doc = { id: null, title: '未命名-1', savedSeq: 0 };
let docOpen = false, editSeq = 0, untitledN = 1;
function readIndex() { try { return JSON.parse(localStorage.getItem(DOCS_KEY)) || []; } catch { return []; } }
function writeIndex(list) { try { localStorage.setItem(DOCS_KEY, JSON.stringify(list)); return true; } catch { return false; } }
function isDirty() { return docOpen && editSeq !== doc.savedSeq; }
function bumpEdit() { editSeq++; syncDocTitle(); }
function syncDocTitle() {
  const t = $('#docTitle'), st = $('#docState');
  if (t && !t.dataset.editing) t.textContent = doc.title;
  if (st) {
    st.classList.toggle('dirty', isDirty());
    st.textContent = !docOpen ? '' : isDirty() ? '未保存' : doc.id ? '已保存' : '尚未保存';
  }
  document.title = docOpen ? `${doc.title}${isDirty() ? '（未保存）' : ''} — 物理实验室` : '物理实验室';
  syncCommands();
  try { localStorage.setItem(CUR_KEY, JSON.stringify({ open: docOpen, id: doc.id, title: doc.title, dirty: isDirty() })); } catch { }
}
function fmtTime(ts) {
  const d = new Date(ts), now = new Date();
  const p = n => String(n).padStart(2, '0');
  if (d.toDateString() === now.toDateString()) return `今天 ${p(d.getHours())}:${p(d.getMinutes())}`;
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}`;
}
/** 当前画布缩略图（JPEG，约 10–20 KB） */
function makeThumb(w = 320, h = 200) {
  try {
    const c = document.createElement('canvas'); c.width = w; c.height = h;
    const g = c.getContext('2d');
    const s = Math.max(w / cv.width, h / cv.height), dw = cv.width * s, dh = cv.height * s;
    g.fillStyle = C['--canvas'] || '#fbfaf6'; g.fillRect(0, 0, w, h);
    g.drawImage(cv, (w - dw) / 2, (h - dh) / 2, dw, dh);
    return c.toDataURL('image/jpeg', 0.78);
  } catch { return ''; }
}
function resetHistory() { undoStack.length = 0; redoStack.length = 0; syncHistoryButtons(); }
/** 打开一个场景作为当前文档 */
function openAsDocument(sceneData, meta) {
  doc = { id: meta.id ?? null, title: meta.title, savedSeq: 0 };
  editSeq = meta.dirty ? 1 : 0;
  docOpen = true;
  setSelection([], null, { deferPanel: true });
  propsTab = 'overview';
  loadScene(sceneData, false);
  if (!sceneData.view) fitView();
  resetHistory();
  hideHome();
  syncDocTitle();
}
function newUntitledTitle() { const names = new Set(readIndex().map(d => d.title)); let t; do { t = `未命名-${untitledN++}`; } while (names.has(t)); return t; }
function saveDocument(asNew = false, title) {
  if (!docOpen) return false;
  const list = readIndex();
  if (asNew || !doc.id) { doc.id = 'd' + Date.now().toString(36) + Math.random().toString(36).slice(2, 6); }
  if (title) doc.title = title;
  const entry = { id: doc.id, title: doc.title, modified: Date.now(), thumb: makeThumb(), objects: scene.objects.length };
  const old = list.find(d => d.id === doc.id);
  entry.created = old ? old.created : Date.now();
  try { localStorage.setItem(docKey(doc.id), serialize()); }
  catch { toast('保存失败：浏览器本地存储已满。可先导出为文件，或在开始页删除旧文档。'); return false; }
  const next = [entry, ...list.filter(d => d.id !== doc.id)];
  if (!writeIndex(next)) { entry.thumb = ''; writeIndex(next); }
  doc.savedSeq = editSeq;
  syncDocTitle();
  toast(`已保存「${doc.title}」到本地文档库`);
  return true;
}
function saveAs() {
  if (!docOpen) return;
  promptDialog({ title: '另存为', label: '文档名称', value: doc.title + ' 副本', ok: '保存' }, v => saveDocument(true, v));
}
function saveOrPrompt() {
  if (!docOpen) return;
  if (doc.id) saveDocument();
  else promptDialog({ title: '保存文档', label: '文档名称', value: doc.title, ok: '保存', hint: '保存到本浏览器的本地文档库。若要换电脑使用，请在「实验」里导出场景文件。' }, v => saveDocument(false, v));
}
function openDocById(id) {
  const meta = readIndex().find(d => d.id === id);
  let data = null;
  try { data = JSON.parse(localStorage.getItem(docKey(id))); } catch { }
  if (!meta || !data) { toast('找不到这个文档，可能已被删除'); return; }
  guardUnsaved(() => { openAsDocument(data, { id, title: meta.title }); toast(`已打开「${meta.title}」`); });
}
function deleteDocById(id) {
  const meta = readIndex().find(d => d.id === id); if (!meta) return;
  confirmDialog({ title: `删除「${meta.title}」？`, body: '将从本地文档库中永久删除，不能撤销。', ok: '删除', danger: true }, () => {
    writeIndex(readIndex().filter(d => d.id !== id));
    try { localStorage.removeItem(docKey(id)); } catch { }
    if (doc.id === id) { doc.id = null; syncDocTitle(); }
    renderHome(); toast(`已删除「${meta.title}」`);
  });
}
/** 有未保存修改时先询问：保存 / 不保存 / 取消 */
function guardUnsaved(next) {
  if (!isDirty()) { next(); return; }
  openModal(`
    <div class="dlg sm"><div class="dlg-head"><h3>保存对「${escapeHtml(doc.title)}」的修改？</h3><p>不保存的话，这些修改将会丢失。</p></div>
    <div class="dlg-foot"><button type="button" class="left" data-act="discard">不保存</button><button type="button" data-act="cancel">取消</button><button type="button" class="primary" data-act="save">保存</button></div></div>`,
  { discard: () => { closeModal(); next(); }, cancel: closeModal, save: () => { closeModal(); if (doc.id) { saveDocument(); next(); } else promptDialog({ title: '保存文档', label: '文档名称', value: doc.title, ok: '保存' }, v => { if (saveDocument(false, v)) next(); }); } });
}
function closeDocument() { guardUnsaved(() => { docOpen = false; setSelection([], null, { deferPanel: true }); sim.running = false; updatePlayBtn(); syncDocTitle(); showHome('home'); }); }
function renameDocument() {
  if (!docOpen) return;
  const t = $('#docTitle');
  const inp = el('input', { type: 'text', id: 'docTitleInput', value: doc.title });
  t.dataset.editing = '1'; t.style.display = 'none'; t.after(inp); inp.focus(); inp.select();
  let done = false;
  const fin = ok => {
    if (done) return; done = true;
    const v = inp.value.trim();
    inp.remove(); t.style.display = ''; delete t.dataset.editing;
    if (ok && v && v !== doc.title) {
      doc.title = v;
      if (doc.id) { const list = readIndex(); const e = list.find(d => d.id === doc.id); if (e) { e.title = v; writeIndex(list); } }
      toast(`已重命名为「${v}」`);
    }
    syncDocTitle();
  };
  inp.addEventListener('keydown', e => { if (e.key === 'Enter') fin(true); if (e.key === 'Escape') fin(false); e.stopPropagation(); });
  inp.addEventListener('blur', () => fin(true));
}

/* ---------------- 文档预设（新建） ---------------- */
const DOC_PRESETS = [
  { id: 'ground', name: '地面与重力', sub: 'g = 9.8 m/s² · 地面 y = 0', gOn: true, g: 9.8, ground: true, groundY: 0, smooth: false, art: 'ground' },
  { id: 'blank', name: '空白画布', sub: '无重力 · 无地面', gOn: false, g: 9.8, ground: false, art: 'blank' },
  { id: 'table', name: '光滑水平面', sub: '光滑地面 · 研究碰撞与振动', gOn: true, g: 9.8, ground: true, groundY: 0, smooth: true, art: 'table' },
  { id: 'space', name: '太空 · 万有引力', sub: '无重力 · 物体相互吸引 G = 1', gOn: false, gravity: true, Gc: 1, ground: false, art: 'space' },
  { id: 'em', name: '电磁场', sub: '无重力 · 全空间匀强磁场 ⊗', gOn: false, ground: false, bAll: true, art: 'em' },
  { id: 'incline', name: '斜面实验台', sub: '地面 + 37° 粗糙斜面', gOn: true, g: 9.8, ground: true, groundY: 0, incline: true, art: 'incline' },
];
const ART = {
  ground: '<svg viewBox="0 0 160 84"><path d="M10 64h140" stroke="currentColor" stroke-width="2"/><path d="M14 68l-6 6M26 68l-6 6M38 68l-6 6M50 68l-6 6M62 68l-6 6M74 68l-6 6M86 68l-6 6M98 68l-6 6M110 68l-6 6M122 68l-6 6M134 68l-6 6M146 68l-6 6" stroke="currentColor" opacity=".45"/><circle cx="58" cy="52" r="8" fill="#d97757"/><path d="M58 40c18-26 46-26 64 10" stroke="#d97757" stroke-dasharray="3 4" fill="none"/></svg>',
  blank: '<svg viewBox="0 0 160 84"><path d="M80 18v48M56 42h48" stroke="currentColor" stroke-width="1.4" opacity=".5"/></svg>',
  table: '<svg viewBox="0 0 160 84"><path d="M10 60h140" stroke="currentColor" stroke-width="2"/><circle cx="50" cy="51" r="9" fill="#4a78b0"/><circle cx="104" cy="51" r="9" fill="#d97757"/><path d="M64 51h18" stroke="#4a8a4f" stroke-width="2"/></svg>',
  space: '<svg viewBox="0 0 160 84"><circle cx="80" cy="42" r="9" fill="#c2973e"/><ellipse cx="80" cy="42" rx="52" ry="22" stroke="currentColor" fill="none" stroke-dasharray="3 4" opacity=".6"/><circle cx="132" cy="42" r="5" fill="#4a78b0"/></svg>',
  em: '<svg viewBox="0 0 160 84"><g stroke="#4f7faf" stroke-width="1.4"><path d="M30 20l8 8M38 20l-8 8M76 20l8 8M84 20l-8 8M122 20l8 8M130 20l-8 8M30 56l8 8M38 56l-8 8M76 56l8 8M84 56l-8 8M122 56l8 8M130 56l-8 8"/></g><circle cx="80" cy="42" r="22" stroke="#d97757" fill="none" stroke-dasharray="3 4"/></svg>',
  incline: '<svg viewBox="0 0 160 84"><path d="M10 70h140" stroke="currentColor" stroke-width="2"/><path d="M40 70h80V28z" fill="currentColor" opacity=".12" stroke="currentColor" stroke-width="1.4"/><rect x="88" y="34" width="16" height="11" fill="#d08a3c" transform="rotate(-27.7 96 40)"/></svg>',
};
function sceneFromPreset(p, over = {}) {
  const q = Object.assign({}, p, over);
  nextId = 1;
  const objs = [];
  if (q.ground) objs.push(mkGround(q.groundY || 0, { smooth: !!q.smooth }));
  if (q.incline) objs.push(mkIncline(-2, q.groundY || 0, { L: 4, ang: 37 }));
  if (q.bAll) objs.push(mkB({ name: '磁场 ⊗', B: 1, dir: -1, all: true }));
  const settings = { gOn: !!q.gOn, g: q.g ?? 9.8, drag: q.drag || 0, gravity: !!q.gravity, Gc: q.Gc ?? 1, maxDt: q.maxDt ?? 0.001, snapStep: q.snapStep ?? 0.1 };
  const view = q.ground ? { cx: 2, cy: 3, scale: 60 } : { cx: 0, cy: 0, scale: 60 };
  return { objects: objs, settings, view, groups: [] };
}
function createFromPreset(p, over = {}, title) {
  guardUnsaved(() => {
    openAsDocument(sceneFromPreset(p, over), { title: title || newUntitledTitle() });
    toast(`已新建「${doc.title}」· 用「添加」放置物体`);
  });
}
function createFromTemplate(key) {
  const P = PRESETS[key]; if (!P) return;
  guardUnsaved(() => {
    const sc = P.make(); sc.objects.forEach(o => { if (isBody(o)) o.st = null; }); sc.info = { key };
    openAsDocument(sc, { title: P.name });
    toast(`已从模板新建「${P.name}」· 空格播放`);
  });
}

/* ---------------- 新建文档对话框 ---------------- */
function openNewDialog(presetId = 'ground') {
  let cur = DOC_PRESETS.find(p => p.id === presetId) || DOC_PRESETS[0];
  const st = { title: newUntitledTitle(), gOn: cur.gOn, g: cur.g ?? 9.8, ground: cur.ground, groundY: cur.groundY ?? 0, smooth: !!cur.smooth, drag: 0, gravity: !!cur.gravity, Gc: cur.Gc ?? 1, maxDt: 0.001, snapStep: 0.1 };
  const html = `
  <div class="dlg"><div class="dlg-head"><h3>新建文档</h3><p>选择一个起点，再按需调整。所有设置之后都能在「属性 → 物理」里修改。</p></div>
  <div class="dlg-body"><div class="nd-grid">
    <div class="nd-list">${DOC_PRESETS.map(p => `<button type="button" data-p="${p.id}" class="${p.id === cur.id ? 'active' : ''}"><b>${p.name}</b><span>${p.sub}</span></button>`).join('')}</div>
    <div class="nd-form">
      <h4>文档</h4>
      <div class="row"><span class="lab">名称</span><input type="text" data-k="title"></div>
      <h4>环境</h4>
      <label class="chk"><input type="checkbox" data-k="gOn">均匀重力场</label>
      <div class="row"><span class="lab">重力加速度 g</span><input type="number" step="0.1" data-k="g"><span class="unit">m/s²</span></div>
      <label class="chk"><input type="checkbox" data-k="ground">地面</label>
      <div class="row"><span class="lab">地面高度</span><input type="number" step="0.1" data-k="groundY"><span class="unit">m</span></div>
      <label class="chk"><input type="checkbox" data-k="smooth">地面光滑</label>
      <div class="row"><span class="lab">线性阻力 b</span><input type="number" step="0.05" min="0" data-k="drag"><span class="unit">N·s/m</span></div>
      <label class="chk"><input type="checkbox" data-k="gravity">物体间万有引力</label>
      <div class="row"><span class="lab">引力常量 G</span><input type="number" step="0.1" data-k="Gc"><span class="unit">N·m²/kg²</span></div>
      <h4>精度</h4>
      <div class="row"><span class="lab">积分步长 Δt</span><select data-k="maxDt"><option value="0.002">2 ms（快速）</option><option value="0.001">1 ms（标准）</option><option value="0.0002">0.2 ms（精细）</option></select></div>
      <div class="row"><span class="lab">吸附步长</span><input type="number" step="0.05" min="0.001" data-k="snapStep"><span class="unit">m</span></div>
      <div class="field-err" data-err></div>
    </div>
  </div></div>
  <div class="dlg-foot"><button type="button" data-act="cancel">取消</button><button type="button" class="primary" data-act="create">创建</button></div></div>`;
  const root = openModal(html, {
    cancel: closeModal,
    create: () => {
      read();
      if (!(st.g >= 0) || !(st.snapStep > 0) || !(st.drag >= 0) || !st.title.trim()) { root.querySelector('[data-err]').textContent = '请检查：名称不能为空，g、阻力不能为负，吸附步长必须大于 0。'; return; }
      closeModal();
      createFromPreset(cur, { gOn: st.gOn, g: st.g, ground: st.ground, groundY: st.groundY, smooth: st.smooth, drag: st.drag, gravity: st.gravity, Gc: st.Gc, maxDt: +st.maxDt, snapStep: st.snapStep, incline: cur.incline && st.ground, bAll: cur.bAll }, st.title.trim());
    },
  });
  const fields = [...root.querySelectorAll('[data-k]')];
  const write = () => fields.forEach(f => { const v = st[f.dataset.k]; if (f.type === 'checkbox') f.checked = !!v; else f.value = v; });
  const read = () => fields.forEach(f => { const k = f.dataset.k; st[k] = f.type === 'checkbox' ? f.checked : f.type === 'number' ? parseFloat(f.value) : f.value; });
  const sync = () => {
    const on = k => root.querySelector(`[data-k="${k}"]`);
    on('g').disabled = !st.gOn; on('groundY').disabled = !st.ground; on('smooth').disabled = !st.ground; on('Gc').disabled = !st.gravity;
  };
  write(); sync();
  fields.forEach(f => f.addEventListener('input', () => { read(); sync(); }));
  root.querySelectorAll('[data-p]').forEach(b => b.addEventListener('click', () => {
    read();
    cur = DOC_PRESETS.find(p => p.id === b.dataset.p);
    Object.assign(st, { gOn: cur.gOn, g: cur.g ?? 9.8, ground: cur.ground, groundY: cur.groundY ?? 0, smooth: !!cur.smooth, gravity: !!cur.gravity, Gc: cur.Gc ?? 1 });
    root.querySelectorAll('[data-p]').forEach(x => x.classList.toggle('active', x === b));
    write(); sync();
  }));
  root.querySelector('[data-k="title"]').select();
}

/* ---------------- 通用对话框 ---------------- */
let modalKeyHandler = null;
function openModal(html, actions = {}) {
  const m = $('#modal');
  m.innerHTML = html; m.classList.add('show');
  m.querySelectorAll('[data-act]').forEach(b => b.addEventListener('click', () => actions[b.dataset.act] && actions[b.dataset.act]()));
  m.onclick = e => { if (e.target === m) closeModal(); };
  modalKeyHandler = e => {
    if (e.key === 'Escape') { e.preventDefault(); e.stopPropagation(); closeModal(); }
    else if (e.key === 'Enter' && e.target.tagName !== 'TEXTAREA' && e.target.tagName !== 'BUTTON') { const p = m.querySelector('button.primary'); if (p) { e.preventDefault(); e.stopPropagation(); p.click(); } }
  };
  document.addEventListener('keydown', modalKeyHandler, true);
  setTimeout(() => { const f = m.querySelector('input, select, button.primary'); if (f) f.focus(); }, 0);
  return m;
}
function closeModal() { const m = $('#modal'); m.classList.remove('show'); m.innerHTML = ''; if (modalKeyHandler) document.removeEventListener('keydown', modalKeyHandler, true); modalKeyHandler = null; }
function modalOpen() { return $('#modal').classList.contains('show'); }
function promptDialog(o, done) {
  const root = openModal(`<div class="dlg sm"><div class="dlg-head"><h3>${escapeHtml(o.title)}</h3>${o.hint ? `<p>${escapeHtml(o.hint)}</p>` : ''}</div>
    <div class="dlg-body nd-form"><div class="row"><span class="lab">${escapeHtml(o.label)}</span><input type="${o.type || 'text'}" data-v ${o.min != null ? `min="${o.min}"` : ''} ${o.step ? `step="${o.step}"` : ''}>${o.unit ? `<span class="unit">${o.unit}</span>` : ''}</div><div class="field-err" data-err></div></div>
    <div class="dlg-foot"><button type="button" data-act="cancel">取消</button><button type="button" class="primary" data-act="ok">${escapeHtml(o.ok || '确定')}</button></div></div>`,
  { cancel: closeModal, ok: () => {
    const raw = root.querySelector('[data-v]').value.trim();
    const v = o.type === 'number' ? parseFloat(raw) : raw;
    const err = !raw ? '不能为空' : o.validate ? o.validate(v) : '';
    if (err) { root.querySelector('[data-err]').textContent = err; return; }
    closeModal(); done(v);
  } });
  const inp = root.querySelector('[data-v]'); inp.value = o.value ?? ''; setTimeout(() => { inp.focus(); inp.select(); }, 0);
}
function confirmDialog(o, done) {
  openModal(`<div class="dlg sm"><div class="dlg-head"><h3>${escapeHtml(o.title)}</h3><p>${escapeHtml(o.body || '')}</p></div>
    <div class="dlg-foot"><button type="button" data-act="cancel">取消</button><button type="button" class="primary${o.danger ? ' danger-fill' : ''}" data-act="ok">${escapeHtml(o.ok || '确定')}</button></div></div>`,
  { cancel: closeModal, ok: () => { closeModal(); done(); } });
}

/* ---------------- 开始页 ---------------- */
let homeView = 'home', homeCat = 'all';
function showHome(view) {
  if (view) homeView = view;
  renderHome();
  $('#home')?.classList.add('show');
  if (sheet !== 'lab') setSheet('lab');
  else updateOptContext();
}
function hideHome() {
  $('#home')?.classList.remove('show');
  if (docOpen && sheet !== 'canvas') setSheet('canvas');
  updateOptContext();
}
function homeShown() { return !!$('#home')?.classList.contains('show'); }
function renderHome() {
  const h = $('#home'); if (!h) return;
  const docs = readIndex();
  const nav = `
    <div class="home-nav">
      <button type="button" class="primary" data-h="new">新建文档…</button>
      <button type="button" data-h="openFile">从文件打开…</button>
      <div class="hn-sep"></div>
      <button type="button" class="hn-link ${homeView === 'home' ? 'active' : ''}" data-v="home">主页</button>
      <button type="button" class="hn-link ${homeView === 'templates' ? 'active' : ''}" data-v="templates">模板（${Object.keys(PRESETS).length}）</button>
      <button type="button" class="hn-link ${homeView === 'docs' ? 'active' : ''}" data-v="docs">我的文档（${docs.length}）</button>
      ${docOpen ? `<div class="hn-sep"></div><button type="button" data-h="back">返回「${escapeHtml(doc.title)}」</button>` : ''}
    </div>`;
  const presetCards = DOC_PRESETS.map(p => `<button type="button" class="pcard" data-preset="${p.id}"><div class="pc-art">${ART[p.art] || ''}</div><div class="pc-name">${p.name}</div><div class="pc-sub">${p.sub}</div></button>`).join('');
  const docCards = list => list.length ? `<div class="doc-grid">${list.map(d => `
      <div class="dcard"><button type="button" class="pcard" data-doc="${d.id}" style="width:100%"><div class="pc-art">${d.thumb ? `<img alt="" src="${d.thumb}">` : ''}</div>
      <div class="pc-name">${escapeHtml(d.title)}</div><div class="pc-sub">${fmtTime(d.modified)} · ${d.objects ?? '?'} 个对象</div></button>
      <button type="button" class="dc-del" data-del="${d.id}" title="删除" aria-label="删除">×</button></div>`).join('')}</div>`
    : '<div class="empty-box">还没有保存过的文档。新建后按 Ctrl+S，或点上方「保存」。</div>';
  const tplCards = cat => {
    const keys = Object.keys(PRESETS).filter(k => cat === 'all' || (PRESET_META[k] || {}).cat === cat);
    return `<div class="tpl-grid">${keys.map(k => { const P = PRESETS[k], M = PRESET_META[k] || {}; return `
      <button type="button" class="tcard" data-tpl="${k}"><div class="tc-top"><span class="tc-name">${escapeHtml(P.name)}</span><span class="tc-cat">${M.cat || ''}</span></div><div class="tc-desc">${escapeHtml(P.desc || '')}</div></button>`; }).join('')}</div>`;
  };
  let main;
  if (homeView === 'templates') {
    main = `<h1>模板</h1><p class="lead">每个模板都是一个可编辑的完整文档，附原理说明、关键公式和可以尝试的改动。</p>
      <div class="tpl-chips">${['all', ...PRESET_CATS].map(c => `<button type="button" class="chip ${homeCat === c ? 'active' : ''}" data-cat="${c}">${c === 'all' ? '全部' : c}</button>`).join('')}</div>${tplCards(homeCat)}`;
  } else if (homeView === 'docs') {
    main = `<h1>我的文档</h1><p class="lead">保存在本浏览器的本地文档库中。换设备时用上方「场景文件」导出。</p>${docCards(docs)}`;
  } else {
    main = `<h1>${docs.length ? '欢迎回来' : '欢迎使用物理实验室'}</h1>
      <p class="lead">新建一个空白文档自己搭建，或者从经典模板开始。</p>
      <h2>新建</h2><div class="preset-cards">${presetCards}</div>
      <h2>最近的文档 ${docs.length > 6 ? '<button type="button" class="ghost h2-act" data-v="docs">全部 →</button>' : ''}</h2>${docCards(docs.slice(0, 6))}
      <h2>从模板开始 <button type="button" class="ghost h2-act" data-v="templates">全部 ${Object.keys(PRESETS).length} 个 →</button></h2>${tplCards('all').replace('<div class="tpl-grid">', '<div class="tpl-grid tpl-preview">')}`;
  }
  h.innerHTML = `<div class="home-wrap">${nav}<div class="home-main">${main}</div></div>`;
  h.querySelectorAll('[data-v]').forEach(b => b.onclick = () => { homeView = b.dataset.v; renderHome(); h.scrollTop = 0; });
  h.querySelectorAll('[data-cat]').forEach(b => b.onclick = () => { homeCat = b.dataset.cat; renderHome(); });
  h.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => createFromPreset(DOC_PRESETS.find(p => p.id === b.dataset.preset)));
  h.querySelectorAll('[data-doc]').forEach(b => b.onclick = () => openDocById(b.dataset.doc));
  h.querySelectorAll('[data-del]').forEach(b => b.onclick = e => { e.stopPropagation(); deleteDocById(b.dataset.del); });
  h.querySelectorAll('[data-tpl]').forEach(b => b.onclick = () => createFromTemplate(b.dataset.tpl));
  const act = { new: () => openNewDialog(), openFile: () => $('#fileIn').click(), back: () => hideHome() };
  h.querySelectorAll('[data-h]').forEach(b => b.onclick = () => act[b.dataset.h]());
}
function openDocsDialog() {
  const docs = readIndex();
  const root = openModal(`<div class="dlg"><div class="dlg-head"><h3>打开</h3><p>本地文档库中的文档，或从电脑上的场景文件打开。</p></div>
    <div class="dlg-body">${docs.length ? `<div class="doc-grid">${docs.map(d => `<button type="button" class="pcard" data-doc="${d.id}"><div class="pc-art">${d.thumb ? `<img alt="" src="${d.thumb}">` : ''}</div><div class="pc-name">${escapeHtml(d.title)}</div><div class="pc-sub">${fmtTime(d.modified)}</div></button>`).join('')}</div>` : '<div class="empty-box">本地文档库是空的。</div>'}</div>
    <div class="dlg-foot"><button type="button" class="left" data-act="file">从文件打开…</button><button type="button" data-act="cancel">取消</button></div></div>`,
  { cancel: closeModal, file: () => { closeModal(); $('#fileIn').click(); } });
  root.querySelectorAll('[data-doc]').forEach(b => b.onclick = () => { closeModal(); openDocById(b.dataset.doc); });
}

/* =====================================================================
 *  导出：PNG（1× / 2×）、CSV 数据、WebM 视频、场景文件
 * ===================================================================== */
function safeName(s) { return String(s).replace(/[\\/:*?"<>|]+/g, '_').slice(0, 80) || '物理实验'; }
function downloadBlob(blob, name) {
  const a = el('a', { href: URL.createObjectURL(blob), download: name });
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 2000);
}
function exportPNG(mult = 1) {
  const oldDPR = DPR;
  try {
    DPR = (window.devicePixelRatio || 1) * mult;
    cv.width = Math.round(W * DPR); cv.height = Math.round(H * DPR);
    const keepHover = hoverObj, keepTrace = traceHit; hoverObj = null; traceHit = null;
    draw();
    cv.toBlob(b => { if (b) downloadBlob(b, `${safeName(doc.title)}${mult > 1 ? '@' + mult + 'x' : ''}.png`); }, 'image/png');
    hoverObj = keepHover; traceHit = keepTrace;
    toast(`已导出 PNG（${cv.width} × ${cv.height} 像素）`);
  } finally { DPR = oldDPR; resize(); }
}
function exportCSV() {
  const list = bodies().filter(b => b.st && b.st.hist.length > 1);
  if (!list.length || sim.t === 0) { toast('还没有运动数据：先播放一段再导出'); return; }
  const rows = ['物体,t (s),x (m),y (m),vx (m/s),vy (m/s),|v| (m/s),Ek (J),|a| (m/s²)'];
  for (const b of list) for (const h of b.st.hist) rows.push([`"${String(b.name).replace(/"/g, '""')}"`, ...h.slice(0, 8).map(v => +(+v).toPrecision(10))].join(','));
  const e = sim.eHist;
  if (e.length > 1) { rows.push('', 't (s),动能 Ek (J),势能 Ep (J),机械能 E (J),E − W (J)'); for (const r of e) rows.push(r.map(v => +(+v).toPrecision(10)).join(',')); }
  downloadBlob(new Blob(['﻿' + rows.join('\n')], { type: 'text/csv;charset=utf-8' }), `${safeName(doc.title)}_数据.csv`);
  toast(`已导出 ${list.length} 个物体的运动数据（CSV，可用 Excel 打开）`);
}
function exportJSON() {
  const data = JSON.parse(serialize()); data.title = doc.title;
  downloadBlob(new Blob([JSON.stringify(data, null, 1)], { type: 'application/json' }), `${safeName(doc.title)}.physim.json`);
  toast('已导出场景文件，可在「实验」里从文件打开');
}
let recording = null;
function exportVideo() {
  if (recording) { recording.stop(); return; }
  if (!window.MediaRecorder || !cv.captureStream) { toast('当前浏览器不支持录制视频（需要 MediaRecorder）'); return; }
  promptDialog({ title: '导出视频', label: '时长', type: 'number', value: '6', unit: 's（模拟时间）', min: 0.5, step: 0.5, ok: '开始录制',
    hint: `从 t = 0 开始，以当前速度倍率（${scene.settings.speed}×）实时录制画布，结束后自动下载 WebM 文件。`, validate: v => !(v > 0) ? '时长必须大于 0' : v > 120 ? '最长 120 s' : '' }, dur => {
    const mime = ['video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm'].find(m => MediaRecorder.isTypeSupported(m)) || '';
    const stream = cv.captureStream(60);
    const rec = new MediaRecorder(stream, mime ? { mimeType: mime, videoBitsPerSecond: 6e6 } : {});
    const chunks = [];
    rec.ondataavailable = e => { if (e.data && e.data.size) chunks.push(e.data); };
    rec.onstop = () => {
      stream.getTracks().forEach(t => t.stop());
      clearInterval(watch); recording = null; $('#stage').classList.remove('recording');
      sim.running = false; updatePlayBtn();
      downloadBlob(new Blob(chunks, { type: 'video/webm' }), `${safeName(doc.title)}.webm`);
      toast('已导出视频');
    };
    resetSim(); selSet.clear(); selId = null; buildPanel();
    rec.start(200); recording = rec; $('#stage').classList.add('recording');
    sim.running = true; lastTs = 0; updatePlayBtn();
    toast(`正在录制（${dur} s）… 再次选择「导出视频」可提前结束`);
    const watch = setInterval(() => { if (sim.t >= dur || !sim.running) rec.stop(); }, 50);
  });
}
function importFile(f) {
  const rd = new FileReader();
  rd.onload = () => {
    let data;
    try { data = JSON.parse(rd.result); if (!data || !Array.isArray(data.objects)) throw 0; }
    catch { toast('打开失败：不是有效的场景文件（.physim.json）'); return; }
    const title = data.title || f.name.replace(/\.physim\.json$|\.json$/i, '');
    guardUnsaved(() => { openAsDocument(data, { title, dirty: true }); toast(`已打开 ${f.name}（尚未保存到文档库）`); });
  };
  rd.readAsText(f);
}

/* =====================================================================
 *  命令：菜单、快捷键、工具条共用一份定义
 * ===================================================================== */
const IS_MAC = /Mac|iPhone|iPad/.test(navigator.platform || navigator.userAgent);
const CMD = {};
function cmd(id, label, run, o = {}) { CMD[id] = Object.assign({ id, label, run }, o); }
const needDoc = () => docOpen;
const hasSel = () => needDoc() && selSet.size > 0;
const S_ = () => scene.settings;
function toggleSetting(k, after) { return () => { S_()[k] = !S_()[k]; syncToolbar(); if (after) after(); markDirty(); requestDraw(); if (!selected()) buildPanel(); }; }
function setSpeed(v) { scene.settings.speed = v; syncToolbar(); markDirty(); requestDraw(); }
function setTheme(dark) {
  document.body.classList.toggle('dark', dark);
  try { localStorage.setItem('physim.theme', dark ? 'dark' : 'light'); } catch { }
  syncThemeMeta(); readColors(); markFieldsDirty(); draw(); drawChart(); updateEnergyReadout(true);
}
function togglePanel(side) {
  const main = $('#main'); main.classList.toggle(side === 'left' ? 'noleft' : 'noright');
  syncPanelToggleButtons(); requestAnimationFrame(() => { resize(); requestDraw(); });
}
// 文件
cmd('file.new', '新建…', () => openNewDialog(), { kbd: 'Ctrl+N' });
cmd('file.home', '开始页', () => showHome('home'));
cmd('file.templates', '从模板新建…', () => showHome('templates'));
cmd('file.open', '打开…', () => openDocsDialog(), { kbd: 'Ctrl+O' });
cmd('file.openFile', '从文件打开…', () => $('#fileIn').click());
cmd('file.save', '保存', saveOrPrompt, { kbd: 'Ctrl+S', enabled: () => docOpen });
cmd('file.saveAs', '另存为…', saveAs, { kbd: 'Ctrl+Shift+S', enabled: () => docOpen });
cmd('file.rename', '重命名文档…', renameDocument, { enabled: () => docOpen });
cmd('file.exportPNG', 'PNG 图片', () => exportPNG(1), { kbd: 'Ctrl+Shift+E', enabled: needDoc });
cmd('file.exportPNG2', 'PNG 图片（2 倍分辨率）', () => exportPNG(2), { enabled: needDoc });
cmd('file.exportVideo', '视频（WebM）…', exportVideo, { enabled: needDoc, label2: () => recording ? '停止录制' : null });
cmd('file.exportCSV', '运动数据（CSV）', exportCSV, { enabled: needDoc });
cmd('file.exportJSON', '场景文件（.physim.json）', exportJSON, { enabled: () => docOpen });
cmd('file.close', '关闭文档', closeDocument, { kbd: 'Ctrl+Alt+W', enabled: () => docOpen });
// 编辑
cmd('edit.undo', '撤销', undo, { kbd: 'Ctrl+Z', enabled: () => needDoc() && undoStack.length > 0, label2: () => undoStack.length && undoStack.at(-1).label ? `撤销 ${undoStack.at(-1).label}` : null });
cmd('edit.redo', '重做', redo, { kbd: 'Ctrl+Shift+Z', alt: ['Ctrl+Y'], enabled: () => needDoc() && redoStack.length > 0, label2: () => redoStack.length && redoStack.at(-1).label ? `重做 ${redoStack.at(-1).label}` : null });
cmd('edit.cut', '剪切', () => copySel(true), { kbd: 'Ctrl+X', enabled: hasSel });
cmd('edit.copy', '拷贝', () => copySel(), { kbd: 'Ctrl+C', enabled: hasSel });
cmd('edit.paste', '粘贴', pasteClip, { kbd: 'Ctrl+V', enabled: () => needDoc() && !!clipboard });
cmd('edit.duplicate', '复制一份', duplicateSel, { kbd: 'Ctrl+D', enabled: hasSel });
cmd('edit.delete', '删除', () => deleteSelection(), { kbd: 'Delete', alt: ['Backspace'], enabled: hasSel });
cmd('edit.selectAll', '全选', selectAll, { kbd: 'Ctrl+A', enabled: needDoc });
cmd('edit.deselect', '取消选择', () => setSelection([]), { kbd: 'Ctrl+Shift+A', enabled: hasSel });
cmd('edit.clear', '清空文档内容', () => {
  if (!scene.objects.length) { toast('文档已经是空的'); return; }
  pushUndo('清空文档'); loadScene({ objects: [], settings: scene.settings, view: scene.view, groups: [] }, true);
  toast('已清空文档内容', { action: UNDO_ACTION });
}, { enabled: needDoc });
// 对象
cmd('obj.group', '编组', groupSelection, { kbd: 'Ctrl+G', enabled: () => needDoc() && selSet.size > 1 });
cmd('obj.ungroup', '取消编组', ungroupSelection, { kbd: 'Ctrl+Shift+G', enabled: () => hasSel() && selection().some(o => o.group != null) });
cmd('obj.front', '置于顶层', () => arrangeSelection('front'), { kbd: 'Ctrl+Shift+]', enabled: hasSel });
cmd('obj.forward', '上移一层', () => arrangeSelection('forward'), { kbd: 'Ctrl+]', enabled: hasSel });
cmd('obj.backward', '下移一层', () => arrangeSelection('backward'), { kbd: 'Ctrl+[', enabled: hasSel });
cmd('obj.back', '置于底层', () => arrangeSelection('back'), { kbd: 'Ctrl+Shift+[', enabled: hasSel });
for (const [m, t] of [['left', '左对齐'], ['hcenter', '水平居中对齐'], ['right', '右对齐'], ['top', '顶对齐'], ['vcenter', '垂直居中对齐'], ['bottom', '底对齐']]) cmd('obj.align.' + m, t, () => alignSelection(m), { enabled: () => needDoc() && selSet.size > 1 });
cmd('obj.dist.h', '水平分布', () => alignSelection('disth'), { enabled: () => needDoc() && selSet.size > 2 });
cmd('obj.dist.v', '垂直分布', () => alignSelection('distv'), { enabled: () => needDoc() && selSet.size > 2 });
cmd('obj.lock', '锁定所选', () => setLocked(selection(), true), { kbd: 'Ctrl+2', enabled: hasSel });
cmd('obj.unlockAll', '全部解锁', () => setLocked(scene.objects.filter(o => o.locked), false), { kbd: 'Ctrl+Alt+2', enabled: () => needDoc() && scene.objects.some(o => o.locked) });
cmd('obj.hide', '隐藏所选', () => setHidden(selection(), true), { kbd: 'Ctrl+3', enabled: hasSel });
cmd('obj.showAll', '全部显示', () => setHidden(scene.objects.filter(o => o.hidden), false), { kbd: 'Ctrl+Alt+3', enabled: () => needDoc() && scene.objects.some(o => o.hidden) });
cmd('obj.fix', '固定 / 取消固定', () => {
  const bs = selection().filter(isBody); if (!bs.length) return;
  pushUndo('固定'); const v = !bs.every(b => b.fixed); bs.forEach(b => { b.fixed = v; }); initChanged(); buildPanel();
}, { enabled: () => hasSel() && selection().some(isBody) });
// 插入（工具）
const TOOL_DEFS = [
  ['select', '选择', 'V', '<path d="M5 3l11 6.5-5 1.2L8.6 16z" fill="currentColor"/>'],
  ['pan', '平移视图', 'H', '<path d="M7 10V5.5a1.2 1.2 0 0 1 2.4 0V9M9.4 9V4.3a1.2 1.2 0 0 1 2.4 0V9M11.8 9V5a1.2 1.2 0 0 1 2.4 0v6.2c0 3-2 5.3-5 5.3-2.4 0-3.6-1.4-4.8-3.4L3 10.4a1.2 1.2 0 0 1 2-1.2L7 11.5" stroke="currentColor" stroke-width="1.3" fill="none" stroke-linecap="round" stroke-linejoin="round"/>'],
  '-',
  ['ball', '小球', '1', '<circle cx="10" cy="10" r="5.5" fill="currentColor"/>'],
  ['particle', '带电粒子', '2', '<circle cx="10" cy="10" r="5.5" stroke="currentColor" stroke-width="1.5" fill="none"/><path d="M10 7.3v5.4M7.3 10h5.4" stroke="currentColor" stroke-width="1.5"/>'],
  ['box', '滑块', '3', '<rect x="4.5" y="6" width="11" height="8" rx="1" fill="currentColor"/>'],
  '-',
  ['ground', '地面', '4', '<path d="M3 11h14" stroke="currentColor" stroke-width="1.8"/><path d="M5 13l-2 2.5M8.5 13l-2 2.5M12 13l-2 2.5M15.5 13l-2 2.5" stroke="currentColor" stroke-width="1.1"/>'],
  ['wall', '挡板', '5', '<path d="M5 16L15 4" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>'],
  ['incline', '斜面', '6', '<path d="M3 15.5h14V5.5z" stroke="currentColor" stroke-width="1.5" fill="currentColor" fill-opacity=".18" stroke-linejoin="round"/>'],
  '-',
  ['spring', '弹簧', '7', '<path d="M3 10h2l1.2-3 2 6 2-6 2 6 2-6 1.2 3H17" stroke="currentColor" stroke-width="1.4" fill="none" stroke-linejoin="round"/>'],
  ['rope', '轻绳', '8', '<path d="M4 5c3 9 9 1 12 10" stroke="currentColor" stroke-width="1.5" fill="none" stroke-linecap="round"/><circle cx="16" cy="15" r="1.8" fill="currentColor"/>'],
  ['rod', '轻杆', '9', '<path d="M5 15L15 5" stroke="currentColor" stroke-width="2.2" stroke-linecap="round"/><circle cx="5" cy="15" r="2" fill="currentColor"/><circle cx="15" cy="5" r="2" fill="currentColor"/>'],
  '-',
  ['efield', '匀强电场', 'E', '<path d="M4 6h10M4 10h10M4 14h10" stroke="currentColor" stroke-width="1.3"/><path d="M13 4.5L15.5 6 13 7.5M13 8.5l2.5 1.5-2.5 1.5M13 12.5l2.5 1.5-2.5 1.5" fill="currentColor"/>'],
  ['bfield_in', '磁场（向里 ⊗）', 'B', '<circle cx="10" cy="10" r="6" stroke="currentColor" stroke-width="1.3" fill="none"/><path d="M7.5 7.5l5 5M12.5 7.5l-5 5" stroke="currentColor" stroke-width="1.4"/>'],
  ['bfield_out', '磁场（向外 ⊙）', 'Shift+B', '<circle cx="10" cy="10" r="6" stroke="currentColor" stroke-width="1.3" fill="none"/><circle cx="10" cy="10" r="1.8" fill="currentColor"/>'],
  ['pointCharge', '点电荷', 'Q', '<circle cx="10" cy="10" r="5.5" fill="currentColor"/><path d="M10 7.5v5M7.5 10h5" stroke="var(--panel)" stroke-width="1.6"/>'],
];
for (const t of TOOL_DEFS) if (t !== '-') cmd('tool.' + t[0], t[1], () => setTool(t[0]), { kbd: t[2], enabled: needDoc, checked: () => tool === t[0] });
// 视图
cmd('view.zoomIn', '放大', () => zoomAt(1.25), { kbd: 'Ctrl+=', alt: ['=', 'Shift+='], enabled: needDoc });
cmd('view.zoomOut', '缩小', () => zoomAt(0.8), { kbd: 'Ctrl+-', alt: ['-'], enabled: needDoc });
cmd('view.fit', '适配视图', () => fitView(), { kbd: 'Ctrl+0', alt: ['F'], enabled: needDoc });
cmd('view.fitSel', '缩放到所选对象', () => fitView(selection()), { enabled: hasSel });
cmd('view.origin', '回到原点', () => { scene.view.cx = 0; scene.view.cy = 0; markFieldsDirty(); requestDraw(); markDirty(); }, { enabled: needDoc });
cmd('view.grid', '网格', toggleSetting('showGrid'), { checked: () => !!S_().showGrid, enabled: needDoc });
for (const type of VIEW_QUICK) cmd('view.ov.' + type, OVERLAY_TYPES[type].name, () => toggleGlobalOverlay(type), { checked: () => globalOverlayOn(type), enabled: needDoc });
cmd('view.display', '显示面板…', () => setLeftTab('display'), { enabled: needDoc });
cmd('view.vars', '变量面板…', () => setLeftTab('vars'), { enabled: needDoc });
cmd('view.recommend', '启用模板推荐显示', applyRecommended, { enabled: () => needDoc() && !!(scene.info && scene.info.recommend && scene.info.recommend.length) });
cmd('view.clearDisplay', '关闭全部显示', () => { if (!overlays().some(o => o.on)) return; pushUndo('关闭全部显示'); overlays().forEach(o => { o.on = false; }); onOverlaysChanged(true); syncViewQuick(); toast('已关闭全部显示项（配置保留）', { action: UNDO_ACTION }); }, { enabled: () => needDoc() && overlays().some(o => o.on) });
cmd('view.snap', '吸附网格', toggleSetting('snap'), { checked: () => !!S_().snap, enabled: needDoc });
cmd('view.layers', '对象列表', () => setSheet('objects'), { checked: () => sheet === 'objects' });
cmd('view.props', '回到画布', () => setSheet('canvas'), { checked: () => sheet === 'canvas' });
cmd('view.dark', '深色模式', () => setTheme(!document.body.classList.contains('dark')), { checked: () => document.body.classList.contains('dark') });
// 模拟
cmd('sim.play', '播放 / 暂停', togglePlay, { kbd: 'Space', enabled: needDoc, label2: () => sim.running ? '暂停' : '播放' });
cmd('sim.step', '前进一帧', stepOnce, { kbd: '.', enabled: needDoc });
cmd('sim.back', '后退一帧', stepBack, { kbd: ',', enabled: () => needDoc() && sim.frameIdx > 0 });
cmd('sim.reset', '重置到 t = 0', () => { resetSim('已重置到 t = 0'); draw(); updateReadouts(); }, { kbd: 'R', enabled: needDoc });
for (const v of [0.1, 0.25, 0.5, 1, 2, 5, 10]) cmd('sim.speed.' + v, `${v}×`, () => setSpeed(v), { checked: () => S_().speed === v, enabled: needDoc });
cmd('sim.settings', '物理设置…', () => { showHome(homeView || 'home'); requestAnimationFrame(() => $('#envBody')?.scrollIntoView({ block: 'start' })); }, { enabled: needDoc });
cmd('sim.energy', '能量与动量…', () => setSheet('analysis'), { enabled: needDoc });
// 帮助
cmd('help.guide', '使用说明', () => $('#help').classList.add('show'), { kbd: 'Shift+/' });
cmd('help.about', '关于物理实验室', () => openModal(`<div class="dlg sm"><div class="dlg-head"><h3>物理实验室</h3><p>浏览器里的二维力学与电磁学模拟器。刚体与接触（含转动）、场与约束、辛积分器、轨迹预测与解析解对照。所有数据只保存在本机浏览器中。</p></div><div class="dlg-foot"><button type="button" class="primary" data-act="ok">好</button></div></div>`, { ok: closeModal }));

const MENUS = [
  ['文件', ['file.new', 'file.templates', '-', 'file.open', 'file.openFile', { label: '最近打开', sub: () => readIndex().slice(0, 8).map(d => ({ label: d.title, run: () => openDocById(d.id), hint: fmtTime(d.modified) })) }, '-', 'file.save', 'file.saveAs', 'file.rename', '-',
    { label: '导出', sub: ['file.exportPNG', 'file.exportPNG2', 'file.exportVideo', '-', 'file.exportCSV', 'file.exportJSON'] }, '-', 'file.home', 'file.close']],
  ['编辑', ['edit.undo', 'edit.redo', '-', 'edit.cut', 'edit.copy', 'edit.paste', 'edit.duplicate', 'edit.delete', '-', 'edit.selectAll', 'edit.deselect', '-', 'edit.clear']],
  ['对象', ['obj.group', 'obj.ungroup', '-', { label: '排列', sub: ['obj.front', 'obj.forward', 'obj.backward', 'obj.back'] },
    { label: '对齐', sub: ['obj.align.left', 'obj.align.hcenter', 'obj.align.right', '-', 'obj.align.top', 'obj.align.vcenter', 'obj.align.bottom', '-', 'obj.dist.h', 'obj.dist.v'] }, '-',
    'obj.lock', 'obj.unlockAll', 'obj.hide', 'obj.showAll', '-', 'obj.fix']],
  ['插入', [{ head: '物体' }, 'tool.ball', 'tool.particle', 'tool.box', { head: '表面' }, 'tool.ground', 'tool.wall', 'tool.incline', { head: '约束' }, 'tool.spring', 'tool.rope', 'tool.rod', { head: '场' }, 'tool.efield', 'tool.bfield_in', 'tool.bfield_out', 'tool.pointCharge']],
  ['视图', ['view.zoomIn', 'view.zoomOut', 'view.fit', 'view.fitSel', 'view.origin', '-', 'view.grid', '-', { head: '显示（全部物体）' }, ...VIEW_QUICK.map(t => 'view.ov.' + t), 'view.display', 'view.recommend', 'view.clearDisplay', '-', 'view.vars', '-', 'view.snap', '-', 'view.layers', 'view.props', 'view.dark']],
  ['模拟', ['sim.play', 'sim.step', 'sim.back', 'sim.reset', '-', { label: '速度', sub: ['sim.speed.0.1', 'sim.speed.0.25', 'sim.speed.0.5', 'sim.speed.1', 'sim.speed.2', 'sim.speed.5', 'sim.speed.10'] }, '-', 'sim.settings', 'sim.energy']],
  ['帮助', ['help.guide', 'help.about']],
];

/* ---------------- 菜单渲染 ---------------- */
function kbdLabel(k) {
  if (!k) return '';
  const map = { Ctrl: IS_MAC ? '⌘' : 'Ctrl', Shift: IS_MAC ? '⇧' : 'Shift', Alt: IS_MAC ? '⌥' : 'Alt', Space: '空格', Delete: 'Del', 'Shift+/': '?' };
  if (map[k]) return map[k];
  return k.split('+').map(p => map[p] || p).join(IS_MAC ? '' : '+');
}
let openMenuIdx = -1;
function closeMenus() {
  const layer = $('#menuLayer'); if (layer) layer.innerHTML = '';
  openMenuIdx = -1;
  document.querySelectorAll('#menus > button').forEach(b => b.classList.remove('open'));
}
function menuItemsEl(items, depth = 0) {
  const box = el('div', { class: 'dmenu', role: 'menu' });
  for (const it of (typeof items === 'function' ? items() : items)) {
    if (it === '-') { box.appendChild(el('div', { class: 'ms' })); continue; }
    if (it.head) { box.appendChild(el('div', { class: 'mh' }, it.head)); continue; }
    const c = typeof it === 'string' ? CMD[it] : null;
    const sub = c ? null : it.sub;   // 注意：字符串自带 String.prototype.sub，不能直接取 it.sub
    const label = c ? (c.label2 && c.label2()) || c.label : it.label;
    const enabled = c ? (!c.enabled || c.enabled()) : true;
    const subItems = sub ? (typeof sub === 'function' ? sub() : sub) : null;
    const b = el('button', { type: 'button', class: 'mi', role: 'menuitem' },
      `<span class="ck">${c && c.checked ? (c.checked() ? '✓' : '') : ''}</span><span>${escapeHtml(label)}</span><span class="mk">${c ? escapeHtml(kbdLabel(c.kbd)) : escapeHtml(it.hint || '')}</span><span class="ar">${sub ? '▸' : ''}</span>`);
    if (!enabled || (sub && !subItems.length)) b.disabled = true;
    if (sub) {
      b.addEventListener('pointerenter', () => openSub(b, subItems, depth));
      b.addEventListener('click', () => openSub(b, subItems, depth));
    } else {
      b.addEventListener('pointerenter', () => closeSubs(depth));
      b.addEventListener('click', () => { closeMenus(); (c ? c.run : it.run)(); });
    }
    box.appendChild(b);
  }
  box.dataset.depth = String(depth);
  return box;
}
function closeSubs(depth) { document.querySelectorAll('#menuLayer .dmenu').forEach(m => { if (+m.dataset.depth > depth) m.remove(); }); document.querySelectorAll(`#menuLayer .dmenu[data-depth="${depth}"] .mi.hot`).forEach(x => x.classList.remove('hot')); }
function openSub(btn, items, depth) {
  closeSubs(depth);
  if (!items.length) return;
  btn.classList.add('hot');
  const m = menuItemsEl(items.map(x => typeof x === 'object' && x.run ? x : x), depth + 1);
  $('#menuLayer').appendChild(m);
  const r = btn.getBoundingClientRect();
  let x = r.right + 2, y = r.top - 5;
  const mw = m.offsetWidth, mh = m.offsetHeight;
  if (x + mw > innerWidth - 6) x = r.left - mw - 2;
  if (y + mh > innerHeight - 6) y = innerHeight - mh - 6;
  m.style.left = Math.max(6, x) + 'px'; m.style.top = Math.max(6, y) + 'px';
}
function openMenu(i) {
  closeMenus(); openMenuIdx = i;
  const btn = document.querySelectorAll('#menus > button')[i]; if (!btn) return; btn.classList.add('open');
  const m = menuItemsEl(MENUS[i][1], 0);
  $('#menuLayer').appendChild(m);
  const r = btn.getBoundingClientRect();
  m.style.left = Math.max(6, Math.min(r.left, innerWidth - m.offsetWidth - 6)) + 'px'; m.style.top = (r.bottom + 4) + 'px';
}
function buildMenubar() {
  const nav = $('#menus'); if (!nav) return; nav.innerHTML = '';
  MENUS.forEach(([label], i) => {
    const b = el('button', { type: 'button', role: 'menuitem', 'aria-haspopup': 'true' }, label);
    b.addEventListener('click', e => { e.stopPropagation(); openMenuIdx === i ? closeMenus() : openMenu(i); });
    b.addEventListener('pointerenter', () => { if (openMenuIdx >= 0 && openMenuIdx !== i) openMenu(i); });
    nav.appendChild(b);
  });
  document.addEventListener('pointerdown', e => { if (openMenuIdx >= 0 && !e.target.closest('#menuLayer') && !e.target.closest('#menus')) closeMenus(); }, true);
  window.addEventListener('resize', closeMenus);
  window.addEventListener('blur', closeMenus);
}

/* ---------------- 工具条与提示 ---------------- */
function buildToolstrip() {
  const ts = $('#toolstrip'); if (!ts) return; ts.innerHTML = '';
  for (const t of TOOL_DEFS) {
    if (t === '-') { ts.appendChild(el('div', { class: 'ts-sep' })); continue; }
    const [id, name, k, svg] = t;
    const b = el('button', { type: 'button', 'data-tool': id, 'aria-label': `${name}（${kbdLabel(k)}）`, 'data-tip': name, 'data-tipk': kbdLabel(k) },
      `<svg viewBox="0 0 20 20">${svg}</svg>${k && k.length === 1 && /\d/.test(k) ? `<span class="tk">${k}</span>` : ''}`);
    b.addEventListener('click', () => setTool(id));
    ts.appendChild(b);
  }
  // 自定义提示（带快捷键）
  let tipEl = null, tipT = 0;
  const hide = () => { clearTimeout(tipT); if (tipEl) { tipEl.remove(); tipEl = null; } };
  document.addEventListener('pointerover', e => {
    const t = e.target.closest && e.target.closest('[data-tip]');
    if (!t) { hide(); return; }
    clearTimeout(tipT);
    tipT = setTimeout(() => {
      hide();
      tipEl = el('div', { class: 'tip' }, `${escapeHtml(t.dataset.tip)}${t.dataset.tipk ? `<kbd>${escapeHtml(t.dataset.tipk)}</kbd>` : ''}`);
      document.body.appendChild(tipEl);
      const r = t.getBoundingClientRect();
      tipEl.style.left = (r.right + 8) + 'px'; tipEl.style.top = (r.top + r.height / 2 - tipEl.offsetHeight / 2) + 'px';
    }, 350);
  });
  document.addEventListener('pointerdown', hide, true);
}
/** 控制栏中部：当前工具的操作提示，或选择状态 */
function updateOptContext() {
  const c = $('#obContext'); if (!c) return;
  if (!docOpen || homeShown()) { c.innerHTML = ''; return; }
  if (tool !== 'select' && tool !== 'pan' && TOOL_HINT[tool]) {
    c.innerHTML = `<b>${escapeHtml(CMD['tool.' + tool] ? CMD['tool.' + tool].label : tool)}</b> · ${escapeHtml(TOOL_HINT[tool])} <span class="th-k">· Alt 关闭吸附 · Esc 取消</span>`;
    return;
  }
  const list = selection(), n = list.length;   // 只认仍然存在的对象
  if (!n) { c.innerHTML = tool === 'pan' ? '<b>平移</b> · 拖动画布移动视图' : '<span class="th-k">单击选择 · Shift 加选 · 空白处拖出选框 · 双击进入编组</span>'; return; }
  const g = groupOf(list[0]);
  const whole = g && list.every(o => o.group === g.id) && n === groupMembers(g.id).length;
  c.innerHTML = n === 1 ? `已选 <b>${escapeHtml(list[0].name)}</b> · ${escapeHtml(KIND_NAME[list[0].kind] || '')}${list[0].locked ? ' · 已锁定' : ''}`
    : whole ? `已选编组 <b>${escapeHtml(g.name)}</b> · ${n} 个对象` : `已选 <b>${n}</b> 个对象`;
}

/* ---------------- 快捷键：统一由命令表分发 ---------------- */
const CODE_KEY = { BracketLeft: '[', BracketRight: ']', Equal: '=', Minus: '-', Comma: ',', Period: '.', Slash: '/', Space: 'Space', Backquote: '`' };
function comboOf(e) {
  // “?” 按字符识别：不同键盘布局下它不一定在 Slash 键位
  if (e.key === '?' && !(e.ctrlKey || e.metaKey || e.altKey)) return 'Shift+/';
  let k = CODE_KEY[e.code];
  if (!k) {
    if (/^Key[A-Z]$/.test(e.code)) k = e.code.slice(3);
    else if (/^Digit\d$/.test(e.code)) k = e.code.slice(5);
    else if (/^Numpad\d$/.test(e.code)) k = e.code.slice(6);
    else k = e.key === 'Del' ? 'Delete' : e.key;
  }
  return `${e.ctrlKey || e.metaKey ? 'Ctrl+' : ''}${e.altKey ? 'Alt+' : ''}${e.shiftKey ? 'Shift+' : ''}${k}`;
}
let KEYMAP = null;
function buildKeymap() {
  KEYMAP = new Map();
  for (const c of Object.values(CMD)) for (const k of [c.kbd, ...(c.alt || [])]) if (k) KEYMAP.set(k, c);
}
function handleShortcut(e) {
  if (!KEYMAP) buildKeymap();
  const c = KEYMAP.get(comboOf(e));
  if (!c) return false;
  e.preventDefault();
  if (c.enabled && !c.enabled()) return true;
  closeMenus();
  c.run();
  return true;
}

function syncCommands() {
  document.querySelectorAll('[data-cmd]').forEach(b => {
    const c = CMD[b.dataset.cmd];
    if (!c) return;
    b.disabled = !!(c.enabled && !c.enabled());
    if (c.checked) b.classList.toggle('active', !!c.checked());
    const label = (c.label2 && c.label2()) || c.label;
    if (label && b.textContent !== label) b.textContent = label;
  });
}
function renderLabActions() {
  const host = $('#labActions'); if (!host) return;
  host.innerHTML = '';
  for (const [label, items] of MENUS) {
    if (label === '插入') continue;
    const ids = [];
    const walk = (arr) => {
      for (const it of (typeof arr === 'function' ? arr() : arr)) {
        if (it === '-' || (it && it.head)) continue;
        if (typeof it === 'string') ids.push(it);
        else if (it && it.sub) walk(typeof it.sub === 'function' ? it.sub() : it.sub);
      }
    };
    walk(items);
    if (!ids.length) continue;
    const sec = el('section', { class: 'flat-sec' });
    sec.appendChild(el('h3', {}, label === '文件' ? '文档' : label === '模拟' ? '播放与环境' : label));
    const grid = el('div', { class: 'flat-grid' });
    for (const id of ids) {
      const c = CMD[id]; if (!c) continue;
      const b = el('button', { type: 'button', 'data-cmd': id }, (c.label2 && c.label2()) || c.label);
      b.addEventListener('click', () => {
        if (c.enabled && !c.enabled()) return;
        c.run();
        syncCommands();
      });
      grid.appendChild(b);
    }
    sec.appendChild(grid);
    host.appendChild(sec);
  }
  syncCommands();
}
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
  readColors();
  if ($('#menus')) buildMenubar();
  if ($('#toolstrip')) buildToolstrip();
  buildKeymap(); initToolbar(); renderLabActions();
  new ResizeObserver(resize).observe($('#stage'));
  resize();
  // 恢复上次打开的文档（包括未保存的修改）；没有则显示开始页
  let restored = false;
  try {
    const cur = JSON.parse(localStorage.getItem(CUR_KEY) || 'null');
    const s = localStorage.getItem(STORE_KEY);
    if (s && (!cur || cur.open)) {
      openAsDocument(JSON.parse(s), { id: cur ? cur.id : null, title: (cur && cur.title) || '恢复的文档', dirty: cur ? !!cur.dirty : true });
      restored = true;
    }
  } catch (e) { restored = false; }
  setTool('select');
  bootHistory();
  bindKeyboardInset();
  syncPanelToggleButtons();
  ensureDesmosKeypadDock(document.getElementById('pageAnalysis') || document.getElementById('right'));
  syncDesmosKeypadForAnalysis(false);
  if (restored) toast(isDirty() ? `已恢复「${doc.title}」（含未保存的修改）` : `已打开「${doc.title}」`);
  else showHome('home');
  syncDocTitle();
  window.addEventListener('beforeunload', () => { try { localStorage.setItem(STORE_KEY, serialize()); } catch { } syncDocTitle(); });
  window.__physim = {
    scene: () => scene, sim, advance, loadPreset, select, resetSim, togglePlay, fieldSys,
    setKeyboardInset: applyKeyboardInset,
    interaction: () => ({ tool, pending: pending ? (pending.type || true) : null, sheet, pinch: !!pinch, drag: drag ? drag.type : null, sel: [...selSet], arm: !$('#armBar')?.hidden }),
    presetKeys: () => Object.keys(PRESETS), energy: systemEnergy, predict: T => { predCache.msPerStep = 0; return computePrediction(T || 3); }, pins: () => pins,
    overlays, addOverlay: (type, target, extra) => addOverlay(type, target, extra), dynamics: id => { frameKey = null; return dyn(byId(id)); },
    forceParts: id => { frameKey = null; return forceParts(byId(id)); }, evalDisplay: (src, id) => { frameKey = null; return evalDisplay(src, byId(id)); },
    addVariable: (n, e) => addVariable(n, e, { focus: false }),
    handles: () => editHandles().map(h => ({ key: h.key, id: h.b.id, x: h.x, y: h.y, hx: h.hx, hy: h.hy, body: w2s(h.b.st.x, h.b.st.y) })), serialize, loadScene: d => loadScene(d, false), byName: n => scene.objects.find(o => o.name === n),
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
