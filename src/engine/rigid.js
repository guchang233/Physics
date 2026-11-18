/**
 * 二维刚体接触：几何检测 + 顺序冲量求解（含转动与库仑摩擦）。
 *
 * 物体约定（与 simulator.js 一致）：
 *   { kind: 'ball' | 'particle' | 'box', m, r, w, h, e, mu, mus, rotate, fixed, _grab,
 *     st: { x, y, vx, vy, ang, omega } }
 * 表面线段：{ x1, y1, x2, y2, smooth, e, ground }
 *
 * 一个子步内的流程（由调用方组织）：
 *   速度半步/力 → 位置漂移 → collectContacts（只收穿透的接触点）
 *   → solveVelocities（迭代求冲量；非弹跳冲量同步修正本步位移，消除蠕滑）
 *   → solvePositions（剩余穿透按质量与转动惯量分摊推出）
 */

export const REST_V = 0.15;        // 法向接近速度低于此值视为静止接触，不反弹
export const SLOP = 1e-4;          // 位置修正允许的残余穿透
const BAUMGARTE = 0.8;

/* ---------------- 质量属性 ---------------- */
export function canRotate(b) { return b.kind !== 'particle' && b.rotate !== false; }
/** 转动惯量（绕质心）：矩形 m(w²+h²)/12；小球按实心球 2/5·m r² */
export function inertia(b) {
  if (!canRotate(b)) return Infinity;
  return b.kind === 'box' ? b.m * (b.w * b.w + b.h * b.h) / 12 : 0.4 * b.m * b.r * b.r;
}
export function invMass(b) { return !b || b.fixed || b._grab ? 0 : 1 / b.m; }
export function invInertia(b) { if (!b || b.fixed || b._grab) return 0; const I = inertia(b); return Number.isFinite(I) && I > 0 ? 1 / I : 0; }
export function boundRadius(b) { return b.kind === 'box' ? Math.hypot(b.w, b.h) / 2 : b.r; }

/* ---------------- 几何 ---------------- */
export function boxCorners(b) {
  const s = b.st, c = Math.cos(s.ang), n = Math.sin(s.ang), hw = b.w / 2, hh = b.h / 2;
  return [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]].map(([lx, ly]) => [s.x + lx * c - ly * n, s.y + lx * n + ly * c]);
}
function toLocal(b, x, y) { const s = b.st, c = Math.cos(s.ang), n = Math.sin(s.ang), dx = x - s.x, dy = y - s.y; return [dx * c + dy * n, -dx * n + dy * c]; }
function toWorldDir(b, lx, ly) { const c = Math.cos(b.st.ang), n = Math.sin(b.st.ang); return [lx * c - ly * n, lx * n + ly * c]; }
/** 矩形在方向 u 上的投影半径 */
function boxRadiusAlong(b, ux, uy) {
  const c = Math.cos(b.st.ang), n = Math.sin(b.st.ang);
  return b.w / 2 * Math.abs(c * ux + n * uy) + b.h / 2 * Math.abs(-n * ux + c * uy);
}

/**
 * 接触点：a 为 null 表示静止表面。n 从 a 指向 b（对表面：从表面指向物体），sep < 0 为穿透深度。
 */
function contact(a, b, nx, ny, px, py, sep, e, mu, mus) { return { a, b, nx, ny, px, py, sep, e, mu, mus }; }

function surfaceFriction(b, sg) {
  if (sg.smooth) return [0, 0];
  const mu = b.mu || 0;
  return [mu, Math.max(mu, b.mus ?? mu)];
}

/** 物体与一条静止表面的接触点 */
export function surfaceContacts(b, sg, out) {
  const n0 = out.length;
  surfaceContacts0(b, sg, out);
  for (let i = n0; i < out.length; i++) out[i].sg = sg;
}
function surfaceContacts0(b, sg, out) {
  const s = b.st, e = Math.min(b.e ?? 0, sg.e ?? 0), [mu, mus] = surfaceFriction(b, sg);
  if (b.kind !== 'box') {
    if (sg.ground) {
      const sep = s.y - sg.y1 - b.r;
      if (sep < 0 && sep > -Math.max(2 * b.r, 0.5)) out.push(contact(null, b, 0, 1, s.x, s.y - b.r, sep, e, mu, mus));
      return;
    }
    const dx = sg.x2 - sg.x1, dy = sg.y2 - sg.y1, L2 = dx * dx + dy * dy;
    let t = L2 > 0 ? ((s.x - sg.x1) * dx + (s.y - sg.y1) * dy) / L2 : 0; t = Math.max(0, Math.min(1, t));
    const cx = sg.x1 + t * dx, cy = sg.y1 + t * dy, ex = s.x - cx, ey = s.y - cy, d = Math.hypot(ex, ey);
    if (d < 1e-12 || d >= b.r) return;
    const nx = ex / d, ny = ey / d;
    out.push(contact(null, b, nx, ny, s.x - nx * b.r, s.y - ny * b.r, d - b.r, e, mu, mus));
    return;
  }
  const corners = boxCorners(b), deep = b.w + b.h + 0.5;
  if (sg.ground) {
    for (const [x, y] of corners) { const sep = y - sg.y1; if (sep < 0 && sep > -deep) out.push(contact(null, b, 0, 1, x, y, sep, e, mu, mus)); }
    return;
  }
  const dx = sg.x2 - sg.x1, dy = sg.y2 - sg.y1, L = Math.hypot(dx, dy);
  if (L < 1e-12) return;
  const ux = dx / L, uy = dy / L;
  let nx = -uy, ny = ux;
  if ((s.x - sg.x1) * nx + (s.y - sg.y1) * ny < 0) { nx = -nx; ny = -ny; }
  for (const [x, y] of corners) {
    const t = (x - sg.x1) * ux + (y - sg.y1) * uy;
    if (t < 0 || t > L) continue;
    const sep = (x - sg.x1) * nx + (y - sg.y1) * ny;
    if (sep < 0 && sep > -deep) out.push(contact(null, b, nx, ny, x, y, sep, e, mu, mus));
  }
  // 线段端点（例如斜面顶点、挡板端头）扎进矩形
  for (const [x, y] of [[sg.x1, sg.y1], [sg.x2, sg.y2]]) {
    const [lx, ly] = toLocal(b, x, y), hw = b.w / 2, hh = b.h / 2;
    if (Math.abs(lx) >= hw || Math.abs(ly) >= hh) continue;
    const px = hw - Math.abs(lx), py = hh - Math.abs(ly);
    const [fx, fy] = px < py ? toWorldDir(b, Math.sign(lx) || 1, 0) : toWorldDir(b, 0, Math.sign(ly) || 1);
    out.push(contact(null, b, -fx, -fy, x, y, -Math.min(px, py), e, mu, mus));
  }
}

/** 两个物体之间的接触点（最多 2 个） */
export function pairContacts(a, b, out) {
  const A = a.st, B = b.st;
  if (Math.hypot(B.x - A.x, B.y - A.y) >= boundRadius(a) + boundRadius(b)) return;
  const e = Math.min(a.e ?? 0, b.e ?? 0), mu = Math.min(a.mu || 0, b.mu || 0);
  const mus = Math.max(mu, Math.min(a.mus ?? a.mu ?? 0, b.mus ?? b.mu ?? 0));
  if (a.kind !== 'box' && b.kind !== 'box') {
    const dx = B.x - A.x, dy = B.y - A.y, d = Math.hypot(dx, dy), R = a.r + b.r;
    if (d >= R) return;
    const nx = d < 1e-12 ? 1 : dx / d, ny = d < 1e-12 ? 0 : dy / d;
    out.push(contact(a, b, nx, ny, A.x + nx * a.r, A.y + ny * a.r, d - R, e, mu, mus));
    return;
  }
  if (a.kind === 'box' && b.kind !== 'box') return boxCircle(a, b, false, e, mu, mus, out);
  if (a.kind !== 'box' && b.kind === 'box') return boxCircle(b, a, true, e, mu, mus, out);
  boxBox(a, b, e, mu, mus, out);
}
function boxCircle(box, c, flip, e, mu, mus, out) {
  const [lx, ly] = toLocal(box, c.st.x, c.st.y), hw = box.w / 2, hh = box.h / 2;
  let nlx, nly, sep, qx, qy;
  if (Math.abs(lx) <= hw && Math.abs(ly) <= hh) {
    const px = hw - Math.abs(lx), py = hh - Math.abs(ly);
    if (px < py) { nlx = Math.sign(lx) || 1; nly = 0; qx = nlx * hw; qy = ly; sep = -(px + c.r); }
    else { nlx = 0; nly = Math.sign(ly) || 1; qx = lx; qy = nly * hh; sep = -(py + c.r); }
  } else {
    qx = Math.max(-hw, Math.min(hw, lx)); qy = Math.max(-hh, Math.min(hh, ly));
    const ex = lx - qx, ey = ly - qy, d = Math.hypot(ex, ey);
    if (d >= c.r) return;
    nlx = ex / d; nly = ey / d; sep = d - c.r;
  }
  let [nx, ny] = toWorldDir(box, nlx, nly);
  const [wx, wy] = toWorldDir(box, qx, qy);
  const px = box.st.x + wx, py = box.st.y + wy;
  if (flip) out.push(contact(c, box, -nx, -ny, px, py, sep, e, mu, mus));
  else out.push(contact(box, c, nx, ny, px, py, sep, e, mu, mus));
}
function boxBox(a, b, e, mu, mus, out) {
  const A = a.st, B = b.st, dx = B.x - A.x, dy = B.y - A.y;
  const axes = [[Math.cos(A.ang), Math.sin(A.ang)], [-Math.sin(A.ang), Math.cos(A.ang)], [Math.cos(B.ang), Math.sin(B.ang)], [-Math.sin(B.ang), Math.cos(B.ang)]];
  let best = null;
  for (const [ux, uy] of axes) {
    const dist = dx * ux + dy * uy;
    const ov = boxRadiusAlong(a, ux, uy) + boxRadiusAlong(b, ux, uy) - Math.abs(dist);
    if (ov <= 0) return;
    if (!best || ov < best.ov - 1e-12) { const sg = dist >= 0 ? 1 : -1; best = { nx: ux * sg, ny: uy * sg, ov }; }
  }
  const { nx, ny } = best;
  const faceA = nx * A.x + ny * A.y + boxRadiusAlong(a, nx, ny);   // A 朝向 B 的面
  const faceB = nx * B.x + ny * B.y - boxRadiusAlong(b, nx, ny);   // B 朝向 A 的面
  const pts = [];
  const tol = 1e-6;
  for (const [x, y] of boxCorners(b)) {
    const [lx, ly] = toLocal(a, x, y);
    if (Math.abs(lx) <= a.w / 2 + tol && Math.abs(ly) <= a.h / 2 + tol) pts.push([x, y, nx * x + ny * y - faceA]);
  }
  for (const [x, y] of boxCorners(a)) {
    const [lx, ly] = toLocal(b, x, y);
    if (Math.abs(lx) <= b.w / 2 + tol && Math.abs(ly) <= b.h / 2 + tol) pts.push([x, y, faceB - (nx * x + ny * y)]);
  }
  if (!pts.length) pts.push([(A.x + B.x) / 2, (A.y + B.y) / 2, -best.ov]);
  pts.sort((p, q) => p[2] - q[2]);
  for (const [x, y, sep] of pts.slice(0, 2)) out.push(contact(a, b, nx, ny, x, y, Math.min(sep, 0), e, mu, mus));
}

/* ---------------- 求解 ---------------- */
function relVel(c) {
  const B = c.b.st, rbx = c.px - B.x, rby = c.py - B.y;
  let vx = B.vx - (B.omega || 0) * rby, vy = B.vy + (B.omega || 0) * rbx;
  if (c.a) { const A = c.a.st, rax = c.px - A.x, ray = c.py - A.y; vx -= A.vx - (A.omega || 0) * ray; vy -= A.vy + (A.omega || 0) * rax; }
  return [vx, vy];
}
function applyImpulse(c, Px, Py, h, redrift) {
  const b = c.b, B = b.st, mb = invMass(b), ib = invInertia(b);
  const rbx = c.px - B.x, rby = c.py - B.y;
  const dvx = Px * mb, dvy = Py * mb, dw = (rbx * Py - rby * Px) * ib;
  B.vx += dvx; B.vy += dvy; B.omega = (B.omega || 0) + dw;
  if (redrift) { B.x += dvx * h; B.y += dvy * h; B.ang += dw * h; }
  if (c.a) {
    const a = c.a, A = a.st, ma = invMass(a), ia = invInertia(a);
    const rax = c.px - A.x, ray = c.py - A.y;
    const ax = -Px * ma, ay = -Py * ma, aw = -(rax * Py - ray * Px) * ia;
    A.vx += ax; A.vy += ay; A.omega = (A.omega || 0) + aw;
    if (redrift) { A.x += ax * h; A.y += ay * h; A.ang += aw * h; }
  }
}
function effMass(c, dx, dy) {
  const b = c.b, B = b.st, rb = (c.px - B.x) * dy - (c.py - B.y) * dx;
  let k = invMass(b) + invInertia(b) * rb * rb;
  if (c.a) { const A = c.a.st, ra = (c.px - A.x) * dy - (c.py - A.y) * dx; k += invMass(c.a) + invInertia(c.a) * ra * ra; }
  return k;
}

/**
 * 顺序冲量迭代。每个接触的恢复目标在第一次处理时按当时的相对速度确定，
 * 这样牛顿摆这类“依次传递”的碰撞仍然正确。
 * 非弹跳冲量（静止接触、摩擦）同时修正本子步的位移，等价于“先解速度、再走位置”，从而没有蠕滑。
 * 返回值写回 c.jn / c.jt（累计冲量），调用方据此统计接触力。
 */
function vnOf(c) { const [vx, vy] = relVel(c); return vx * c.nx + vy * c.ny; }
function applyNormal(c, d, h) { if (d) applyImpulse(c, d * c.nx, d * c.ny, h, !c.bounce); }
/** 两点耦合系数 A12 */
function crossK(c1, c2) {
  const nx = c1.nx, ny = c1.ny;
  const rn = (body, c) => body ? (c.px - body.st.x) * ny - (c.py - body.st.y) * nx : 0;
  return invMass(c1.a) + invMass(c1.b) + invInertia(c1.a) * rn(c1.a, c1) * rn(c1.a, c2) + invInertia(c1.b) * rn(c1.b, c1) * rn(c1.b, c2);
}
/**
 * 2×2 线性互补：w = A·x + b，x ≥ 0，w ≥ 0，x·w = 0。返回 [x1, x2] 或 null。
 */
function lcp2(A11, A12, A22, b1, b2) {
  const det = A11 * A22 - A12 * A12;
  if (det > 1e-10 * A11 * A22) { const x1 = -(A22 * b1 - A12 * b2) / det, x2 = -(A11 * b2 - A12 * b1) / det; if (x1 >= 0 && x2 >= 0) return [x1, x2]; }
  { const x1 = -b1 / A11; if (x1 >= 0 && A12 * x1 + b2 >= 0) return [x1, 0]; }
  { const x2 = -b2 / A22; if (x2 >= 0 && A12 * x2 + b1 >= 0) return [0, x2]; }
  if (b1 >= 0 && b2 >= 0) return [0, 0];
  return null;
}

/* ---- 第一阶段：弹跳（一遍顺序处理，恢复系数作用一次） ---- */
function bounceSingle(c, h) {
  const vn = vnOf(c);
  if (!(vn < -REST_V && c.e > 0)) return;
  c.bounce = true;
  const j = -(1 + c.e) * vn / c.kn;
  c.jb += j; applyNormal(c, j, h);
}
function bounceBlock(c1, c2, h) {
  const v1 = vnOf(c1), v2 = vnOf(c2);
  const in1 = v1 < -REST_V && c1.e > 0, in2 = v2 < -REST_V && c2.e > 0;
  if (!in1 && !in2) return;
  if (!(in1 && in2)) { bounceSingle(c1, h); bounceSingle(c2, h); return; }
  // 两角同时着地：联立求解，使两点都达到 −e·vn 的分离速度（不会凭空产生转动）
  const x = lcp2(c1.kn, crossK(c1, c2), c2.kn, v1 + c1.e * v1, v2 + c2.e * v2);
  if (!x) { bounceSingle(c1, h); bounceSingle(c2, h); return; }
  c1.bounce = c2.bounce = true;
  c1.jb += x[0]; c2.jb += x[1];
  applyNormal(c1, x[0], h); applyNormal(c2, x[1], h);
}

/* ---- 第二阶段：非穿透 vn ≥ 0（累计冲量 ≥ 0，不会抵消第一阶段的弹跳冲量） ---- */
function solveNormalSingle(c, h) {
  const jn0 = c.jn, jn = Math.max(0, jn0 - vnOf(c) / c.kn);
  c.jn = jn; applyNormal(c, jn - jn0, h);
}
/** 两点块求解（同一对物体、同一法向的两个接触点，例如平放的矩形） */
function solveNormalBlock(c1, c2, h) {
  const A11 = c1.kn, A22 = c2.kn, A12 = crossK(c1, c2);
  const a1 = c1.jn, a2 = c2.jn;
  const x = lcp2(A11, A12, A22, vnOf(c1) - (A11 * a1 + A12 * a2), vnOf(c2) - (A12 * a1 + A22 * a2));
  if (!x) { solveNormalSingle(c1, h); solveNormalSingle(c2, h); return; }
  c1.jn = x[0]; c2.jn = x[1];
  applyNormal(c1, x[0] - a1, h); applyNormal(c2, x[1] - a2, h);
}
/**
 * 库仑摩擦：迭代过程中一律按静摩擦锥 |f| ≤ μₛN 求解（法向冲量尚未收敛时不能过早判定滑动）；
 * 最后一轮若仍需超过 μₛN 才能锁住，则判定为滑动，改用动摩擦 μN。
 */
function solveFriction(c, h, final) {
  if (!(c.mu > 0 || c.mus > 0) || !(c.kt > 0)) return;
  const [vx, vy] = relVel(c);
  const vt = vx * c.tx + vy * c.ty, N = c.jn + c.jb;
  const jt0 = c.jt, need = jt0 - vt / c.kt;
  c.sticking = Math.abs(need) <= c.mus * N + 1e-15;
  const lim = (final && !c.sticking ? c.mu : c.mus) * N;
  const jt = Math.max(-lim, Math.min(lim, need));
  c.jt = jt;
  if (jt !== jt0) applyImpulse(c, (jt - jt0) * c.tx, (jt - jt0) * c.ty, h, true);
}

/**
 * 两点接触的摩擦：两点在同一条与切向平行的直线上，切向力对质心的力臂（r·n）相同，
 * 因此两点摩擦之和严格等价于作用在中点的单个摩擦力。合成后按 N₁+N₂ 判定静/动摩擦，
 * 避免逐点求解时两角摩擦分配不均而误判为滑动。
 */
function solveFrictionBlock(c1, c2, h, final) {
  if (!c1.m2) {
    c1.m2 = { a: c1.a, b: c1.b, px: (c1.px + c2.px) / 2, py: (c1.py + c2.py) / 2, nx: c1.nx, ny: c1.ny, tx: c1.tx, ty: c1.ty,
      mu: c1.mu, mus: c1.mus, jn: 0, jb: 0, jt: 0, bounce: false };
    c1.m2.kt = effMass(c1.m2, c1.tx, c1.ty);
  }
  const m = c1.m2;
  m.jn = c1.jn + c2.jn; m.jb = c1.jb + c2.jb;
  solveFriction(m, h, final);
  c1.jt = m.jt; c2.jt = 0; c1.sticking = c2.sticking = m.sticking;
}

/**
 * 接触速度求解，分两阶段：
 *  1. 弹跳：按列表顺序处理一遍，每个接触按“当时”的相对速度施加 (1+e) 冲量——碰撞可以像牛顿摆那样依次传递；
 *  2. 非穿透 + 摩擦：迭代求解，两点接触用块求解；非弹跳冲量同步修正本子步位移（无蠕滑）。
 * 累计法向冲量写回 c.jn（两阶段之和）、切向写回 c.jt，供调用方统计接触力。
 */
export function solveVelocities(contacts, h, iters = 8) {
  for (const c of contacts) {
    c.kn = effMass(c, c.nx, c.ny); c.tx = -c.ny; c.ty = c.nx; c.kt = effMass(c, c.tx, c.ty);
    c.jn = 0; c.jb = 0; c.jt = 0; c.bounce = false; c.sticking = true;
  }
  // 组装求解单元：同一对象对、同一表面、同一法向的两点合成一个块
  const units = [], used = new Set();
  for (let i = 0; i < contacts.length; i++) {
    if (used.has(i)) continue;
    const c = contacts[i];
    let mate = -1;
    for (let j = i + 1; j < contacts.length; j++) {
      const d = contacts[j];
      if (used.has(j) || d.a !== c.a || d.b !== c.b || d.sg !== c.sg) continue;
      if (Math.abs(d.nx * c.nx + d.ny * c.ny - 1) > 1e-9) continue;
      if (Math.hypot(d.px - c.px, d.py - c.py) < 1e-9) continue;
      mate = j; break;
    }
    used.add(i);
    if (mate >= 0) { used.add(mate); units.push([c, contacts[mate]]); } else units.push([c]);
  }
  const live = units.filter(u => u[0].kn > 0 && (u.length < 2 || u[1].kn > 0));
  for (const u of live) { if (u.length === 2) bounceBlock(u[0], u[1], h); else bounceSingle(u[0], h); }
  for (let it = 0; it < iters; it++) {
    const n = live.length;
    for (let q = 0; q < n; q++) {
      const u = live[it % 2 ? n - 1 - q : q];   // 正反交替，消除顺序偏置
      if (u.length === 2) solveNormalBlock(u[0], u[1], h); else solveNormalSingle(u[0], h);
      if (u.length === 2) solveFrictionBlock(u[0], u[1], h, it === iters - 1);
      else solveFriction(u[0], h, it === iters - 1);
    }
  }
  // 收尾：法向 ↔ 摩擦交替若干轮并以法向结束。两者互相扰动，交替投影按约 0.6 倍/轮收敛
  const SWEEPS = 12;
  for (let k = 0; k < SWEEPS; k++) {
    for (const u of live) { if (u.length === 2) solveNormalBlock(u[0], u[1], h); else solveNormalSingle(u[0], h); }
    if (k === SWEEPS - 1) break;
    for (const u of live) { if (u.length === 2) solveFrictionBlock(u[0], u[1], h, true); else solveFriction(u[0], h, true); }
  }
  for (const c of contacts) c.jn += c.jb;
  // 弹跳：本子步内物体已穿入 |sep|，真实轨迹应在接触面处反射，落在 e·|sep| 之外。按镜像位置补偿，避免每次弹跳损失/增加势能
  for (const c of contacts) {
    if (!c.bounce || !(c.kn > 0) || c.sep >= 0) continue;
    const lam = -c.sep * (1 + c.e) / c.kn;
    const b = c.b, B = b.st, rb = (c.px - B.x) * c.ny - (c.py - B.y) * c.nx;
    B.x += c.nx * lam * invMass(b); B.y += c.ny * lam * invMass(b); B.ang += invInertia(b) * rb * lam;
    if (c.a) {
      const a = c.a, A = a.st, ra = (c.px - A.x) * c.ny - (c.py - A.y) * c.nx;
      A.x -= c.nx * lam * invMass(a); A.y -= c.ny * lam * invMass(a); A.ang -= invInertia(a) * ra * lam;
    }
  }
}

/** 非线性高斯-赛德尔位置修正：只处理超出 SLOP 的穿透 */
export function solvePositions(collect, iters = 3) {
  for (let it = 0; it < iters; it++) {
    const cs = collect();
    let worst = 0;
    for (const c of cs) {
      const C = -(c.sep + SLOP);
      if (C <= 0) continue;
      worst = Math.max(worst, C);
      const k = effMass(c, c.nx, c.ny);
      if (!(k > 0)) continue;
      const lam = BAUMGARTE * C / k;
      const b = c.b, B = b.st, rb = (c.px - B.x) * c.ny - (c.py - B.y) * c.nx;
      B.x += c.nx * lam * invMass(b); B.y += c.ny * lam * invMass(b); B.ang += invInertia(b) * rb * lam;
      if (c.a) {
        const a = c.a, A = a.st, ra = (c.px - A.x) * c.ny - (c.py - A.y) * c.nx;
        A.x -= c.nx * lam * invMass(a); A.y -= c.ny * lam * invMass(a); A.ang -= invInertia(a) * ra * lam;
      }
    }
    if (worst < SLOP) break;
  }
}

/** 收集一组物体与表面、物体之间的全部穿透接触 */
export function collectContacts(movers, colliders, segs) {
  const out = [];
  for (const b of movers) { if (b._grab) continue; for (const sg of segs) surfaceContacts(b, sg, out); }
  for (let i = 0; i < colliders.length; i++) for (let j = i + 1; j < colliders.length; j++) {
    const a = colliders[i], b = colliders[j];
    if ((a.fixed || a._grab) && (b.fixed || b._grab)) continue;
    pairContacts(a, b, out);
  }
  return out;
}

/** 转动动能 ½Iω² */
export function rotationalKE(b) { const I = inertia(b); return Number.isFinite(I) && b.st ? 0.5 * I * (b.st.omega || 0) ** 2 : 0; }
