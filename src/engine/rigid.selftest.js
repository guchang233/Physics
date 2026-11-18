/**
 * 刚体接触自测：与经典力学结论逐项对照。
 *   node src/engine/rigid.selftest.js
 */
import { collectContacts, solveVelocities, solvePositions, rotationalKE } from './rigid.js';

const g = 9.8, DEG = Math.PI / 180;
function body(kind, o) {
  const b = Object.assign({ kind, m: 1, r: 0.25, w: 0.6, h: 0.4, e: 0, mu: 0, rotate: true, fixed: false }, o);
  b.st = { x: o.x || 0, y: o.y || 0, vx: o.vx || 0, vy: o.vy || 0, ang: o.ang || 0, omega: o.omega || 0 };
  return b;
}
const ground = { x1: -1e4, y1: 0, x2: 1e4, y2: 0, ground: true, e: 1, smooth: false };
function incline(th) {
  // 过原点、倾角 th 的长斜面（向右上），只用斜边
  const L = 40; return { x1: -L * Math.cos(th), y1: -L * Math.sin(th), x2: L * Math.cos(th), y2: L * Math.sin(th), e: 0, smooth: false };
}
function step(bodies, segs, h) {
  for (const b of bodies) { const s = b.st; s.vy -= g * h; s.x += s.vx * h; s.y += s.vy * h; s.ang += s.omega * h; }
  const cs = collectContacts(bodies, bodies, segs);
  solveVelocities(cs, h, bodies.length > 1 ? 16 : 8);
  solvePositions(() => collectContacts(bodies, bodies, segs), 3);
}
function run(bodies, segs, T, h = 1e-3) { const n = Math.round(T / h); for (let i = 0; i < n; i++) step(bodies, segs, h); }

let fails = 0;
const check = (name, ok, detail) => { console.log(`${ok ? '✓' : '✗'} ${name}  ${detail}`); if (!ok) fails++; };

// 1. 实心球沿粗糙斜面纯滚动：a = (5/7) g sinθ
{
  const th = 30 * DEG, r = 0.25;
  const b = body('ball', { r, mu: 1, x: -r * Math.sin(th), y: r * Math.cos(th) });
  run([b], [incline(th)], 0.05);
  const v0 = Math.hypot(b.st.vx, b.st.vy), t0 = 0.05;
  run([b], [incline(th)], 0.5);
  const a = (Math.hypot(b.st.vx, b.st.vy) - v0) / 0.5, th5 = 5 / 7 * g * Math.sin(th);
  check('实心球纯滚动 a = 5/7·g·sinθ', Math.abs(a - th5) / th5 < 0.01, `${a.toFixed(4)} vs ${th5.toFixed(4)}`);
  const roll = Math.abs(Math.hypot(b.st.vx, b.st.vy) - Math.abs(b.st.omega) * r);
  check('纯滚动约束 v = ωr', roll < 0.01, `|v − ωr| = ${roll.toExponential(2)}`);
  void t0;
}
// 2. 光滑斜面上的球只滑不转：a = g sinθ, ω = 0
{
  const th = 30 * DEG, r = 0.25, seg = { ...incline(th), smooth: true };
  const b = body('ball', { r, x: -r * Math.sin(th), y: r * Math.cos(th) });
  run([b], [seg], 0.5);
  const a = Math.hypot(b.st.vx, b.st.vy) / 0.5;
  check('光滑斜面 a = g·sinθ 且不转', Math.abs(a - g * 0.5) < 0.02 && Math.abs(b.st.omega) < 1e-9, `${a.toFixed(4)}, ω = ${b.st.omega}`);
}
// 3. 滑动的球在粗糙地面上变为纯滚动：v_end = 5/7 v0
{
  const r = 0.25, b = body('ball', { r, mu: 0.3, y: r, vx: 7 });
  run([b], [ground], 3);
  check('滑动→纯滚动 v = 5/7·v₀', Math.abs(b.st.vx - 5) < 0.02 && Math.abs(b.st.vx - b.st.omega * -r) < 0.02, `v = ${b.st.vx.toFixed(4)}, ωr = ${(-b.st.omega * r).toFixed(4)}`);
}
// 4. 地面上静置的矩形：不漂移、不转动、不下沉
{
  const b = body('box', { w: 1, h: 0.5, mu: 0.5, y: 0.25 });
  run([b], [ground], 3);
  check('静置矩形保持静止', Math.abs(b.st.x) < 1e-9 && Math.abs(b.st.y - 0.25) < 2e-4 && Math.abs(b.st.ang) < 1e-9, `x=${b.st.x.toExponential(1)} y=${b.st.y.toFixed(5)} θ=${b.st.ang.toExponential(1)}`);
}
// 5. 倾倒判据：斜面上不滑动时，tanθ > w/h 会倾倒，否则保持
for (const [w, h, expectTip] of [[0.4, 1.0, true], [0.6, 0.4, false]]) {
  const th = 30 * DEG;
  const nx = -Math.sin(th), ny = Math.cos(th);
  const b = body('box', { w, h, mu: 1.2, mus: 1.2, ang: th, x: nx * h / 2, y: ny * h / 2 });
  run([b], [incline(th)], 2.5);
  const tipped = Math.abs(b.st.ang - th) > 20 * DEG;
  check(`矩形 w/h = ${(w / h).toFixed(2)}，tan30° = 0.577：${expectTip ? '应倾倒' : '应保持'}`, tipped === expectTip, `转过 ${((b.st.ang - th) / DEG).toFixed(1)}°`);
}
// 6. 完全弹性弹跳：机械能守恒
{
  const b = body('ball', { r: 0.2, e: 1, y: 2 });
  const E = s => 0.5 * (s.vx ** 2 + s.vy ** 2) + g * s.y;
  const E0 = E(b.st);
  run([b], [ground], 4);
  const rel = Math.abs(E(b.st) - E0) / E0;
  check('e = 1 弹跳 4 s 机械能守恒', rel < 5e-3, `相对误差 ${rel.toExponential(2)}`);
}
// 7. 两球斜碰：动量守恒，弹性时动能守恒
{
  const a = body('ball', { r: 0.3, e: 1, x: -2, y: 5, vx: 4 }), b = body('ball', { r: 0.3, e: 1, m: 2, x: 0, y: 5.25 });
  const P0 = [a.m * a.st.vx + b.m * b.st.vx, 0], K0 = 0.5 * a.m * 16;
  for (let i = 0; i < 1000; i++) {
    for (const o of [a, b]) { o.st.x += o.st.vx * 1e-3; o.st.y += o.st.vy * 1e-3; }
    const cs = collectContacts([a, b], [a, b], []); solveVelocities(cs, 1e-3, 8); solvePositions(() => collectContacts([a, b], [a, b], []), 3);
  }
  const P = [a.m * a.st.vx + b.m * b.st.vx, a.m * a.st.vy + b.m * b.st.vy];
  const K = 0.5 * a.m * (a.st.vx ** 2 + a.st.vy ** 2) + 0.5 * b.m * (b.st.vx ** 2 + b.st.vy ** 2) + rotationalKE(a) + rotationalKE(b);
  check('斜碰：动量守恒', Math.hypot(P[0] - P0[0], P[1] - P0[1]) < 1e-9, `Δp = ${Math.hypot(P[0] - P0[0], P[1] - P0[1]).toExponential(2)}`);
  check('斜碰（光滑、e = 1）：动能守恒', Math.abs(K - K0) / K0 < 1e-9, `ΔK/K = ${(Math.abs(K - K0) / K0).toExponential(2)}`);
}
// 8. 叠放的两个矩形：静止且不下沉
{
  const lo = body('box', { w: 1, h: 0.5, mu: 0.5, y: 0.25 }), hi = body('box', { w: 0.6, h: 0.4, mu: 0.5, x: 0.15, y: 0.7 });
  run([lo, hi], [ground], 3);
  check('叠放矩形静止', Math.abs(hi.st.y - 0.7) < 5e-4 && Math.abs(hi.st.vy) < 5e-4 && Math.abs(hi.st.ang) < 1e-4, `上 y=${hi.st.y.toFixed(5)} vy=${hi.st.vy.toExponential(1)} θ=${hi.st.ang.toExponential(1)}`);
}

// 9. 一排相互接触的等质量弹性球（牛顿摆）：入射速度完整传给最后一个，其余静止
{
  const B = [];
  for (let i = 0; i < 5; i++) B.push(body('ball', { r: 0.2, e: 1, x: i * 0.4 - (i === 0 ? 0.5 : 0), vx: i === 0 ? 4 : 0 }));
  for (let s = 0; s < 400; s++) {
    for (const b of B) b.st.x += b.st.vx * 1e-3;
    const cs = collectContacts(B, B, []); solveVelocities(cs, 1e-3, 16); solvePositions(() => collectContacts(B, B, []), 3);
  }
  const v = B.map(b => b.st.vx);
  check('牛顿摆：v = (0, 0, 0, 0, 4)', Math.abs(v[4] - 4) < 1e-9 && v.slice(0, 4).every(x => Math.abs(x) < 1e-9), v.map(x => x.toFixed(4)).join(', '));
}
// 10. 平放的矩形以 e = 0.5 落地：两角同时着地，不应产生转动
{
  const b = body('box', { w: 1, h: 0.4, e: 0.5, mu: 0.4, y: 1.5 });
  const gnd = { ...ground, e: 1 };
  run([b], [gnd], 2);
  check('矩形平着落地不转动', Math.abs(b.st.ang) < 1e-9 && Math.abs(b.st.omega) < 1e-9, `θ = ${b.st.ang.toExponential(1)}, ω = ${b.st.omega.toExponential(1)}`);
}

console.log(fails ? `\n${fails} 项失败` : '\nrigid selftest OK');
process.exit(fails ? 1 : 0);
