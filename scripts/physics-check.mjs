#!/usr/bin/env node
/**
 * 物理回归测试：在真实浏览器里加载应用，逐项核对数值结果与理论值。
 *   npm run test:physics
 * 会自动启动一个临时 Vite 服务器；失败时以非零码退出。
 */
import { createServer } from 'vite';
import { chromium } from 'playwright';
import fs from 'node:fs';

const server = await createServer({ server: { port: 5199, host: '127.0.0.1', strictPort: false }, logLevel: 'error' });
await server.listen();
const url = server.resolvedUrls.local[0];
// 优先用 Playwright 自带浏览器；若版本不匹配，可用 CHROMIUM_PATH 指定本机 Chrome/Chromium
const fallback = process.env.CHROMIUM_PATH || ['/opt/pw-browsers/chromium'].find(p => fs.existsSync(p));
const browser = await chromium.launch().catch(err => {
  if (!fallback) throw err;
  return chromium.launch({ executablePath: fallback, args: ['--no-sandbox'] });
});
const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
const pageErrors = [];
page.on('pageerror', e => pageErrors.push(e.message));
await page.addInitScript(() => { try { localStorage.clear(); localStorage.setItem('physim.seenHelp', '1'); } catch { /* */ } });
await page.goto(url, { waitUntil: 'networkidle' });
await page.waitForFunction(() => window.__physim);

const results = await page.evaluate(() => {
  const P = window.__physim, out = [];
  const DEG = Math.PI / 180;
  const check = (name, ok, detail) => out.push({ name, ok: !!ok, detail });
  const run = T => { while (P.sim.t < T - 1e-9) P.advance(1 / 60); };
  const runSteps = n => { const h = P.scene().settings.maxDt; let d = 0; while (d < n) { const k = Math.min(17, n - d); P.advance(k * h); d += k; } };

  // 1. 所有实验：无 NaN，保守系统能量守恒
  for (const k of P.presetKeys()) {
    P.loadPreset(k);
    const e0 = P.energy(); run(3); const e1 = P.energy();
    const bad = P.scene().objects.some(o => o.st && (!Number.isFinite(o.st.x) || !Number.isFinite(o.st.y)));
    check(`${k}: 无 NaN`, !bad);
    if (['collision', 'springosc', 'coupled', 'pendulum', 'rodpendulum', 'pendlen', 'doublepend', 'kepler', 'binary', 'figure8', 'bbound', 'coulomb', 'rutherford'].includes(k)) {
      const rel = Math.abs((e1.E - e1.W) - e0.E) / Math.max(Math.abs(e0.E), 1e-9);
      check(`${k}: 能量守恒`, rel < 2e-3, `相对漂移 ${rel.toExponential(2)}`);
    }
  }

  // 2. 数值预测与实际运行逐步一致
  for (const k of P.presetKeys()) {
    P.loadPreset(k);
    const sc = P.scene(); sc.settings.predictT = 1;
    const pred = P.predict(); runSteps(Math.round(1 / sc.settings.maxDt));
    let err = 0;
    for (const o of sc.objects) { if (!o.st || o.fixed) continue; const a = pred.get(o.id), e = a[a.length - 1]; err = Math.max(err, Math.hypot(e[0] - o.st.x, e[1] - o.st.y)); }
    check(`${k}: 预测 = 实际`, err < 1e-12, `max|Δr| = ${err.toExponential(2)}`);
  }

  // 3. 解析结果
  P.loadPreset('collision'); run(2);
  { const [A, B] = P.scene().objects.filter(o => o.st); check('弹性碰撞 v₁′ = −2, v₂′ = 2', Math.abs(A.st.vx + 2) < 1e-6 && Math.abs(B.st.vx - 2) < 1e-6, `${A.st.vx}, ${B.st.vx}`); }
  P.loadPreset('inelastic'); run(3);
  { const [A, B] = P.scene().objects.filter(o => o.st); check('完全非弹性：共同速度 2 m/s', Math.abs(A.st.vx - 2) < 1e-6 && Math.abs(B.st.vx - 2) < 1e-6); }
  P.loadPreset('figure8'); run(6.32591398);
  // 二阶辛积分器 Δt = 2×10⁻⁴：一个周期后的位置误差应在 10⁻² 量级以内
  { const b1 = P.scene().objects.find(o => o.st); const d = Math.hypot(b1.st.x - b1.x0, b1.st.y - b1.y0); check('三体 8 字形：一个周期后回到起点', d < 1e-2, d.toExponential(2)); }
  P.loadPreset('incline');
  { const box = P.scene().objects.find(o => o.kind === 'box'); run(0.6);
    const a = Math.hypot(box.st.ax, box.st.ay), th = 37 * DEG, theory = 9.8 * (Math.sin(th) - 0.5 * Math.cos(th));
    check('斜面下滑加速度 g(sinθ − μcosθ)', Math.abs(a - theory) < 1e-3, `${a.toFixed(4)} vs ${theory.toFixed(4)}`); }
  P.loadPreset('incline');
  { const box = P.scene().objects.find(o => o.kind === 'box'); box.mus = 0.8; P.resetSim(); run(0.2); const x0 = box.st.x; run(3);   // 先落座，再测蠕滑
    check('静摩擦 μₛ > tanθ 时静止（无蠕滑）', Math.abs(box.st.x - x0) < 1e-6, `位移 ${(box.st.x - x0).toExponential(2)} m`); }
  P.loadPreset('newton'); run(1.2);
  { const bs = P.scene().objects.filter(o => o.st); const v5 = Math.hypot(bs[4].st.vx, bs[4].st.vy), vMid = Math.max(...bs.slice(1, 4).map(b => Math.hypot(b.st.vx, b.st.vy)));
    check('牛顿摆：动量传到最后一个球', v5 > 1 && vMid < 0.05, `v5 = ${v5.toFixed(3)}, 中间 ${vMid.toFixed(3)}`); }
  P.loadPreset('massspec');
  { const ion = P.scene().objects.find(o => o.name === 'm = 1'); let landed = null;
    while (P.sim.t < 8) { P.advance(1 / 240); if (P.sim.t > 2 && ion.st.vy < 0 && ion.st.y < 0.12 && !landed) landed = ion.st.x; }
    const r = Math.sqrt(2 * 1 * 2 * 1.95);
    check('质谱仪：落点 x = −2r', landed != null && Math.abs(landed + 2 * r) < 0.02, `${landed} vs ${-2 * r}`); }
  // 4. 转动：滚动、倾倒、多米诺
  P.loadPreset('rolling');
  { const bs = P.scene().objects.filter(o => o.st); run(0.3); const v0 = bs.map(b => Math.hypot(b.st.vx, b.st.vy)); run(1.0);
    const a = bs.map((b, i) => (Math.hypot(b.st.vx, b.st.vy) - v0[i]) / 0.7), s20 = 9.8 * Math.sin(20 * DEG), c20 = 9.8 * Math.cos(20 * DEG);
    check('光滑斜面：a = g sinθ', Math.abs(a[0] - s20) < 1e-3, `${a[0].toFixed(4)} vs ${s20.toFixed(4)}`);
    check('实心球纯滚动：a = 5/7 g sinθ', Math.abs(a[1] - s20 * 5 / 7) < 1e-3, `${a[1].toFixed(4)} vs ${(s20 * 5 / 7).toFixed(4)}`);
    check('滑块：a = g(sinθ − μcosθ)', Math.abs(a[2] - (s20 - 0.2 * c20)) < 1e-3, `${a[2].toFixed(4)} vs ${(s20 - 0.2 * c20).toFixed(4)}`);
    const ball = bs[1]; check('纯滚动约束 v = ωr', Math.abs(Math.hypot(ball.st.vx, ball.st.vy) - Math.abs(ball.st.omega) * ball.r) < 1e-3); }
  P.loadPreset('topple'); run(3);
  { const [tall, wide] = P.scene().objects.filter(o => o.st);
    check('tanθ > w/h 的木块倾倒', Math.abs(tall.st.ang / DEG - tall.ang0) > 60, `${(tall.st.ang / DEG - tall.ang0).toFixed(1)}°`);
    check('tanθ < w/h 的木块不倒不滑', Math.abs(wide.st.ang / DEG - wide.ang0) < 0.01 && Math.hypot(wide.st.vx, wide.st.vy) < 1e-4, `${(wide.st.ang / DEG - wide.ang0).toExponential(1)}°`); }
  P.loadPreset('dominoes'); run(4);
  { const ds = P.scene().objects.filter(o => o.st); check('多米诺骨牌全部倒下', ds.every(b => Math.abs(b.st.ang) > 60 * DEG), ds.map(b => (b.st.ang / DEG).toFixed(0)).join(' ')); }
  return out;
});

await browser.close();
await server.close();
let fail = 0;
for (const r of results) { if (!r.ok) fail++; console.log(`${r.ok ? '✓' : '✗'} ${r.name}${r.detail ? `  (${r.detail})` : ''}`); }
if (pageErrors.length) { fail++; console.log('✗ 页面报错：\n  ' + pageErrors.join('\n  ')); }
console.log(`\n${results.length - fail}/${results.length} 通过`);
process.exit(fail ? 1 : 0);
