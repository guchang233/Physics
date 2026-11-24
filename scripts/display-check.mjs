#!/usr/bin/env node
/**
 * 显示系统 / 全局变量 / 画布拾取 回归测试
 *   npm run test:display
 */
import { createServer } from 'vite';
import { chromium } from 'playwright';
import fs from 'node:fs';

const server = await createServer({ server: { port: 5197, host: '127.0.0.1', strictPort: false }, logLevel: 'error' });
await server.listen();
const url = server.resolvedUrls.local[0];
const fallback = process.env.CHROMIUM_PATH || ['/opt/pw-browsers/chromium'].find(p => fs.existsSync(p));
const launch = () => chromium.launch().catch(err => { if (!fallback) throw err; return chromium.launch({ executablePath: fallback, args: ['--no-sandbox'] }); });
const R = []; const ok = (n, c, x = '') => R.push(`${c ? 'PASS' : 'FAIL'}  ${n}${x ? '  — ' + x : ''}`);
const b = await launch();
const p = await b.newPage({ viewport: { width: 1440, height: 900 } });
const errs = []; p.on('pageerror', e => errs.push(e.message)); p.on('console', m => { if (m.type() === 'error') errs.push(m.text()); });
await p.addInitScript(() => { try { if (!sessionStorage.getItem('dc')) { localStorage.clear(); sessionStorage.setItem('dc', '1'); } localStorage.setItem('physim.seenHelp', '1'); } catch { /* */ } });
await p.goto(url, { waitUntil: 'networkidle' }); await p.waitForTimeout(300);
const cv = await p.locator('#cv').boundingBox();
const E = (fn, arg) => p.evaluate(fn, arg);
const w2s = (x, y) => E(([x, y]) => { const v = window.__physim.scene().view, c = document.getElementById('cv').getBoundingClientRect(); return [c.left + c.width / 2 + (x - v.cx) * v.scale, c.top + c.height / 2 - (y - v.cy) * v.scale]; }, [x, y]);

// ---------- 1. 默认不显示；模板给出推荐显示 ----------
await E(() => window.__physim.loadPreset('incline')); await p.waitForTimeout(150);
ok('模板载入后没有任何显示项', await E(() => window.__physim.overlays().length === 0));
await p.locator('[data-tab="analysis"]').click(); await p.waitForTimeout(150);
ok('教学卡片提供“启用推荐显示”', await p.locator('.lesson .l-actions button.primary').isVisible());
await p.locator('.lesson .l-actions button.primary').click(); await p.waitForTimeout(100);
{ const t = await E(() => window.__physim.overlays().map(o => o.type).join(',')); ok('一键启用推荐显示', t.includes('trail') && t.includes('v') && t.includes('forces'), t); }
await p.keyboard.press('Control+z'); await p.waitForTimeout(100);
ok('撤销推荐显示', await E(() => window.__physim.overlays().length === 0));

// ---------- 2. t = 0 时的瞬时动力学（试走一个子步，不改变状态） ----------
{
  const r = await E(() => {
    const P = window.__physim, box = P.scene().objects.find(o => o.kind === 'box'), x = box.st.x, vx = box.st.vx;
    const d = P.dynamics(box.id), parts = P.forceParts(box.id).map(q => q.label);
    return { a: Math.hypot(d.ax, d.ay), parts, unchanged: box.st.x === x && box.st.vx === vx && P.sim.t === 0 };
  });
  const th = 37 * Math.PI / 180, a = 9.8 * (Math.sin(th) - 0.5 * Math.cos(th));
  ok('t = 0 的加速度 = g(sinθ − μcosθ)', Math.abs(r.a - a) < 2e-3, `${r.a.toFixed(4)} vs ${a.toFixed(4)}`);
  ok('t = 0 的受力含 G、N、f', ['G', 'N', 'f'].every(k => r.parts.includes(k)), r.parts.join(','));
  ok('试算不改变状态', r.unchanged);
}
// 静止在地面上的物块：N = mg，a = 0
await E(() => {
  const P = window.__physim;
  P.loadScene({ version: 4, settings: {}, objects: [
    { id: 1, kind: 'ground', name: '地面', y: 0 },
    { id: 2, kind: 'box', name: '物块', m: 2, w: 0.6, h: 0.4, x0: 0, y0: 0.2, vx0: 0, vy0: 0, mu: 0.3 },
    { id: 3, kind: 'ball', name: '球', m: 1, r: 0.25, x0: 3, y0: 2, vx0: 2, vy0: 0 },
  ], overlays: [] });
});
{
  const r = await E(() => { const P = window.__physim, ps = P.forceParts(2), N = ps.find(q => q.label === 'N'), d = P.dynamics(2); return { N: N ? N.fy : 0, a: Math.hypot(d.ax, d.ay) }; });
  ok('静止物块：N = mg，a = 0', Math.abs(r.N - 2 * 9.8) < 1e-6 && r.a < 1e-9, JSON.stringify(r));
}

// ---------- 3. 编辑手柄与显示分离：v₀ 只在 t = 0 出现，并画在物体当前位置 ----------
await E(() => window.__physim.select(3));
{
  const h = await E(() => window.__physim.handles());
  const v0 = h.find(x => x.key === 'v0');
  ok('选中后出现 v₀ 手柄（无需开启任何显示）', !!v0 && Math.abs(v0.x - v0.body[0]) < 1e-9 && Math.abs(v0.y - v0.body[1]) < 1e-9);
  // 拖动手柄改 v₀
  const c = await p.locator('#cv').boundingBox();
  await p.mouse.move(c.x + v0.hx, c.y + v0.hy); await p.mouse.down();
  await p.mouse.move(c.x + v0.hx + 40, c.y + v0.hy - 40, { steps: 5 }); await p.mouse.up();
  const v = await E(() => { const o = window.__physim.byName('球'); return [o.vx0, o.vy0]; });
  ok('拖手柄设置 v₀', v[0] > 3 && v[1] > 1.5, v.join(', '));
}
await E(() => { const P = window.__physim; P.togglePlay(); });
await p.waitForTimeout(400);
await E(() => { const P = window.__physim; if (P.sim.running) P.togglePlay(); });
await p.waitForTimeout(80);
{
  const h = await E(() => window.__physim.handles());
  ok('运行后暂停：不再有残留的 v₀ 手柄', !h.some(x => x.key === 'v0'), JSON.stringify(h.map(x => x.key)));
}
await E(() => window.__physim.resetSim());

// ---------- 4. 显示面板：添加、配置、自定义公式 ----------
await p.locator('[data-tab="objects"]').click(); await p.waitForTimeout(80);
await p.locator('#leftTabs [data-left-tab="display"]').click(); await p.waitForTimeout(80);
ok('显示面板空状态提示', await p.locator('.ovpanel .empty-hint').isVisible());
await p.locator('#bAddOverlay').click(); await p.waitForTimeout(60);
ok('添加菜单按类别分组', (await p.locator('.ctx-menu.ov-menu .ctx-head').count()) === 4);
await p.locator('.ctx-menu.ov-menu button', { hasText: '数值标签' }).click(); await p.waitForTimeout(80);
ok('添加数值标签并展开设置', await E(() => window.__physim.overlays().some(o => o.type === 'value')) && await p.locator('.ovrow.open').isVisible());
await p.locator('.ovrow.open .ov-opts button', { hasText: '动能' }).click(); await p.waitForTimeout(60);
{
  const r = await E(() => { const P = window.__physim, o = P.byName('球'); const e = P.evalDisplay('Ek', 3); return { e: e.value, th: 0.5 * o.m * (o.vx0 ** 2 + o.vy0 ** 2) }; });
  ok('自定义显示公式 Ek', Math.abs(r.e - r.th) < 1e-9, JSON.stringify(r));
}
{
  const r = await E(() => window.__physim.evalDisplay('m*a', 2));
  ok('显示公式可用加速度 a', r.ok && Math.abs(r.value) < 1e-9, JSON.stringify(r));
}
// 视图弹出框的快捷开关
await p.locator('#bView').click(); await p.waitForTimeout(60);
await p.locator('#viewQuick label', { hasText: '加速度' }).click(); await p.waitForTimeout(60);
ok('视图弹出框：一键给全部物体显示加速度', await E(() => window.__physim.overlays().some(o => o.type === 'a' && o.target === 'all' && o.on)));
await p.keyboard.press('Escape');

// ---------- 5. Desmos 式“创建变量” ----------
await E(() => window.__physim.select(3)); await p.waitForTimeout(80);
const vxRow = p.locator('#right .row', { hasText: '分量 vx₀' });
await vxRow.locator('.expr-toggle').click(); await p.waitForTimeout(60);
const inp = p.locator('#right .row', { hasText: '分量 vx₀' }).locator('input.expr-input');
await inp.fill('v0*cos(θ)'); await p.waitForTimeout(60);
{
  const names = await p.locator('#right .expr-assist .mk-var b').allTextContents();
  ok('公式里出现未知名字时提示创建变量', names.join(',') === 'v0,θ', names.join(','));
}
await p.locator('#right .expr-assist .mk-var', { hasText: 'v0' }).click(); await p.waitForTimeout(60);
await p.locator('#right .expr-assist .mk-var', { hasText: 'θ' }).click(); await p.waitForTimeout(60);
{
  const r = await E(() => { const P = window.__physim, sc = P.scene(); return { vars: (sc.vars || []).map(v => v.name).join(','), vx0: P.byName('球').vx0, err: (P.byName('球')._exprErr || {}).vx0 || '' }; });
  ok('一键创建 v0、θ 后公式生效', r.vars === 'v0,θ' && Math.abs(r.vx0 - Math.cos(1)) < 1e-9 && !r.err, JSON.stringify(r));
  ok('提示随之消失', (await p.locator('#right .expr-assist .mk-var').count()) === 0);
}
// 变量面板滑块驱动
await p.locator('[data-tab="objects"]').click(); await p.waitForTimeout(80);
await p.locator('#leftTabs [data-left-tab="vars"]').click(); await p.waitForTimeout(80);
ok('变量面板列出变量与滑块', (await p.locator('.vrow').count()) === 2 && (await p.locator('.vrow .v-slider').count()) === 2);
await p.locator('.vrow').first().locator('.v-expr').fill('5'); await p.locator('.vrow').first().locator('.v-expr').press('Enter'); await p.waitForTimeout(80);
{ const vx0 = await E(() => window.__physim.byName('球').vx0); ok('改变量值，引用它的初速度随之更新', Math.abs(vx0 - 5 * Math.cos(1)) < 1e-9, String(vx0)); }
// 改名同步更新公式
await p.locator('.vrow').first().locator('.v-name').fill('u'); await p.locator('.vrow').first().locator('.v-name').press('Enter'); await p.waitForTimeout(80);
{ const ex = await E(() => window.__physim.byName('球').expr.vx0); ok('变量改名同步更新公式', ex === 'u*cos(θ)', ex); }

// ---------- 6. 画布拾取：外力指向某个物体 ----------
await E(() => window.__physim.select(3)); await p.waitForTimeout(60);
await p.locator('#right .force-chip[data-mode="aim"]').click(); await p.waitForTimeout(60);
await p.locator('[data-tab="canvas"]').click(); await p.waitForTimeout(60);
await p.locator('#right .force-aim-select .pick-btn').click(); await p.waitForTimeout(60);
ok('进入拾取模式', await E(() => document.body.classList.contains('picking')));
{
  const q = await w2s(0, 0.2);
  await p.mouse.click(q[0], q[1]); await p.waitForTimeout(80);
  const r = await E(() => ({ aim: window.__physim.byName('球').FaimId, picking: document.body.classList.contains('picking'), sel: [...document.querySelectorAll('.lrow.sel[data-id]')].length }));
  ok('点画布上的物块 → 外力指向它（不改变选择）', r.aim === 2 && !r.picking, JSON.stringify(r));
}
await p.locator('#right .force-aim-select .pick-btn').click(); await p.waitForTimeout(60);
await p.mouse.click(cv.x + 80, cv.y + 120); await p.waitForTimeout(80);
{ const r = await E(() => { const o = window.__physim.byName('球'); return { id: o.FaimId, x: o.FaimX, y: o.FaimY }; }); ok('点空白处 → 指向一个固定点', r.id === null && Number.isFinite(r.x) && Number.isFinite(r.y), JSON.stringify(r)); }
await p.locator('#right .force-aim-select .pick-btn').click(); await p.keyboard.press('Escape'); await p.waitForTimeout(60);
ok('Esc 取消拾取', await E(() => !document.body.classList.contains('picking')));

// ---------- 7. 旧文档迁移 ----------
await E(() => window.__physim.loadScene({ version: 3, settings: { showVel: true, showTrail: true, showPredict: false, showForce: true, showLabels: false, showFieldLines: false, strobe: 0.05 }, objects: [{ id: 1, kind: 'ball', name: 'A', x0: 0, y0: 1 }] }));
{ const t = await E(() => window.__physim.overlays().map(o => o.type).sort().join(',')); ok('旧版显示开关迁移为显示项', t === 'analytic,forces,strobe,trail,v', t); }
{ const t = await E(() => Object.keys(window.__physim.scene().settings).filter(k => k.startsWith('show') && k !== 'showGrid').join(',')); ok('迁移后设置里不再有旧开关', t === '', t); }
// 保存再读回
{ const r = await E(() => { const P = window.__physim, s = JSON.parse(P.serialize()); P.loadScene(s); return { v: s.version, n: P.overlays().length }; }); ok('显示项随文档保存', r.v === 4 && r.n === 5, JSON.stringify(r)); }

ok('无页面报错', errs.length === 0, errs.join(' | '));
await b.close();
await server.close();
console.log(R.join('\n'));
const fails = R.filter(r => r.startsWith('FAIL')).length;
console.log(`\n${R.length - fails}/${R.length} 通过`);
process.exit(fails ? 1 : 0);
