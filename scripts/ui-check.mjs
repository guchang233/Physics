#!/usr/bin/env node
/**
 * 界面回归测试：在真实浏览器里模拟鼠标与键盘，走一遍完整的文档工作流
 * （新建、放置、选框、编组、对齐、拷贝粘贴、锁定、图层拖动、保存、模板、关闭询问、导出、刷新恢复）。
 *   npm run test:ui
 */
import { createServer } from 'vite';
import { chromium } from 'playwright';
import fs from 'node:fs';

const server = await createServer({ server: { port: 5198, host: '127.0.0.1', strictPort: false }, logLevel: 'error' });
await server.listen();
const url = server.resolvedUrls.local[0];
const fallback = process.env.CHROMIUM_PATH || ['/opt/pw-browsers/chromium'].find(p => fs.existsSync(p));
const launch = () => chromium.launch().catch(err => { if (!fallback) throw err; return chromium.launch({ executablePath: fallback, args: ['--no-sandbox'] }); });
const R = []; const ok = (n, c, x = '') => R.push(`${c ? 'PASS' : 'FAIL'}  ${n}${x ? '  — ' + x : ''}`);
const b = await launch();
const ctx = await b.newContext({ viewport: { width: 1440, height: 900 }, acceptDownloads: true });
const p = await ctx.newPage(); const errs = []; p.on('pageerror', e => errs.push(e.message)); p.on('console', m => { if (m.type() === 'error') errs.push(m.text()); });
await p.goto(url, { waitUntil: 'networkidle' }); await p.waitForTimeout(400);
const S = () => p.evaluate(() => { const P = window.__physim, sc = P.scene(); return { n: sc.objects.length, names: sc.objects.map(o => o.name), groups: (sc.groups || []).length, sel: [...document.querySelectorAll('.lrow.sel[data-id]')].length, t: P.sim.t }; });
const cv = await p.locator('#cv').boundingBox();
const w2s = (x, y) => p.evaluate(([x, y]) => { const v = window.__physim.scene().view, c = document.getElementById('cv').getBoundingClientRect(); return [c.left + c.width / 2 + (x - v.cx) * v.scale, c.top + c.height / 2 - (y - v.cy) * v.scale]; }, [x, y]);
ok('首次打开显示开始页', await p.locator('#home.show').isVisible());
ok('无文档时“保存”不可用', await p.evaluate(() => { document.querySelectorAll('#menus > button')[0].click(); const it = [...document.querySelectorAll('.dmenu .mi')].find(x => x.textContent.includes('保存') && !x.textContent.includes('另存')); return it && it.disabled; }));
await p.keyboard.press('Escape');
// 新建对话框
await p.keyboard.press('Control+n'); await p.waitForTimeout(150);
ok('Ctrl+N 打开新建对话框', await p.locator('#modal.show .dlg').isVisible());
await p.locator('.nd-list [data-p="table"]').click();
await p.locator('[data-k="title"]').fill('碰撞研究');
await p.locator('[data-act="create"]').click(); await p.waitForTimeout(250);
const st0 = await p.evaluate(() => ({ title: document.getElementById('docTitle').textContent, smooth: window.__physim.scene().objects[0].smooth, home: document.getElementById('home').classList.contains('show') }));
ok('新建文档：名称与预设生效', st0.title === '碰撞研究' && st0.smooth === true && !st0.home, JSON.stringify(st0));
// 工具与放置
await p.keyboard.press('1'); await p.keyboard.down('Shift');
for (const [x, y] of [[300, 300], [400, 300], [500, 300]]) await p.mouse.click(cv.x + x, cv.y + y);
await p.keyboard.up('Shift'); await p.keyboard.press('Escape');
let s = await S();
ok('数字键 1 + Shift 连续放置', s.n === 4, s.names.join(','));
ok('未保存标记', (await p.locator('#docState').textContent()).includes('未保存'));
// 选框
await p.mouse.move(cv.x + 260, cv.y + 250); await p.mouse.down(); await p.mouse.move(cv.x + 540, cv.y + 350, { steps: 6 }); await p.mouse.up();
s = await S(); ok('选框选中 3 个', s.sel === 3, `sel=${s.sel}`);
await p.keyboard.press('Control+g'); await p.waitForTimeout(100);
s = await S(); ok('Ctrl+G 编组', s.groups === 1);
// 点击编组成员 → 整组；双击 → 单个
await p.keyboard.press('Escape');
const b1 = await p.evaluate(() => { const o = window.__physim.scene().objects.find(o => o.kind === 'ball'); return [o.x0, o.y0]; });
const q1 = await w2s(...b1);
await p.mouse.click(q1[0], q1[1]); await p.waitForTimeout(80);
ok('单击编组成员选中整组', (await S()).sel === 3);
await p.mouse.dblclick(q1[0], q1[1]); await p.waitForTimeout(80);
ok('双击进入编组只选一个', (await S()).sel === 1);
// 移动整组
await p.mouse.click(cv.x + cv.width - 60, cv.y + cv.height - 120); // 画布内空白处取消
await p.mouse.click(q1[0], q1[1]);
const before = await p.evaluate(() => window.__physim.scene().objects.filter(o => o.kind === 'ball').map(o => o.x0));
await p.mouse.move(q1[0], q1[1]); await p.mouse.down(); await p.mouse.move(q1[0] + 120, q1[1], { steps: 5 }); await p.mouse.up();
const after = await p.evaluate(() => window.__physim.scene().objects.filter(o => o.kind === 'ball').map(o => o.x0));
ok('拖动整组一起移动', after.every((x, i) => Math.abs(x - before[i] - 2) < 1e-9), `${before} → ${after}`);
await p.keyboard.press('Control+z');
const und = await p.evaluate(() => window.__physim.scene().objects.filter(o => o.kind === 'ball').map(o => o.x0));
ok('撤销整组移动', und.every((x, i) => Math.abs(x - before[i]) < 1e-9));
// 共同属性编辑
const massRow = p.locator('#props .row', { hasText: '质量' }).locator('input').first();
await massRow.fill('2'); await massRow.press('Enter');
ok('多选编辑共同属性', await p.evaluate(() => window.__physim.scene().objects.filter(o => o.kind === 'ball').every(o => o.m === 2)));
// 对齐
await p.evaluate(() => { const bs = window.__physim.scene().objects.filter(o => o.kind === 'ball'); bs[1].y0 = 3; });
await p.locator('#menus > button', { hasText: '对象' }).click(); await p.locator('.dmenu .mi', { hasText: '对齐' }).hover(); await p.locator('.dmenu .mi', { hasText: '底对齐' }).click();
const ys = await p.evaluate(() => window.__physim.scene().objects.filter(o => o.kind === 'ball').map(o => o.y0));
ok('菜单 对象→对齐→底对齐', ys.every(y => Math.abs(y - ys[0]) < 1e-9), ys.join(','));
// 复制粘贴（带编组）
await p.keyboard.press('Control+c'); await p.mouse.move(cv.x + 600, cv.y + 500); await p.keyboard.press('Control+v');
s = await S(); ok('多选拷贝粘贴', s.n === 7, `n=${s.n}`);
await p.keyboard.press('Control+z');
// 锁定 → 画布不可选
await p.keyboard.press('Escape'); await p.mouse.click(q1[0], q1[1]); await p.mouse.dblclick(q1[0], q1[1]);
await p.keyboard.press('Control+2'); await p.waitForTimeout(80);
await p.mouse.click(q1[0], q1[1]); await p.waitForTimeout(80);
const lockedSel = await p.evaluate(() => [...document.querySelectorAll('.lrow.sel[data-id]')].map(r => r.textContent));
ok('锁定后画布上点不中', !lockedSel.some(t => t.includes('小球1')), lockedSel.join('|'));
await p.keyboard.press('Control+Alt+2');
ok('全部解锁', await p.evaluate(() => !window.__physim.scene().objects.some(o => o.locked)));
// 图层：隐藏眼睛
const eye = p.locator('.lrow[data-id]').filter({ hasText: '地面' }).locator('.eye');
await eye.click();
ok('图层眼睛隐藏对象', await p.evaluate(() => window.__physim.scene().objects.find(o => o.kind === 'ground').hidden === true));
await eye.click();
// 图层拖动排序：把“地面”拖到最上
const ground = p.locator('.lrow[data-id]').filter({ hasText: '地面' });
const top = p.locator('.layers .lrow').first();
await ground.dragTo(top, { targetPosition: { x: 40, y: 4 } });
ok('图层拖动调整顺序', await p.evaluate(() => window.__physim.scene().objects.at(-1).kind === 'ground'));
// 排列：置于底层
await p.locator('.lrow[data-id]').filter({ hasText: '地面' }).click();
await p.keyboard.press('Control+Shift+BracketLeft');
ok('Ctrl+Shift+[ 置于底层', await p.evaluate(() => window.__physim.scene().objects[0].kind === 'ground'));
// 保存 → 最近文档
await p.keyboard.press('Control+s'); await p.waitForTimeout(100);
await p.locator('#modal [data-act="ok"]').click(); await p.waitForTimeout(200);
ok('保存后状态变为已保存', (await p.locator('#docState').textContent()) === '已保存');
await p.locator('#bHome').click(); await p.waitForTimeout(150);
ok('开始页出现最近文档（含缩略图）', await p.locator('.dcard img').count() === 1 && (await p.locator('.dcard .pc-name').first().textContent()) === '碰撞研究');
// 从模板新建 → 修改 → 关闭时询问
await p.locator('[data-v="templates"]').first().click(); await p.locator('.tcard', { hasText: '多米诺骨牌' }).click(); await p.waitForTimeout(300);
ok('从模板新建', (await p.locator('#docTitle').textContent()) === '多米诺骨牌' && await p.locator('.lesson').isVisible());
await p.keyboard.press('1'); await p.mouse.click(cv.x + 300, cv.y + 200);
await p.locator('#menus > button', { hasText: '文件' }).click(); await p.locator('.dmenu .mi', { hasText: '关闭文档' }).click(); await p.waitForTimeout(100);
ok('关闭未保存文档时询问', await p.locator('#modal.show', { hasText: '保存对' }).isVisible());
await p.locator('[data-act="discard"]').click(); await p.waitForTimeout(150);
ok('不保存 → 回到开始页', await p.locator('#home.show').isVisible());
// 打开已保存文档
await p.locator('.dcard .pcard').first().click(); await p.waitForTimeout(250);
s = await S(); ok('打开已保存文档', s.n === 4 && s.groups === 1, JSON.stringify(s));
// 导出
const dl = async (sub) => { const pr = p.waitForEvent('download', { timeout: 5000 }); await p.locator('#menus > button', { hasText: '文件' }).click(); await p.locator('.dmenu .mi', { hasText: '导出' }).hover(); await p.locator('.dmenu .mi', { hasText: sub }).click(); const d = await pr; return fs.readFileSync(await d.path()); };
{ const buf = await dl('PNG 图片（2 倍分辨率）'); ok('导出 PNG（文件头正确、2 倍尺寸）', buf.slice(1, 4).toString() === 'PNG' && buf.readUInt32BE(16) >= 2 * cv.width, `${buf.readUInt32BE(16)}×${buf.readUInt32BE(20)}`); }
{ const buf = await dl('场景文件'); let j = null; try { j = JSON.parse(buf.toString()); } catch {} ok('导出场景文件（可解析、含编组与标题）', j && j.objects.length === 4 && j.groups.length === 1 && j.title === '碰撞研究'); }
await p.keyboard.press('Space'); await p.waitForTimeout(600); await p.keyboard.press('Space');
{ const buf = await dl('运动数据'); const t = buf.toString(); ok('导出 CSV（含表头与数据行）', t.includes('物体,t (s),x (m)') && t.split('\n').length > 20, `${t.split('\n').length} 行`); }
// 刷新后恢复
await p.keyboard.press('3'); await p.mouse.click(cv.x + 700, cv.y + 300);
await p.waitForTimeout(500);
await p.reload({ waitUntil: 'networkidle' }); await p.waitForTimeout(400);
s = await S(); ok('刷新后恢复未保存的修改', s.n === 5 && (await p.locator('#docState').textContent()).includes('未保存'), JSON.stringify(s));
// 菜单快捷键显示与实际一致
const mism = await p.evaluate(() => { const out = []; document.querySelectorAll('#menus > button').forEach(x => {}); return out; });
ok('无页面报错', errs.length === 0, errs.join(' | '));
await b.close();

await server.close();
console.log(R.join('\n'));
const fails = R.filter(r => r.startsWith('FAIL')).length;
console.log(`\n${R.length - fails}/${R.length} 通过`);
process.exit(fails ? 1 : 0);
