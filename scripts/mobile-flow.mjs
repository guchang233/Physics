#!/usr/bin/env node
/**
 * 移动端交互回归（模拟触控，不是真机，也不是系统软键盘）。
 * 覆盖 360×800、390×844、412×915 与横屏。
 *   npm run test:mobile
 */
import { createServer } from 'vite';
import { chromium } from 'playwright';
import fs from 'node:fs';

const server = await createServer({ server: { port: 5188, host: '127.0.0.1', strictPort: false }, logLevel: 'error' });
await server.listen();
const url = server.resolvedUrls.local[0];
const fallback = process.env.CHROMIUM_PATH || ['/opt/pw-browsers/chromium'].find(p => fs.existsSync(p));
const launch = () => chromium.launch().catch(err => { if (!fallback) throw err; return chromium.launch({ executablePath: fallback, args: ['--no-sandbox'] }); });
const browser = await launch();
const R = [];
const ok = (n, c, x = '') => R.push(`${c ? 'PASS' : 'FAIL'}  ${n}${x ? '  — ' + x : ''}`);

function ptr(page, type, x, y, id = 1) {
  return page.evaluate(({ type, x, y, id }) => {
    const cv = document.getElementById('cv');
    cv.dispatchEvent(new PointerEvent(type, {
      bubbles: true, cancelable: true, pointerId: id, pointerType: 'touch', isPrimary: id === 1,
      clientX: x, clientY: y, button: 0, buttons: type === 'pointerup' || type === 'pointercancel' ? 0 : 1,
    }));
  }, { type, x, y, id });
}
const down = (page, x, y, id) => ptr(page, 'pointerdown', x, y, id);
const move = (page, x, y, id) => ptr(page, 'pointermove', x, y, id);
const up = (page, x, y, id) => ptr(page, 'pointerup', x, y, id);

async function worldPoint(page, kind, which = 0) {
  return page.evaluate(({ kind, which }) => {
    const P = window.__physim;
    const list = P.scene().objects.filter(o => o.kind === kind && o.st);
    const o = list[which] || list[0];
    const v = P.scene().view;
    const c = document.getElementById('cv').getBoundingClientRect();
    return {
      x: c.left + c.width / 2 + (o.st.x - v.cx) * v.scale,
      y: c.top + c.height / 2 - (o.st.y - v.cy) * v.scale,
      id: o.id, x0: o.x0, y0: o.y0, n: list.length,
    };
  }, { kind, which });
}

async function flow(width, height, tag) {
  const ctx = await browser.newContext({
    viewport: { width, height },
    hasTouch: true,
    isMobile: height >= width,
  });
  const page = await ctx.newPage();
  const errs = [];
  page.on('pageerror', e => errs.push(e.message));
  page.on('console', m => { if (m.type() === 'error') errs.push(m.text()); });
  await page.addInitScript(() => { try { localStorage.clear(); } catch { /* */ } });
  await page.goto(url, { waitUntil: 'networkidle' });
  await page.waitForSelector('#geombar');
  const pre = tag + ' ';

  ok(pre + '没有五入口底栏', await page.locator('#tabbar').count() === 0);
  ok(pre + '工具条与对象列表常驻', await page.locator('#geombar').isVisible() && await page.locator('#listDock').isVisible());
  ok(pre + '没有旧菜单栏', await page.locator('#menus, #menubar, #toolstrip').count() === 0);
  ok(pre + '实验库空状态可见', await page.locator('#home.show').isVisible() && await page.locator('.tcard').count() > 10);

  await page.locator('[data-tpl="projectile"]').click();
  await page.waitForTimeout(250);
  let n0 = await page.evaluate(() => window.__physim.scene().objects.length);
  ok(pre + '载入平抛后工具未武装', await page.evaluate(() => window.__physim.interaction().tool === 'select' && !window.__physim.interaction().arm));

  await page.locator('#geombar [data-geom="body"]').click();
  await page.locator('#toolPop [data-tool="ball"]').click();
  await page.waitForTimeout(150);
  ok(pre + '选中小球后出现取消条', await page.locator('#armBar').isVisible() && await page.evaluate(() => window.__physim.interaction().tool === 'ball'));

  const cv = await page.locator('#cv').boundingBox();
  const panX = cv.x + cv.width * 0.35, panY = cv.y + Math.min(80, cv.height * 0.25);
  await down(page, panX, panY);
  await move(page, panX + 36, panY + 8);
  await up(page, panX + 36, panY + 8);
  ok(pre + '武装后拖动只平移、不创建', await page.evaluate(n => window.__physim.scene().objects.length === n, n0));

  await page.locator('#armCancel').click();
  await page.waitForTimeout(80);
  ok(pre + '取消后不再武装', await page.evaluate(() => window.__physim.interaction().tool === 'select' && !window.__physim.interaction().arm));
  await down(page, panX, panY); await up(page, panX, panY);
  ok(pre + '取消后点画布不会创建', await page.evaluate(n => window.__physim.scene().objects.length === n, n0));

  await page.locator('#geombar [data-geom="body"]').click();
  await page.locator('#toolPop [data-tool="ball"]').click();
  await page.waitForTimeout(100);
  await page.evaluate(() => history.back());
  await page.waitForTimeout(120);
  ok(pre + '返回取消放置', await page.evaluate(() => window.__physim.interaction().tool === 'select'));
  await down(page, panX, panY + 30); await up(page, panX, panY + 30);
  ok(pre + '返回后点画布不会创建', await page.evaluate(n => window.__physim.scene().objects.length === n, n0));

  await page.locator('#geombar [data-geom="body"]').click();
  await page.locator('#toolPop [data-tool="ball"]').click();
  await page.waitForTimeout(80);
  const place = { x: cv.x + cv.width * 0.42, y: cv.y + cv.height * 0.42 };
  await down(page, place.x, place.y);
  await up(page, place.x, place.y);
  await page.waitForTimeout(150);
  const placed = await page.evaluate(() => {
    const P = window.__physim, balls = P.scene().objects.filter(o => o.kind === 'ball');
    return { n: P.scene().objects.length, balls: balls.length, tool: P.interaction().tool, sel: P.interaction().sel.length, sheet: document.body.dataset.sheet };
  });
  ok(pre + '点按落点并保持工具武装', placed.n === n0 + 1 && placed.tool === 'ball' && placed.sel === 1 && placed.sheet === 'canvas', JSON.stringify(placed));
  ok(pre + '紧凑参数与选中同步', await page.locator('#quick').isVisible() && await page.locator('#quick input[aria-label="名称"]').inputValue() !== '' && !(await page.locator('#propSheet').isVisible()));

  const ball = await worldPoint(page, 'ball', 1);
  const before = await page.evaluate(id => window.__physim.scene().objects.find(o => o.id === id).x0, ball.id);
  await down(page, ball.x, ball.y);
  await move(page, ball.x + 48, ball.y);
  await up(page, ball.x + 48, ball.y);
  await page.waitForTimeout(80);
  const after = await page.evaluate(id => window.__physim.scene().objects.find(o => o.id === id).x0, ball.id);
  ok(pre + '单指拖动改变位置', after > before + 0.2, `${before} → ${after}`);
  await page.locator('#bUndo').click();
  await page.waitForTimeout(80);
  const undone = await page.evaluate(id => window.__physim.scene().objects.find(o => o.id === id).x0, ball.id);
  ok(pre + '撤销恢复拖动', Math.abs(undone - before) < 1e-6, String(undone));

  const xPin = await page.evaluate(id => window.__physim.scene().objects.find(o => o.id === id).x0, ball.id);
  const p2 = await worldPoint(page, 'ball', 1);
  await down(page, p2.x, p2.y, 1);
  await move(page, p2.x + 28, p2.y, 1);
  await down(page, p2.x + 10, p2.y + 70, 2);
  await move(page, p2.x + 40, p2.y, 1);
  await move(page, p2.x + 30, p2.y + 110, 2);
  await up(page, p2.x + 40, p2.y, 1);
  await up(page, p2.x + 30, p2.y + 110, 2);
  await page.waitForTimeout(80);
  const xBack = await page.evaluate(id => window.__physim.scene().objects.find(o => o.id === id).x0, ball.id);
  const pinched = await page.evaluate(() => window.__physim.scene().view.scale);
  ok(pre + '拖动中加第二指不提交位移', Math.abs(xBack - xPin) < 1e-6, `${xPin} → ${xBack}`);
  ok(pre + '双指改变了缩放', pinched > 0);

  await page.evaluate(id => window.__physim.select(id), ball.id);
  await page.waitForTimeout(80);
  const mass = page.locator('#quick .row', { hasText: '质量' }).locator('input').first();
  const charge = page.locator('#quick .row', { hasText: '电荷量' }).locator('input').first();
  const v0 = page.locator('#quick .row', { hasText: '速率' }).locator('input').first();
  await mass.fill('2.5'); await mass.blur();
  await charge.fill('0.003'); await charge.blur();
  await v0.fill('4'); await v0.blur();
  await page.waitForTimeout(50);
  const edited = await page.evaluate(id => {
    const o = window.__physim.scene().objects.find(o => o.id === id);
    return { m: o.m, q: o.q, vx: o.vx0 };
  }, ball.id);
  ok(pre + '编辑 m / q / v0', edited.m === 2.5 && Math.abs(edited.q - 0.003) < 1e-9 && Math.abs(edited.vx - 4) < 1e-6, JSON.stringify(edited));

  await mass.focus();
  await page.evaluate(() => window.__physim.setKeyboardInset(280));
  await page.waitForTimeout(80);
  const ib = await mass.boundingBox();
  const kb = await page.evaluate(() => ({ h: window.innerHeight, kb: parseFloat(getComputedStyle(document.documentElement).getPropertyValue('--kb')) || 0, open: document.body.classList.contains('kb-open') }));
  ok(pre + '模拟键盘垫高后输入框仍在可视区', kb.open && ib && ib.y + ib.height <= kb.h - kb.kb + 8, JSON.stringify({ top: ib && ib.y, bottom: ib && ib.y + ib.height, ...kb }));
  await page.evaluate(() => window.__physim.setKeyboardInset(0));
  await mass.blur();

  await page.locator('#bPlay').click();
  await page.waitForTimeout(400);
  const running = await page.evaluate(() => ({ t: window.__physim.sim.t, run: window.__physim.sim.running }));
  ok(pre + '播放推进时间', running.run && running.t > 0.05, JSON.stringify(running));
  await page.locator('#bPlay').click();
  await page.locator('#bReset').click();
  await page.waitForTimeout(80);
  ok(pre + '复位回到 t = 0', await page.evaluate(() => window.__physim.sim.t === 0 && !window.__physim.sim.running));

  await page.locator('#geombar [data-geom="select"]').click();
  await page.locator('#bOverflow').click();
  await page.locator('#bOpenAnalysis').click();
  await page.waitForTimeout(250);
  const anText = await page.locator('#analysisBody').innerText();
  ok(pre + '分析识别抛体', /抛体|运动分析/.test(anText), anText.slice(0, 40));
  ok(pre + '分析页默认不铺满公式键盘', await page.evaluate(() => { const d = document.getElementById('desmosKeypadDock'); return !d || d.hidden; }));
  const scratch = page.locator('.an-scratch [data-mq="edit"], .an-scratch .mq-editable-field').first();
  if (await scratch.count()) {
    await scratch.click();
    await page.waitForTimeout(200);
    ok(pre + '聚焦公式后打开数学键盘', await page.evaluate(() => { const d = document.getElementById('desmosKeypadDock'); return !!(d && !d.hidden); }));
    await page.locator('#bOverlayClose').click();
    await page.waitForTimeout(100);
    ok(pre + '离开分析后收起数学键盘', await page.evaluate(() => { const d = document.getElementById('desmosKeypadDock'); return !d || d.hidden; }));
  } else ok(pre + '聚焦公式后打开数学键盘', false, '没有演算草稿');

  await page.evaluate(() => window.__physim.select(null));
  await page.locator('#bOverflow').click();
  await page.locator('#bOpenAnalysis').click();
  await page.waitForTimeout(150);
  ok(pre + '未选中时分析为空状态', /选中一个物体/.test(await page.locator('#analysisBody').innerText()));

  await page.locator('#bOverlayClose').click();
  await page.locator('#geombar [data-geom="select"]').click();
  const a = await worldPoint(page, 'ball', 0);
  await down(page, a.x, a.y); await up(page, a.x, a.y);
  await page.waitForTimeout(40);
  // 选中后紧凑参数出现，画布高度可能变化，第二下要按新位置算
  const b2 = await worldPoint(page, 'ball', 1);
  await down(page, b2.x, b2.y);
  await page.waitForTimeout(520);
  await up(page, b2.x, b2.y);
  await page.waitForTimeout(80);
  const multi = await page.evaluate(() => window.__physim.interaction().sel.length);
  ok(pre + '长按加入多选', multi >= 2, 'sel=' + multi);
  const nBeforeDel = await page.evaluate(() => window.__physim.scene().objects.length);
  if (multi >= 1) await page.locator('#ctxDelete').click();
  await page.waitForTimeout(80);
  ok(pre + '多选删除', await page.evaluate(n => window.__physim.scene().objects.length < n, nBeforeDel));
  await page.locator('#bUndo').click();
  await page.waitForTimeout(80);
  ok(pre + '撤销删除', await page.evaluate(n => window.__physim.scene().objects.length === n, nBeforeDel));

  await page.locator('#bOverflow').click();
  await page.locator('#bHome').click();
  await page.locator('[data-tpl="incline"]').click();
  await page.waitForTimeout(200);
  if (await page.locator('#modal.show').count()) {
    await page.locator('#modal [data-act="discard"]').click();
    await page.waitForTimeout(200);
  }
  const switched = await page.evaluate(() => {
    const P = window.__physim;
    return { box: P.scene().objects.some(o => o.kind === 'box'), tool: P.interaction().tool, arm: P.interaction().arm, sheet: document.body.dataset.sheet };
  });
  ok(pre + '换实验后状态干净', switched.box && switched.tool === 'select' && !switched.arm && switched.sheet === 'canvas', JSON.stringify(switched));

  await page.locator('#bOverflow').click();
  await page.locator('#bHome').click();
  await page.locator('[data-cmd="file.save"]').click();
  await page.waitForTimeout(150);
  ok(pre + '保存打开对话框', await page.locator('#modal.show').isVisible());
  const name = '手机回归-' + tag;
  await page.locator('#modal input[data-v]').fill(name);
  await page.locator('#modal [data-act="ok"]').click();
  await page.waitForTimeout(200);
  ok(pre + '文档已保存', (await page.locator('#docState').textContent()) === '已保存' && (await page.locator('#docTitle').textContent()) === name);
  await page.locator('#bOverlayClose').click();
  await page.locator('#bCollapse').click();
  ok(pre + '列表可收起', (await page.evaluate(() => document.body.dataset.list)) === 'closed' && await page.locator('#bListExpand').isVisible());
  await page.locator('#bListExpand').click();
  ok(pre + '列表可展开', await page.locator('#listDock').isVisible());
  await page.evaluate(() => window.__physim.addVariable('k', '1'));
  await page.locator('#leftTabs [data-left-tab="vars"]').click();
  await page.locator('.v-play').first().click();
  ok(pre + '滑块行可播放', await page.locator('.v-play.on').count() > 0);
  const colorBefore = await page.evaluate(() => window.__physim.scene().objects.find(o => o.kind === 'box').color);
  await page.locator('#leftTabs [data-left-tab="objects"]').click();
  await page.evaluate(() => {
    const box = window.__physim.scene().objects.find(o => o.kind === 'box');
    document.querySelector(`.lrow[data-id="${box.id}"] .lsw`).click();
  });
  ok(pre + '色点打开样式层', await page.locator('#stylePop').isVisible());
  await page.locator('#stylePop .swatch-btn').nth(2).click();
  const colorAfter = await page.evaluate(() => window.__physim.scene().objects.find(o => o.kind === 'box').color);
  ok(pre + '样式层改颜色', colorAfter && colorAfter !== colorBefore, `${colorBefore} → ${colorAfter}`);

  ok(pre + '无页面报错', errs.length === 0, errs.join(' | '));
  await ctx.close();
}

try {
  await flow(360, 800, '360x800');
  await flow(390, 844, '390x844');
  await flow(412, 915, '412x915');
  await flow(844, 390, '844x390横屏');
} catch (err) {
  ok('脚本未中断', false, err && err.message ? err.message.split('\n')[0] : String(err));
} finally {
  await browser.close();
  await server.close();
}
console.log(R.join('\n'));
const fails = R.filter(r => r.startsWith('FAIL')).length;
console.log(`\n${R.length - fails}/${R.length} 通过`);
process.exit(fails ? 1 : 0);
