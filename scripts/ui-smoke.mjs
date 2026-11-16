import { chromium, devices } from 'playwright';
import { mkdir } from 'fs/promises';

const out = '/opt/cursor/artifacts';
await mkdir(out, { recursive: true });

const browser = await chromium.launch();
const results = [];

async function desktop() {
  const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
  page.on('pageerror', e => results.push('pageerror: ' + e.message));
  await page.goto('http://127.0.0.1:5173/', { waitUntil: 'networkidle' });
  await page.waitForTimeout(500);
  // close help if open
  if (await page.locator('#help.show').count()) {
    await page.locator('#bHelpClose').click();
  }
  const brand = await page.locator('.brand').innerText();
  results.push('brand: ' + brand.replace(/\s+/g, ' '));
  await page.locator('#leftTabs button[data-left-tab="add"]').click();
  await page.waitForTimeout(200);
  const addCats = await page.locator('.add-cat').allTextContents();
  results.push('add-cats: ' + addCats.join('|'));
  await page.locator('#leftTabs button[data-left-tab="presets"]').click();
  await page.waitForTimeout(200);
  await page.locator('#qLeft').fill('摆线');
  await page.waitForTimeout(200);
  const presets = await page.locator('.preset-item .p-name').allTextContents();
  results.push('preset-search: ' + presets.join('|'));
  await page.locator('.preset-item').first().click();
  await page.waitForTimeout(400);
  // select particle via left objects
  await page.locator('#leftTabs button[data-left-tab="objects"]').click();
  await page.locator('#qLeft').fill('');
  await page.waitForTimeout(200);
  await page.locator('.objlist li').first().click();
  await page.waitForTimeout(300);
  const tabs = await page.locator('#props .tabs button').allTextContents();
  results.push('props-tabs: ' + tabs.join('|'));
  await page.locator('#props .tabs button[data-tab="analysis"]').click();
  await page.waitForTimeout(300);
  const title = await page.locator('.analysis-panel .an-title').innerText();
  results.push('analysis: ' + title);
  await page.screenshot({ path: `${out}/ui_desktop_analysis.png`, fullPage: false });
  await page.close();
}

async function mobile() {
  const iPhone = devices['iPhone 12'];
  const context = await browser.newContext({ ...iPhone });
  const page = await context.newPage();
  page.on('pageerror', e => results.push('mobile-pageerror: ' + e.message));
  await page.goto('http://127.0.0.1:5173/', { waitUntil: 'networkidle' });
  await page.waitForTimeout(500);
  if (await page.locator('#help.show').count()) await page.locator('#bHelpClose').click();
  const dock = await page.locator('#mobile-dock button').allTextContents();
  results.push('dock: ' + dock.map(t => t.replace(/\s+/g, '')).join('|'));
  await page.locator('#mobile-dock button[data-dock="objects"]').click();
  await page.waitForTimeout(300);
  const leftOpen = await page.locator('#left.open').count();
  results.push('left-open: ' + leftOpen);
  await page.screenshot({ path: `${out}/ui_mobile_objects.png` });
  await page.locator('#mobile-dock button[data-dock="inspect"]').click();
  await page.waitForTimeout(300);
  const rightOpen = await page.locator('#right.open').count();
  results.push('right-open: ' + rightOpen);
  await page.screenshot({ path: `${out}/ui_mobile_inspect.png` });
  await page.locator('#mobile-dock button[data-dock="stage"]').click();
  await page.waitForTimeout(200);
  results.push('closed: ' + (await page.locator('#left.open, #right.open').count()));
  await context.close();
}

try {
  await desktop();
  await mobile();
} finally {
  await browser.close();
}
console.log(results.join('\n'));
