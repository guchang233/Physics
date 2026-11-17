#!/usr/bin/env node
/**
 * 打包哔哩哔哩 Toy 上传包：
 * - publish/h5/ 下是拆分后的 index.html + assets（入口 HTML 远小于 1MB）
 * - publish/Physics-toy.zip 的根目录直接包含 index.html
 * - icon.png / cover.jpg 单独留给发布页上传
 */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { spawnSync } from 'node:child_process';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const pub = path.join(root, 'publish');
const h5Dir = path.join(pub, 'h5');
const htmlPath = path.join(h5Dir, 'index.html');
const zipPath = path.join(pub, 'Physics-toy.zip');
const LIMIT = 1024 * 1024; // Toy 入口 HTML 上限 1MB

if (!fs.existsSync(htmlPath)) {
  console.error('missing', htmlPath, '— run npm run build:toy first');
  process.exit(1);
}

const htmlSize = fs.statSync(htmlPath).size;
if (htmlSize >= LIMIT) {
  console.error(
    `index.html is ${(htmlSize / 1024 / 1024).toFixed(2)} MB, ` +
    `must be under 1 MB. Check that JS/CSS are not being inlined.`,
  );
  process.exit(1);
}

const html = fs.readFileSync(htmlPath, 'utf8');
if (/(?:src|href)="\/(?!\/)/.test(html)) {
  console.error('index.html still contains root-absolute asset URLs; base must be "./"');
  process.exit(1);
}

if (fs.existsSync(zipPath)) fs.unlinkSync(zipPath);
const zip = spawnSync('zip', ['-r', '-q', zipPath, '.'], { cwd: h5Dir, stdio: 'inherit' });
if (zip.status !== 0) process.exit(zip.status || 1);

function kb(bytes) {
  return `${(bytes / 1024).toFixed(1)} KB`;
}

const files = [];
const walk = (dir) => {
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) walk(full);
    else files.push({ name: path.relative(h5Dir, full), size: fs.statSync(full).size });
  }
};
walk(h5Dir);
files.sort((a, b) => b.size - a.size);

console.log(JSON.stringify({
  html: htmlPath,
  htmlSize: kb(htmlSize),
  htmlLimit: '1 MB',
  zip: zipPath,
  zipSize: kb(fs.statSync(zipPath).size),
  assets: files,
}, null, 2));
