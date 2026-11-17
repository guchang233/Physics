#!/usr/bin/env node
/**
 * 打包哔哩哔哩 Toy 上传包：
 * - 单文件 index.html（功能完整）
 * - icon.png / cover.jpg
 * - Physics-toy.zip（可直接上传）
 */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { spawnSync } from 'node:child_process';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const pub = path.join(root, 'publish');
const h5Dir = path.join(pub, 'h5');
const htmlSrc = path.join(h5Dir, 'index.html');
const htmlOut = path.join(pub, 'index.html');
const zipOut = path.join(pub, 'Physics-toy.zip');

if (!fs.existsSync(htmlSrc)) {
  console.error('missing', htmlSrc, '— run npm run build:h5 first');
  process.exit(1);
}

let html = fs.readFileSync(htmlSrc, 'utf8');

// 去掉未内联的 Symbola.svg 源（woff2/ttf 已在 CSS 里）
html = html.replace(/,?\s*url\(\.?\/?Symbola-[^)]+\.svg[^)]*\)\s*format\(["']svg["']\)/g, '');
html = html.replace(/url\(\.?\/?Symbola-[^)]+\.svg[^)]*\)/g, 'url()');

// favicon 内联为 data URI，保证单文件自包含
{
  const fav = fs.readFileSync(path.join(root, 'public/favicon.svg'));
  const data = `data:image/svg+xml;base64,${fav.toString('base64')}`;
  html = html
    .replace(/href="\/favicon\.svg"/g, `href="${data}"`)
    .replace(/href="\.\/favicon\.svg"/g, `href="${data}"`)
    .replace(/href="favicon\.svg"/g, `href="${data}"`);
}

fs.writeFileSync(htmlOut, html);

const readme = `# 物理模拟器 · 哔哩哔哩 Toy 发布包

## 上传文件
- \`index.html\`：单文件 H5（全部功能内联）
- 或直接上传 \`Physics-toy.zip\`

## 素材
- \`icon.png\`：Toy 图标 500×500（1:1）
- \`cover.jpg\`：封面 1200×900（4:3）

## 建议填写
- 名称：物理模拟器
- 页面地址：Physics
- 分类：工具
- 展示主题：深色

生成时间：${new Date().toISOString()}
`;
fs.writeFileSync(path.join(pub, 'README-toy.md'), readme);

if (fs.existsSync(zipOut)) fs.unlinkSync(zipOut);
const zip = spawnSync('zip', ['-j', zipOut, htmlOut, path.join(pub, 'icon.png'), path.join(pub, 'cover.jpg'), path.join(pub, 'README-toy.md')], {
  stdio: 'inherit',
});
if (zip.status !== 0) process.exit(zip.status || 1);

const size = fs.statSync(htmlOut).size;
const zsize = fs.statSync(zipOut).size;
console.log(JSON.stringify({
  html: htmlOut,
  htmlMB: +(size / 1024 / 1024).toFixed(2),
  zip: zipOut,
  zipMB: +(zsize / 1024 / 1024).toFixed(2),
  icon: path.join(pub, 'icon.png'),
  cover: path.join(pub, 'cover.jpg'),
}, null, 2));
