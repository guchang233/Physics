# 物理模拟器 · 哔哩哔哩 Toy 发布包

## 直接上传

| 文件 | 用途 |
|------|------|
| `Physics-toy.zip` | 推荐：整包上传（含 HTML + 图标说明） |
| `index.html` | 单文件 H5（全部功能内联，约 5.2 MB，&lt; 50 MB） |
| `icon.png` | Toy 图标 · **500×500** · 1:1 |
| `cover.jpg` | 封面 · **1200×900** · 4:3 |

## 发布页建议填写

- **Toy 名称**：物理模拟器
- **页面地址**：`Physics` → `https://www.bilibili.com/toy/Physics`
- **分类**：工具
- **展示主题**：深色
- **谁可以看**：按需选择

## 本地重新打包

```bash
npm run pack:toy
```

会执行：`tsc` → 单文件 Vite 构建 → 生成 icon/cover → 写出 `publish/index.html` 与 `Physics-toy.zip`。
