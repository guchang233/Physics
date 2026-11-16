# 物理运动模拟器

浏览器端 2D 力学 / 电磁场交互模拟器。选中物体后，右侧 **运动分析 · 参数方程** 会根据当前场与受力自动识别运动类型（圆周、正交电磁场摆线、电场类平抛、重力抛体等），给出符号 + 数值形式的轨迹参数方程，并可在画布上叠加解析轨迹虚线。

## 开发

```bash
npm install
npm run dev
```

```bash
npm run build
npm run test:analysis
```

## 功能要点

- 搭建：质点 / 带电粒子 / 滑块、地面、挡板、斜面、匀强 E/B、**点电荷**、弹簧与轻绳
- 场引擎：`FieldSystem` 统一采样 E/B/φ（默认三维库仑 \(1/r^2\)）；RK4 电场线；可选 \(1/r\) 线电荷截面
- 接触：原生冲量求解；可选 Rapier2D 后端（`__physim.enableRapier()`）
- 分析：与选中物体运动状态绑定的参数方程；**分析**页公式以 LaTeX 书写，经 MathQuill 渲染（可复制 LaTeX，含演算草稿）

## 引擎分层

```
src/engine/fields   FieldSystem + PointCharge + 力线（EFSim 思路）
src/engine/rapier   Rapier 接触适配（刚体/碰撞，不做 EM）
src/app             UI 与编排
```
