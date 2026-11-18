/**
 * 显示项（overlay）类型表。
 *
 * 画布上除了对象本身，其余所有“附加显示”——速度、加速度、受力、轨迹、预测、频闪、数值标签……
 * 都是文档里的一条显示项：scene.overlays = [{ id, type, target, ... }]。
 * 新文档里一条都没有（默认什么都不显示），需要什么就加什么，每条都能单独设置。
 *
 * target：'all' 全部物体 · 'sel' 当前选中的物体 · 数字 = 某个物体的 id
 * scale ：矢量箭头比例（px / 单位）；0 = 自动（让最长的箭头约 80 px）
 */

export const OVERLAY_TYPES = {
  // ---------- 矢量 ----------
  v:      { cat: 'vec', name: '速度', sym: 'v', unit: 'm/s', color: '#4a8a4f', scale: 20, desc: '瞬时速度' },
  a:      { cat: 'vec', name: '加速度', sym: 'a', unit: 'm/s²', color: '#c0603a', scale: 10, desc: '瞬时加速度（含接触力的效果）' },
  p:      { cat: 'vec', name: '动量', sym: 'p', unit: 'kg·m/s', color: '#3d6fa8', scale: 10, desc: 'p = mv' },
  Fnet:   { cat: 'vec', name: '合力', sym: 'F合', unit: 'N', color: '#7a4a9e', scale: 0, desc: 'F合 = ma（含支持力、摩擦力、约束力）' },
  forces: { cat: 'vec', name: '受力分析', sym: '', unit: 'N', color: '', scale: 0, desc: '逐个画出物体受到的每一个力' },
  F:      { cat: 'vec', name: '外力', sym: 'F', unit: 'N', color: '#8a6bbe', scale: 0, desc: '在属性里设置的外力' },
  vec:    { cat: 'vec', name: '自定义矢量', sym: 'u', unit: '', color: '#5e6b78', scale: 20, desc: '用公式写出 x、y 分量', custom: true },
  // ---------- 曲线 ----------
  trail:    { cat: 'curve', name: '已走轨迹', desc: '实线：物体实际走过的路径' },
  predict:  { cat: 'curve', name: '预测轨迹', desc: '虚线：暂停时从当前状态向前数值积分（与实际运行逐步一致）' },
  analytic: { cat: 'curve', name: '解析轨迹', desc: '点线：按运动类型给出的闭式解（遇到表面即截断）' },
  strobe:   { cat: 'curve', name: '频闪点', desc: '每隔 Δt 标出一次位置，像课本上的频闪照片' },
  // ---------- 标注 ----------
  names:  { cat: 'label', name: '名称', desc: '在物体下方标出名称' },
  value:  { cat: 'label', name: '数值标签', desc: '在物体旁显示一个公式的实时值', custom: true },
  ghost:  { cat: 'label', name: '初始位置', desc: '在 t = 0 的位置画出半透明的影子' },
  // ---------- 场 ----------
  fieldLines: { cat: 'field', name: '电场线', desc: '由电场与点电荷生成', noTarget: true },
};

export const OVERLAY_CATS = [
  ['vec', '矢量'],
  ['curve', '轨迹与曲线'],
  ['label', '标注'],
  ['field', '场'],
];

export function isVecType(type) { return OVERLAY_TYPES[type] && OVERLAY_TYPES[type].cat === 'vec'; }

/** 一条显示项的完整默认值 */
export function overlayDefaults(type) {
  const T = OVERLAY_TYPES[type] || {};
  const o = { type, on: true, target: T.noTarget ? null : 'all' };
  if (T.cat === 'vec') Object.assign(o, { color: '', scale: T.scale ?? 0, value: false, comps: false });
  if (type === 'vec') Object.assign(o, { label: 'u', ex: 'vx', ey: 'vy', unit: 'm/s' });
  if (type === 'trail') Object.assign(o, { color: '' });
  if (type === 'predict') Object.assign(o, { color: '', T: 3 });
  if (type === 'analytic') Object.assign(o, { color: '' });
  if (type === 'strobe') Object.assign(o, { dt: 0.1 });
  if (type === 'value') Object.assign(o, { label: 'v', expr: 'v', unit: 'm/s', digits: 4 });
  return o;
}

const NUMS = { scale: [0, 1e6], T: [0.05, 120], dt: [0.001, 100], digits: [1, 8] };

/** 读文档时清洗：未知类型丢弃，缺的字段补默认值，越界的数值夹回范围 */
export function normalizeOverlay(raw, nextId) {
  if (!raw || !OVERLAY_TYPES[raw.type]) return null;
  const d = overlayDefaults(raw.type), o = { id: Number.isFinite(+raw.id) && +raw.id > 0 ? +raw.id : nextId() };
  for (const k of Object.keys(d)) {
    let v = raw[k] === undefined ? d[k] : raw[k];
    if (typeof d[k] === 'boolean') v = !!v;
    else if (typeof d[k] === 'number') { v = +v; if (!Number.isFinite(v)) v = d[k]; const r = NUMS[k]; if (r) v = Math.min(r[1], Math.max(r[0], v)); }
    else if (k === 'target') v = v === 'all' || v === 'sel' ? v : (Number.isFinite(+v) && v !== null && v !== '' ? +v : d[k]);
    else if (typeof d[k] === 'string') v = String(v ?? '');
    o[k] = v;
  }
  return o;
}

/**
 * 旧版文档（v3 及以前）用一组全局开关控制显示；读入时转换成等效的显示项，保证旧文件打开后看起来一样。
 */
export function overlaysFromLegacy(settings) {
  const S = settings || {}, out = [];
  const add = (type, extra) => out.push(Object.assign(overlayDefaults(type), extra || {}));
  if (S.showLabels !== false) add('names');
  if (S.showTrail !== false) add('trail');
  if (S.showPredict !== false) add('predict', { T: +S.predictT > 0 ? +S.predictT : 3 });
  if (S.strobe > 0) add('strobe', { dt: +S.strobe });
  if (S.showVel !== false) add('v', { scale: +S.vPx > 0 ? +S.vPx : 20 });
  if (S.showForce) add('forces');
  if (S.showAnalytic !== false) add('analytic', { target: 'sel' });
  if (S.showFieldLines !== false) add('fieldLines');
  return out;
}
export const LEGACY_DISPLAY_KEYS = ['showLabels', 'showTrail', 'showPredict', 'predictT', 'strobe', 'showVel', 'showForce', 'showAnalytic', 'showFieldLines', '_strobeLast'];

/** 一键“推荐显示”：用于模板的教学卡片 */
export function recommendedOverlays(keys) {
  return (keys || []).map(k => typeof k === 'string' ? overlayDefaults(k) : Object.assign(overlayDefaults(k.type), k));
}
