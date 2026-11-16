/**
 * 情境感知运动分析：根据选中物体当前受力/场环境，推导轨迹参数方程与特征量。
 * 与数值仿真解耦，只读场景状态。
 */

export type Vec2 = { x: number; y: number };

export type FieldSample = { Ex: number; Ey: number; Bz: number };

export type BodySnapshot = {
  id: number;
  name: string;
  kind: string;
  m: number;
  q: number;
  /** 参考时刻位置（分析原点） */
  x: number;
  y: number;
  vx: number;
  vy: number;
  /** 仿真当前时刻（用于标注） */
  t: number;
  fixed: boolean;
};

export type SpringInfo = {
  k: number;
  L0: number;
  rope: boolean;
  /** 另一端固定点或对面物体位置 */
  anchor: Vec2;
  otherBody: boolean;
};

export type AnalysisContext = {
  gOn: boolean;
  g: number;
  body: BodySnapshot;
  /** 参考点处的实际场抽样 */
  field: FieldSample;
  /**
   * 场景中的「模型场」（例如有界匀强 B 的标称值）。
   * 当参考点尚未进入场区时，仍可用它给出进入场区后的参数方程。
   */
  modelField?: FieldSample;
  modelFieldNote?: string;
  /** 恒定外力 */
  Fext: Vec2;
  springs: SpringInfo[];
  /** 是否用「当前状态」作为参数方程 t'=0；否则用场景初始条件 */
  refMode: 'initial' | 'current';
  /** 空气阻力系数（线性） */
  drag: number;
};

export type EquationBlock = {
  /** 符号形式 */
  symbolic: string;
  /** 代入数值后 */
  numeric: string;
};

export type DerivedQty = {
  name: string;
  formula: string;
  value: string;
};

export type MotionAnalysis = {
  regimeId: string;
  title: string;
  /** 一句话说明当前运动类型 */
  summary: string;
  assumptions: string[];
  /** 参数方程（以参考时刻为 τ=0） */
  parametric: EquationBlock[];
  /** 其它有用方程 */
  extras: EquationBlock[];
  derived: DerivedQty[];
  /** 适用范围 / 注意 */
  validity: string;
  /** 当前模型里计入的力 */
  forcesInModel: string[];
};

const DEG = Math.PI / 180;

export function fmt(v: number, d = 4): string {
  if (!isFinite(v)) return '—';
  const a = Math.abs(v);
  if (a < 1e-14) return '0';
  if (a !== 0 && (a < 1e-3 || a >= 1e5)) return v.toExponential(3);
  return (+v.toFixed(d)).toString();
}

function hypot(x: number, y: number) {
  return Math.hypot(x, y);
}

function phaseForm(cosAmp: number, sinAmp: number): { R: number; phi: number } {
  const R = Math.hypot(cosAmp, sinAmp);
  const phi = Math.atan2(sinAmp, cosAmp);
  return { R, phi };
}

/** 供画布叠加：按识别出的闭式解采样世界坐标点列 */
export function sampleAnalyticPath(
  ctx: AnalysisContext,
  opts: { duration?: number; steps?: number } = {},
): { regimeId: string; points: Vec2[] } | null {
  const report = analyzeMotion(ctx);
  const steps = opts.steps ?? 240;
  const b = ctx.body;
  const points: Vec2[] = [];

  if (report.regimeId === 'cyclotron') {
    const Bz = ctx.field.Bz;
    const ω = (b.q * Bz) / b.m;
    if (Math.abs(ω) < 1e-15) return null;
    const xc = b.x + b.vy / ω;
    const yc = b.y - b.vx / ω;
    const δx0 = b.x - xc;
    const δy0 = b.y - yc;
    const duration = opts.duration ?? (2 * Math.PI) / Math.abs(ω);
    for (let i = 0; i <= steps; i++) {
      const τ = (duration * i) / steps;
      const c = Math.cos(ω * τ);
      const s = Math.sin(ω * τ);
      points.push({ x: xc + δx0 * c + δy0 * s, y: yc + δy0 * c - δx0 * s });
    }
    return { regimeId: report.regimeId, points };
  }

  if (report.regimeId === 'crossed_eb') {
    const { Ex, Ey, Bz } = ctx.field;
    const ω = (b.q * Bz) / b.m;
    if (Math.abs(ω) < 1e-15) return null;
    const B2 = Bz * Bz;
    const vdx = (Ey * Bz) / B2;
    const vdy = (-Ex * Bz) / B2;
    const ux = b.vx - vdx;
    const uy = b.vy - vdy;
    const xc0 = b.x + uy / ω;
    const yc0 = b.y - ux / ω;
    const δx0 = b.x - xc0;
    const δy0 = b.y - yc0;
    const duration = opts.duration ?? (4 * Math.PI) / Math.abs(ω);
    for (let i = 0; i <= steps; i++) {
      const τ = (duration * i) / steps;
      const c = Math.cos(ω * τ);
      const s = Math.sin(ω * τ);
      points.push({
        x: xc0 + vdx * τ + δx0 * c + δy0 * s,
        y: yc0 + vdy * τ + δy0 * c - δx0 * s,
      });
    }
    return { regimeId: report.regimeId, points };
  }

  if (report.regimeId === 'uniform_e') {
    const ax = (b.q * ctx.field.Ex) / b.m;
    const ay = (b.q * ctx.field.Ey) / b.m;
    const duration = opts.duration ?? 2;
    for (let i = 0; i <= steps; i++) {
      const τ = (duration * i) / steps;
      points.push({
        x: b.x + b.vx * τ + 0.5 * ax * τ * τ,
        y: b.y + b.vy * τ + 0.5 * ay * τ * τ,
      });
    }
    return { regimeId: report.regimeId, points };
  }

  if (report.regimeId === 'projectile') {
    const g = ctx.g;
    const duration = opts.duration ?? Math.max(2, (b.vy + Math.sqrt(Math.max(0, b.vy * b.vy + 2 * g * Math.max(0, b.y)))) / g);
    for (let i = 0; i <= steps; i++) {
      const τ = (duration * i) / steps;
      points.push({
        x: b.x + b.vx * τ,
        y: b.y + b.vy * τ - 0.5 * g * τ * τ,
      });
    }
    return { regimeId: report.regimeId, points };
  }

  return null;
}

/**
 * 主入口：根据上下文分类并生成分析报告
 */
function effectiveField(ctx: AnalysisContext): { F: FieldSample; usedModel: boolean; note?: string } {
  const local = ctx.field;
  const localMag = Math.hypot(local.Ex, local.Ey) + Math.abs(local.Bz);
  if (localMag > 1e-15) return { F: local, usedModel: false };
  if (ctx.modelField) {
    const m = ctx.modelField;
    if (Math.hypot(m.Ex, m.Ey) + Math.abs(m.Bz) > 1e-15) {
      return { F: m, usedModel: true, note: ctx.modelFieldNote };
    }
  }
  return { F: local, usedModel: false };
}

export function analyzeMotion(ctx: AnalysisContext): MotionAnalysis {
  const { body: b, gOn, g, Fext, springs, drag } = ctx;
  const { F, usedModel, note: modelNote } = effectiveField(ctx);
  const forcesInModel: string[] = [];
  if (gOn) forcesInModel.push('重力 mg');
  if (b.q && (F.Ex || F.Ey)) forcesInModel.push('电场力 qE');
  if (b.q && F.Bz) forcesInModel.push('洛伦兹力 qv×B');
  if (Fext.x || Fext.y) forcesInModel.push('恒定外力 F');
  if (springs.length) forcesInModel.push(springs.some(s => s.rope) ? '绳/弹簧约束' : '弹簧力');
  if (drag) forcesInModel.push('线性阻力 −bv');

  const withModelNote = (report: MotionAnalysis): MotionAnalysis => {
    if (!usedModel || !modelNote) return report;
    return {
      ...report,
      assumptions: [modelNote, ...report.assumptions],
      validity: `${modelNote} ${report.validity}`,
      title: report.title + '（按场景场模型）',
    };
  };

  if (b.fixed) {
    return {
      regimeId: 'fixed',
      title: '固定点',
      summary: '物体被固定，不参与动力学积分。',
      assumptions: ['fixed = true'],
      parametric: [
        { symbolic: 'x(τ) = x₀', numeric: `x(τ) = ${fmt(b.x)}` },
        { symbolic: 'y(τ) = y₀', numeric: `y(τ) = ${fmt(b.y)}` },
      ],
      extras: [],
      derived: [],
      validity: '取消「固定不动」后才会运动。',
      forcesInModel,
    };
  }

  const E = hypot(F.Ex, F.Ey);
  const B = F.Bz;
  const v = hypot(b.vx, b.vy);
  const hasSpring = springs.length > 0;
  const hasDrag = drag > 0;
  const hasFext = !!(Fext.x || Fext.y);

  // 闭式解使用 effective field（含「场外但按场景模型」）
  const ctxF: AnalysisContext = { ...ctx, field: F };

  // —— 匀强 B、无 E、无 g（或 g 关）、无弹簧：圆周 ——
  if (
    b.q &&
    Math.abs(B) > 1e-15 &&
    E < 1e-15 &&
    !hasSpring &&
    !hasDrag &&
    !hasFext &&
    (!gOn || Math.abs(g) < 1e-15)
  ) {
    return withModelNote(analyzeCyclotron(ctxF, forcesInModel));
  }

  // —— 正交电磁场（E⊥B，g 关）：摆线 / 漂移 ——
  if (
    b.q &&
    Math.abs(B) > 1e-15 &&
    E > 1e-15 &&
    !hasSpring &&
    !hasDrag &&
    !hasFext &&
    (!gOn || Math.abs(g) < 1e-15)
  ) {
    return withModelNote(analyzeCrossedEB(ctxF, forcesInModel));
  }

  // —— 匀强 E、无 B、g 关：类平抛 ——
  if (
    b.q &&
    E > 1e-15 &&
    Math.abs(B) < 1e-15 &&
    !hasSpring &&
    !hasDrag &&
    !hasFext &&
    (!gOn || Math.abs(g) < 1e-15)
  ) {
    return withModelNote(analyzeUniformE(ctxF, forcesInModel));
  }

  // —— 重力投射（无电荷场/弹簧）——
  if (
    gOn &&
    Math.abs(g) > 1e-15 &&
    (!b.q || (E < 1e-15 && Math.abs(B) < 1e-15)) &&
    !hasSpring &&
    !hasDrag &&
    !hasFext
  ) {
    return analyzeProjectile(ctx, forcesInModel);
  }

  // —— 单弹簧水平/近似简谐 ——
  if (hasSpring && springs.length === 1 && !springs[0].rope && !hasDrag) {
    return analyzeSpring(ctx, forcesInModel);
  }

  // —— 轻绳单摆 ——
  if (hasSpring && springs.length === 1 && springs[0].rope && gOn) {
    return analyzePendulum(ctx, forcesInModel);
  }

  return analyzeGeneral(ctx, forcesInModel, v);
}

function analyzeCyclotron(ctx: AnalysisContext, forcesInModel: string[]): MotionAnalysis {
  const b = ctx.body;
  const Bz = ctx.field.Bz;
  const ω = (b.q * Bz) / b.m; // signed
  const v = hypot(b.vx, b.vy);
  const R = Math.abs(ω) > 1e-15 ? v / Math.abs(ω) : Infinity;
  const xc = b.x + b.vy / ω;
  const yc = b.y - b.vx / ω;
  const δx0 = b.x - xc;
  const δy0 = b.y - yc;
  const { phi } = phaseForm(δx0, δy0);
  const sense = ω > 0 ? '顺时针（本约定：ω=qB_z/m>0）' : '逆时针';
  const T = Math.abs(ω) > 1e-15 ? (2 * Math.PI) / Math.abs(ω) : Infinity;
  const refNote = ctx.refMode === 'current' ? `以当前时刻 t=${fmt(b.t, 3)}s 为 τ=0` : '以初始条件 t=0 为 τ=0';

  return {
    regimeId: 'cyclotron',
    title: '匀强磁场中的圆周运动',
    summary: `带电粒子在匀强 B 中做匀速圆周运动；回旋方向：${sense}。`,
    assumptions: [
      'B 沿 z（垂直纸面）且在粒子处均匀',
      '忽略重力与电场',
      '无阻力、无其它约束',
      refNote,
    ],
    parametric: [
      {
        symbolic: 'x(τ) = x_c + δx₀ cos(ωτ) + δy₀ sin(ωτ)',
        numeric: `x(τ) = ${fmt(xc)} + (${fmt(δx0)}) cos(ωτ) + (${fmt(δy0)}) sin(ωτ)`,
      },
      {
        symbolic: 'y(τ) = y_c + δy₀ cos(ωτ) − δx₀ sin(ωτ)',
        numeric: `y(τ) = ${fmt(yc)} + (${fmt(δy0)}) cos(ωτ) − (${fmt(δx0)}) sin(ωτ)`,
      },
      {
        symbolic: '亦作：x(τ)=x_c + R cos(ωτ+φ), y(τ)=y_c + R sin(ωτ+φ)（相位依初速）',
        numeric: `R=${fmt(R)} m，φ≈${fmt(phi / DEG, 2)}°，ω=${fmt(ω)} rad/s`,
      },
    ],
    extras: [
      {
        symbolic: 'ω = q B_z / m，  R = m|v| / |q B_z|，  T = 2π / |ω|',
        numeric: `ω = ${fmt(ω)} rad/s，R = ${fmt(R)} m，T = ${fmt(T)} s`,
      },
      {
        symbolic: '圆心 (x_c, y_c) = (x + v_y/ω, y − v_x/ω)',
        numeric: `(x_c, y_c) = (${fmt(xc)}, ${fmt(yc)}) m`,
      },
    ],
    derived: [
      { name: '回旋半径 R', formula: 'mv/|qB|', value: `${fmt(R)} m` },
      { name: '周期 T', formula: '2πm/|qB|', value: `${fmt(T)} s` },
      { name: '角频率 |ω|', formula: '|qB|/m', value: `${fmt(Math.abs(ω))} rad/s` },
      { name: '动能 Ek', formula: '½mv²', value: `${fmt(0.5 * b.m * v * v)} J（磁场不做功）` },
      { name: '比荷 q/m', formula: 'q/m', value: `${fmt(b.q / b.m)} C/kg` },
    ],
    validity: '粒子离开有界磁场区域后公式失效；若存在电场或重力，请看「正交电磁场」或一般数值解。',
    forcesInModel,
  };
}

function analyzeCrossedEB(ctx: AnalysisContext, forcesInModel: string[]): MotionAnalysis {
  const b = ctx.body;
  const { Ex, Ey, Bz } = ctx.field;
  const ω = (b.q * Bz) / b.m;
  // E×B drift: v_d = E×B / B²  with B=(0,0,Bz): E×B = (Ey Bz, -Ex Bz, 0)? 
  // E×B = |i     j     k|
  //       |Ex   Ey    0|
  //       |0     0   Bz| = i(Ey Bz) - j(Ex Bz) = (Ey Bz, -Ex Bz, 0)
  // v_d = E×B / B²
  const B2 = Bz * Bz;
  const vdx = (Ey * Bz) / B2;
  const vdy = (-Ex * Bz) / B2;
  const vd = hypot(vdx, vdy);
  // velocity in drift frame
  const ux = b.vx - vdx;
  const uy = b.vy - vdy;
  const u = hypot(ux, uy);
  const R = Math.abs(ω) > 1e-15 ? u / Math.abs(ω) : Infinity;
  // guiding center at ref time in lab: same cyclotron formula on (u)
  const xc = b.x + uy / ω;
  const yc = b.y - ux / ω;
  // In lab frame, guiding center drifts: x_c(τ)=xc0+vd τ
  const T = Math.abs(ω) > 1e-15 ? (2 * Math.PI) / Math.abs(ω) : Infinity;
  const E = hypot(Ex, Ey);

  return {
    regimeId: 'crossed_eb',
    title: '正交电磁场：漂移 + 回旋（摆线族）',
    summary:
      u < 1e-9
        ? '相对漂移系静止 → 实验室系匀速直线运动（v = E×B/B²）。'
        : Math.abs(u - vd) < 1e-6 && Math.abs(b.vx * vdx + b.vy * vdy) >= 0
          ? '一般情形：回旋叠加 E×B 漂移，轨迹为摆线/长短辐圆滚线。'
          : '回旋叠加 E×B 漂移；若由静止释放且 E⊥B，轨迹为摆线。',
    assumptions: [
      'E 在纸面内、B 垂直纸面（自动 E⊥B）',
      '场在粒子处均匀、忽略重力',
      ctx.refMode === 'current' ? `τ=0 取当前 t=${fmt(b.t, 3)}s` : 'τ=0 取初始条件',
    ],
    parametric: [
      {
        symbolic: 'v_d = E×B / B²  （漂移速度）',
        numeric: `v_d = (${fmt(vdx)}, ${fmt(vdy)}) m/s，|v_d|=${fmt(vd)}（=E/|B|=${fmt(E / Math.abs(Bz))}）`,
      },
      {
        symbolic: 'x(τ) = x_{c0} + v_{dx} τ + δx₀ cos(ωτ) + δy₀ sin(ωτ)',
        numeric: `x(τ) = ${fmt(xc)} + (${fmt(vdx)})τ + (${fmt(b.x - xc)}) cos(ωτ) + (${fmt(b.y - yc)}) sin(ωτ)`,
      },
      {
        symbolic: 'y(τ) = y_{c0} + v_{dy} τ + δy₀ cos(ωτ) − δx₀ sin(ωτ)',
        numeric: `y(τ) = ${fmt(yc)} + (${fmt(vdy)})τ + (${fmt(b.y - yc)}) cos(ωτ) − (${fmt(b.x - xc)}) sin(ωτ)`,
      },
      {
        symbolic: '漂移系：匀速圆周，R = m|v−v_d|/|qB|，ω = qB/m',
        numeric: `R = ${fmt(R)} m，ω = ${fmt(ω)} rad/s`,
      },
    ],
    extras: [
      {
        symbolic: '由静止释放：普通摆线，滚轮半径 R=mE/(qB²)，尖点间距 2πR',
        numeric:
          hypot(b.vx, b.vy) < 1e-9
            ? `R_摆 = ${fmt((b.m * E) / (Math.abs(b.q) * B2))} m，尖点距 ${fmt((2 * Math.PI * b.m * E) / (Math.abs(b.q) * B2))} m`
            : '（当前并非静止释放；上式仅作对照）',
      },
    ],
    derived: [
      { name: '漂移速度 |E×B|/B²', formula: 'E/|B|', value: `${fmt(vd)} m/s` },
      { name: '回旋半径（漂移系）', formula: 'm|v−vd|/|qB|', value: `${fmt(R)} m` },
      { name: '回旋周期', formula: '2πm/|qB|', value: `${fmt(T)} s` },
      { name: '速度选择器平衡', formula: 'v=E/B', value: `${fmt(E / Math.abs(Bz))} m/s（当 v∥且 F电+F洛=0）` },
    ],
    validity: '场区有界或存在重力时，解析式仅在均匀区内近似成立。',
    forcesInModel,
  };
}

function analyzeUniformE(ctx: AnalysisContext, forcesInModel: string[]): MotionAnalysis {
  const b = ctx.body;
  const { Ex, Ey } = ctx.field;
  const ax = (b.q * Ex) / b.m;
  const ay = (b.q * Ey) / b.m;
  const a = hypot(ax, ay);

  return {
    regimeId: 'uniform_e',
    title: '匀强电场中的匀加速（类平抛）',
    summary: '电场力恒定 ⇒ 加速度恒定，轨迹一般为抛物线。',
    assumptions: ['E 均匀', '无磁场、无重力（或已关闭）', '无阻力'],
    parametric: [
      {
        symbolic: 'x(τ) = x₀ + v_{x0} τ + ½ a_x τ²',
        numeric: `x(τ) = ${fmt(b.x)} + (${fmt(b.vx)})τ + ½(${fmt(ax)})τ²`,
      },
      {
        symbolic: 'y(τ) = y₀ + v_{y0} τ + ½ a_y τ²',
        numeric: `y(τ) = ${fmt(b.y)} + (${fmt(b.vy)})τ + ½(${fmt(ay)})τ²`,
      },
      {
        symbolic: 'a = qE / m',
        numeric: `a = (${fmt(ax)}, ${fmt(ay)}) m/s²，|a|=${fmt(a)}`,
      },
    ],
    extras: [
      {
        symbolic: '若 v₀ ⊥ a，等价于重力场中的平抛（g_eff = |a|）',
        numeric: `平抛类比 g_eff = ${fmt(a)} m/s²`,
      },
    ],
    derived: [
      { name: '加速度 |a|', formula: '|q|E/m', value: `${fmt(a)} m/s²` },
      { name: '比荷 q/m', formula: 'q/m', value: `${fmt(b.q / b.m)} C/kg` },
      {
        name: '动能变化',
        formula: 'ΔEk = qE·Δr',
        value: '电场力做功，动能不守恒',
      },
    ],
    validity: '离开电场区域后 a→0，改为匀速直线运动。',
    forcesInModel,
  };
}

function analyzeProjectile(ctx: AnalysisContext, forcesInModel: string[]): MotionAnalysis {
  const b = ctx.body;
  const g = ctx.g;
  const v = hypot(b.vx, b.vy);
  const ang = Math.atan2(b.vy, b.vx) / DEG;
  const H = b.vy > 0 ? (b.vy * b.vy) / (2 * g) : 0;
  const T_up = b.vy > 0 ? b.vy / g : 0;

  return {
    regimeId: 'projectile',
    title: '重力场中的抛体运动',
    summary: '仅受重力时，轨迹为抛物线（理想无阻力）。',
    assumptions: ['重力场均匀', '无电磁场有效作用', '无空气阻力', '着陆/碰撞前'],
    parametric: [
      {
        symbolic: 'x(τ) = x₀ + v_{x0} τ',
        numeric: `x(τ) = ${fmt(b.x)} + (${fmt(b.vx)})τ`,
      },
      {
        symbolic: 'y(τ) = y₀ + v_{y0} τ − ½ g τ²',
        numeric: `y(τ) = ${fmt(b.y)} + (${fmt(b.vy)})τ − ½(${fmt(g)})τ²`,
      },
    ],
    extras: [
      {
        symbolic: '消去 τ：y − y₀ = (v_{y0}/v_{x0})(x−x₀) − g(x−x₀)²/(2 v_{x0}²)  （v_x≠0）',
        numeric:
          Math.abs(b.vx) > 1e-12
            ? `y = ${fmt(b.y)} + (${fmt(b.vy / b.vx)})(x−${fmt(b.x)}) − (${fmt(g / (2 * b.vx * b.vx))})(x−${fmt(b.x)})²`
            : 'v_x=0：竖直抛体，x 恒定',
      },
    ],
    derived: [
      { name: '速率 |v|', formula: '|v|', value: `${fmt(v)} m/s` },
      { name: '方向角', formula: 'atan2(vy,vx)', value: `${fmt(ang, 2)}°` },
      { name: '相对上升高度', formula: 'v_y²/2g', value: b.vy > 0 ? `${fmt(H)} m` : '—' },
      { name: '到最高点时间', formula: 'v_y/g', value: b.vy > 0 ? `${fmt(T_up)} s` : '—' },
    ],
    validity: '触地、碰挡板或开启阻力后不再适用上述闭式解。',
    forcesInModel,
  };
}

function analyzeSpring(ctx: AnalysisContext, forcesInModel: string[]): MotionAnalysis {
  const b = ctx.body;
  const s = ctx.springs[0];
  const dx = b.x - s.anchor.x;
  const dy = b.y - s.anchor.y;
  const L = hypot(dx, dy);
  const ω0 = Math.sqrt(s.k / b.m);
  const T = (2 * Math.PI) / ω0;
  const gOn = ctx.gOn;

  return {
    regimeId: 'spring',
    title: '弹簧振子',
    summary: s.otherBody
      ? '两端皆为可动体：一般为二体耦合振荡（无简单单频闭式解）。'
      : gOn
        ? '固定端弹簧：竖直/斜向时平衡点含重力；小振动近似简谐。'
        : '固定端弹簧、无重力：沿连线方向为简谐振动（若约束在线上）。',
    assumptions: ['理想弹簧 F=−k(L−L₀) r̂', '本模拟为二维自由质点+弹簧，未强制单轴约束'],
    parametric: [
      {
        symbolic: '一维水平简谐（若限制在 x 轴）：x(τ)=A cos(ω₀τ+φ)，ω₀=√(k/m)',
        numeric: `ω₀ = ${fmt(ω0)} rad/s，T = ${fmt(T)} s，当前 L=${fmt(L)} m，L₀=${fmt(s.L0)} m`,
      },
      {
        symbolic: '二维自由端：需解 ẍ = −(k/m)(1−L₀/L)(x−x_a) 等耦合 ODE',
        numeric: '右侧图像为数值积分结果；解析参数方程仅在一维约束下严格成立',
      },
    ],
    extras: [],
    derived: [
      { name: '固有角频 ω₀', formula: '√(k/m)', value: `${fmt(ω0)} rad/s` },
      { name: '周期 T', formula: '2π√(m/k)', value: `${fmt(T)} s` },
      { name: '伸长量', formula: 'L−L₀', value: `${fmt(L - s.L0)} m` },
    ],
    validity: '二维无轨道约束时轨迹不一定是直线简谐；请结合数值轨迹理解。',
    forcesInModel,
  };
}

function analyzePendulum(ctx: AnalysisContext, forcesInModel: string[]): MotionAnalysis {
  const b = ctx.body;
  const s = ctx.springs[0];
  const dx = b.x - s.anchor.x;
  const dy = b.y - s.anchor.y;
  const L = hypot(dx, dy);
  const theta = Math.atan2(dx, -dy); // 相对竖直向下的偏角
  const ω0 = Math.sqrt(ctx.g / Math.max(s.L0, 1e-9));
  const T_small = (2 * Math.PI) / ω0;

  return {
    regimeId: 'pendulum',
    title: '单摆（轻绳）',
    summary: '轻绳约束 + 重力；大角度为椭圆积分，小角度近似简谐。',
    assumptions: ['绳不可伸长（本引擎用硬弹簧近似）', '固定悬点', '平面摆'],
    parametric: [
      {
        symbolic: 'θ̈ + (g/L) sinθ = 0',
        numeric: `当前 θ ≈ ${fmt(theta / DEG, 2)}°，L₀=${fmt(s.L0)} m`,
      },
      {
        symbolic: '小角度：θ(τ) ≈ Θ cos(ω₀τ+φ)，ω₀=√(g/L)',
        numeric: `ω₀ ≈ ${fmt(ω0)} rad/s，T≈${fmt(T_small)} s`,
      },
      {
        symbolic: 'x = x_a + L sinθ，  y = y_a − L cosθ',
        numeric: `(x_a,y_a)=(${fmt(s.anchor.x)}, ${fmt(s.anchor.y)})，当前 L=${fmt(L)} m`,
      },
    ],
    extras: [],
    derived: [
      { name: '小角度周期', formula: '2π√(L/g)', value: `${fmt(T_small)} s` },
      { name: '摆角 θ', formula: 'atan2(x−xa, ya−y)', value: `${fmt(theta / DEG, 2)}°` },
    ],
    validity: '绳松弛（T=0）后变为抛体；刚度不足时会有弹性拉伸误差。',
    forcesInModel,
  };
}

function analyzeGeneral(ctx: AnalysisContext, forcesInModel: string[], v: number): MotionAnalysis {
  const b = ctx.body;
  const F = ctx.field;
  const parts: string[] = [];
  if (ctx.gOn) parts.push('m(0, −g)');
  if (b.q) parts.push('qE + qv×B');
  if (ctx.Fext.x || ctx.Fext.y) parts.push('F_ext');
  if (ctx.springs.length) parts.push('Σ F_spring');
  if (ctx.drag) parts.push('−b v');

  return {
    regimeId: 'general',
    title: '一般受力：数值积分',
    summary: '当前力组合没有单一初等参数方程；运动由微分方程数值给出。',
    assumptions: ['下面写出的是微分方程，不是闭式轨迹'],
    parametric: [
      {
        symbolic: 'm a = Σ F',
        numeric: `m=${fmt(b.m)}，当前位置 (${fmt(b.x)}, ${fmt(b.y)})，v=(${fmt(b.vx)}, ${fmt(b.vy)})`,
      },
      {
        symbolic: parts.length ? `m(ẍ, ÿ) = ${parts.join(' + ')}` : 'm a = 0',
        numeric: `场抽样 E=(${fmt(F.Ex)}, ${fmt(F.Ey)})，B_z=${fmt(F.Bz)}，|v|=${fmt(v)}`,
      },
    ],
    extras: [
      {
        symbolic: '可尝试：关闭部分力 / 使场均匀全域，以落入圆周、平抛、摆线等可析情形',
        numeric: '例如仅保留匀强 B → 圆周参数方程；仅重力 → 抛体参数方程',
      },
    ],
    derived: [
      { name: '速率', formula: '|v|', value: `${fmt(v)} m/s` },
      { name: '动量', formula: 'm|v|', value: `${fmt(b.m * v)} kg·m/s` },
      { name: '动能', formula: '½mv²', value: `${fmt(0.5 * b.m * v * v)} J` },
    ],
    validity: '请用轨迹与 v–t / x–t 图像观察；解析式需满足更强对称性。',
    forcesInModel,
  };
}
