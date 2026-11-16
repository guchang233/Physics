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

export type PointChargeInfo = {
  id: number | string;
  x: number;
  y: number;
  q: number;
  /** 是否固定（可作中心力心） */
  fixed?: boolean;
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
  /** 场景中的点电荷源（含「带电体发射」），供非匀强库仑分析 */
  pointCharges?: PointChargeInfo[];
  /** Coulomb 力律与常量（默认 1/r²） */
  coulombLaw?: 'coulomb3d' | 'coulomb2d';
  kCoulomb?: number;
  /** 是否存在匀强 E 源（有则优先走匀强 E 闭式，而非局部抽样） */
  hasUniformE?: boolean;
  hasUniformB?: boolean;
};

export type EquationBlock = {
  /**
   * 符号形式（MathQuill / LaTeX）。
   * 纯中文说明请放 note，勿塞进 latex。
   */
  symbolic: string;
  /** 代入数值后的 LaTeX */
  numeric: string;
  /** 可选中文旁注（渲染在公式外） */
  note?: string;
};

export type DerivedQty = {
  /** 中文名称 */
  name: string;
  /** 公式 LaTeX */
  formula: string;
  /** 数值 LaTeX（可含单位 \\text{ m}） */
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

/** Format a number as MathQuill-friendly LaTeX (scientific → a\\times10^{b}). */
export function fmtTex(v: number, d = 4): string {
  if (!isFinite(v)) return '\\text{—}';
  const a = Math.abs(v);
  if (a < 1e-14) return '0';
  if (a !== 0 && (a < 1e-3 || a >= 1e5)) {
    const exp = Number(v).toExponential(3);
    const [mant, e] = exp.split('e');
    const expNum = Number(e);
    return `${mant}\\times10^{${expNum}}`;
  }
  return String(+v.toFixed(d));
}

export function unitTex(u: string) {
  return `\\text{ ${u}}`;
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

  // 近似圆轨道：绕固定点电荷匀速圆周
  if (report.regimeId === 'coulomb_circular') {
    const center = dominantFixedCharge(ctx);
    if (!center) return null;
    const rx = b.x - center.x;
    const ry = b.y - center.y;
    const r = hypot(rx, ry);
    if (r < 1e-12) return null;
    const Lz = b.m * (rx * b.vy - ry * b.vx);
    const ω = Lz / (b.m * r * r);
    if (Math.abs(ω) < 1e-15) return null;
    const duration = opts.duration ?? (2 * Math.PI) / Math.abs(ω);
    for (let i = 0; i <= steps; i++) {
      const τ = (duration * i) / steps;
      const c = Math.cos(ω * τ);
      const s = Math.sin(ω * τ);
      points.push({
        x: center.x + rx * c - ry * s,
        y: center.y + rx * s + ry * c,
      });
    }
    return { regimeId: report.regimeId, points };
  }

  return null;
}

function dominantFixedCharge(ctx: AnalysisContext): PointChargeInfo | null {
  const list = (ctx.pointCharges || []).filter(c => c.id !== ctx.body.id && c.q && (c.fixed !== false));
  if (!list.length) return null;
  // Prefer truly fixed; else nearest
  const fixed = list.filter(c => c.fixed);
  const pool = fixed.length ? fixed : list;
  let best = pool[0];
  let bestD = Infinity;
  for (const c of pool) {
    const d = hypot(ctx.body.x - c.x, ctx.body.y - c.y);
    if (d < bestD) {
      bestD = d;
      best = c;
    }
  }
  return best;
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
        { symbolic: 'x(\\tau)=x_0', numeric: `x(\\tau)=${fmtTex(b.x)}` },
        { symbolic: 'y(\\tau)=y_0', numeric: `y(\\tau)=${fmtTex(b.y)}` },
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
  const pointCharges = (ctx.pointCharges || []).filter(c => c.id !== b.id && Math.abs(c.q) > 0);
  const hasPointCharges = pointCharges.length > 0;
  const hasUniformE = !!ctx.hasUniformE;
  const hasUniformB = !!ctx.hasUniformB;

  // 闭式解使用 effective field（含「场外但按场景模型」）
  const ctxF: AnalysisContext = { ...ctx, field: F };

  // —— 非匀强库仑场（点电荷）：优先于「局部 E 当作匀强」——
  if (
    b.q &&
    hasPointCharges &&
    !hasUniformB &&
    Math.abs(B) < 1e-15 &&
    !hasSpring &&
    !hasDrag &&
    !hasFext &&
    (!gOn || Math.abs(g) < 1e-15)
  ) {
    if (!hasUniformE) {
      return analyzeCentralCoulomb(ctxF, forcesInModel, pointCharges);
    }
  }

  // —— 匀强 B、无 E、无 g（或 g 关）、无弹簧：圆周 ——
  if (
    b.q &&
    Math.abs(B) > 1e-15 &&
    E < 1e-15 &&
    !hasPointCharges &&
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
    !hasPointCharges &&
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
    (hasUniformE || !hasPointCharges) &&
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
  const ω = (b.q * Bz) / b.m;
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
        symbolic: 'x(\\tau)=x_c+\\delta x_0\\cos(\\omega\\tau)+\\delta y_0\\sin(\\omega\\tau)',
        numeric: `x(\\tau)=${fmtTex(xc)}+(${fmtTex(δx0)})\\cos(\\omega\\tau)+(${fmtTex(δy0)})\\sin(\\omega\\tau)`,
      },
      {
        symbolic: 'y(\\tau)=y_c+\\delta y_0\\cos(\\omega\\tau)-\\delta x_0\\sin(\\omega\\tau)',
        numeric: `y(\\tau)=${fmtTex(yc)}+(${fmtTex(δy0)})\\cos(\\omega\\tau)-(${fmtTex(δx0)})\\sin(\\omega\\tau)`,
      },
      {
        symbolic: 'x(\\tau)=x_c+R\\cos(\\omega\\tau+\\phi),\\quad y(\\tau)=y_c+R\\sin(\\omega\\tau+\\phi)',
        numeric: `R=${fmtTex(R)}${unitTex('m')},\\ \\phi\\approx${fmtTex(phi / DEG, 2)}^\\circ,\\ \\omega=${fmtTex(ω)}${unitTex('rad/s')}`,
        note: '相位依初速',
      },
    ],
    extras: [
      {
        symbolic: '\\omega=\\frac{qB_z}{m},\\quad R=\\frac{m|v|}{|qB_z|},\\quad T=\\frac{2\\pi}{|\\omega|}',
        numeric: `\\omega=${fmtTex(ω)}${unitTex('rad/s')},\\ R=${fmtTex(R)}${unitTex('m')},\\ T=${fmtTex(T)}${unitTex('s')}`,
      },
      {
        symbolic: '(x_c,y_c)=\\left(x+\\frac{v_y}{\\omega},\\ y-\\frac{v_x}{\\omega}\\right)',
        numeric: `(x_c,y_c)=(${fmtTex(xc)},\\ ${fmtTex(yc)})${unitTex('m')}`,
      },
    ],
    derived: [
      { name: '回旋半径 R', formula: 'R=\\frac{mv}{|qB|}', value: `${fmtTex(R)}${unitTex('m')}` },
      { name: '周期 T', formula: 'T=\\frac{2\\pi m}{|qB|}', value: `${fmtTex(T)}${unitTex('s')}` },
      { name: '角频率 |ω|', formula: '|\\omega|=\\frac{|qB|}{m}', value: `${fmtTex(Math.abs(ω))}${unitTex('rad/s')}` },
      { name: '动能 Ek', formula: 'E_k=\\frac{1}{2}mv^2', value: `${fmtTex(0.5 * b.m * v * v)}${unitTex('J')}`, },
      { name: '比荷 q/m', formula: '\\frac{q}{m}', value: `${fmtTex(b.q / b.m)}${unitTex('C/kg')}` },
    ],
    validity: '粒子离开有界磁场区域后公式失效；若存在电场或重力，请看「正交电磁场」或一般数值解。',
    forcesInModel,
  };
}

function analyzeCrossedEB(ctx: AnalysisContext, forcesInModel: string[]): MotionAnalysis {
  const b = ctx.body;
  const { Ex, Ey, Bz } = ctx.field;
  const ω = (b.q * Bz) / b.m;
  const B2 = Bz * Bz;
  const vdx = (Ey * Bz) / B2;
  const vdy = (-Ex * Bz) / B2;
  const vd = hypot(vdx, vdy);
  const ux = b.vx - vdx;
  const uy = b.vy - vdy;
  const u = hypot(ux, uy);
  const R = Math.abs(ω) > 1e-15 ? u / Math.abs(ω) : Infinity;
  const xc = b.x + uy / ω;
  const yc = b.y - ux / ω;
  const T = Math.abs(ω) > 1e-15 ? (2 * Math.PI) / Math.abs(ω) : Infinity;
  const E = hypot(Ex, Ey);

  return {
    regimeId: 'crossed_eb',
    title: '正交电磁场：漂移 + 回旋（摆线族）',
    summary:
      u < 1e-9
        ? '相对漂移系静止 → 实验室系匀速直线运动（v = E×B/B²）。'
        : '回旋叠加 E×B 漂移；若由静止释放且 E⊥B，轨迹为摆线。',
    assumptions: [
      'E 在纸面内、B 垂直纸面（自动 E⊥B）',
      '场在粒子处均匀、忽略重力',
      ctx.refMode === 'current' ? `τ=0 取当前 t=${fmt(b.t, 3)}s` : 'τ=0 取初始条件',
    ],
    parametric: [
      {
        symbolic: '\\vec{v}_d=\\frac{\\vec{E}\\times\\vec{B}}{B^2}',
        numeric: `\\vec{v}_d=(${fmtTex(vdx)},\\ ${fmtTex(vdy)})${unitTex('m/s')},\\ |\\vec{v}_d|=${fmtTex(vd)}=\\frac{E}{|B|}=${fmtTex(E / Math.abs(Bz))}`,
        note: '漂移速度',
      },
      {
        symbolic: 'x(\\tau)=x_{c0}+v_{dx}\\tau+\\delta x_0\\cos(\\omega\\tau)+\\delta y_0\\sin(\\omega\\tau)',
        numeric: `x(\\tau)=${fmtTex(xc)}+(${fmtTex(vdx)})\\tau+(${fmtTex(b.x - xc)})\\cos(\\omega\\tau)+(${fmtTex(b.y - yc)})\\sin(\\omega\\tau)`,
      },
      {
        symbolic: 'y(\\tau)=y_{c0}+v_{dy}\\tau+\\delta y_0\\cos(\\omega\\tau)-\\delta x_0\\sin(\\omega\\tau)',
        numeric: `y(\\tau)=${fmtTex(yc)}+(${fmtTex(vdy)})\\tau+(${fmtTex(b.y - yc)})\\cos(\\omega\\tau)-(${fmtTex(b.x - xc)})\\sin(\\omega\\tau)`,
      },
      {
        symbolic: 'R=\\frac{m|\\vec{v}-\\vec{v}_d|}{|qB|},\\quad\\omega=\\frac{qB}{m}',
        numeric: `R=${fmtTex(R)}${unitTex('m')},\\ \\omega=${fmtTex(ω)}${unitTex('rad/s')}`,
        note: '漂移系匀速圆周',
      },
    ],
    extras: [
      {
        symbolic: 'R_{\\mathrm{cyc}}=\\frac{mE}{|q|B^2},\\quad\\Delta=2\\pi R_{\\mathrm{cyc}}',
        numeric:
          hypot(b.vx, b.vy) < 1e-9
            ? `R_{\\mathrm{cyc}}=${fmtTex((b.m * E) / (Math.abs(b.q) * B2))}${unitTex('m')},\\ \\Delta=${fmtTex((2 * Math.PI * b.m * E) / (Math.abs(b.q) * B2))}${unitTex('m')}`
            : '\\text{(not released from rest)}',
        note: '由静止释放时的普通摆线',
      },
    ],
    derived: [
      { name: '漂移速度 |E×B|/B²', formula: '|\\vec{v}_d|=\\frac{E}{|B|}', value: `${fmtTex(vd)}${unitTex('m/s')}` },
      { name: '回旋半径（漂移系）', formula: 'R=\\frac{m|\\vec{v}-\\vec{v}_d|}{|qB|}', value: `${fmtTex(R)}${unitTex('m')}` },
      { name: '回旋周期', formula: 'T=\\frac{2\\pi m}{|qB|}', value: `${fmtTex(T)}${unitTex('s')}` },
      { name: '速度选择器平衡', formula: 'v=\\frac{E}{B}', value: `${fmtTex(E / Math.abs(Bz))}${unitTex('m/s')}` },
    ],
    validity: '场区有界或存在重力时，解析式仅在均匀区内近似成立。',
    forcesInModel,
  };
}

function analyzeCentralCoulomb(
  ctx: AnalysisContext,
  forcesInModel: string[],
  charges: PointChargeInfo[],
): MotionAnalysis {
  const b = ctx.body;
  const law = ctx.coulombLaw || 'coulomb3d';
  const k = ctx.kCoulomb ?? 8.99e9;
  const refNote = ctx.refMode === 'current' ? `以当前时刻 t=${fmt(b.t, 3)}s 为 τ=0` : '以初始条件 t=0 为 τ=0';
  const forces = forcesInModel.includes('电场力 qE')
    ? forcesInModel
    : [...forcesInModel, '库仑力（非匀强）'];

  const fixed = charges.filter(c => c.fixed);
  const center = fixed.length === 1
    ? fixed[0]
    : (fixed.length ? dominantFixedCharge({ ...ctx, pointCharges: fixed }) : null);

  if (center && (fixed.length === 1 || charges.length === 1)) {
    const Q = center.q;
    const α = k * b.q * Q;
    const rx = b.x - center.x;
    const ry = b.y - center.y;
    const r = hypot(rx, ry);
    const vr = r > 1e-15 ? (b.vx * rx + b.vy * ry) / r : 0;
    const Lz = b.m * (rx * b.vy - ry * b.vx);
    const L = Math.abs(Lz);
    const v2 = b.vx * b.vx + b.vy * b.vy;
    const U = law === 'coulomb2d'
      ? -2 * k * b.q * Q * Math.log(Math.max(r, 1e-12))
      : α / Math.max(r, 1e-12);
    const Ek = 0.5 * b.m * v2;
    const Etot = Ek + U;
    const attract = α < 0;

    let circular = false;
    let ωCirc = 0;
    let rCirc = r;
    if (law === 'coulomb3d' && attract && r > 1e-12) {
      const vθ = L / (b.m * r);
      const aNeed = (Math.abs(α) / (b.m * r * r));
      const aHave = (vθ * vθ) / r;
      if (Math.abs(vr) < 0.05 * Math.max(Math.sqrt(v2), 1e-6) && Math.abs(aHave - aNeed) / aNeed < 0.08) {
        circular = true;
        ωCirc = Lz / (b.m * r * r);
        rCirc = r;
      }
    }

    let ecc = NaN;
    let orbitKind = '一般有心力轨道';
    if (law === 'coulomb3d' && Math.abs(α) > 1e-30 && L > 1e-30) {
      const disc = 1 + (2 * Etot * L * L) / (b.m * α * α);
      ecc = Math.sqrt(Math.max(0, disc));
      if (attract) {
        if (ecc < 1e-3) orbitKind = '近圆轨道';
        else if (ecc < 1) orbitKind = '椭圆（束缚）';
        else if (Math.abs(ecc - 1) < 1e-3) orbitKind = '抛物线（E≈0）';
        else orbitKind = '双曲线（散射）';
      } else {
        orbitKind = '同号排斥 · 双曲散射';
      }
    } else if (law === 'coulomb2d') {
      orbitKind = '二维/线电荷势（对数）· 无初等圆锥曲线分类';
    }

    if (circular) {
      const T = (2 * Math.PI) / Math.abs(ωCirc);
      return {
        regimeId: 'coulomb_circular',
        title: '库仑中心力 · 圆轨道',
        summary: `试探电荷绕固定点电荷做近似匀速圆周运动（${orbitKind}）。`,
        assumptions: [
          '中心电荷固定',
          law === 'coulomb3d' ? '三维点电荷 Coulomb 1/r²' : '二维 1/r 力律',
          '忽略重力与磁场',
          refNote,
        ],
        parametric: [
          {
            symbolic: 'x(\\tau)=x_c+r\\cos(\\omega\\tau+\\phi),\\quad y(\\tau)=y_c+r\\sin(\\omega\\tau+\\phi)',
            numeric: `(x_c,y_c)=(${fmtTex(center.x)},\\ ${fmtTex(center.y)}),\\ r=${fmtTex(rCirc)},\\ \\omega=${fmtTex(ωCirc)}${unitTex('rad/s')}`,
          },
          {
            symbolic: '\\omega^2=\\frac{|kqQ|}{mr^3}',
            numeric: `|\\omega|=${fmtTex(Math.abs(ωCirc))},\\ T=${fmtTex(T)}${unitTex('s')}`,
            note: '向心力 = 库仑力',
          },
        ],
        extras: [
          {
            symbolic: 'v=\\sqrt{\\frac{|kqQ|}{mr}},\\quad L=mvr',
            numeric: `|v|=${fmtTex(Math.sqrt(v2))}${unitTex('m/s')},\\ L_z=${fmtTex(Lz)}${unitTex('kg·m²/s')}`,
          },
        ],
        derived: [
          { name: '轨道半径 r', formula: 'r', value: `${fmtTex(rCirc)}${unitTex('m')}` },
          { name: '周期 T', formula: 'T=\\frac{2\\pi}{|\\omega|}', value: `${fmtTex(T)}${unitTex('s')}` },
          { name: '机械能 E', formula: 'E=\\frac{1}{2}mv^2+\\frac{kqQ}{r}', value: `${fmtTex(Etot)}${unitTex('J')}` },
          { name: '角动量 L_z', formula: 'L_z=m(xv_y-yv_x)', value: `${fmtTex(Lz)}${unitTex('kg·m²/s')}` },
        ],
        validity: '中心若可动或存在其它电荷，圆轨道条件被破坏；软核半径内力被截断。',
        forcesInModel: forces,
      };
    }

    const Fr = law === 'coulomb3d' ? '\\frac{kqQ}{r^2}' : '\\frac{2kqQ}{r}';
    const phiLaw = law === 'coulomb3d'
      ? '\\varphi=\\frac{kQ}{r},\\quad U=q\\varphi=\\frac{kqQ}{r}'
      : '\\varphi=-2kQ\\ln r,\\quad U=q\\varphi';
    return {
      regimeId: 'coulomb_central',
      title: '库仑中心力（固定点电荷）',
      summary: `单固定源下的二维有心力问题：${orbitKind}。能量与角动量守恒；轨迹一般非初等 τ 参数式。`,
      assumptions: [
        '中心电荷固定、其余电荷可忽略',
        law === 'coulomb3d' ? 'F = k q Q / r² r̂' : 'F ∝ 1/r（二维截面）',
        '平面运动、无耗散',
        refNote,
      ],
      parametric: [
        {
          symbolic: `m\\ddot{r}=\\frac{L^2}{mr^3}+F_r(r),\\quad F_r=${Fr}`,
          numeric: `r=${fmtTex(r)}${unitTex('m')},\\ v_r=${fmtTex(vr)},\\ L_z=${fmtTex(Lz)}`,
          note: `相对心 (${fmt(center.x)}, ${fmt(center.y)})`,
        },
        {
          symbolic: '\\dot{\\theta}=\\frac{L_z}{mr^2}',
          numeric: `\\dot{\\theta}=${fmtTex(Lz / (b.m * Math.max(r * r, 1e-18)))}${unitTex('rad/s')}`,
        },
        {
          symbolic: law === 'coulomb3d'
            ? 'r=\\frac{L^2/(m|\\alpha|)}{1+e\\cos\\theta}'
            : '\\text{no elementary conic for log potential}',
          numeric: Number.isFinite(ecc) ? `e\\approx${fmtTex(ecc, 4)},\\ \\alpha=kqQ=${fmtTex(α)}` : `\\alpha=${fmtTex(α)}`,
          note: law === 'coulomb3d' ? '吸引时 α=kqQ<0' : undefined,
        },
      ],
      extras: [
        { symbolic: phiLaw, numeric: `U=${fmtTex(U)}${unitTex('J')},\\ E_k=${fmtTex(Ek)}${unitTex('J')},\\ E=${fmtTex(Etot)}${unitTex('J')}` },
        {
          symbolic: 'E=\\frac{1}{2}m\\dot{r}^2+\\frac{L^2}{2mr^2}+U(r),\\quad L_z=\\mathrm{const}',
          numeric: '\\text{numeric trajectory; analytic overlay when near-circular}',
          note: '守恒量',
        },
      ],
      derived: [
        { name: '势能 U', formula: law === 'coulomb3d' ? 'U=\\frac{kqQ}{r}' : 'U=-2kqQ\\ln r', value: `${fmtTex(U)}${unitTex('J')}` },
        { name: '机械能 E', formula: 'E=E_k+U', value: `${fmtTex(Etot)}${unitTex('J')}` },
        { name: '角动量 |L|', formula: '|\\vec{r}\\times\\vec{p}|', value: `${fmtTex(L)}${unitTex('kg·m²/s')}` },
        { name: '偏心率 e', formula: 'e=\\sqrt{1+\\frac{2EL^2}{m\\alpha^2}}', value: Number.isFinite(ecc) ? fmtTex(ecc, 4) : '\\text{—}' },
        { name: `轨道类型 · ${orbitKind}`, formula: 'e,\\ \\mathrm{sign}(\\alpha)', value: Number.isFinite(ecc) ? fmtTex(ecc, 4) : '\\text{—}' },
        { name: '比荷 q/m', formula: '\\frac{q}{m}', value: `${fmtTex(b.q / b.m)}${unitTex('C/kg')}` },
      ],
      validity: '多电荷、可动源或软核截断时，上式仅为以最近固定源为心的近似。',
      forcesInModel: forces,
    };
  }

  return {
    regimeId: 'coulomb_field',
    title: '非匀强静电场（多点电荷）',
    summary: `场由 ${charges.length} 个点电荷叠加；一般无初等轨迹参数方程，按 F=qE(r) 数值积分。`,
    assumptions: [
      law === 'coulomb3d' ? '各源 Coulomb 1/r² 叠加' : '各源 1/r 叠加',
      '线性叠加、准静态（源瞬时位置）',
      refNote,
    ],
    parametric: [
      {
        symbolic: 'm\\vec{a}=q\\sum_i\\vec{E}_i(\\vec{r}),\\quad\\vec{E}_i=kQ_i\\frac{\\vec{r}-\\vec{r}_i}{|\\vec{r}-\\vec{r}_i|^3}',
        numeric: `\\vec{r}=(${fmtTex(b.x)},\\ ${fmtTex(b.y)}),\\ \\vec{E}=(${fmtTex(ctx.field.Ex)},\\ ${fmtTex(ctx.field.Ey)})`,
      },
      {
        symbolic: '\\varphi=\\sum_i\\frac{kQ_i}{|\\vec{r}-\\vec{r}_i|},\\quad U=q\\varphi',
        numeric: `k=${fmtTex(k)},\\ N=${charges.length}`,
        note: '无 B 时机械能守恒',
      },
    ],
    extras: [
      {
        symbolic: '\\text{reduce to one fixed source for conic analysis}',
        numeric: charges.map(c => `Q(${fmtTex(c.x)},${fmtTex(c.y)})=${fmtTex(c.q)}`).join(',\\ '),
        note: '场景源',
      },
    ],
    derived: [
      { name: '|E|', formula: '|\\sum_i\\vec{E}_i|', value: `${fmtTex(hypot(ctx.field.Ex, ctx.field.Ey))}${unitTex('N/C')}` },
      { name: '|a|', formula: '|\\vec{a}|=\\frac{|qE|}{m}', value: `${fmtTex((Math.abs(b.q) * hypot(ctx.field.Ex, ctx.field.Ey)) / b.m)}${unitTex('m/s²')}` },
      { name: '动能', formula: 'E_k=\\frac{1}{2}mv^2', value: `${fmtTex(0.5 * b.m * (b.vx * b.vx + b.vy * b.vy))}${unitTex('J')}` },
    ],
    validity: '电场线为可视化；解析轨迹需对称性（单固定心、匀强区等）。',
    forcesInModel: forces,
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
        symbolic: 'x(\\tau)=x_0+v_{x0}\\tau+\\frac{1}{2}a_x\\tau^2',
        numeric: `x(\\tau)=${fmtTex(b.x)}+(${fmtTex(b.vx)})\\tau+\\frac{1}{2}(${fmtTex(ax)})\\tau^2`,
      },
      {
        symbolic: 'y(\\tau)=y_0+v_{y0}\\tau+\\frac{1}{2}a_y\\tau^2',
        numeric: `y(\\tau)=${fmtTex(b.y)}+(${fmtTex(b.vy)})\\tau+\\frac{1}{2}(${fmtTex(ay)})\\tau^2`,
      },
      {
        symbolic: '\\vec{a}=\\frac{q\\vec{E}}{m}',
        numeric: `\\vec{a}=(${fmtTex(ax)},\\ ${fmtTex(ay)})${unitTex('m/s²')},\\ |\\vec{a}|=${fmtTex(a)}`,
      },
    ],
    extras: [
      {
        symbolic: '\\vec{v}_0\\perp\\vec{a}\\ \\Rightarrow\\ g_{\\mathrm{eff}}=|\\vec{a}|',
        numeric: `g_{\\mathrm{eff}}=${fmtTex(a)}${unitTex('m/s²')}`,
        note: '类平抛',
      },
    ],
    derived: [
      { name: '加速度 |a|', formula: '|\\vec{a}|=\\frac{|q|E}{m}', value: `${fmtTex(a)}${unitTex('m/s²')}` },
      { name: '比荷 q/m', formula: '\\frac{q}{m}', value: `${fmtTex(b.q / b.m)}${unitTex('C/kg')}` },
      { name: '动能变化', formula: '\\Delta E_k=q\\vec{E}\\cdot\\Delta\\vec{r}', value: '\\text{not conserved}' },
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
        symbolic: 'x(\\tau)=x_0+v_{x0}\\tau',
        numeric: `x(\\tau)=${fmtTex(b.x)}+(${fmtTex(b.vx)})\\tau`,
      },
      {
        symbolic: 'y(\\tau)=y_0+v_{y0}\\tau-\\frac{1}{2}g\\tau^2',
        numeric: `y(\\tau)=${fmtTex(b.y)}+(${fmtTex(b.vy)})\\tau-\\frac{1}{2}(${fmtTex(g)})\\tau^2`,
      },
    ],
    extras: [
      {
        symbolic: 'y-y_0=\\frac{v_{y0}}{v_{x0}}(x-x_0)-\\frac{g(x-x_0)^2}{2v_{x0}^2}',
        numeric:
          Math.abs(b.vx) > 1e-12
            ? `y=${fmtTex(b.y)}+(${fmtTex(b.vy / b.vx)})(x-${fmtTex(b.x)})-(${fmtTex(g / (2 * b.vx * b.vx))})(x-${fmtTex(b.x)})^2`
            : 'v_x=0\\Rightarrow\\text{vertical throw}',
        note: 'v_x≠0 时消去 τ',
      },
    ],
    derived: [
      { name: '速率 |v|', formula: '|\\vec{v}|', value: `${fmtTex(v)}${unitTex('m/s')}` },
      { name: '方向角', formula: '\\theta=\\mathrm{atan2}(v_y,v_x)', value: `${fmtTex(ang, 2)}^\\circ` },
      { name: '相对上升高度', formula: 'H=\\frac{v_y^2}{2g}', value: b.vy > 0 ? `${fmtTex(H)}${unitTex('m')}` : '\\text{—}' },
      { name: '到最高点时间', formula: 't_{\\uparrow}=\\frac{v_y}{g}', value: b.vy > 0 ? `${fmtTex(T_up)}${unitTex('s')}` : '\\text{—}' },
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
        symbolic: 'x(\\tau)=A\\cos(\\omega_0\\tau+\\phi),\\quad\\omega_0=\\sqrt{k/m}',
        numeric: `\\omega_0=${fmtTex(ω0)}${unitTex('rad/s')},\\ T=${fmtTex(T)}${unitTex('s')},\\ L=${fmtTex(L)}${unitTex('m')},\\ L_0=${fmtTex(s.L0)}${unitTex('m')}`,
        note: '一维水平简谐（若限制在 x 轴）',
      },
      {
        symbolic: '\\ddot{x}=-\\frac{k}{m}\\left(1-\\frac{L_0}{L}\\right)(x-x_a)',
        numeric: '\\text{2D free end: numeric ODE}',
        note: '二维自由端耦合',
      },
    ],
    extras: [],
    derived: [
      { name: '固有角频 ω₀', formula: '\\omega_0=\\sqrt{k/m}', value: `${fmtTex(ω0)}${unitTex('rad/s')}` },
      { name: '周期 T', formula: 'T=2\\pi\\sqrt{m/k}', value: `${fmtTex(T)}${unitTex('s')}` },
      { name: '伸长量', formula: '\\Delta L=L-L_0', value: `${fmtTex(L - s.L0)}${unitTex('m')}` },
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
  const theta = Math.atan2(dx, -dy);
  const ω0 = Math.sqrt(ctx.g / Math.max(s.L0, 1e-9));
  const T_small = (2 * Math.PI) / ω0;

  return {
    regimeId: 'pendulum',
    title: '单摆（轻绳）',
    summary: '轻绳约束 + 重力；大角度为椭圆积分，小角度近似简谐。',
    assumptions: ['绳不可伸长（本引擎用硬弹簧近似）', '固定悬点', '平面摆'],
    parametric: [
      {
        symbolic: '\\ddot{\\theta}+\\frac{g}{L}\\sin\\theta=0',
        numeric: `\\theta\\approx${fmtTex(theta / DEG, 2)}^\\circ,\\ L_0=${fmtTex(s.L0)}${unitTex('m')}`,
      },
      {
        symbolic: '\\theta(\\tau)\\approx\\Theta\\cos(\\omega_0\\tau+\\phi),\\quad\\omega_0=\\sqrt{g/L}',
        numeric: `\\omega_0\\approx${fmtTex(ω0)}${unitTex('rad/s')},\\ T\\approx${fmtTex(T_small)}${unitTex('s')}`,
        note: '小角度',
      },
      {
        symbolic: 'x=x_a+L\\sin\\theta,\\quad y=y_a-L\\cos\\theta',
        numeric: `(x_a,y_a)=(${fmtTex(s.anchor.x)},\\ ${fmtTex(s.anchor.y)}),\\ L=${fmtTex(L)}${unitTex('m')}`,
      },
    ],
    extras: [],
    derived: [
      { name: '小角度周期', formula: 'T=2\\pi\\sqrt{L/g}', value: `${fmtTex(T_small)}${unitTex('s')}` },
      { name: '摆角 θ', formula: '\\theta=\\mathrm{atan2}(x-x_a,y_a-y)', value: `${fmtTex(theta / DEG, 2)}^\\circ` },
    ],
    validity: '绳松弛（T=0）后变为抛体；刚度不足时会有弹性拉伸误差。',
    forcesInModel,
  };
}

function analyzeGeneral(ctx: AnalysisContext, forcesInModel: string[], v: number): MotionAnalysis {
  const b = ctx.body;
  const F = ctx.field;
  const parts: string[] = [];
  if (ctx.gOn) parts.push('m(0,-g)');
  if (b.q) parts.push('q\\vec{E}+q\\vec{v}\\times\\vec{B}');
  if (ctx.Fext.x || ctx.Fext.y) parts.push('\\vec{F}_{\\mathrm{ext}}');
  if (ctx.springs.length) parts.push('\\sum\\vec{F}_{\\mathrm{spring}}');
  if (ctx.drag) parts.push('-b\\vec{v}');

  return {
    regimeId: 'general',
    title: '一般受力：数值积分',
    summary: '当前力组合没有单一初等参数方程；运动由微分方程数值给出。',
    assumptions: ['下面写出的是微分方程，不是闭式轨迹'],
    parametric: [
      {
        symbolic: 'm\\vec{a}=\\sum\\vec{F}',
        numeric: `m=${fmtTex(b.m)},\\ \\vec{r}=(${fmtTex(b.x)},\\ ${fmtTex(b.y)}),\\ \\vec{v}=(${fmtTex(b.vx)},\\ ${fmtTex(b.vy)})`,
      },
      {
        symbolic: parts.length ? `m(\\ddot{x},\\ddot{y})=${parts.join('+')}` : 'm\\vec{a}=0',
        numeric: `\\vec{E}=(${fmtTex(F.Ex)},\\ ${fmtTex(F.Ey)}),\\ B_z=${fmtTex(F.Bz)},\\ |\\vec{v}|=${fmtTex(v)}`,
      },
    ],
    extras: [
      {
        symbolic: '\\text{reduce forces / make fields uniform for closed form}',
        numeric: '\\text{e.g. uniform }B\\to\\text{cyclotron};\\ \\text{gravity only}\\to\\text{projectile}',
      },
    ],
    derived: [
      { name: '速率', formula: '|\\vec{v}|', value: `${fmtTex(v)}${unitTex('m/s')}` },
      { name: '动量', formula: 'p=m|\\vec{v}|', value: `${fmtTex(b.m * v)}${unitTex('kg·m/s')}` },
      { name: '动能', formula: 'E_k=\\frac{1}{2}mv^2', value: `${fmtTex(0.5 * b.m * v * v)}${unitTex('J')}` },
    ],
    validity: '请用轨迹与 v–t / x–t 图像观察；解析式需满足更强对称性。',
    forcesInModel,
  };
}
