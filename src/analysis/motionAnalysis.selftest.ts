import { analyzeMotion, sampleAnalyticPath, type AnalysisContext, type BodySnapshot } from './motionAnalysis.ts';

function assert(cond: unknown, msg: string) {
  if (!cond) throw new Error(msg);
}

function base(
  bodyOver: Partial<BodySnapshot> = {},
  field: AnalysisContext['field'] = { Ex: 0, Ey: 0, Bz: 0 },
  extra: Partial<AnalysisContext> = {},
): AnalysisContext {
  return {
    gOn: false,
    g: 9.8,
    drag: 0,
    refMode: 'initial',
    body: {
      id: 1,
      name: 'p',
      kind: 'particle',
      m: 1,
      q: 1,
      x: 0,
      y: 0,
      vx: 2,
      vy: 0,
      t: 0,
      fixed: false,
      ...bodyOver,
    },
    field,
    Fext: { x: 0, y: 0 },
    springs: [],
    ...extra,
  };
}

const cyclo = analyzeMotion(base({}, { Ex: 0, Ey: 0, Bz: -1 }));
assert(cyclo.regimeId === 'cyclotron', 'expected cyclotron');
assert(cyclo.parametric.length >= 2, 'cyclotron eqs');
assert(cyclo.derived.some(d => d.value.includes('2')), 'R≈2');

const path = sampleAnalyticPath(base({}, { Ex: 0, Ey: 0, Bz: -1 }));
assert(path && path.points.length > 10, 'path samples');
const mid = path!.points[Math.floor(path!.points.length / 2)];
// ω=qBz/m=-1, center (xc,yc)=(0,2), radius 2
assert(Math.abs(Math.hypot(mid.x - 0, mid.y - 2) - 2) < 0.05, `point on circle, got ${mid.x},${mid.y}`);

const crossed = analyzeMotion(base({ vx: 0, vy: 0 }, { Ex: 0, Ey: -2, Bz: -1 }));
assert(crossed.regimeId === 'crossed_eb', 'crossed eb');

const proj = analyzeMotion(base({ q: 0, vx: 3, vy: 4, y: 5 }, { Ex: 0, Ey: 0, Bz: 0 }, { gOn: true }));
assert(proj.regimeId === 'projectile', 'projectile');


const outside = analyzeMotion({
  ...base({ x: -2, y: 0, vx: 2, vy: 0 }, { Ex: 0, Ey: 0, Bz: 0 }),
  modelField: { Ex: 0, Ey: 0, Bz: -1 },
  modelFieldNote: '参考点处场为零；按场景匀强 B',
});
assert(outside.regimeId === 'cyclotron', 'model field cyclotron');
assert(outside.title.includes('场景场模型') || outside.assumptions.some(a => a.includes('参考点')), 'model note');

// 点电荷非匀强：不得误判为匀强 E
const k = 8.99e9;
const Q = 1e-4;
const q = -1e-5;
const r = 2;
const vCirc = Math.sqrt((k * Math.abs(q * Q)) / (1 * r));
const coulombCirc = analyzeMotion(base(
  { m: 1, q, x: r, y: 0, vx: 0, vy: vCirc },
  { Ex: (k * Q) / (r * r), Ey: 0, Bz: 0 },
  {
    pointCharges: [{ id: 99, x: 0, y: 0, q: Q, fixed: true }],
    coulombLaw: 'coulomb3d',
    kCoulomb: k,
    hasUniformE: false,
  },
));
assert(coulombCirc.regimeId === 'coulomb_circular' || coulombCirc.regimeId === 'coulomb_central', `expected coulomb orbit, got ${coulombCirc.regimeId}`);
assert(coulombCirc.regimeId !== 'uniform_e', 'must not classify point-charge field as uniform_e');

const pathC = sampleAnalyticPath(base(
  { m: 1, q, x: r, y: 0, vx: 0, vy: vCirc },
  { Ex: (k * Q) / (r * r), Ey: 0, Bz: 0 },
  {
    pointCharges: [{ id: 99, x: 0, y: 0, q: Q, fixed: true }],
    coulombLaw: 'coulomb3d',
    kCoulomb: k,
  },
));
if (coulombCirc.regimeId === 'coulomb_circular') {
  assert(pathC && pathC.points.length > 10, 'circular coulomb path');
  const p = pathC!.points[Math.floor(pathC!.points.length / 2)];
  assert(Math.abs(Math.hypot(p.x, p.y) - r) < 0.08, `point on coulomb circle, r=${Math.hypot(p.x, p.y)}`);
}

const multi = analyzeMotion(base(
  { q: 1e-8, x: 0, y: 1.2, vx: 0.8, vy: 0 },
  { Ex: 1, Ey: 0, Bz: 0 },
  {
    pointCharges: [
      { id: 2, x: -1, y: 0, q: 1e-6, fixed: true },
      { id: 3, x: 1, y: 0, q: -1e-6, fixed: true },
    ],
    coulombLaw: 'coulomb3d',
    kCoulomb: k,
  },
));
assert(multi.regimeId === 'coulomb_field' || multi.regimeId === 'coulomb_central', `dipole field regime got ${multi.regimeId}`);

console.log('motionAnalysis selftest OK');

