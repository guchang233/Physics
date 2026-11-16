import { FieldSystem2, PointChargeSource2, UniformBSource2, UniformESource2 } from './fieldSystem.ts';

function assert(cond: unknown, msg: string) {
  if (!cond) throw new Error(msg);
}

// Single +1 C charge at origin: |E| at (1,0) = k / r²
{
  const fs = new FieldSystem2({ law: 'coulomb3d', k: 9e9, softening: 1e-6 });
  fs.setSources([new PointChargeSource2(1, 0, 0, 1, { softRadius: 1e-6 })]);
  const s = fs.sample(1, 0);
  assert(Math.abs(s.Ex - 9e9) / 9e9 < 1e-6, `Ex≈k, got ${s.Ex}`);
  assert(Math.abs(s.Ey) < 1e-3, `Ey≈0, got ${s.Ey}`);
}

// Dipole field lines should produce multiple polylines
{
  const fs = new FieldSystem2({ law: 'coulomb3d', k: 1, softening: 0.05 });
  fs.setSources([
    new PointChargeSource2('p', -1, 0, 1, { softRadius: 0.05 }),
    new PointChargeSource2('n', 1, 0, -1, { softRadius: 0.05 }),
  ]);
  fs.setLineOptions({
    linesPerUnitCharge: 8,
    step: 0.05,
    maxSteps: 400,
    bounds: { x0: -5, y0: -5, x1: 5, y1: 5 },
  });
  const lines = fs.getFieldLines();
  assert(lines.length >= 4, `expected several field lines, got ${lines.length}`);
  assert(lines.every(l => l.length > 5), 'lines should have multiple samples');
}

// Uniform E + sample
{
  const fs = new FieldSystem2();
  fs.setSources([new UniformESource2('e', 0, -2, { all: true })]);
  const s = fs.sample(3, 4);
  assert(s.Ex === 0 && s.Ey === -2, 'uniform E');
}

// Force: qE
{
  const fs = new FieldSystem2();
  fs.setSources([new UniformESource2('e', 3, 0, { all: true })]);
  const f = fs.forceOnCharge(2, 0, 0, 0, 0);
  assert(f.Fx === 6 && f.Fy === 0, 'F=qE');
}

// Lorentz: Bz=1, v=(0,1), q=1 → F = q v×B = (vy Bz, -vx Bz)=(1,0)
{
  const fs = new FieldSystem2();
  fs.setSources([new UniformBSource2('b', 1, { all: true })]);
  const f = fs.forceOnCharge(1, 0, 0, 0, 1);
  assert(Math.abs(f.Fx - 1) < 1e-9 && Math.abs(f.Fy) < 1e-9, `Lorentz Fx=1, got ${f.Fx},${f.Fy}`);
}

console.log('fieldSystem selftest OK');
