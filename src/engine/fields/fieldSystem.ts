import { collectLineSeeds, PointChargeSource2, UniformBSource2, UniformESource2 } from './sources.ts';
import { traceFieldLines } from './fieldLines.ts';
import type { CoulombLaw, FieldSample2, FieldSource2, FieldSystemOptions } from './types.ts';

const K_DEFAULT = 8.99e9;

/**
 * Unified electrostatic/magnetostatic sampler.
 * Dynamics, analysis, and field-line views all read through this.
 */
export class FieldSystem2 {
  sources: FieldSource2[] = [];
  law: CoulombLaw;
  k: number;
  softening: number;

  private dirty = true;
  private cachedLines: { x: number; y: number }[][] = [];
  private lineOpts = {
    enabled: true,
    linesPerUnitCharge: 4,
    step: 0.04,
    maxSteps: 500,
    bounds: { x0: -20, y0: -20, x1: 20, y1: 20 },
  };

  constructor(opts: FieldSystemOptions = {}) {
    this.law = opts.law ?? 'coulomb3d';
    this.k = opts.k ?? K_DEFAULT;
    this.softening = opts.softening ?? 0.02;
  }

  clear() {
    this.sources = [];
    this.markDirty();
  }

  setSources(sources: FieldSource2[]) {
    this.sources = sources;
    this.markDirty();
  }

  markDirty() {
    this.dirty = true;
  }

  setLineOptions(partial: Partial<typeof this.lineOpts>) {
    Object.assign(this.lineOpts, partial);
    this.markDirty();
  }

  sample(x: number, y: number): FieldSample2 {
    let Ex = 0;
    let Ey = 0;
    let Bz = 0;
    let phi = 0;
    for (const s of this.sources) {
      const e = s.sampleE(x, y, this.law, this.k);
      Ex += e.x;
      Ey += e.y;
      if (s.sampleB) Bz += s.sampleB(x, y).Bz;
      if (s.samplePhi) phi += s.samplePhi(x, y, this.law, this.k);
    }
    return { Ex, Ey, Bz, phi };
  }

  /** Lorentz + electric force on charge q with velocity (vx,vy). */
  forceOnCharge(q: number, x: number, y: number, vx: number, vy: number) {
    const f = this.sample(x, y);
    // F = qE + q v × B  (B = Bz ẑ) → (q Ey B? wait): v×B = (vy Bz, -vx Bz, 0) for B=(0,0,Bz)? 
    // v×B = |i     j     k|
    //       |vx   vy    0|
    //       |0     0   Bz| = i(vy Bz) - j(vx Bz) = (vy Bz, -vx Bz, 0)
    const Fx = q * f.Ex + q * vy * f.Bz;
    const Fy = q * f.Ey - q * vx * f.Bz;
    return { Fx, Fy, ...f };
  }

  getFieldLines(): { x: number; y: number }[][] {
    if (!this.lineOpts.enabled) return [];
    if (!this.dirty) return this.cachedLines;
    const seeds = collectLineSeeds(this.sources, this.lineOpts.linesPerUnitCharge);
    this.cachedLines = traceFieldLines(seeds, (x, y) => this.sample(x, y), this.sources, {
      step: this.lineOpts.step,
      maxSteps: this.lineOpts.maxSteps,
      bounds: this.lineOpts.bounds,
    });
    this.dirty = false;
    return this.cachedLines;
  }
}

/** Build FieldSystem2 sources from a plain scene object list (simulator bridge). */
export function sourcesFromSceneObjects(
  objects: any[],
  opts: { includeBodyCharges?: boolean; deg?: number } = {},
): FieldSource2[] {
  const DEG = opts.deg ?? Math.PI / 180;
  const out: FieldSource2[] = [];
  for (const o of objects) {
    if (o.kind === 'pointCharge') {
      out.push(new PointChargeSource2(o.id, o.x, o.y, o.q, { softRadius: o.softRadius ?? 0.05 }));
    } else if (o.kind === 'efield') {
      const Ex = o.E * Math.cos(o.ang * DEG);
      const Ey = o.E * Math.sin(o.ang * DEG);
      const region = o.all
        ? ({ all: true } as const)
        : ({ all: false, x: o.x, y: o.y, w: o.w, h: o.h } as const);
      out.push(new UniformESource2(o.id, Ex, Ey, region));
    } else if (o.kind === 'bfield') {
      const region = o.all
        ? ({ all: true } as const)
        : ({ all: false, x: o.x, y: o.y, w: o.w, h: o.h } as const);
      out.push(new UniformBSource2(o.id, o.B * o.dir, region));
    } else if (opts.includeBodyCharges && o.q && o.st && (o.kind === 'particle' || o.kind === 'ball' || o.kind === 'box')) {
      // Charged bodies act as Coulomb emitters when scene.settings.coulomb is on
      out.push(new PointChargeSource2(o.id, o.st.x, o.st.y, o.q, {
        softRadius: Math.max(o.r || 0.05, o.softRadius || 0.02),
      }));
    }
  }
  return out;
}

export { PointChargeSource2, UniformESource2, UniformBSource2 };
