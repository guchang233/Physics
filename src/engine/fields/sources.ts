import { hypot2, normalize2, type Vec2 } from '../math/vec.ts';
import type { CoulombLaw, FieldSource2 } from './types.ts';

const TWO_PI = Math.PI * 2;

/** Softened 3D Coulomb / 2D (log) contributions from a point charge in the plane. */
function pointE(
  qx: number,
  qy: number,
  q: number,
  x: number,
  y: number,
  law: CoulombLaw,
  k: number,
  soft: number,
): Vec2 {
  const dx = x - qx;
  const dy = y - qy;
  const r2 = dx * dx + dy * dy + soft * soft;
  const r = Math.sqrt(r2);
  if (r < 1e-15) return { x: 0, y: 0 };
  if (law === 'coulomb2d') {
    // E = (2 k q / r) r̂  with 2k = 1/(2πϵ₀)
    const mag = (2 * k * q) / r2;
    return { x: mag * dx, y: mag * dy };
  }
  const mag = (k * q) / (r2 * r);
  return { x: mag * dx, y: mag * dy };
}

function pointPhi(
  qx: number,
  qy: number,
  q: number,
  x: number,
  y: number,
  law: CoulombLaw,
  k: number,
  soft: number,
): number {
  const dx = x - qx;
  const dy = y - qy;
  const r = Math.sqrt(dx * dx + dy * dy + soft * soft);
  if (law === 'coulomb2d') return -2 * k * q * Math.log(Math.max(r, soft));
  return (k * q) / Math.max(r, soft);
}

/** Point charge source (EFSim-style). */
export class PointChargeSource2 implements FieldSource2 {
  id: string | number;
  x: number;
  y: number;
  q: number;
  softRadius: number;

  constructor(id: string | number, x: number, y: number, q: number, opts: { softRadius?: number } = {}) {
    this.id = id;
    this.x = x;
    this.y = y;
    this.q = q;
    this.softRadius = opts.softRadius ?? 0.05;
  }

  sampleE(x: number, y: number, law: CoulombLaw, k: number): Vec2 {
    return pointE(this.x, this.y, this.q, x, y, law, k, this.softRadius);
  }

  samplePhi(x: number, y: number, law: CoulombLaw, k: number): number {
    return pointPhi(this.x, this.y, this.q, x, y, law, k, this.softRadius);
  }

  signedCharge() {
    return this.q;
  }

  isSink(x: number, y: number, threshold: number) {
    if (this.q >= 0) return false;
    return hypot2({ x: x - this.x, y: y - this.y }) < Math.max(threshold, this.softRadius);
  }

  lineSeeds(linesPerUnitCharge: number) {
    if (this.q <= 0) return [];
    const n = Math.max(4, Math.round(Math.abs(this.q) * linesPerUnitCharge));
    const r0 = Math.max(this.softRadius * 1.5, 0.02);
    const out: { x: number; y: number; dirx: number; diry: number }[] = [];
    for (let i = 0; i < n; i++) {
      const ang = (TWO_PI * i) / n;
      const dirx = Math.cos(ang);
      const diry = Math.sin(ang);
      out.push({ x: this.x + r0 * dirx, y: this.y + r0 * diry, dirx, diry });
    }
    return out;
  }
}

type RectRegion = { all: true } | { all: false; x: number; y: number; w: number; h: number };

/** Uniform E region. */
export class UniformESource2 implements FieldSource2 {
  id: string | number;
  Ex: number;
  Ey: number;
  region: RectRegion;

  constructor(id: string | number, Ex: number, Ey: number, region: RectRegion) {
    this.id = id;
    this.Ex = Ex;
    this.Ey = Ey;
    this.region = region;
  }

  private inside(x: number, y: number) {
    if (this.region.all) return true;
    const r = this.region;
    return x >= r.x && x <= r.x + r.w && y >= r.y && y <= r.y + r.h;
  }

  sampleE(x: number, y: number, _law?: CoulombLaw, _k?: number): Vec2 {
    if (!this.inside(x, y)) return { x: 0, y: 0 };
    return { x: this.Ex, y: this.Ey };
  }

  samplePhi(x: number, y: number, _law?: CoulombLaw, _k?: number): number {
    if (!this.inside(x, y)) return 0;
    return -(this.Ex * x + this.Ey * y);
  }

  signedCharge() {
    return 0;
  }

  lineSeeds() {
    return [] as { x: number; y: number; dirx: number; diry: number }[];
  }
}

/** Uniform B_z region. */
export class UniformBSource2 implements FieldSource2 {
  id: string | number;
  Bz: number;
  region: RectRegion;

  constructor(id: string | number, Bz: number, region: RectRegion) {
    this.id = id;
    this.Bz = Bz;
    this.region = region;
  }

  private inside(x: number, y: number) {
    if (this.region.all) return true;
    const r = this.region;
    return x >= r.x && x <= r.x + r.w && y >= r.y && y <= r.y + r.h;
  }

  sampleE(_x?: number, _y?: number, _law?: CoulombLaw, _k?: number): Vec2 {
    return { x: 0, y: 0 };
  }

  sampleB(x: number, y: number) {
    if (!this.inside(x, y)) return { Bz: 0 };
    return { Bz: this.Bz };
  }

  signedCharge() {
    return 0;
  }
}

/** Build flux-aware seeds from sources. */
export function collectLineSeeds(sources: FieldSource2[], linesPerUnitCharge: number) {
  const seeds: { x: number; y: number; dirx: number; diry: number }[] = [];
  for (const s of sources) {
    if (s.lineSeeds) seeds.push(...s.lineSeeds(linesPerUnitCharge));
    if (s instanceof UniformESource2 && !s.region.all) {
      const r = s.region;
      const e = normalize2({ x: s.Ex, y: s.Ey });
      if (hypot2(e) < 1e-12) continue;
      for (let i = 0; i <= 6; i++) {
        const t = i / 6;
        let x: number;
        let y: number;
        if (Math.abs(e.x) >= Math.abs(e.y)) {
          x = e.x > 0 ? r.x : r.x + r.w;
          y = r.y + t * r.h;
        } else {
          y = e.y > 0 ? r.y : r.y + r.h;
          x = r.x + t * r.w;
        }
        seeds.push({ x, y, dirx: e.x, diry: e.y });
      }
    }
  }
  return seeds;
}
