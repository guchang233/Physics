import { hypot2, normalize2 } from '../math/vec.ts';
import type { FieldSample2, FieldSource2 } from './types.ts';

export type TraceOptions = {
  step: number;
  maxSteps: number;
  bounds: { x0: number; y0: number; x1: number; y1: number };
  sinkThreshold?: number;
};

/**
 * RK4 streamline integration of unit Ê (same approach as EFSim / field-sandbox tutorials).
 * Integrates both forward (+E) from seeds; sinks terminate on negative charges.
 */
export function traceFieldLines(
  seeds: { x: number; y: number; dirx: number; diry: number }[],
  sample: (x: number, y: number) => FieldSample2,
  sources: FieldSource2[],
  opts: TraceOptions,
): { x: number; y: number }[][] {
  const lines: { x: number; y: number }[][] = [];
  const thr = opts.sinkThreshold ?? 0.08;
  for (const s of seeds) {
    const path = integrateOne(s.x, s.y, 1, sample, sources, opts, thr);
    if (path.length > 2) lines.push(path);
  }
  return lines;
}

function dirAt(
  x: number,
  y: number,
  sample: (x: number, y: number) => FieldSample2,
  sign: number,
) {
  const f = sample(x, y);
  const n = normalize2({ x: sign * f.Ex, y: sign * f.Ey });
  return n;
}

function integrateOne(
  x0: number,
  y0: number,
  sign: number,
  sample: (x: number, y: number) => FieldSample2,
  sources: FieldSource2[],
  opts: TraceOptions,
  thr: number,
): { x: number; y: number }[] {
  const path: { x: number; y: number }[] = [{ x: x0, y: y0 }];
  let x = x0;
  let y = y0;
  const h = opts.step;
  const { x0: bx0, y0: by0, x1: bx1, y1: by1 } = opts.bounds;

  for (let i = 0; i < opts.maxSteps; i++) {
    const f = sample(x, y);
    if (hypot2({ x: f.Ex, y: f.Ey }) < 1e-14) break;

    const k1 = dirAt(x, y, sample, sign);
    if (hypot2(k1) < 1e-15) break;
    const k2 = dirAt(x + 0.5 * h * k1.x, y + 0.5 * h * k1.y, sample, sign);
    const k3 = dirAt(x + 0.5 * h * k2.x, y + 0.5 * h * k2.y, sample, sign);
    const k4 = dirAt(x + h * k3.x, y + h * k3.y, sample, sign);

    x += (h / 6) * (k1.x + 2 * k2.x + 2 * k3.x + k4.x);
    y += (h / 6) * (k1.y + 2 * k2.y + 2 * k3.y + k4.y);
    path.push({ x, y });

    if (x < bx0 || x > bx1 || y < by0 || y > by1) break;
    let hit = false;
    for (const s of sources) {
      if (s.isSink && s.isSink(x, y, thr)) {
        hit = true;
        break;
      }
    }
    if (hit) break;
  }
  return path;
}
