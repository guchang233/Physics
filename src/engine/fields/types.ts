import type { Vec2, Vec3 } from '../math/vec.ts';

/** Coulomb law: 3D point-charge (default) or true-2D / line-charge cross-section. */
export type CoulombLaw = 'coulomb3d' | 'coulomb2d';

export type FieldSample2 = {
  Ex: number;
  Ey: number;
  Bz: number;
  /** Electrostatic potential (V), optional; 0 if unsupported */
  phi: number;
};

export type FieldSample3 = {
  Ex: number;
  Ey: number;
  Ez: number;
  Bx: number;
  By: number;
  Bz: number;
  phi: number;
};

/**
 * Field source contract (EFSim-inspired): each source contributes E (and optionally B, φ).
 * Core stays free of Three.js; views adapt vectors as needed.
 */
export interface FieldSource2 {
  readonly id: string | number;
  /** Soft radius used by field-line termination / singularity clamp */
  softRadius?: number;
  sampleE(x: number, y: number, law: CoulombLaw, k: number): Vec2;
  sampleB?(x: number, y: number): { Bz: number };
  samplePhi?(x: number, y: number, law: CoulombLaw, k: number): number;
  /** Seeds for field-line tracing (world coords + unit direction) */
  lineSeeds?(linesPerUnitCharge: number): { x: number; y: number; dirx: number; diry: number }[];
  /** True if a field-line tip has entered this sink */
  isSink?(x: number, y: number, threshold: number): boolean;
  /** Signed charge magnitude for flux-proportional line counts; 0 if N/A */
  signedCharge?(): number;
}

export interface FieldSource3 {
  readonly id: string | number;
  softRadius?: number;
  sampleE(p: Vec3, law: CoulombLaw, k: number): Vec3;
  sampleB?(p: Vec3): Vec3;
  samplePhi?(p: Vec3, law: CoulombLaw, k: number): number;
}

export type FieldSystemOptions = {
  law?: CoulombLaw;
  /** Coulomb constant k (N·m²/C²). Default 8.99e9 */
  k?: number;
  /** Softening length (m) near point singularities */
  softening?: number;
};
