/** Minimal 2D/3D vector helpers for the field engine (no Three dependency in core). */

export type Vec2 = { x: number; y: number };
export type Vec3 = { x: number; y: number; z: number };

export function v2(x = 0, y = 0): Vec2 {
  return { x, y };
}

export function v3(x = 0, y = 0, z = 0): Vec3 {
  return { x, y, z };
}

export function add2(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x + b.x, y: a.y + b.y };
}

export function sub2(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x - b.x, y: a.y - b.y };
}

export function scale2(a: Vec2, s: number): Vec2 {
  return { x: a.x * s, y: a.y * s };
}

export function hypot2(a: Vec2): number {
  return Math.hypot(a.x, a.y);
}

export function normalize2(a: Vec2, eps = 1e-15): Vec2 {
  const L = hypot2(a);
  if (L < eps) return { x: 0, y: 0 };
  return { x: a.x / L, y: a.y / L };
}

export function add3(a: Vec3, b: Vec3): Vec3 {
  return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
}

export function sub3(a: Vec3, b: Vec3): Vec3 {
  return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
}

export function scale3(a: Vec3, s: number): Vec3 {
  return { x: a.x * s, y: a.y * s, z: a.z * s };
}

export function hypot3(a: Vec3): number {
  return Math.hypot(a.x, a.y, a.z);
}

export function normalize3(a: Vec3, eps = 1e-15): Vec3 {
  const L = hypot3(a);
  if (L < eps) return { x: 0, y: 0, z: 0 };
  return { x: a.x / L, y: a.y / L, z: a.z / L };
}

export function cross3(a: Vec3, b: Vec3): Vec3 {
  return {
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x,
  };
}
