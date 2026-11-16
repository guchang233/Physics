/**
 * Rapier contact/rigid-body adapter (scaffold).
 *
 * Role in the stack:
 *   FieldSystem  → EM forces (custom) + optional Boris for charged particles
 *   ContactWorld → collisions / friction / future 3D rigid bodies (Rapier)
 *
 * This module intentionally does NOT simulate Coulomb or Lorentz forces.
 * Callers apply EM forces externally, then step Rapier for contacts.
 */

export type ContactBodyDesc = {
  id: number;
  kind: 'ball' | 'box';
  x: number;
  y: number;
  vx: number;
  vy: number;
  m: number;
  r?: number;
  w?: number;
  h?: number;
  ang?: number;
  fixed?: boolean;
  restitution?: number;
  friction?: number;
};

export type ContactSegmentDesc = {
  id: number;
  x1: number;
  y1: number;
  x2: number;
  y2: number;
  restitution?: number;
  friction?: number;
};

export type ContactStepResult = {
  id: number;
  x: number;
  y: number;
  vx: number;
  vy: number;
  ang: number;
};

export interface ContactWorld {
  readonly backend: 'rapier2d' | 'stub';
  clear(): void;
  syncFromScene(bodies: ContactBodyDesc[], segments: ContactSegmentDesc[]): void;
  /** Apply external force for this step only (N). Cleared after step. */
  applyForce(id: number, fx: number, fy: number): void;
  step(dt: number): ContactStepResult[];
  dispose(): void;
}

/** Stub used until Rapier WASM is initialized / opted-in. */
export class StubContactWorld implements ContactWorld {
  readonly backend = 'stub' as const;
  clear() {}
  syncFromScene() {}
  applyForce() {}
  step(): ContactStepResult[] {
    return [];
  }
  dispose() {}
}

let rapierModule: typeof import('@dimforge/rapier2d-compat') | null = null;

/** Lazy-load Rapier WASM. Safe to call multiple times. */
export async function loadRapier2d() {
  if (rapierModule) return rapierModule;
  const RAPIER = await import('@dimforge/rapier2d-compat');
  await RAPIER.init();
  rapierModule = RAPIER;
  return RAPIER;
}

/**
 * Rapier2D-backed contact world.
 * Create via `createRapierContactWorld()` after `loadRapier2d()`.
 */
export class RapierContactWorld implements ContactWorld {
  readonly backend = 'rapier2d' as const;
  private world: import('@dimforge/rapier2d-compat').World;
  private RAPIER: typeof import('@dimforge/rapier2d-compat');
  private bodies = new Map<number, import('@dimforge/rapier2d-compat').RigidBody>();
  private colliders: import('@dimforge/rapier2d-compat').Collider[] = [];
  private pendingForce = new Map<number, { fx: number; fy: number }>();

  constructor(RAPIER: typeof import('@dimforge/rapier2d-compat'), gravityY = 0) {
    this.RAPIER = RAPIER;
    this.world = new RAPIER.World({ x: 0, y: gravityY });
  }

  clear() {
    for (const b of this.bodies.values()) this.world.removeRigidBody(b);
    this.bodies.clear();
    this.colliders = [];
    this.pendingForce.clear();
  }

  syncFromScene(bodies: ContactBodyDesc[], segments: ContactSegmentDesc[]) {
    this.clear();
    const R = this.RAPIER;
    for (const s of segments) {
      const dx = s.x2 - s.x1;
      const dy = s.y2 - s.y1;
      const len = Math.hypot(dx, dy) || 1e-6;
      const cx = (s.x1 + s.x2) / 2;
      const cy = (s.y1 + s.y2) / 2;
      const ang = Math.atan2(dy, dx);
      const body = this.world.createRigidBody(R.RigidBodyDesc.fixed().setTranslation(cx, cy).setRotation(ang));
      const col = this.world.createCollider(
        R.ColliderDesc.cuboid(len / 2, 0.02)
          .setRestitution(s.restitution ?? 0.2)
          .setFriction(s.friction ?? 0.3),
        body,
      );
      this.colliders.push(col);
    }
    for (const b of bodies) {
      const desc = b.fixed
        ? R.RigidBodyDesc.fixed()
        : R.RigidBodyDesc.dynamic().setLinvel(b.vx, b.vy).setAdditionalMass(b.m);
      desc.setTranslation(b.x, b.y);
      if (b.ang != null) desc.setRotation(b.ang);
      const rb = this.world.createRigidBody(desc);
      let collider;
      if (b.kind === 'box') {
        collider = R.ColliderDesc.cuboid((b.w ?? 0.6) / 2, (b.h ?? 0.4) / 2);
      } else {
        collider = R.ColliderDesc.ball(b.r ?? 0.25);
      }
      collider.setRestitution(b.restitution ?? 0.4).setFriction(b.friction ?? 0.2);
      this.world.createCollider(collider, rb);
      this.bodies.set(b.id, rb);
    }
  }

  applyForce(id: number, fx: number, fy: number) {
    const cur = this.pendingForce.get(id) || { fx: 0, fy: 0 };
    cur.fx += fx;
    cur.fy += fy;
    this.pendingForce.set(id, cur);
  }

  step(dt: number): ContactStepResult[] {
    for (const [id, f] of this.pendingForce) {
      const rb = this.bodies.get(id);
      if (!rb || rb.isFixed()) continue;
      rb.resetForces(true);
      rb.addForce({ x: f.fx, y: f.fy }, true);
    }
    this.pendingForce.clear();
    this.world.timestep = dt;
    this.world.step();
    const out: ContactStepResult[] = [];
    for (const [id, rb] of this.bodies) {
      const t = rb.translation();
      const v = rb.linvel();
      out.push({ id, x: t.x, y: t.y, vx: v.x, vy: v.y, ang: rb.rotation() });
      rb.resetForces(false);
    }
    return out;
  }

  dispose() {
    this.clear();
    this.world.free();
  }
}

export async function createRapierContactWorld(gravityY = 0): Promise<ContactWorld> {
  const R = await loadRapier2d();
  return new RapierContactWorld(R, gravityY);
}
