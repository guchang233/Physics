/**
 * Physim engine façade.
 *
 * Integration map:
 *   FieldSystem2     — Coulomb / uniform E·B sampling + field lines (EFSim-style)
 *   ContactWorld     — Rapier2D contacts (optional); stub until enabled
 *   App simulator    — orchestration, UI, Boris for EM particles when not using Rapier EM step
 */

export * from './math/vec.ts';
export * from './fields/types.ts';
export * from './fields/sources.ts';
export * from './fields/fieldLines.ts';
export * from './fields/fieldSystem.ts';
export * from './rapier/contactWorld.ts';
