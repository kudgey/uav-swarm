/**
 * Turbulence generator with optional spatial correlation.
 * STATUS: experimental (Phase 11: lattice OU field option)
 *
 * Base: OU colored noise per-axis (global, all drones identical).
 * Spatial mode: preallocated lattice of independent OU cells with trilinear interpolation.
 */

import { v3Create } from '@lib/math';
import type { Vec3, TurbulenceConfig } from '@sim/core/types';
import type { DeterministicRNG } from '@sim/core/rng';
import { DeterministicRNG as RNG } from '@sim/core/rng';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('turbulence', 'experimental',
  'OU colored noise + optional spatial lattice correlation');

export class TurbulenceGenerator {
  // Global OU state (used when spatialCorrelation = false)
  private state: Float64Array;
  private rng: DeterministicRNG;
  private intensity: number;
  private tau: number;
  private enabled: boolean;
  private _output: Vec3;

  // Spatial lattice (Phase 11)
  private spatial: boolean;
  private L: number; // integral length scale
  private lattice: Float64Array[] | null = null; // each cell: Float64Array(3) OU state
  private latticeRngs: DeterministicRNG[] | null = null;
  private nx = 0; private ny = 0; private nz = 0;
  private minX = 0; private minY = 0; private minZ = 0;

  constructor(config: TurbulenceConfig, rng: DeterministicRNG) {
    this.state = new Float64Array(3);
    this.rng = rng;
    this.intensity = config.intensity;
    this.tau = config.correlationTime;
    this.enabled = config.enabled;
    this._output = v3Create();
    this.spatial = config.spatialCorrelation ?? false;
    this.L = config.integralLengthScale ?? 5;
  }

  /** Initialize spatial lattice from scene bounds. Must be called after construction if spatial enabled. */
  initLattice(bounds: { min: Vec3; max: Vec3 }, masterSeed: number): void {
    if (!this.spatial) return;
    const L = this.L;
    this.minX = Math.floor(bounds.min[0] / L); this.minY = Math.floor(bounds.min[1] / L); this.minZ = Math.floor(bounds.min[2] / L);
    const maxCX = Math.ceil(bounds.max[0] / L); const maxCY = Math.ceil(bounds.max[1] / L); const maxCZ = Math.ceil(bounds.max[2] / L);
    this.nx = maxCX - this.minX + 1; this.ny = maxCY - this.minY + 1; this.nz = maxCZ - this.minZ + 1;
    const total = this.nx * this.ny * this.nz;
    this.lattice = new Array(total);
    this.latticeRngs = new Array(total);
    for (let i = 0; i < total; i++) {
      this.lattice[i] = new Float64Array(3);
      // Deterministic seed per cell
      const cellSeed = masterSeed ^ (i * 2654435761); // hash
      this.latticeRngs[i] = new RNG(cellSeed >>> 0);
    }
  }

  updateConfig(config: TurbulenceConfig): void {
    this.intensity = config.intensity;
    this.tau = config.correlationTime;
    this.enabled = config.enabled;
    this.spatial = config.spatialCorrelation ?? false;
    this.L = config.integralLengthScale ?? 5;
  }

  /** Advance turbulence state by dt. Call once per physics step. */
  step(dt: number): void {
    if (!this.enabled || this.tau <= 0) return;
    const alpha = dt / this.tau;
    const decay = 1 - alpha;
    const diffusion = this.intensity * Math.sqrt(2 * alpha);

    // Global OU
    for (let i = 0; i < 3; i++) {
      this.state[i] = decay * this.state[i] + diffusion * this.rng.gaussian();
    }
    this._output[0] = this.state[0]; this._output[1] = this.state[1]; this._output[2] = this.state[2];

    // Advance all lattice cells
    if (this.spatial && this.lattice && this.latticeRngs) {
      for (let c = 0; c < this.lattice.length; c++) {
        const cell = this.lattice[c];
        const cellRng = this.latticeRngs[c];
        for (let i = 0; i < 3; i++) {
          cell[i] = decay * cell[i] + diffusion * cellRng.gaussian();
        }
      }
    }
  }

  /** Get global turbulence output (no spatial variation). */
  getOutput(): Vec3 { return this._output; }

  /** Sample turbulence at a position. If spatial disabled, returns global output. Read-only. */
  sampleAt(position: Vec3): Vec3 {
    if (!this.spatial || !this.lattice) return this._output;

    const L = this.L;
    const fx = position[0] / L - this.minX;
    const fy = position[1] / L - this.minY;
    const fz = position[2] / L - this.minZ;

    const ix = Math.floor(fx); const iy = Math.floor(fy); const iz = Math.floor(fz);
    const tx = fx - ix; const ty = fy - iy; const tz = fz - iz;

    // Trilinear interpolation from 8 cells
    const result = v3Create();
    for (let dx = 0; dx <= 1; dx++) {
      for (let dy = 0; dy <= 1; dy++) {
        for (let dz = 0; dz <= 1; dz++) {
          const cx = Math.max(0, Math.min(this.nx - 1, ix + dx));
          const cy = Math.max(0, Math.min(this.ny - 1, iy + dy));
          const cz = Math.max(0, Math.min(this.nz - 1, iz + dz));
          const idx = cx + cy * this.nx + cz * this.nx * this.ny;
          const w = (dx ? tx : 1 - tx) * (dy ? ty : 1 - ty) * (dz ? tz : 1 - tz);
          const cell = this.lattice[idx];
          if (cell) {
            result[0] += w * cell[0]; result[1] += w * cell[1]; result[2] += w * cell[2];
          }
        }
      }
    }
    return result;
  }

  reset(): void {
    this.state[0] = 0; this.state[1] = 0; this.state[2] = 0;
    this._output[0] = 0; this._output[1] = 0; this._output[2] = 0;
    if (this.lattice) for (const c of this.lattice) { c[0] = 0; c[1] = 0; c[2] = 0; }
  }
}
