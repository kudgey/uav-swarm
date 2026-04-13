/**
 * Inter-drone downwash model.
 * STATUS: simplified — empirical 1/r^2 falloff along vertical axis.
 *
 * When drone B is below drone A and within rotor radius horizontally,
 * B's rotors see reduced inflow velocity → thrust reduction.
 *
 * Simplification: we model this as a thrust multiplier < 1 applied to B.
 * k_dw(dz, dr) = 1 - alpha * exp(-dr² / R²) * 1/(1 + (dz/R)²)
 *
 * Where:
 *   dz = vertical separation (B below A)
 *   dr = horizontal separation
 *   R  = rotor disk radius
 *   alpha = downwash strength (0..0.4)
 *
 * Effective range: ~5 rotor radii vertically.
 *
 * This captures the dominant effect (thrust loss) without modeling
 * full wake turbulence or angle-of-attack changes.
 */

import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('downwash', 'simplified',
  'Inter-drone downwash: thrust reduction when below another drone', {
    simplifications: ['Empirical 1/r^2 falloff', 'No wake turbulence or AoA change'],
  });

export interface DownwashConfig {
  enabled: boolean;
  strength: number;       // alpha (0..0.4). Default 0.2
  verticalRange: number;  // max dz to consider (m). Default 5.0
}

export function defaultDownwashConfig(): DownwashConfig {
  return { enabled: false, strength: 0.2, verticalRange: 5.0 };
}

export interface DroneLikeForDownwash {
  id: number;
  position: Float64Array;
  destroyed?: boolean;
}

/**
 * Compute downwash thrust multiplier for target drone.
 * NED: positive z = down, so "above" means z_other < z_target.
 *
 * @param target       Target drone (thrust being reduced)
 * @param allDrones    All drones (including target — self is skipped)
 * @param propRadius   Propeller radius (m)
 * @param config       Downwash model config
 * @returns Multiplier in (1-alpha, 1]
 */
export function computeDownwashMultiplier(
  target: DroneLikeForDownwash,
  allDrones: DroneLikeForDownwash[],
  propRadius: number,
  config: DownwashConfig,
): number {
  if (!config.enabled || propRadius <= 0) return 1.0;

  const R = propRadius;
  const tx = target.position[0];
  const ty = target.position[1];
  const tz = target.position[2];

  let totalReduction = 0;
  for (const other of allDrones) {
    if (other.id === target.id) continue;
    if (other.destroyed) continue;

    const dx = tx - other.position[0];
    const dy = ty - other.position[1];
    const dz = tz - other.position[2]; // >0 if target is below (NED z down)

    if (dz <= 0) continue; // other is at same level or below — no downwash on us
    if (dz > config.verticalRange) continue;

    const dr = Math.sqrt(dx * dx + dy * dy);
    // Horizontal falloff: exp(-(dr/R)²)
    const horizFactor = Math.exp(-(dr / R) * (dr / R));
    // Vertical falloff: 1/(1 + (dz/R)²)
    const vertFactor = 1 / (1 + (dz / R) * (dz / R));
    totalReduction += config.strength * horizFactor * vertFactor;
  }

  // Cap reduction to avoid negative thrust
  const multiplier = Math.max(0.5, 1 - totalReduction);
  return Math.min(1.0, multiplier);
}

/**
 * Compute downwash-induced wind velocity perturbation for target drone.
 * Downward (positive z in NED) flow from drones above, scaled by proximity.
 * Written to `out` Vec3, added to env wind downstream.
 */
export function computeDownwashWindPerturbation(
  out: Float64Array,
  target: DroneLikeForDownwash,
  allDrones: DroneLikeForDownwash[],
  propRadius: number,
  config: DownwashConfig,
): void {
  out[0] = 0; out[1] = 0; out[2] = 0;
  if (!config.enabled || propRadius <= 0) return;

  const R = propRadius;
  const tx = target.position[0], ty = target.position[1], tz = target.position[2];

  // Approx induced downwash velocity = thrust-to-weight * v_ind (Glauert).
  // At hover v_ind ≈ 5 m/s for typical quad. Scale by proximity.
  // Strong near-field component: wake can reach ~2*v_ind then decay.
  const MAX_WAKE_SPEED = 4.0; // m/s, peak inside wake column

  for (const other of allDrones) {
    if (other.id === target.id || other.destroyed) continue;
    const dz = tz - other.position[2];
    if (dz <= 0 || dz > config.verticalRange) continue;
    const dx = tx - other.position[0];
    const dy = ty - other.position[1];
    const dr = Math.sqrt(dx * dx + dy * dy);

    // Horizontal: Gaussian with sigma ~= R (inside wake column)
    const horizFactor = Math.exp(-(dr * dr) / (R * R));
    // Vertical: 1/(1 + (dz/R)²) — close drones see strong wake, distant weak
    const vertFactor = 1 / (1 + (dz / R) * (dz / R));
    // Downward wind (positive z in NED) — outflow from rotors above
    const wakeSpeed = MAX_WAKE_SPEED * config.strength * horizFactor * vertFactor;
    out[2] += wakeSpeed;

    // Lateral shear: near edge of wake column, outflow flares slightly outward.
    // Simplified: small lateral component in direction of horizontal offset.
    if (dr > 0.01 && dr < R * 2) {
      const edgeFactor = (dr / R) * Math.exp(-(dr / R) * (dr / R)) * config.strength * 0.5;
      out[0] += (dx / dr) * edgeFactor * MAX_WAKE_SPEED * vertFactor;
      out[1] += (dy / dr) * edgeFactor * MAX_WAKE_SPEED * vertFactor;
    }
  }
}
