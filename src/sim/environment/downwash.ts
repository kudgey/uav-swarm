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
