/**
 * Ground effect thrust multiplier (Cheeseman-Bennett 1955).
 * STATUS: experimental
 *
 * k_ge(h) = 1 / (1 - (R_prop / (4h))^2)
 *
 * Active only for 0 < h < 4 * R_prop.
 * h <= 0 (on/below ground): returns 1.0 (disabled — no contact model yet).
 * h >= 4 * R_prop: returns 1.0 (no ground effect at altitude).
 * Clamped to maximum 1.5 to avoid singularity near ground.
 */

import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('ground-effect', 'experimental', 'Cheeseman-Bennett ground effect + optional wall/ceiling');

/**
 * Compute ground effect thrust multiplier.
 * @param heightAboveGround  Height in meters (>= 0 from environment)
 * @param propRadius         Propeller radius in meters
 * @returns Multiplier >= 1.0
 */
export function computeGroundEffectMultiplier(
  heightAboveGround: number,
  propRadius: number,
): number {
  // Disabled if on/below ground or above influence zone
  if (heightAboveGround <= 0 || propRadius <= 0) return 1.0;

  const hMax = 4 * propRadius;
  if (heightAboveGround >= hMax) return 1.0;

  // Cheeseman-Bennett formula
  const ratio = propRadius / (4 * heightAboveGround);
  const denominator = 1 - ratio * ratio;

  // Clamp to avoid singularity and unrealistic values
  if (denominator <= 0) return 1.5;
  const multiplier = 1 / denominator;
  return Math.min(multiplier, 1.5);
}

/**
 * Compute wall/ceiling proximity factor.
 * @param wallDist  Nearest wall distance (m). Infinity = no wall.
 * @param ceilDist  Nearest ceiling distance (m). Infinity = no ceiling.
 * @param propRadius  Propeller radius (m).
 * @returns Multiplicative factor >= 1.0
 */
export function computeWallCeilingFactor(
  wallDist: number,
  ceilDist: number,
  propRadius: number,
): number {
  if (propRadius <= 0) return 1.0;
  let factor = 1.0;
  // Wall blockage: Gaussian decay
  if (isFinite(wallDist) && wallDist > 0 && wallDist < 4 * propRadius) {
    factor *= 1 + 0.15 * Math.exp(-wallDist / propRadius);
  }
  // Ceiling recirculation: Gaussian decay
  if (isFinite(ceilDist) && ceilDist > 0 && ceilDist < 4 * propRadius) {
    factor *= 1 + 0.2 * Math.exp(-ceilDist / propRadius);
  }
  return Math.min(factor, 1.5);
}
