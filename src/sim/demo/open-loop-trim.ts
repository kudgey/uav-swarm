/**
 * Open-loop hover trim calculator.
 * STATUS: experimental, demoOnly
 *
 * NOT in acceptance path. This is a demo/exploration tool.
 */

import type { DroneParams } from '@sim/core/types';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('demo-trim', 'experimental', 'Open-loop hover trim calculator', { demoOnly: true });

export interface TrimResult {
  feasible: boolean;
  omegaHover: number;       // rad/s required for hover
  omegaMax: number;          // rad/s available
  utilizationPercent: number; // omegaHover / omegaMax * 100
  deficitPercent?: number;   // how much over max, if infeasible
}

/**
 * Compute hover trim speed for all rotors.
 *
 * Omega_hover = sqrt(m * g / (N * kT))
 *
 * Returns feasibility info. Does NOT silently clamp.
 */
export function computeHoverTrim(params: DroneParams): TrimResult {
  const totalWeightN = params.mass * GRAVITY_MPS2;
  const thrustPerRotor = totalWeightN / params.numRotors;
  const omegaHover = Math.sqrt(thrustPerRotor / params.kT);
  const utilization = (omegaHover / params.motorOmegaMax) * 100;
  const feasible = omegaHover <= params.motorOmegaMax;

  return {
    feasible,
    omegaHover,
    omegaMax: params.motorOmegaMax,
    utilizationPercent: utilization,
    deficitPercent: feasible ? undefined : utilization - 100,
  };
}

/**
 * Fill motor commands with hover trim speed.
 * Returns TrimResult for UI feedback.
 */
export function applyHoverTrim(
  motorCommands: Float64Array,
  params: DroneParams,
): TrimResult {
  const result = computeHoverTrim(params);
  const omega = result.feasible ? result.omegaHover : params.motorOmegaMax;
  for (let i = 0; i < params.numRotors; i++) {
    motorCommands[i] = omega;
  }
  return result;
}
