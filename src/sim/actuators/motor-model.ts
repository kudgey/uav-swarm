/**
 * Hybrid actuator model: exponential first-order lag core + discrete constraints.
 * STATUS: experimental
 *
 * Pipeline per motor per step:
 *   1. Dead zone check on command
 *   2. Command saturation clamp
 *   3. First-order lag (analytical exponential, exact for unconstrained linear ODE)
 *   4. Rate limit (discrete clamp on delta-omega per dt)
 *   5. Output saturation clamp
 *
 * The exponential formula in step 3 is exact ONLY for the unconstrained linear ODE.
 * Steps 1, 4, 5 are discrete constraints, making the overall model hybrid.
 */

import { clamp } from '@lib/math';
import type { DroneParams } from '@sim/core/types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('motor-model', 'experimental', 'Hybrid actuator: exp lag + saturation + rate limit + dead zone');

/**
 * Step one motor for one timestep.
 *
 * @param omega     Current rotor speed (rad/s)
 * @param omegaCmd  Commanded rotor speed (rad/s)
 * @param dt        Timestep (s)
 * @param params    Drone parameters
 * @returns         New rotor speed (rad/s)
 */
export function stepMotor(
  omega: number,
  omegaCmd: number,
  dt: number,
  params: DroneParams,
): number {
  // 1. Dead zone: command below threshold treated as zero
  if (Math.abs(omegaCmd) < params.motorDeadZone) {
    omegaCmd = 0;
  }

  // 2. Command saturation
  omegaCmd = clamp(omegaCmd, params.motorOmegaMin, params.motorOmegaMax);

  // 3. Analytical first-order lag (exact for linear ODE: dω/dt = (ωcmd - ω) / τ)
  const expFactor = Math.exp(-dt / params.motorTau);
  const omegaLag = omegaCmd + (omega - omegaCmd) * expFactor;

  // 4. Rate limit (discrete constraint on change per step)
  let delta = omegaLag - omega;
  const maxDelta = params.motorRateLimit * dt;
  delta = clamp(delta, -maxDelta, maxDelta);
  let omegaNew = omega + delta;

  // 5. Output saturation
  omegaNew = clamp(omegaNew, 0, params.motorOmegaMax);

  return omegaNew;
}

/**
 * Step all motors for one timestep.
 * Writes new speeds into motorSpeeds array in-place.
 */
export function stepAllMotors(
  motorSpeeds: Float64Array,
  motorCommands: Float64Array,
  dt: number,
  params: DroneParams,
): void {
  for (let i = 0; i < params.numRotors; i++) {
    motorSpeeds[i] = stepMotor(motorSpeeds[i], motorCommands[i], dt, params);
  }
}
