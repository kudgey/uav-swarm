/**
 * Rotor aerodynamic force and torque models.
 * STATUS: experimental
 *
 * T_i = kT * Omega_i^2    (thrust, always positive in -z body direction)
 * Q_i = kQ * Omega_i^2    (drag torque magnitude)
 * tau_z_i = s_i * Q_i     (yaw torque contribution, s_i = spin direction)
 */

import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('rotor-physics', 'experimental', 'Quadratic thrust/torque model');

/** Compute thrust from a single rotor. Always non-negative. */
export function computeThrust(omega: number, kT: number): number {
  return kT * omega * omega;
}

/** Compute drag torque magnitude from a single rotor. Always non-negative. */
export function computeDragTorque(omega: number, kQ: number): number {
  return kQ * omega * omega;
}

/** Compute yaw torque contribution from a single rotor. */
export function computeYawTorque(omega: number, kQ: number, direction: number): number {
  return direction * kQ * omega * omega;
}

/** Compute total thrust from all rotors. */
export function computeTotalThrust(motorSpeeds: Float64Array, kT: number, numRotors: number): number {
  let total = 0;
  for (let i = 0; i < numRotors; i++) {
    total += kT * motorSpeeds[i] * motorSpeeds[i];
  }
  return total;
}
