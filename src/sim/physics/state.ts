/**
 * DroneState factory, clone, and reset.
 * All arrays are pre-allocated Float64Array — no per-frame allocations.
 */

import { v3Create } from '@lib/math';
import { qCreate } from './quaternion';
import type { DroneState, DroneStateDerivative } from '@sim/core/types';

/** Create a fresh drone state at origin, identity orientation, all zeros. */
export function createDroneState(numRotors = 4): DroneState {
  return {
    position: v3Create(0, 0, 0),
    velocity: v3Create(0, 0, 0),
    quaternion: qCreate(1, 0, 0, 0),
    angularVelocity: v3Create(0, 0, 0),
    motorSpeeds: new Float64Array(numRotors),
    motorCommands: new Float64Array(numRotors),
  };
}

/** Create a pre-allocated derivative struct. */
export function createDroneStateDerivative(numRotors = 4): DroneStateDerivative {
  return {
    dPosition: v3Create(),
    dVelocity: v3Create(),
    dQuaternion: new Float64Array(4),
    dAngularVelocity: v3Create(),
    dMotorSpeeds: new Float64Array(numRotors),
  };
}

/** Deep copy state from src to dst (must have same numRotors). */
export function copyDroneState(dst: DroneState, src: DroneState): void {
  dst.position.set(src.position);
  dst.velocity.set(src.velocity);
  dst.quaternion.set(src.quaternion);
  dst.angularVelocity.set(src.angularVelocity);
  dst.motorSpeeds.set(src.motorSpeeds);
  dst.motorCommands.set(src.motorCommands);
}

/** Reset state to origin, identity attitude, zero velocities and motors. */
export function resetDroneState(state: DroneState): void {
  state.position[0] = 0; state.position[1] = 0; state.position[2] = 0;
  state.velocity[0] = 0; state.velocity[1] = 0; state.velocity[2] = 0;
  state.quaternion[0] = 1; state.quaternion[1] = 0;
  state.quaternion[2] = 0; state.quaternion[3] = 0;
  state.angularVelocity[0] = 0; state.angularVelocity[1] = 0; state.angularVelocity[2] = 0;
  state.motorSpeeds.fill(0);
  state.motorCommands.fill(0);
}

/** Apply derivative: out = state + derivative * dt (for RK4 intermediate states). */
export function applyDerivative(
  out: DroneState,
  state: DroneState,
  deriv: DroneStateDerivative,
  dt: number,
): void {
  for (let i = 0; i < 3; i++) {
    out.position[i] = state.position[i] + deriv.dPosition[i] * dt;
    out.velocity[i] = state.velocity[i] + deriv.dVelocity[i] * dt;
    out.angularVelocity[i] = state.angularVelocity[i] + deriv.dAngularVelocity[i] * dt;
  }
  for (let i = 0; i < 4; i++) {
    out.quaternion[i] = state.quaternion[i] + deriv.dQuaternion[i] * dt;
  }
  const n = state.motorSpeeds.length;
  for (let i = 0; i < n; i++) {
    out.motorSpeeds[i] = state.motorSpeeds[i] + deriv.dMotorSpeeds[i] * dt;
  }
  // Commands don't change during integration — they're latched
  out.motorCommands.set(state.motorCommands);
}

/** Accumulate weighted derivative: accum += deriv * weight. */
export function accumDerivative(
  accum: DroneStateDerivative,
  deriv: DroneStateDerivative,
  weight: number,
): void {
  for (let i = 0; i < 3; i++) {
    accum.dPosition[i] += deriv.dPosition[i] * weight;
    accum.dVelocity[i] += deriv.dVelocity[i] * weight;
    accum.dAngularVelocity[i] += deriv.dAngularVelocity[i] * weight;
  }
  for (let i = 0; i < 4; i++) {
    accum.dQuaternion[i] += deriv.dQuaternion[i] * weight;
  }
  const n = accum.dMotorSpeeds.length;
  for (let i = 0; i < n; i++) {
    accum.dMotorSpeeds[i] += deriv.dMotorSpeeds[i] * weight;
  }
}

/** Zero out a derivative struct. */
export function zeroDerivative(deriv: DroneStateDerivative): void {
  deriv.dPosition[0] = 0; deriv.dPosition[1] = 0; deriv.dPosition[2] = 0;
  deriv.dVelocity[0] = 0; deriv.dVelocity[1] = 0; deriv.dVelocity[2] = 0;
  deriv.dQuaternion[0] = 0; deriv.dQuaternion[1] = 0;
  deriv.dQuaternion[2] = 0; deriv.dQuaternion[3] = 0;
  deriv.dAngularVelocity[0] = 0; deriv.dAngularVelocity[1] = 0; deriv.dAngularVelocity[2] = 0;
  deriv.dMotorSpeeds.fill(0);
}
