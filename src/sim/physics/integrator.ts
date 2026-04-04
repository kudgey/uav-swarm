/**
 * RK4 integrator for 6DoF drone state.
 * STATUS: experimental
 *
 * Classic 4th-order Runge-Kutta with:
 *   - Pre-allocated scratch buffers (no per-frame allocations)
 *   - Quaternion normalization after each full step
 *   - Motor dynamics integrated within each RK4 sub-evaluation
 *
 * Algorithm:
 *   k1 = f(S)
 *   k2 = f(S + 0.5*h*k1)
 *   k3 = f(S + 0.5*h*k2)
 *   k4 = f(S + h*k3)
 *   S_new = S + (h/6)*(k1 + 2*k2 + 2*k3 + k4)
 *   S_new.quaternion = normalize(S_new.quaternion)
 */

import { qNormalize } from './quaternion';
import {
  createDroneState,
  createDroneStateDerivative,
  applyDerivative,
  zeroDerivative,
  accumDerivative,
} from './state';
import { computeDerivative } from './rigid-body';
import type { DroneState, DroneStateDerivative, DroneParams, EnvironmentOutput } from '@sim/core/types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('integrator', 'experimental', 'RK4 integrator with quaternion normalization');

/**
 * Pre-allocated scratch space for RK4.
 * Created once per integrator instance.
 */
export interface RK4Scratch {
  k1: DroneStateDerivative;
  k2: DroneStateDerivative;
  k3: DroneStateDerivative;
  k4: DroneStateDerivative;
  kAccum: DroneStateDerivative;
  tmpState: DroneState;
}

export function createRK4Scratch(numRotors = 4): RK4Scratch {
  return {
    k1: createDroneStateDerivative(numRotors),
    k2: createDroneStateDerivative(numRotors),
    k3: createDroneStateDerivative(numRotors),
    k4: createDroneStateDerivative(numRotors),
    kAccum: createDroneStateDerivative(numRotors),
    tmpState: createDroneState(numRotors),
  };
}

/**
 * Perform one RK4 integration step. Updates state IN-PLACE.
 *
 * @param state    Drone state (modified in place)
 * @param params   Drone parameters
 * @param env      Environment output
 * @param dt       Physics timestep (s)
 * @param scratch  Pre-allocated scratch buffers
 */
export function rk4Step(
  state: DroneState,
  params: DroneParams,
  env: EnvironmentOutput,
  dt: number,
  scratch: RK4Scratch,
): void {
  const { k1, k2, k3, k4, kAccum, tmpState } = scratch;
  const halfDt = dt * 0.5;

  // k1 = f(state)
  computeDerivative(k1, state, params, env, dt);

  // k2 = f(state + 0.5*dt*k1)
  applyDerivative(tmpState, state, k1, halfDt);
  computeDerivative(k2, tmpState, params, env, dt);

  // k3 = f(state + 0.5*dt*k2)
  applyDerivative(tmpState, state, k2, halfDt);
  computeDerivative(k3, tmpState, params, env, dt);

  // k4 = f(state + dt*k3)
  applyDerivative(tmpState, state, k3, dt);
  computeDerivative(k4, tmpState, params, env, dt);

  // Accumulate: kAccum = (k1 + 2*k2 + 2*k3 + k4) / 6
  zeroDerivative(kAccum);
  accumDerivative(kAccum, k1, 1 / 6);
  accumDerivative(kAccum, k2, 2 / 6);
  accumDerivative(kAccum, k3, 2 / 6);
  accumDerivative(kAccum, k4, 1 / 6);

  // Apply: state = state + dt * kAccum
  applyDerivative(state, state, kAccum, dt);

  // Normalize quaternion to prevent drift
  qNormalize(state.quaternion, state.quaternion);
}
