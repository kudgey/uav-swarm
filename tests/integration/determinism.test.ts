/**
 * Integration test: Determinism.
 * Two runs with identical seed and config. Compare full state at t=10s.
 * Must be bit-identical within the same Node/V8 runtime.
 */

import { describe, it, expect } from 'vitest';
import { createRK4Scratch, rk4Step } from '@sim/physics/integrator';
import { createDroneState } from '@sim/physics/state';
import { defaultDroneParams } from '@sim/core/config-defaults';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import { createDefaultEnv } from '@sim/core/config-defaults';
import type { DroneState } from '@sim/core/types';

const env = createDefaultEnv();

function runSimulation(dt: number, totalTime: number): DroneState {
  const params = defaultDroneParams();
  const state = createDroneState(4);
  const scratch = createRK4Scratch(4);
  const steps = Math.round(totalTime / dt);

  // Set hover trim with a small perturbation for interesting dynamics
  const omegaHover = Math.sqrt(params.mass * GRAVITY_MPS2 / (4 * params.kT));
  for (let i = 0; i < 4; i++) {
    state.motorSpeeds[i] = omegaHover;
    state.motorCommands[i] = omegaHover * (1 + (i - 1.5) * 0.02);
  }

  for (let i = 0; i < steps; i++) {
    rk4Step(state, params, env, dt, scratch);
  }

  return state;
}

describe('Determinism', () => {
  it('two identical runs produce bit-identical state at t=10s', () => {
    const dt = 0.001;
    const T = 10.0;

    const stateA = runSimulation(dt, T);
    const stateB = runSimulation(dt, T);

    // Bit-identical comparison
    for (let i = 0; i < 3; i++) {
      expect(stateA.position[i]).toBe(stateB.position[i]);
      expect(stateA.velocity[i]).toBe(stateB.velocity[i]);
      expect(stateA.angularVelocity[i]).toBe(stateB.angularVelocity[i]);
    }
    for (let i = 0; i < 4; i++) {
      expect(stateA.quaternion[i]).toBe(stateB.quaternion[i]);
    }
    for (let i = 0; i < stateA.motorSpeeds.length; i++) {
      expect(stateA.motorSpeeds[i]).toBe(stateB.motorSpeeds[i]);
    }
  }, 30000);
});
