/**
 * Integration test: Torque sign convention.
 * From hover trim, add differential thrust per axis.
 * After 0.1s, verify angular velocity direction matches expected sign.
 */

import { describe, it, expect } from 'vitest';
import { createRK4Scratch, rk4Step } from '@sim/physics/integrator';
import { createDroneState } from '@sim/physics/state';
import { defaultDroneParams } from '@sim/core/config-defaults';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import { createDefaultEnv } from '@sim/core/config-defaults';

const env = createDefaultEnv();

describe('Torque sign conventions', () => {
  const params = defaultDroneParams();
  const dt = 0.001;
  const omegaHover = Math.sqrt(params.mass * GRAVITY_MPS2 / (4 * params.kT));
  const delta = omegaHover * 0.1; // 10% differential

  function runWithDifferential(motorOffsets: number[]): Float64Array {
    const state = createDroneState(4);
    const scratch = createRK4Scratch(4);
    for (let i = 0; i < 4; i++) {
      state.motorSpeeds[i] = omegaHover + motorOffsets[i];
      state.motorCommands[i] = omegaHover + motorOffsets[i];
    }
    // Run 0.1 seconds
    for (let i = 0; i < 100; i++) {
      rk4Step(state, params, env, dt, scratch);
    }
    return state.angularVelocity;
  }

  it('positive roll torque produces positive roll rate', () => {
    // tau_x = d*(T1 - T2 - T3 + T4)
    // To get positive tau_x: increase rotors 1,4 and decrease 2,3
    const omega = runWithDifferential([+delta, -delta, -delta, +delta]);
    expect(omega[0]).toBeGreaterThan(0); // positive roll rate
  });

  it('positive pitch torque produces positive pitch rate', () => {
    // tau_y = d*(T1 + T2 - T3 - T4)
    // To get positive tau_y: increase rotors 1,2 and decrease 3,4
    const omega = runWithDifferential([+delta, +delta, -delta, -delta]);
    expect(omega[1]).toBeGreaterThan(0); // positive pitch rate
  });

  it('positive yaw torque produces positive yaw rate', () => {
    // tau_z from reactive torques: s_i * kQ * omega_i^2
    // s = [+1, -1, +1, -1], so increase CCW rotors (1,3) for positive yaw
    const omega = runWithDifferential([+delta, -delta, +delta, -delta]);
    expect(omega[2]).toBeGreaterThan(0); // positive yaw rate
  });
});
