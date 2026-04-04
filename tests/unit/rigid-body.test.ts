import { describe, it, expect } from 'vitest';
import { computeDerivative } from '@sim/physics/rigid-body';
import { createDroneState, createDroneStateDerivative } from '@sim/physics/state';
import { defaultDroneParams, createDefaultEnv } from '@sim/core/config-defaults';
import { GRAVITY_MPS2 } from '@sim/core/frames';

const stubEnv = createDefaultEnv();

describe('Rigid-Body 6DoF Dynamics', () => {
  it('derivative at hover trim has near-zero translational acceleration', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    const deriv = createDroneStateDerivative(4);

    // Set motors to hover trim: omega = sqrt(mg / (4*kT))
    const omegaHover = Math.sqrt(params.mass * GRAVITY_MPS2 / (4 * params.kT));
    for (let i = 0; i < 4; i++) {
      state.motorSpeeds[i] = omegaHover;
      state.motorCommands[i] = omegaHover;
    }

    computeDerivative(deriv, state, params, stubEnv, 0.001);

    // dVelocity should be near zero (gravity balanced by thrust)
    expect(Math.abs(deriv.dVelocity[0])).toBeLessThan(0.01);
    expect(Math.abs(deriv.dVelocity[1])).toBeLessThan(0.01);
    expect(Math.abs(deriv.dVelocity[2])).toBeLessThan(0.01);
  });

  it('zero motors gives gravity-only acceleration', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    const deriv = createDroneStateDerivative(4);

    computeDerivative(deriv, state, params, stubEnv, 0.001);

    // Only gravity: dv/dt = [0, 0, +g]
    expect(deriv.dVelocity[0]).toBeCloseTo(0);
    expect(deriv.dVelocity[1]).toBeCloseTo(0);
    expect(deriv.dVelocity[2]).toBeCloseTo(GRAVITY_MPS2, 3);
  });

  it('angular acceleration is zero at hover trim with zero omega', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    const deriv = createDroneStateDerivative(4);

    const omegaHover = Math.sqrt(params.mass * GRAVITY_MPS2 / (4 * params.kT));
    for (let i = 0; i < 4; i++) {
      state.motorSpeeds[i] = omegaHover;
      state.motorCommands[i] = omegaHover;
    }

    computeDerivative(deriv, state, params, stubEnv, 0.001);

    // All equal thrusts, zero omega -> zero angular acceleration
    expect(Math.abs(deriv.dAngularVelocity[0])).toBeLessThan(1e-6);
    expect(Math.abs(deriv.dAngularVelocity[1])).toBeLessThan(1e-6);
    // Yaw: check if reactive torques cancel (they should for alternating dirs)
    expect(Math.abs(deriv.dAngularVelocity[2])).toBeLessThan(1e-6);
  });
});
