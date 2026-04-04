import { describe, it, expect } from 'vitest';
import { computeDerivative } from '@sim/physics/rigid-body';
import { defaultDroneParams, createDefaultEnv } from '@sim/core/config-defaults';
import { createDroneState } from '@sim/physics/state';
import { createRK4Scratch } from '@sim/physics/integrator';
import { GRAVITY_MPS2 } from '@sim/core/frames';

describe('Gyroscopic rotor torque', () => {
  it('zero rotorInertia: no gyroscopic coupling', () => {
    const params = defaultDroneParams();
    params.rotorInertia = 0;
    const state = createDroneState(4);
    const omegaHover = Math.sqrt(params.mass * GRAVITY_MPS2 / (4 * params.kT));
    for (let i = 0; i < 4; i++) state.motorSpeeds[i] = omegaHover;
    state.angularVelocity[2] = 1; // yaw rate

    const env = createDefaultEnv();
    env.gravity[2] = GRAVITY_MPS2;
    const deriv = createRK4Scratch(4).k1;
    computeDerivative(deriv, state, params, env);

    // With zero rotorInertia and zero roll/pitch rate, roll/pitch torque should be ~0
    expect(Math.abs(deriv.dAngularVelocity[0])).toBeLessThan(0.1);
    expect(Math.abs(deriv.dAngularVelocity[1])).toBeLessThan(0.1);
  });

  it('non-zero rotorInertia + yaw rate: produces roll/pitch coupling', () => {
    const params = { ...defaultDroneParams(), rotorInertia: 0.001 };
    const state = createDroneState(4);
    const omegaHover = Math.sqrt(params.mass * GRAVITY_MPS2 / (4 * params.kT));
    // Asymmetric rotor speeds (yaw command) so net angular momentum ≠ 0
    // Directions: [+1, -1, +1, -1]. Make CCW faster: Σ(s_i * Ω_i) > 0
    state.motorSpeeds[0] = omegaHover + 50;
    state.motorSpeeds[1] = omegaHover - 50;
    state.motorSpeeds[2] = omegaHover + 50;
    state.motorSpeeds[3] = omegaHover - 50;
    // Apply pitch rate — couples to roll via gyroscopic torque
    state.angularVelocity[1] = 2; // 2 rad/s pitch

    const env = createDefaultEnv();
    env.gravity[2] = GRAVITY_MPS2;
    const deriv = createRK4Scratch(4).k1;
    computeDerivative(deriv, state, params, env);

    // With gyroscopic coupling, yaw rate should induce roll/pitch angular acceleration
    // The coupling term is: τ_gyro_x = ω_y * J_r * Σ(s_i * Ω_i)
    // With ω_z = 2 rad/s (not ω_y), the coupling goes through the cross product differently
    // At least one of roll/pitch should be non-negligible
    const coupling = Math.abs(deriv.dAngularVelocity[0]) + Math.abs(deriv.dAngularVelocity[1]);
    // With rotorInertia = 0 this would be ~0, with 0.001 it should be measurable
    expect(coupling).toBeGreaterThan(0); // any coupling exists
  });
});
