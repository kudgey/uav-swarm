/**
 * Integration test: Free-fall behavior.
 * Zero motor commands. After 1s: z increased ~4.9m (NED), velocity.z ~ 9.81 m/s.
 * No attitude drift from identity quaternion.
 */

import { describe, it, expect } from 'vitest';
import { createRK4Scratch, rk4Step } from '@sim/physics/integrator';
import { createDroneState } from '@sim/physics/state';
import { defaultDroneParams } from '@sim/core/config-defaults';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import { v3Create } from '@lib/math';
import { qNorm } from '@sim/physics/quaternion';
import { createDefaultEnv } from '@sim/core/config-defaults';

const env = createDefaultEnv();

describe('Free-fall integration test', () => {
  it('1 second free-fall produces correct position and velocity', () => {
    const params = defaultDroneParams();
    // Zero drag for pure gravitational free-fall test
    params.dragCoeffLinear = v3Create(0, 0, 0);
    params.dragCoeffQuadratic = v3Create(0, 0, 0);
    const state = createDroneState(4);
    const scratch = createRK4Scratch(4);
    const dt = 0.001;
    const T = 1.0;
    const steps = Math.round(T / dt);

    for (let i = 0; i < steps; i++) {
      rk4Step(state, params, env, dt, scratch);
    }

    // Position: z should be ~4.9 m (NED: +z is down)
    const expectedZ = 0.5 * GRAVITY_MPS2 * T * T;
    expect(state.position[2]).toBeCloseTo(expectedZ, 2);
    expect(state.position[0]).toBeCloseTo(0, 5);
    expect(state.position[1]).toBeCloseTo(0, 5);

    // Velocity: vz should be ~9.81 m/s
    expect(state.velocity[2]).toBeCloseTo(GRAVITY_MPS2 * T, 2);
    expect(state.velocity[0]).toBeCloseTo(0, 5);
    expect(state.velocity[1]).toBeCloseTo(0, 5);
  });

  it('no attitude drift from identity quaternion during free-fall', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    const scratch = createRK4Scratch(4);
    const dt = 0.001;

    for (let i = 0; i < 1000; i++) {
      rk4Step(state, params, env, dt, scratch);
    }

    // Quaternion should remain identity [1,0,0,0]
    expect(state.quaternion[0]).toBeCloseTo(1, 10);
    expect(state.quaternion[1]).toBeCloseTo(0, 10);
    expect(state.quaternion[2]).toBeCloseTo(0, 10);
    expect(state.quaternion[3]).toBeCloseTo(0, 10);
    expect(qNorm(state.quaternion)).toBeCloseTo(1, 10);
  });

  it('angular velocity stays zero during free-fall', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    const scratch = createRK4Scratch(4);
    const dt = 0.001;

    for (let i = 0; i < 1000; i++) {
      rk4Step(state, params, env, dt, scratch);
    }

    expect(state.angularVelocity[0]).toBeCloseTo(0, 10);
    expect(state.angularVelocity[1]).toBeCloseTo(0, 10);
    expect(state.angularVelocity[2]).toBeCloseTo(0, 10);
  });
});
