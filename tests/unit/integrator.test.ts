import { describe, it, expect } from 'vitest';
import { createRK4Scratch, rk4Step } from '@sim/physics/integrator';
import { createDroneState } from '@sim/physics/state';
import { defaultDroneParams } from '@sim/core/config-defaults';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import { v3Create } from '@lib/math';
import { qNorm } from '@sim/physics/quaternion';
import { createDefaultEnv } from '@sim/core/config-defaults';

const stubEnv = createDefaultEnv();

describe('RK4 Integrator', () => {
  it('free-fall position matches analytical 0.5*g*t^2', () => {
    const params = defaultDroneParams();
    // Zero drag for analytical comparison
    params.dragCoeffLinear = v3Create(0, 0, 0);
    params.dragCoeffQuadratic = v3Create(0, 0, 0);
    const state = createDroneState(4);
    const scratch = createRK4Scratch(4);
    const dt = 0.001;
    const T = 1.0;
    const steps = Math.round(T / dt);

    for (let i = 0; i < steps; i++) {
      rk4Step(state, params, stubEnv, dt, scratch);
    }

    // In NED, free-fall z increases (downward)
    const expectedZ = 0.5 * GRAVITY_MPS2 * T * T;
    expect(state.position[2]).toBeCloseTo(expectedZ, 3);
    expect(state.velocity[2]).toBeCloseTo(GRAVITY_MPS2 * T, 3);
  });

  it('quaternion stays normalized after 10000 steps', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    const scratch = createRK4Scratch(4);
    // Give it some angular velocity
    state.angularVelocity[0] = 1.0;
    state.angularVelocity[1] = 0.5;

    const dt = 0.001;
    for (let i = 0; i < 10000; i++) {
      rk4Step(state, params, stubEnv, dt, scratch);
    }

    expect(qNorm(state.quaternion)).toBeCloseTo(1.0, 10);
  });
});
