/**
 * Integration test: Hover trim consistency.
 * Set all motors to Omega_hover = sqrt(mg/(4*kT)). Run 5s.
 * Verify: net acceleration ~0, position drift < tolerance.
 * This tests physics trim, NOT controller stability.
 */

import { describe, it, expect } from 'vitest';
import { createRK4Scratch, rk4Step } from '@sim/physics/integrator';
import { createDroneState } from '@sim/physics/state';
import { defaultDroneParams } from '@sim/core/config-defaults';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import { v3Len } from '@lib/math';
import { createDefaultEnv } from '@sim/core/config-defaults';

const env = createDefaultEnv();

describe('Hover trim consistency', () => {
  it('trim motor speeds maintain near-zero drift over 5 seconds', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    const scratch = createRK4Scratch(4);
    const dt = 0.001;
    const T = 5.0;
    const steps = Math.round(T / dt);

    // Set hover trim
    const omegaHover = Math.sqrt(params.mass * GRAVITY_MPS2 / (4 * params.kT));
    for (let i = 0; i < 4; i++) {
      state.motorSpeeds[i] = omegaHover;
      state.motorCommands[i] = omegaHover;
    }

    for (let i = 0; i < steps; i++) {
      rk4Step(state, params, env, dt, scratch);
    }

    // Position should not drift significantly
    const posDrift = v3Len(state.position);
    expect(posDrift).toBeLessThan(0.01); // < 1cm drift

    // Velocity should be near zero
    const velMag = v3Len(state.velocity);
    expect(velMag).toBeLessThan(0.01); // < 1cm/s
  });

  it('attitude stays at identity during open-loop hover trim', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    const scratch = createRK4Scratch(4);
    const dt = 0.001;

    const omegaHover = Math.sqrt(params.mass * GRAVITY_MPS2 / (4 * params.kT));
    for (let i = 0; i < 4; i++) {
      state.motorSpeeds[i] = omegaHover;
      state.motorCommands[i] = omegaHover;
    }

    for (let i = 0; i < 5000; i++) {
      rk4Step(state, params, env, dt, scratch);
    }

    expect(state.quaternion[0]).toBeCloseTo(1, 8);
    expect(state.quaternion[1]).toBeCloseTo(0, 8);
    expect(state.quaternion[2]).toBeCloseTo(0, 8);
    expect(state.quaternion[3]).toBeCloseTo(0, 8);
  });
});
