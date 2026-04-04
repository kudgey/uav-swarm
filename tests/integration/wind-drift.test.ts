/**
 * Integration test: Hover trim in wind causes drift.
 * Open-loop hover trim with 5 m/s wind → drone drifts downwind.
 */
import { describe, it, expect } from 'vitest';
import { createRK4Scratch, rk4Step } from '@sim/physics/integrator';
import { createDroneState } from '@sim/physics/state';
import { defaultDroneParams, createDefaultEnv } from '@sim/core/config-defaults';
import { GRAVITY_MPS2 } from '@sim/core/frames';

describe('Wind drift integration', () => {
  it('drone drifts in wind direction under open-loop hover trim', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    state.position[2] = -5; // 5m altitude
    const env = createDefaultEnv();
    env.heightAboveGround = 5;
    env.wind[0] = 5; // 5 m/s North wind

    const scratch = createRK4Scratch(4);
    const omegaHover = Math.sqrt(params.mass * GRAVITY_MPS2 / (4 * params.kT));
    for (let i = 0; i < 4; i++) {
      state.motorSpeeds[i] = omegaHover;
      state.motorCommands[i] = omegaHover;
    }

    // Run 2 seconds
    const dt = 0.001;
    for (let i = 0; i < 2000; i++) {
      rk4Step(state, params, env, dt, scratch);
    }

    // Drone should have drifted in the wind direction (positive x = North)
    // With drag opposing motion, drift won't be 5*2=10m, but should be measurable
    expect(state.position[0]).toBeGreaterThan(0.1); // Drifted north
  });
});
