/**
 * Integration test: Long-run numerical stability.
 * Set exact hover trim motor commands (open-loop). Run 60s.
 * Quaternion norm stays within 1e-10 of 1.0. No NaN/Inf.
 * Motor speeds stay constant within floating-point tolerance.
 */

import { describe, it, expect } from 'vitest';
import { createRK4Scratch, rk4Step } from '@sim/physics/integrator';
import { createDroneState } from '@sim/physics/state';
import { defaultDroneParams } from '@sim/core/config-defaults';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import { qNorm } from '@sim/physics/quaternion';
import { createDefaultEnv } from '@sim/core/config-defaults';

const env = createDefaultEnv();

describe('Long-run numerical stability', () => {
  it('60s hover trim: no NaN, quaternion normalized, no drift', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    const scratch = createRK4Scratch(4);
    const dt = 0.001;
    const T = 60.0;
    const steps = Math.round(T / dt);

    const omegaHover = Math.sqrt(params.mass * GRAVITY_MPS2 / (4 * params.kT));
    for (let i = 0; i < 4; i++) {
      state.motorSpeeds[i] = omegaHover;
      state.motorCommands[i] = omegaHover;
    }

    for (let i = 0; i < steps; i++) {
      rk4Step(state, params, env, dt, scratch);

      // Check for NaN/Inf every 1000 steps
      if (i % 1000 === 0) {
        for (let j = 0; j < 3; j++) {
          expect(isFinite(state.position[j])).toBe(true);
          expect(isFinite(state.velocity[j])).toBe(true);
          expect(isFinite(state.angularVelocity[j])).toBe(true);
        }
        for (let j = 0; j < 4; j++) {
          expect(isFinite(state.quaternion[j])).toBe(true);
        }
      }
    }

    // Final quaternion norm
    expect(qNorm(state.quaternion)).toBeCloseTo(1.0, 10);

    // Motor speeds should be at trim (commands are constant)
    for (let i = 0; i < 4; i++) {
      expect(state.motorSpeeds[i]).toBeCloseTo(omegaHover, 3);
    }

    // Position should not have drifted significantly
    expect(Math.abs(state.position[0])).toBeLessThan(0.1);
    expect(Math.abs(state.position[1])).toBeLessThan(0.1);
    expect(Math.abs(state.position[2])).toBeLessThan(0.1);
  }, 30000); // 30s timeout for 60k steps
});
