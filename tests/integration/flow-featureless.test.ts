/**
 * Integration test: Optical flow invalid over featureless surface.
 */
import { describe, it, expect } from 'vitest';
import { readOpticalFlow } from '@sim/sensors/optical-flow';
import { createDroneState } from '@sim/physics/state';
import { defaultSensorSuiteConfig, createDefaultEnv } from '@sim/core/config-defaults';
import { DeterministicRNG } from '@sim/core/rng';

describe('Optical flow featureless', () => {
  it('textureQuality=0: all readings invalid', () => {
    const state = createDroneState(4);
    state.position[2] = -2;
    const env = createDefaultEnv();
    env.heightAboveGround = 2;
    env.surfaceTextureQuality = 0;
    const config = { ...defaultSensorSuiteConfig().opticalFlow, textureThreshold: 0.2 };

    let invalidCount = 0;
    for (let i = 0; i < 100; i++) {
      const r = readOpticalFlow(state, env, config, new DeterministicRNG(i), i * 0.033);
      if (!r.valid) invalidCount++;
    }
    expect(invalidCount).toBe(100);
  });
});
