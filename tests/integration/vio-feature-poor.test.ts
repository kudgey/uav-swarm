/**
 * Integration test: VIO degrades in feature-poor scenes.
 */
import { describe, it, expect } from 'vitest';
import { readCameraVIO } from '@sim/sensors/camera-vio';
import { createDroneState } from '@sim/physics/state';
import { createDefaultEnv } from '@sim/core/config-defaults';
import { DeterministicRNG } from '@sim/core/rng';
import type { CameraVIOSensorConfig } from '@sim/core/types';

describe('VIO feature-poor degradation', () => {
  it('good texture → valid, poor texture → invalid', () => {
    const state = createDroneState(4);
    state.position[2] = -2;
    const env = createDefaultEnv();
    env.heightAboveGround = 2;
    const config: CameraVIOSensorConfig = {
      enabled: true, rate: 30, positionNoiseBase: 0.05, attitudeNoiseBase: 0.01,
      featureThreshold: 0.15, maxAngularRateForQuality: 3, maxHeightForQuality: 15,
      latencyFrames: 1, dropoutProbability: 0,
    };

    // Good texture
    env.surfaceTextureQuality = 0.8;
    const good = readCameraVIO(state, env, config, new DeterministicRNG(42), 0);
    expect(good.valid).toBe(true);

    // Poor texture
    env.surfaceTextureQuality = 0.05;
    const poor = readCameraVIO(state, env, config, new DeterministicRNG(42), 0);
    expect(poor.valid).toBe(false);
  });
});
