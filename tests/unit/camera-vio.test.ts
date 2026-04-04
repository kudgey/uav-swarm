import { describe, it, expect } from 'vitest';
import { readCameraVIO } from '@sim/sensors/camera-vio';
import { createDroneState } from '@sim/physics/state';
import { createDefaultEnv } from '@sim/core/config-defaults';
import { DeterministicRNG } from '@sim/core/rng';
import type { CameraVIOSensorConfig } from '@sim/core/types';

const defaultVIOConfig: CameraVIOSensorConfig = {
  enabled: true, rate: 30, positionNoiseBase: 0.05, attitudeNoiseBase: 0.01,
  featureThreshold: 0.15, maxAngularRateForQuality: 3.0,
  maxHeightForQuality: 15.0, latencyFrames: 1, dropoutProbability: 0,
};

describe('Camera/VIO Sensor', () => {
  it('good conditions: valid output with reasonable noise', () => {
    const state = createDroneState(4);
    state.position[2] = -2;
    const env = createDefaultEnv();
    env.heightAboveGround = 2;
    env.surfaceTextureQuality = 0.8;
    const r = readCameraVIO(state, env, defaultVIOConfig, new DeterministicRNG(42), 0);
    expect(r.valid).toBe(true);
    expect(r.measurement.featureQuality).toBeGreaterThan(0.5);
    expect(Math.abs(r.measurement.position[2] - (-2))).toBeLessThan(0.5);
  });

  it('low texture: invalid', () => {
    const state = createDroneState(4);
    state.position[2] = -2;
    const env = createDefaultEnv();
    env.heightAboveGround = 2;
    env.surfaceTextureQuality = 0.05;
    const r = readCameraVIO(state, env, defaultVIOConfig, new DeterministicRNG(42), 0);
    expect(r.valid).toBe(false);
  });

  it('high angular rate degrades quality', () => {
    const state = createDroneState(4);
    state.position[2] = -2;
    state.angularVelocity[0] = 5; // high rate
    const env = createDefaultEnv();
    env.heightAboveGround = 2;
    env.surfaceTextureQuality = 0.8;
    const r = readCameraVIO(state, env, defaultVIOConfig, new DeterministicRNG(42), 0);
    // Quality should be low due to motion blur
    if (r.valid) {
      expect(r.measurement.featureQuality).toBeLessThan(0.3);
    }
  });

  it('noise scales inversely with quality', () => {
    const state = createDroneState(4);
    state.position[2] = -2;
    const env = createDefaultEnv();
    env.heightAboveGround = 2;

    // High quality
    env.surfaceTextureQuality = 1.0;
    const errs1: number[] = [];
    for (let i = 0; i < 100; i++) {
      const r = readCameraVIO(state, env, defaultVIOConfig, new DeterministicRNG(i), 0);
      if (r.valid) errs1.push(Math.abs(r.measurement.position[0]));
    }

    // Lower quality
    env.surfaceTextureQuality = 0.3;
    const errs2: number[] = [];
    for (let i = 0; i < 100; i++) {
      const r = readCameraVIO(state, env, defaultVIOConfig, new DeterministicRNG(i), 0);
      if (r.valid) errs2.push(Math.abs(r.measurement.position[0]));
    }

    const rms1 = Math.sqrt(errs1.reduce((s, e) => s + e * e, 0) / errs1.length);
    const rms2 = Math.sqrt(errs2.reduce((s, e) => s + e * e, 0) / errs2.length);
    expect(rms2).toBeGreaterThan(rms1); // worse quality = more noise
  });
});
