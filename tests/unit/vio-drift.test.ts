/**
 * Test: VIO odometric drift model.
 * With vioDriftEnabled=true, VIO position diverges from truth over distance.
 */
import { describe, it, expect } from 'vitest';
import { readCameraVIO, createVIODriftState } from '@sim/sensors/camera-vio';
import { createDroneState } from '@sim/physics/state';
import { createDefaultEnv } from '@sim/core/config-defaults';
import { DeterministicRNG } from '@sim/core/rng';
import type { CameraVIOSensorConfig } from '@sim/core/types';

const baseCfg: CameraVIOSensorConfig = {
  enabled: true, rate: 30, positionNoiseBase: 0.05, attitudeNoiseBase: 0.01,
  featureThreshold: 0.05, maxAngularRateForQuality: 3, maxHeightForQuality: 15,
  latencyFrames: 1, dropoutProbability: 0,
  cameraIntrinsicsEnabled: false, focalLength: 400,
  principalPoint: [320, 240], imageSize: [640, 480], distortionK1: 0,
  extrinsicsRotation: [0, 0, 0], extrinsicsTranslation: [0, 0, 0],
  vioDriftEnabled: false, vioScaleBiasRW: 0.001, vioYawDriftRW: 0.0005, vioPositionRW: 0.02,
};

describe('VIO drift model', () => {
  it('legacy mode (vioDriftEnabled=false): position tracks truth closely', () => {
    const cfg = { ...baseCfg, vioDriftEnabled: false };
    const state = createDroneState(4);
    state.position[0] = 100; state.position[1] = 50; state.position[2] = -2;
    const env = createDefaultEnv(); env.heightAboveGround = 2; env.surfaceTextureQuality = 0.8;
    const r = readCameraVIO(state, env, cfg, new DeterministicRNG(42), 1.0);
    expect(r.valid).toBe(true);
    // Legacy: position ≈ truth + small noise
    expect(Math.abs(r.measurement.position[0] - 100)).toBeLessThan(1);
    expect(Math.abs(r.measurement.position[1] - 50)).toBeLessThan(1);
  });

  it('drift mode: VIO position diverges from truth over simulated flight', () => {
    const cfg = { ...baseCfg, vioDriftEnabled: true, vioScaleBiasRW: 0.005, vioYawDriftRW: 0.002, vioPositionRW: 0.05 };
    const state = createDroneState(4);
    state.position[0] = 0; state.position[1] = 0; state.position[2] = -2;
    state.velocity[0] = 2; // flying North at 2 m/s
    const env = createDefaultEnv(); env.heightAboveGround = 2; env.surfaceTextureQuality = 0.8;
    const rng = new DeterministicRNG(42);
    const vioDrift = createVIODriftState(state.position);
    const dt = 1 / 30;

    // Simulate 30s of flight (60m distance)
    let lastVioX = 0;
    for (let t = 0; t < 30; t += dt) {
      state.position[0] = 2 * t; // truth moves at 2 m/s
      readCameraVIO(state, env, cfg, rng, t, undefined, vioDrift);
      lastVioX = vioDrift.position[0];
    }

    const truthX = 2 * 30; // 60m
    const driftError = Math.abs(lastVioX - truthX);
    // With aggressive drift params, expect > 0.5m error over 60m
    expect(driftError).toBeGreaterThan(0.5);
    // But not completely wrong (< 30m for 60m flight)
    expect(driftError).toBeLessThan(30);
  });

  it('feature-poor scene causes faster drift than high-feature scene', () => {
    const cfg = { ...baseCfg, vioDriftEnabled: true };
    const dt = 1 / 30;

    // Flight A: high texture quality
    const stateA = createDroneState(4); stateA.position[2] = -2; stateA.velocity[0] = 1;
    const envHigh = createDefaultEnv(); envHigh.heightAboveGround = 2; envHigh.surfaceTextureQuality = 0.9;
    const rngA = new DeterministicRNG(42);
    const driftA = createVIODriftState(stateA.position);
    for (let t = 0; t < 10; t += dt) {
      stateA.position[0] = t;
      readCameraVIO(stateA, envHigh, cfg, rngA, t, undefined, driftA);
    }
    const errorA = Math.abs(driftA.position[0] - 10);

    // Flight B: feature-poor scene
    const stateB = createDroneState(4); stateB.position[2] = -2; stateB.velocity[0] = 1;
    const envLow = createDefaultEnv(); envLow.heightAboveGround = 2; envLow.surfaceTextureQuality = 0.1;
    const rngB = new DeterministicRNG(42);
    const driftB = createVIODriftState(stateB.position);
    for (let t = 0; t < 10; t += dt) {
      stateB.position[0] = t;
      readCameraVIO(stateB, envLow, cfg, rngB, t, undefined, driftB);
    }
    const errorB = Math.abs(driftB.position[0] - 10);

    // Low quality should have meaningfully more drift
    expect(errorB).toBeGreaterThan(errorA);
  });

  it('drift grows with distance traveled, not just time', () => {
    const cfg = { ...baseCfg, vioDriftEnabled: true };
    const env = createDefaultEnv(); env.heightAboveGround = 2; env.surfaceTextureQuality = 0.8;
    const dt = 1 / 30;

    // Flight A: 2 m/s for 10s = 20m
    const stateA = createDroneState(4); stateA.position[2] = -2; stateA.velocity[0] = 2;
    const rngA = new DeterministicRNG(42);
    const driftA = createVIODriftState(stateA.position);
    for (let t = 0; t < 10; t += dt) {
      stateA.position[0] = 2 * t;
      readCameraVIO(stateA, env, cfg, rngA, t, undefined, driftA);
    }
    const errorA = Math.abs(driftA.position[0] - 20);

    // Flight B: 0.5 m/s for 10s = 5m (same time, less distance)
    const stateB = createDroneState(4); stateB.position[2] = -2; stateB.velocity[0] = 0.5;
    const rngB = new DeterministicRNG(42);
    const driftB = createVIODriftState(stateB.position);
    for (let t = 0; t < 10; t += dt) {
      stateB.position[0] = 0.5 * t;
      readCameraVIO(stateB, env, cfg, rngB, t, undefined, driftB);
    }
    const errorB = Math.abs(driftB.position[0] - 5);

    // Longer distance → more scale/yaw accumulated drift
    // errorA should be larger than errorB (for same time, more distance = more drift)
    expect(errorA).toBeGreaterThan(errorB);
  });
});
