import { describe, it, expect } from 'vitest';
import { readOpticalFlow } from '@sim/sensors/optical-flow';
import { createDroneState } from '@sim/physics/state';
import { defaultSensorSuiteConfig, createDefaultEnv } from '@sim/core/config-defaults';
import { DeterministicRNG } from '@sim/core/rng';
import { qRotateVectorInverse } from '@sim/physics/quaternion';
import { v3Create } from '@lib/math';

describe('Optical Flow Sensor', () => {
  it('height scaling: flow halves when height doubles', () => {
    const config = { ...defaultSensorSuiteConfig().opticalFlow,
      noiseBase: 0, dropoutProbability: 0, heightScaleCoeff: 0 };

    // 1m height, 1 m/s forward
    const state1 = createDroneState(4);
    state1.position[2] = -1;
    state1.velocity[0] = 1; // North = body forward at identity quat
    const env1 = createDefaultEnv();
    env1.heightAboveGround = 1;
    const r1 = readOpticalFlow(state1, env1, config, new DeterministicRNG(42), 0);

    // 2m height, same velocity
    const state2 = createDroneState(4);
    state2.position[2] = -2;
    state2.velocity[0] = 1;
    const env2 = createDefaultEnv();
    env2.heightAboveGround = 2;
    const r2 = readOpticalFlow(state2, env2, config, new DeterministicRNG(42), 0);

    expect(r1.valid).toBe(true);
    expect(r2.valid).toBe(true);
    expect(Math.abs(r1.measurement.flowX)).toBeCloseTo(2 * Math.abs(r2.measurement.flowX), 1);
  });

  it('pure rotation produces nonzero flow', () => {
    const state = createDroneState(4);
    state.position[2] = -2;
    state.angularVelocity[1] = 0.5; // pitch rate (omega_y)
    const env = createDefaultEnv();
    env.heightAboveGround = 2;
    const config = { ...defaultSensorSuiteConfig().opticalFlow,
      noiseBase: 0, dropoutProbability: 0 };
    const r = readOpticalFlow(state, env, config, new DeterministicRNG(42), 0);
    expect(r.valid).toBe(true);
    // flow_x = vx/h + omega_y = 0/2 + 0.5 = 0.5
    expect(r.measurement.flowX).toBeCloseTo(0.5, 2);
  });

  it('featureless surface (textureQuality=0): invalid', () => {
    const state = createDroneState(4);
    state.position[2] = -2;
    const env = createDefaultEnv();
    env.heightAboveGround = 2;
    env.surfaceTextureQuality = 0;
    const config = { ...defaultSensorSuiteConfig().opticalFlow, textureThreshold: 0.2 };
    const r = readOpticalFlow(state, env, config, new DeterministicRNG(42), 0);
    expect(r.valid).toBe(false);
  });

  it('zero velocity + zero rotation = near-zero flow', () => {
    const state = createDroneState(4);
    state.position[2] = -2;
    const env = createDefaultEnv();
    env.heightAboveGround = 2;
    const config = { ...defaultSensorSuiteConfig().opticalFlow,
      noiseBase: 0, dropoutProbability: 0 };
    const r = readOpticalFlow(state, env, config, new DeterministicRNG(42), 0);
    expect(r.valid).toBe(true);
    expect(r.measurement.flowX).toBeCloseTo(0, 5);
    expect(r.measurement.flowY).toBeCloseTo(0, 5);
  });
});
