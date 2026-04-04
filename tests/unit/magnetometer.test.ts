import { describe, it, expect } from 'vitest';
import { readMagnetometer } from '@sim/sensors/magnetometer';
import { createDroneState } from '@sim/physics/state';
import { defaultDroneParams, defaultSensorSuiteConfig, createDefaultEnv } from '@sim/core/config-defaults';
import { DeterministicRNG } from '@sim/core/rng';
import { v3Create, m3Identity, m3Create } from '@lib/math';

describe('Magnetometer Sensor', () => {
  it('identity quaternion + zero noise = earth field in body = earth field in world', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    const env = createDefaultEnv();
    env.earthMagneticField[0] = 20e-6; // North
    env.earthMagneticField[1] = 5e-6;  // East
    env.earthMagneticField[2] = 40e-6; // Down
    const softIron = m3Create(); m3Identity(softIron);
    const config = { ...defaultSensorSuiteConfig().magnetometer,
      noiseDensity: 0, hardIron: v3Create(0,0,0), softIron, emiCoefficient: 0, dropoutProbability: 0 };
    const rng = new DeterministicRNG(42);

    const r = readMagnetometer(state, params, env, config, rng, 0);
    expect(r.valid).toBe(true);
    expect(r.measurement.field[0]).toBeCloseTo(20e-6, 9);
    expect(r.measurement.field[1]).toBeCloseTo(5e-6, 9);
    expect(r.measurement.field[2]).toBeCloseTo(40e-6, 9);
  });

  it('hard-iron bias shifts measurement', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    const env = createDefaultEnv();
    env.earthMagneticField[0] = 20e-6;
    const softIron = m3Create(); m3Identity(softIron);
    const config = { ...defaultSensorSuiteConfig().magnetometer,
      noiseDensity: 0, hardIron: v3Create(5e-6, 0, 0), softIron, emiCoefficient: 0, dropoutProbability: 0 };
    const rng = new DeterministicRNG(42);

    const r = readMagnetometer(state, params, env, config, rng, 0);
    expect(r.measurement.field[0]).toBeCloseTo(25e-6, 9); // 20 + 5
  });

  it('motor EMI changes measurement proportional to speed', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    const env = createDefaultEnv();
    env.earthMagneticField[2] = 40e-6;
    const softIron = m3Create(); m3Identity(softIron);
    const config = { ...defaultSensorSuiteConfig().magnetometer,
      noiseDensity: 0, hardIron: v3Create(0,0,0), softIron, emiCoefficient: 1e-10, dropoutProbability: 0 };

    // Motors off
    const rng1 = new DeterministicRNG(42);
    const r1 = readMagnetometer(state, params, env, config, rng1, 0);
    const fieldZ_off = r1.measurement.field[2]; // capture before next call overwrites scratch

    // Motors at 500 rad/s
    for (let i = 0; i < 4; i++) state.motorSpeeds[i] = 500;
    const rng2 = new DeterministicRNG(42);
    const r2 = readMagnetometer(state, params, env, config, rng2, 0);
    const fieldZ_on = r2.measurement.field[2];

    // z-component should differ due to EMI
    expect(Math.abs(fieldZ_on - fieldZ_off)).toBeGreaterThan(1e-8);
  });
});
