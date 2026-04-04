/**
 * Integration test: Magnetometer readings differ with motor speed (EMI).
 */
import { describe, it, expect } from 'vitest';
import { readMagnetometer } from '@sim/sensors/magnetometer';
import { createDroneState } from '@sim/physics/state';
import { defaultDroneParams, defaultSensorSuiteConfig, createDefaultEnv } from '@sim/core/config-defaults';
import { DeterministicRNG } from '@sim/core/rng';
import { v3Create, m3Create, m3Identity } from '@lib/math';

describe('Magnetometer EMI', () => {
  it('motors off vs full speed produce different readings', () => {
    const params = defaultDroneParams();
    const env = createDefaultEnv();
    env.earthMagneticField[2] = 40e-6;
    const softIron = m3Create(); m3Identity(softIron);
    const config = { ...defaultSensorSuiteConfig().magnetometer,
      noiseDensity: 0, hardIron: v3Create(0,0,0), softIron, emiCoefficient: 1e-9, dropoutProbability: 0 };

    // Motors off
    const stateOff = createDroneState(4);
    const rOff = readMagnetometer(stateOff, params, env, config, new DeterministicRNG(42), 0);
    const fieldZ_off = rOff.measurement.field[2]; // capture before next call

    // Motors at max
    const stateOn = createDroneState(4);
    for (let i = 0; i < 4; i++) stateOn.motorSpeeds[i] = params.motorOmegaMax;
    const rOn = readMagnetometer(stateOn, params, env, config, new DeterministicRNG(42), 0);
    const fieldZ_on = rOn.measurement.field[2];

    const diff = Math.abs(fieldZ_on - fieldZ_off);
    expect(diff).toBeGreaterThan(1e-7);
  });
});
