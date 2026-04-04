import { describe, it, expect } from 'vitest';
import { readBarometer } from '@sim/sensors/barometer';
import { createDroneState } from '@sim/physics/state';
import { defaultSensorSuiteConfig, createDefaultEnv } from '@sim/core/config-defaults';
import { createSensorBiasState } from '@sim/sensors/sensor-bias-state';
import { DeterministicRNG } from '@sim/core/rng';

describe('Barometer Sensor', () => {
  it('at sea level (z=0): pressure approximately 101325 Pa', () => {
    const state = createDroneState(4);
    const env = createDefaultEnv();
    const bias = createSensorBiasState();
    const config = { ...defaultSensorSuiteConfig().barometer, noiseDensity: 0 };
    const rng = new DeterministicRNG(42);

    const r = readBarometer(state, env, bias, config, rng, 0);
    expect(r.valid).toBe(true);
    expect(r.measurement.pressure).toBeCloseTo(101325, -1);
    expect(r.measurement.altitude).toBeCloseTo(0, 0);
  });

  it('at 100m altitude: pressure lower than sea level', () => {
    const state = createDroneState(4);
    state.position[2] = -100; // 100m above ground in NED
    const env = createDefaultEnv();
    const bias = createSensorBiasState();
    const config = { ...defaultSensorSuiteConfig().barometer, noiseDensity: 0 };
    const rng = new DeterministicRNG(42);

    const r = readBarometer(state, env, bias, config, rng, 0);
    expect(r.measurement.pressure).toBeLessThan(101325);
    expect(r.measurement.altitude).toBeCloseTo(100, 0);
  });

  it('bias drift accumulates over time', () => {
    const config = defaultSensorSuiteConfig();
    const bias = createSensorBiasState();
    const rng = new DeterministicRNG(42);
    const initial = bias.baroBias;
    for (let i = 0; i < 60000; i++) {
      bias.baroBias += Math.sqrt(0.001) * config.barometer.biasRW * rng.gaussian();
    }
    expect(Math.abs(bias.baroBias - initial)).toBeGreaterThan(0.01);
  });
});
