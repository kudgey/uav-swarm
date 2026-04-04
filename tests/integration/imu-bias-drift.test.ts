/**
 * Integration test: IMU bias drift over 60 seconds.
 */
import { describe, it, expect } from 'vitest';
import { createSensorBiasState, stepBiasRandomWalks } from '@sim/sensors/sensor-bias-state';
import { defaultSensorSuiteConfig } from '@sim/core/config-defaults';
import { DeterministicRNG } from '@sim/core/rng';
import { v3Len } from '@lib/math';

describe('IMU bias drift', () => {
  it('gyro and accel biases drift measurably over 60 seconds', () => {
    const config = defaultSensorSuiteConfig();
    const bias = createSensorBiasState();
    const rng = new DeterministicRNG(42);
    const dt = 0.001;

    for (let i = 0; i < 60000; i++) {
      stepBiasRandomWalks(bias, dt, rng, config);
    }

    expect(v3Len(bias.gyroBias)).toBeGreaterThan(0.001);
    expect(v3Len(bias.accelBias)).toBeGreaterThan(0.01);
    expect(Math.abs(bias.baroBias)).toBeGreaterThan(0.01);
  });
});
