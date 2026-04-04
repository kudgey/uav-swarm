import { describe, it, expect } from 'vitest';
import { readImu } from '@sim/sensors/imu';
import { createDroneState } from '@sim/physics/state';
import { defaultDroneParams, defaultSensorSuiteConfig, createDefaultEnv } from '@sim/core/config-defaults';
import { createSensorBiasState, stepBiasRandomWalks } from '@sim/sensors/sensor-bias-state';
import { DeterministicRNG } from '@sim/core/rng';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import { v3Len } from '@lib/math';

describe('IMU Sensor', () => {
  it('hover reads approximately [0, 0, -g] specific force in body frame', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    const env = createDefaultEnv();
    const bias = createSensorBiasState();
    const config = { ...defaultSensorSuiteConfig().imu, gyroNoiseDensity: 0, accelNoiseDensity: 0 };
    const rng = new DeterministicRNG(42);

    // Set hover trim motors
    const omegaHover = Math.sqrt(params.mass * GRAVITY_MPS2 / (4 * params.kT));
    for (let i = 0; i < 4; i++) state.motorSpeeds[i] = omegaHover;

    const result = readImu(state, params, env, bias, config, rng, 0);
    expect(result.valid).toBe(true);
    // At hover: specific force = thrust/m = [0, 0, -g] in body (FRD)
    expect(result.measurement.accel[0]).toBeCloseTo(0, 2);
    expect(result.measurement.accel[1]).toBeCloseTo(0, 2);
    expect(result.measurement.accel[2]).toBeCloseTo(-GRAVITY_MPS2, 1);
  });

  it('gyro bias drift over 60s is measurable', () => {
    const config = defaultSensorSuiteConfig();
    const bias = createSensorBiasState();
    const rng = new DeterministicRNG(42);
    for (let i = 0; i < 60000; i++) { // 60s at 1000Hz
      stepBiasRandomWalks(bias, 0.001, rng, config);
    }
    expect(v3Len(bias.gyroBias)).toBeGreaterThan(0.001);
  });

  it('saturation clips at configured limit', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    state.angularVelocity[0] = 100; // way above saturation
    const env = createDefaultEnv();
    const bias = createSensorBiasState();
    const config = { ...defaultSensorSuiteConfig().imu, gyroNoiseDensity: 0, gyroSaturation: 35 };
    const rng = new DeterministicRNG(42);

    const result = readImu(state, params, env, bias, config, rng, 0);
    expect(Math.abs(result.measurement.gyro[0])).toBeLessThanOrEqual(35);
  });

  it('deterministic: same seed produces same readings', () => {
    const params = defaultDroneParams();
    const state = createDroneState(4);
    const env = createDefaultEnv();
    const bias = createSensorBiasState();
    const config = defaultSensorSuiteConfig().imu;

    const r1 = readImu(state, params, env, bias, config, new DeterministicRNG(42), 0);
    const r2 = readImu(state, params, env, bias, config, new DeterministicRNG(42), 0);
    expect(r1.measurement.gyro[0]).toBe(r2.measurement.gyro[0]);
    expect(r1.measurement.accel[2]).toBe(r2.measurement.accel[2]);
  });
});
