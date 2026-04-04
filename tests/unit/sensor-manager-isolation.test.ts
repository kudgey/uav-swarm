import { describe, expect, it } from 'vitest';
import { createDroneState } from '@sim/physics/state';
import { createDefaultEnv, defaultDroneParams, defaultEnvironmentConfig, defaultSensorSuiteConfig } from '@sim/core/config-defaults';
import { SensorManager } from '@sim/sensors/sensor-manager';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { DeterministicRNG } from '@sim/core/rng';

describe('SensorManager isolation', () => {
  it('keeps per-drone IMU and magnetometer readings isolated', () => {
    const params = defaultDroneParams();
    const sensors = defaultSensorSuiteConfig();
    const worldGeo = new WorldGeometry(defaultEnvironmentConfig().scene);

    const mgr0 = new SensorManager(JSON.parse(JSON.stringify(sensors)), new DeterministicRNG(1), worldGeo, params);
    const mgr1 = new SensorManager(JSON.parse(JSON.stringify(sensors)), new DeterministicRNG(2), worldGeo, params);

    const env0 = createDefaultEnv();
    const env1 = createDefaultEnv();
    env0.earthMagneticField[0] = 40e-6;
    env1.earthMagneticField[1] = 55e-6;

    const s0 = createDroneState(4);
    const s1 = createDroneState(4);
    s0.angularVelocity[0] = 0.1;
    s1.angularVelocity[0] = 2.0;
    s0.angularVelocity[2] = 0.2;
    s1.angularVelocity[2] = -1.5;
    for (let i = 0; i < 4; i++) {
      s0.motorSpeeds[i] = 520;
      s1.motorSpeeds[i] = 780;
    }

    mgr0.readImu(s0, env0, 0);
    mgr0.readMag(s0, env0, 0);
    const imu0GyroX = mgr0.getLatestReadings().imu?.measurement.gyro[0] ?? NaN;
    const imu0GyroZ = mgr0.getLatestReadings().imu?.measurement.gyro[2] ?? NaN;
    const mag0Y = mgr0.getLatestReadings().magnetometer?.measurement.field[1] ?? NaN;

    mgr1.readImu(s1, env1, 0);
    mgr1.readMag(s1, env1, 0);

    expect(mgr0.getLatestReadings().imu?.measurement.gyro[0]).toBe(imu0GyroX);
    expect(mgr0.getLatestReadings().imu?.measurement.gyro[2]).toBe(imu0GyroZ);
    expect(mgr0.getLatestReadings().magnetometer?.measurement.field[1]).toBe(mag0Y);

    expect(mgr1.getLatestReadings().imu?.measurement.gyro[0]).not.toBe(imu0GyroX);
    expect(mgr1.getLatestReadings().magnetometer?.measurement.field[1]).not.toBe(mag0Y);
  });

  it('keeps per-drone VIO readings isolated', () => {
    const params = defaultDroneParams();
    const sensors = defaultSensorSuiteConfig();
    sensors.cameraVIO!.enabled = true;
    sensors.cameraVIO!.dropoutProbability = 0;
    const worldGeo = new WorldGeometry(defaultEnvironmentConfig().scene);

    const mgr0 = new SensorManager(JSON.parse(JSON.stringify(sensors)), new DeterministicRNG(3), worldGeo, params);
    const mgr1 = new SensorManager(JSON.parse(JSON.stringify(sensors)), new DeterministicRNG(4), worldGeo, params);

    const env = createDefaultEnv();
    env.heightAboveGround = 2;
    env.surfaceTextureQuality = 0.9;

    const s0 = createDroneState(4);
    const s1 = createDroneState(4);
    s0.position[0] = 1;
    s0.position[2] = -2;
    s1.position[0] = 7;
    s1.position[1] = 3;
    s1.position[2] = -2;

    mgr0.readCamera(s0, env, 0);
    const vio0x = mgr0.getLatestReadings().cameraVIO?.measurement.position[0] ?? NaN;
    const vio0y = mgr0.getLatestReadings().cameraVIO?.measurement.position[1] ?? NaN;

    mgr1.readCamera(s1, env, 0);

    expect(mgr0.getLatestReadings().cameraVIO?.measurement.position[0]).toBe(vio0x);
    expect(mgr0.getLatestReadings().cameraVIO?.measurement.position[1]).toBe(vio0y);
    expect(mgr1.getLatestReadings().cameraVIO?.measurement.position[0]).not.toBe(vio0x);
  });
});
