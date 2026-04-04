/**
 * Sensor bias truth state.
 * Evolves via Euler-stepped random walks (NOT in RK4 — stochastic, not smooth ODE).
 * Lives parallel to DroneState, stepped once per physics tick.
 */

import { v3Create } from '@lib/math';
import type { Vec3, SensorSuiteConfig } from '@sim/core/types';
import type { DeterministicRNG } from '@sim/core/rng';

export interface SensorBiasState {
  gyroBias: Vec3;      // rad/s
  accelBias: Vec3;     // m/s^2
  magBias: Vec3;       // T (hard-iron drift component)
  baroBias: number;    // Pa
}

export function createSensorBiasState(): SensorBiasState {
  return {
    gyroBias: v3Create(0, 0, 0),
    accelBias: v3Create(0, 0, 0),
    magBias: v3Create(0, 0, 0),
    baroBias: 0,
  };
}

/**
 * Advance bias random walks by dt.
 * bias += sqrt(dt) * rw_rate * N(0,1) per axis.
 */
export function stepBiasRandomWalks(
  state: SensorBiasState,
  dt: number,
  rng: DeterministicRNG,
  config: SensorSuiteConfig,
): void {
  const sqrtDt = Math.sqrt(dt);

  // Gyro bias random walk
  if (config.imu.enabled) {
    const gRW = config.imu.gyroBiasRW;
    for (let i = 0; i < 3; i++) {
      state.gyroBias[i] += sqrtDt * gRW * rng.gaussian();
    }
  }

  // Accel bias random walk
  if (config.imu.enabled) {
    const aRW = config.imu.accelBiasRW;
    for (let i = 0; i < 3; i++) {
      state.accelBias[i] += sqrtDt * aRW * rng.gaussian();
    }
  }

  // Barometer bias random walk
  if (config.barometer.enabled) {
    state.baroBias += sqrtDt * config.barometer.biasRW * rng.gaussian();
  }
}

/** Reset all biases to initial values. */
export function resetSensorBiasState(state: SensorBiasState, config: SensorSuiteConfig): void {
  state.gyroBias[0] = 0; state.gyroBias[1] = 0; state.gyroBias[2] = 0;
  state.accelBias[0] = 0; state.accelBias[1] = 0; state.accelBias[2] = 0;
  state.magBias[0] = 0; state.magBias[1] = 0; state.magBias[2] = 0;
  state.baroBias = config.barometer.biasInitial;
}
