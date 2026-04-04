/**
 * IMU sensor model: gyroscope + accelerometer.
 * STATUS: experimental
 *
 * Gyro: omega_meas = omega_true + bias_gyro + noise
 * Accel: a_meas = R_bw^T * (F_thrust+F_drag)/m + bias_accel + noise
 *   (specific force in body frame — gravity excluded)
 *
 * Both: saturation clipping, white noise from DeterministicRNG.
 */

import { v3Create, v3Zero, clamp } from '@lib/math';
import { qRotateVector, qRotateVectorInverse } from '@sim/physics/quaternion';
import { computeBodyDrag } from '@sim/physics/drag';
import type { Vec3, DroneState, DroneParams, EnvironmentOutput, ImuSensorConfig } from '@sim/core/types';
import type { DeterministicRNG } from '@sim/core/rng';
import type { SensorBiasState } from './sensor-bias-state';
import type { SensorOutput, ImuMeasurement } from './sensor-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('sensor-imu', 'experimental', 'IMU with bias, white noise, saturation');

// Pre-allocated scratch
const _thrustBody = v3Create();
const _thrustWorld = v3Create();
const _dragBody = v3Create();
const _specificForceWorld = v3Create();
const _specificForceBody = v3Create();
const _dragWorld = v3Create();
const _vRelWorld = v3Create();
const _vRelBody = v3Create();
const _gyroOut = v3Create();
const _accelOut = v3Create();

export function readImu(
  state: DroneState,
  params: DroneParams,
  env: EnvironmentOutput,
  bias: SensorBiasState,
  config: ImuSensorConfig,
  rng: DeterministicRNG,
  simTime: number,
): SensorOutput<ImuMeasurement> {
  if (!config.enabled) {
    return { timestamp: simTime, valid: false, measurement: { gyro: _gyroOut, accel: _accelOut } };
  }

  const sqrtRate = Math.sqrt(config.rate);

  // ── Gyroscope ──
  for (let i = 0; i < 3; i++) {
    let val = state.angularVelocity[i] + bias.gyroBias[i]
      + config.gyroNoiseDensity * sqrtRate * rng.gaussian();
    val = clamp(val, -config.gyroSaturation, config.gyroSaturation);
    _gyroOut[i] = val;
  }

  // ── Accelerometer (specific force in body frame) ──
  // Recompute thrust + drag from current state to get specific force
  // Specific force = (F_thrust + F_drag) / m  (excludes gravity)

  // Total thrust in body frame
  let totalThrust = 0;
  for (let i = 0; i < params.numRotors; i++) {
    totalThrust += params.kT * state.motorSpeeds[i] * state.motorSpeeds[i];
  }
  totalThrust *= env.groundEffectMultiplier;
  v3Zero(_thrustBody);
  _thrustBody[2] = -totalThrust; // -z in FRD = upward

  // Thrust in world frame
  qRotateVector(_thrustWorld, state.quaternion, _thrustBody);

  // Body drag: need air-relative velocity in body frame
  _vRelWorld[0] = state.velocity[0] - env.wind[0];
  _vRelWorld[1] = state.velocity[1] - env.wind[1];
  _vRelWorld[2] = state.velocity[2] - env.wind[2];
  qRotateVectorInverse(_vRelBody, state.quaternion, _vRelWorld);
  computeBodyDrag(_dragBody, _vRelBody, params);

  // Drag in world frame (uses module-level scratch)
  qRotateVector(_dragWorld, state.quaternion, _dragBody);

  // Specific force in world frame = (thrust + drag) / mass
  _specificForceWorld[0] = (_thrustWorld[0] + _dragWorld[0]) / params.mass;
  _specificForceWorld[1] = (_thrustWorld[1] + _dragWorld[1]) / params.mass;
  _specificForceWorld[2] = (_thrustWorld[2] + _dragWorld[2]) / params.mass;

  // Rotate to body frame
  qRotateVectorInverse(_specificForceBody, state.quaternion, _specificForceWorld);

  // Add bias + noise + saturation
  for (let i = 0; i < 3; i++) {
    let val = _specificForceBody[i] + bias.accelBias[i]
      + config.accelNoiseDensity * sqrtRate * rng.gaussian();
    val = clamp(val, -config.accelSaturation, config.accelSaturation);
    _accelOut[i] = val;
  }

  return {
    timestamp: simTime,
    valid: true,
    measurement: { gyro: _gyroOut, accel: _accelOut },
  };
}
