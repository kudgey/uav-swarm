/**
 * Magnetometer sensor model.
 * STATUS: experimental
 *
 * m_meas = M_soft * (R_bw^T * B_earth_w + B_motor_b) + b_hard + noise
 *
 * Motor EMI computed here (body frame): B_motor_b = [0, 0, emiCoeff * sum(omega_i^2)]
 * Environment provides earthMagneticField in world frame.
 */

import { v3Create, m3MulV3, m3Create } from '@lib/math';
import { qRotateVectorInverse } from '@sim/physics/quaternion';
import type { Vec3, DroneState, DroneParams, EnvironmentOutput, MagSensorConfig } from '@sim/core/types';
import type { DeterministicRNG } from '@sim/core/rng';
import type { SensorOutput, MagMeasurement } from './sensor-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('sensor-mag', 'experimental', 'Magnetometer with hard/soft iron, motor EMI');

// Pre-allocated scratch
const _earthBody = v3Create();
const _preSoftIron = v3Create();
const _measurement = v3Create();

export function readMagnetometer(
  state: DroneState,
  params: DroneParams,
  env: EnvironmentOutput,
  config: MagSensorConfig,
  rng: DeterministicRNG,
  simTime: number,
): SensorOutput<MagMeasurement> {
  if (!config.enabled) {
    return { timestamp: simTime, valid: false, measurement: { field: _measurement } };
  }

  // Dropout check
  if (config.dropoutProbability > 0 && rng.next() < config.dropoutProbability) {
    return { timestamp: simTime, valid: false, measurement: { field: _measurement } };
  }

  const sqrtRate = Math.sqrt(config.rate);

  // Rotate earth field from world to body frame
  qRotateVectorInverse(_earthBody, state.quaternion, env.earthMagneticField);

  // Motor EMI in body frame: along body z-axis
  let motorOmegaSqSum = 0;
  for (let i = 0; i < params.numRotors; i++) {
    motorOmegaSqSum += state.motorSpeeds[i] * state.motorSpeeds[i];
  }
  const emiZ = config.emiCoefficient * motorOmegaSqSum;

  // Pre soft-iron: earth_body + motor_emi_body
  _preSoftIron[0] = _earthBody[0];
  _preSoftIron[1] = _earthBody[1];
  _preSoftIron[2] = _earthBody[2] + emiZ;

  // Apply soft-iron distortion: M_soft * pre_soft_iron
  m3MulV3(_measurement, config.softIron, _preSoftIron);

  // Add hard-iron bias + noise
  for (let i = 0; i < 3; i++) {
    _measurement[i] += config.hardIron[i]
      + config.noiseDensity * sqrtRate * rng.gaussian();
  }

  return {
    timestamp: simTime,
    valid: true,
    measurement: { field: _measurement },
  };
}
