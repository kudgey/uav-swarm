/**
 * Barometric altitude sensor.
 * STATUS: experimental
 *
 * p_meas = p_isa(h) + bias_baro + noise
 * altitude_meas = isaPressureToAltitude(p_meas)
 */

import { isaPressure, isaPressureToAltitude } from '@sim/environment/atmosphere';
import type { DroneState, EnvironmentOutput, BaroSensorConfig } from '@sim/core/types';
import type { DeterministicRNG } from '@sim/core/rng';
import type { SensorBiasState } from './sensor-bias-state';
import type { SensorOutput, BaroMeasurement } from './sensor-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('sensor-baro', 'experimental', 'Barometric altimeter with bias drift');

export function readBarometer(
  state: DroneState,
  _env: EnvironmentOutput,
  bias: SensorBiasState,
  config: BaroSensorConfig,
  rng: DeterministicRNG,
  simTime: number,
): SensorOutput<BaroMeasurement> {
  if (!config.enabled) {
    return { timestamp: simTime, valid: false, measurement: { pressure: 0, altitude: 0 } };
  }

  const sqrtRate = Math.sqrt(config.rate);

  // True altitude = -position_z (NED convention, floor at z=0)
  const hTrue = Math.max(0, -state.position[2]);
  const pTrue = isaPressure(hTrue);

  // Measured pressure with bias + noise
  const pMeas = pTrue + bias.baroBias + config.noiseDensity * sqrtRate * rng.gaussian();

  // Derive altitude from measured pressure
  const altMeas = isaPressureToAltitude(pMeas);

  return {
    timestamp: simTime,
    valid: true,
    measurement: { pressure: pMeas, altitude: altMeas },
  };
}
