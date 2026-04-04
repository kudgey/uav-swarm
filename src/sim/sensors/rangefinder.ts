/**
 * Downward rangefinder sensor.
 * STATUS: experimental
 *
 * Beam along body z-axis. Range via WorldGeometry raycast (floor + obstacles).
 * Invalid on: no hit, out of range, steep incidence, dropout, beam pointing up.
 */

import { v3Create } from '@lib/math';
import { qRotateVector } from '@sim/physics/quaternion';
import type { DroneState, RangeSensorConfig } from '@sim/core/types';
import type { DeterministicRNG } from '@sim/core/rng';
import type { WorldGeometry } from '@sim/environment/world-geometry';
import type { SensorOutput, RangeMeasurement } from './sensor-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('sensor-range', 'experimental', 'Downward rangefinder with incidence-angle limits');

// Beam direction in body frame: [0, 0, 1] (FRD: z = down)
const _beamBody = v3Create(0, 0, 1);
const _beamWorld = v3Create();

export function readRangefinder(
  state: DroneState,
  config: RangeSensorConfig,
  rng: DeterministicRNG,
  worldGeo: WorldGeometry,
  simTime: number,
): SensorOutput<RangeMeasurement> {
  if (!config.enabled) {
    return { timestamp: simTime, valid: false, measurement: { range: 0 } };
  }

  // Dropout check
  if (config.dropoutProbability > 0 && rng.next() < config.dropoutProbability) {
    return { timestamp: simTime, valid: false, measurement: { range: 0 } };
  }

  // Beam direction in world frame
  qRotateVector(_beamWorld, state.quaternion, _beamBody);

  // Beam must point at least partially downward (positive z in NED)
  if (_beamWorld[2] <= 0) {
    return { timestamp: simTime, valid: false, measurement: { range: 0 } };
  }

  // Raycast from drone position along beam direction
  const result = worldGeo.raycast(state.position, _beamWorld);
  if (!result.hit) {
    return { timestamp: simTime, valid: false, measurement: { range: 0 } };
  }

  const rangeTrue = result.distance;

  // Range limits
  if (rangeTrue < config.minRange || rangeTrue > config.maxRange) {
    return { timestamp: simTime, valid: false, measurement: { range: 0 } };
  }

  // Incidence angle: angle between beam direction and the hit surface normal.
  // The raycast returns the outward surface normal (e.g. [0,0,-1] for floor in NED).
  // Incidence angle = angle between beam and the inward normal (toward surface).
  // cos(incidence) = dot(beam, -normal) = -(beam·normal)
  const normal = result.normal;
  const dotBN = _beamWorld[0] * normal[0] + _beamWorld[1] * normal[1] + _beamWorld[2] * normal[2];
  const cosIncidence = -dotBN; // beam points toward surface, normal points away
  if (cosIncidence <= 0) {
    // Beam is parallel or pointing away from surface — shouldn't happen after raycast hit, but guard
    return { timestamp: simTime, valid: false, measurement: { range: 0 } };
  }
  const incidenceAngle = Math.acos(Math.min(1, cosIncidence));
  if (incidenceAngle > config.maxIncidenceAngle) {
    return { timestamp: simTime, valid: false, measurement: { range: 0 } };
  }

  // Noisy measurement
  const rangeMeas = rangeTrue + config.noise * rng.gaussian();

  return {
    timestamp: simTime,
    valid: true,
    measurement: { range: Math.max(0, rangeMeas) },
  };
}
