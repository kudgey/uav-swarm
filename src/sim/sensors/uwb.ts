/**
 * UWB ranging sensor: infrastructure-based anchor ranging.
 * STATUS: simplified — fixed anchors, truth NLOS flag, no MAC contention.
 *
 * LOS/NLOS determined by WorldGeometry.hasLineOfSight().
 * NLOS produces asymmetric positive bias via folded normal |N(mean, std²)|.
 */

import type { DroneState, UWBAnchorDef, UWBSensorConfig } from '@sim/core/types';
import type { DeterministicRNG } from '@sim/core/rng';
import type { WorldGeometry } from '@sim/environment/world-geometry';
import type { SensorOutput, UWBMeasurement } from './sensor-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('sensor-uwb', 'simplified',
  'Fixed anchor UWB, truth NLOS flag, no MAC contention', {
    simplifications: ['Infrastructure anchors only (no inter-drone)',
      'NLOS detected from truth geometry, not estimated',
      'No MAC/channel congestion model'],
  });

/**
 * Read UWB ranges to all anchors. Returns one SensorOutput per anchor.
 */
export function readUWB(
  state: DroneState,
  anchors: UWBAnchorDef[],
  config: UWBSensorConfig,
  worldGeo: WorldGeometry,
  rng: DeterministicRNG,
  simTime: number,
): SensorOutput<UWBMeasurement>[] {
  const results: SensorOutput<UWBMeasurement>[] = [];

  for (const anchor of anchors) {
    // True range
    const dx = state.position[0] - anchor.position[0];
    const dy = state.position[1] - anchor.position[1];
    const dz = state.position[2] - anchor.position[2];
    const trueRange = Math.sqrt(dx * dx + dy * dy + dz * dz);

    // Range limits
    if (trueRange < config.minRange || trueRange > config.maxRange) {
      results.push({
        timestamp: simTime, valid: false,
        measurement: { anchorId: anchor.id, range: 0, isNLOS: false },
      });
      continue;
    }

    // Packet loss
    if (rng.next() < config.packetLossProbability) {
      results.push({
        timestamp: simTime, valid: false,
        measurement: { anchorId: anchor.id, range: 0, isNLOS: false },
      });
      continue;
    }

    // LOS/NLOS check
    const hasLOS = worldGeo.hasLineOfSight(state.position, anchor.position);
    let bias: number;
    let noise: number;

    if (hasLOS) {
      bias = config.losBias;
      noise = config.losNoiseStdDev * rng.gaussian();
    } else {
      // Folded normal: |N(mean, std²)| → always positive bias
      const nlosBiasExtra = Math.abs(rng.gaussian(config.nlosBiasBase, config.nlosBiasStdDev));
      bias = config.losBias + nlosBiasExtra;
      noise = config.nlosNoiseStdDev * rng.gaussian();
    }

    const rangeMeas = Math.max(0, trueRange + bias + noise);

    results.push({
      timestamp: simTime,
      valid: true,
      measurement: { anchorId: anchor.id, range: rangeMeas, isNLOS: !hasLOS },
    });
  }

  return results;
}
