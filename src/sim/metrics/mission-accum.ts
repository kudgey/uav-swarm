/**
 * Shared mission metrics accumulator.
 * Used by both ScenarioRunner (batch) and visual mission worker (live).
 */

import { v3Create, v3Sub, v3Len } from '@lib/math';
import type { Vec3 } from '@sim/core/types';

export interface DroneAccum {
  missionTrackingSqSum: number; executedTrackingSqSum: number;
  estimationSqSum: number; altitudeSqSum: number; maxAltErr: number;
  attitudeSqSum: number; velocitySqSum: number;
  maxHorizDrift: number; saturatedTicks: number;
  flowValidCount: number; vioValidCount: number;
  innovGatedCount: number; innovTotalCount: number;
  maxInnovNorm: number; sampleCount: number;
  uwbRangeSqSum: number; uwbRangeCount: number;
  biasSqSum: number;
  driftSamples: { t: number; d: number }[];
}

export function newAccum(): DroneAccum {
  return {
    missionTrackingSqSum: 0, executedTrackingSqSum: 0, estimationSqSum: 0,
    altitudeSqSum: 0, maxAltErr: 0, attitudeSqSum: 0, velocitySqSum: 0,
    maxHorizDrift: 0, saturatedTicks: 0, flowValidCount: 0, vioValidCount: 0,
    innovGatedCount: 0, innovTotalCount: 0, maxInnovNorm: 0, sampleCount: 0,
    uwbRangeSqSum: 0, uwbRangeCount: 0, biasSqSum: 0, driftSamples: [],
  };
}

const _diff = v3Create();

/**
 * Sample metrics for one drone into its accumulator.
 * Call at controller rate (~250Hz or every Nth physics tick).
 */
export function sampleDroneMetrics(
  a: DroneAccum,
  truthPos: Vec3,
  estPos: Vec3,
  estVel: Vec3,
  truthVel: Vec3,
  truthQuat: Float64Array,
  estQuat: Float64Array,
  guidancePos: Vec3,
  safeGuidancePos: Vec3 | undefined,
  motorSpeeds: Float64Array,
  motorOmegaMax: number,
  numRotors: number,
  simTime: number,
  readings: {
    flowValid: boolean;
    vioValid: boolean;
    innovGated: boolean;
    innovNorm: number;
    hasInnov: boolean;
  },
): void {
  // Tracking errors
  v3Sub(_diff, guidancePos, truthPos);
  a.missionTrackingSqSum += v3Len(_diff) ** 2;
  const execRef = safeGuidancePos ?? guidancePos;
  v3Sub(_diff, execRef, truthPos);
  a.executedTrackingSqSum += v3Len(_diff) ** 2;

  // Estimation error
  v3Sub(_diff, estPos, truthPos);
  a.estimationSqSum += v3Len(_diff) ** 2;

  // Horizontal drift
  const horizDrift = Math.sqrt(_diff[0] ** 2 + _diff[1] ** 2);
  if (horizDrift > a.maxHorizDrift) a.maxHorizDrift = horizDrift;

  // Altitude error
  const altErr = Math.abs(-truthPos[2] - (-execRef[2]));
  a.altitudeSqSum += altErr ** 2;
  if (altErr > a.maxAltErr) a.maxAltErr = altErr;

  // Attitude error
  const dotQ = Math.abs(
    estQuat[0] * truthQuat[0] + estQuat[1] * truthQuat[1] +
    estQuat[2] * truthQuat[2] + estQuat[3] * truthQuat[3],
  );
  a.attitudeSqSum += (2 * Math.acos(Math.min(1, dotQ))) ** 2;

  // Velocity error
  v3Sub(_diff, estVel, truthVel);
  a.velocitySqSum += v3Len(_diff) ** 2;

  // Saturation
  for (let m = 0; m < numRotors; m++) {
    if (motorSpeeds[m] >= motorOmegaMax * 0.99) { a.saturatedTicks++; break; }
  }

  // Sensor validity
  if (readings.flowValid) a.flowValidCount++;
  if (readings.vioValid) a.vioValidCount++;
  if (readings.hasInnov) {
    a.innovTotalCount++;
    if (readings.innovGated) a.innovGatedCount++;
    if (readings.innovNorm > a.maxInnovNorm) a.maxInnovNorm = readings.innovNorm;
  }

  // Drift rate samples
  if (a.sampleCount % 10 === 0) {
    a.driftSamples.push({ t: simTime, d: horizDrift });
  }

  a.sampleCount++;
}

export function rms(sqSum: number, count: number): number {
  return count > 0 ? Math.sqrt(sqSum / count) : 0;
}

export function meanOf(vals: number[]): number {
  return vals.length > 0 ? vals.reduce((s, v) => s + v, 0) / vals.length : 0;
}

export function maxOf(vals: number[]): number {
  return vals.length > 0 ? Math.max(...vals) : 0;
}
