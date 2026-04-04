/**
 * Swarm-level truth-based safety metrics and event logging.
 */

import { v3Len, v3Sub, v3Create } from '@lib/math';
import type { DroneInstance } from '@sim/swarm/drone-instance';
import type { SafetyConfig, SafetyEvent, SwarmSafetyMetrics } from './safety-types';

const _diff = v3Create();

/** Compute truth-based swarm safety metrics (pairwise). */
export function computeSwarmSafetyMetrics(
  drones: DroneInstance[],
  config: SafetyConfig,
): SwarmSafetyMetrics {
  let minSep = Infinity;
  let totalSep = 0;
  let pairCount = 0;
  let collisionCount = 0;
  let overrideCount = 0;
  let emergencyCount = 0;

  for (let i = 0; i < drones.length; i++) {
    for (let j = i + 1; j < drones.length; j++) {
      v3Sub(_diff, drones[i].state.position, drones[j].state.position);
      const sep = v3Len(_diff);
      if (sep < minSep) minSep = sep;
      totalSep += sep;
      pairCount++;
      if (sep < config.minSeparation) collisionCount++;
    }
    if (drones[i].lastSafetyState?.active) {
      overrideCount++;
      if (drones[i].lastSafetyState?.reason === 'emergency') emergencyCount++;
    }
  }

  return {
    minSeparation: pairCount > 0 ? minSep : Infinity,
    collisionCount,
    avgSeparation: pairCount > 0 ? totalSep / pairCount : Infinity,
    safetyOverrideCount: overrideCount,
    emergencyStopCount: emergencyCount,
  };
}

/** Ring buffer of safety events. */
export class SafetyEventLog {
  private events: SafetyEvent[];
  private writeIdx = 0;
  private count = 0;
  private capacity: number;

  constructor(capacity = 500) {
    this.capacity = capacity;
    this.events = new Array(capacity);
    for (let i = 0; i < capacity; i++) {
      this.events[i] = { timestamp: 0, droneId: 0, type: 'orca-override', detail: '' };
    }
  }

  record(event: SafetyEvent): void {
    const e = this.events[this.writeIdx];
    e.timestamp = event.timestamp; e.droneId = event.droneId;
    e.type = event.type; e.detail = event.detail;
    this.writeIdx = (this.writeIdx + 1) % this.capacity;
    if (this.count < this.capacity) this.count++;
  }

  getRecent(n: number): SafetyEvent[] {
    const actual = Math.min(n, this.count);
    const result: SafetyEvent[] = [];
    for (let i = 0; i < actual; i++) {
      const idx = (this.writeIdx - actual + i + this.capacity) % this.capacity;
      result.push({ ...this.events[idx] });
    }
    return result;
  }

  reset(): void { this.writeIdx = 0; this.count = 0; }
}
