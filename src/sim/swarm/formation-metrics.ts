/**
 * Formation truth-based validation metrics.
 * This file IS allowed to import DroneState — it computes acceptance metrics.
 * Formation control logic in formation.ts does NOT import DroneState.
 */

import type { Vec3 } from '@sim/core/types';
import type { DroneInstance } from './drone-instance';
import type { FormationTopology } from './formation-types';

/**
 * Compute truth-based formation error for each drone.
 * Returns per-drone error vectors (truth position - desired formation position).
 */
export function computeTruthFormationErrors(
  drones: DroneInstance[],
  topology: FormationTopology,
  leaderId: number,
): { droneId: number; error: Vec3 }[] {
  const errors: { droneId: number; error: Vec3 }[] = [];
  const leader = drones.find(d => d.id === leaderId);
  if (!leader) return errors;

  const leaderOffset = topology.offsets[leaderId] ?? new Float64Array(3);

  for (const d of drones) {
    const myOffset = topology.offsets[d.id] ?? new Float64Array(3);
    // Desired truth position = leader truth pos + (myOffset - leaderOffset)
    const desX = leader.state.position[0] + myOffset[0] - leaderOffset[0];
    const desY = leader.state.position[1] + myOffset[1] - leaderOffset[1];
    const desZ = leader.state.position[2] + myOffset[2] - leaderOffset[2];

    errors.push({
      droneId: d.id,
      error: new Float64Array([
        d.state.position[0] - desX,
        d.state.position[1] - desY,
        d.state.position[2] - desZ,
      ]),
    });
  }

  return errors;
}

/** Compute RMS formation error from truth. */
export function computeTruthFormationRMS(
  drones: DroneInstance[],
  topology: FormationTopology,
  leaderId: number,
): number {
  const errors = computeTruthFormationErrors(drones, topology, leaderId);
  if (errors.length === 0) return 0;
  let sumSq = 0;
  for (const e of errors) {
    sumSq += e.error[0] ** 2 + e.error[1] ** 2 + e.error[2] ** 2;
  }
  return Math.sqrt(sumSq / errors.length);
}
