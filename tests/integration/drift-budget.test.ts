/**
 * Drift budget experiment: measure return drift vs patrol distance.
 *
 * Runs GPS-denied patrol (VIO drift, no anchors) at multiple distances
 * with multiple seeds, prints aggregate stats. This is a research-grade
 * measurement, not a pass/fail test — assertions are loose sanity bounds.
 *
 * Usage: `npx vitest run tests/integration/drift-budget.test.ts`
 * Output is printed to console for analysis.
 */

import { describe, it, expect } from 'vitest';
import { SimContext } from '@sim/core/sim-context';
import { buildPatrolConfig, buildPatrolMissionSteps } from '@sim/missions/mission-builder';
import type { MissionStep } from '@sim/scenarios/scenario-types';

interface DriftRun {
  distance: number;
  seed: number;
  returnDrift: number;      // distance from launch centroid at end
  maxDrift: number;         // max horizontal drift during flight
  rmsEstError: number;      // mean RMS estimation error across drones
  completed: boolean;       // did leader finish all waypoints
}

function applyStep(ctx: SimContext, step: MissionStep): void {
  const drones = step.droneId === 'all'
    ? ctx.swarmManager.drones
    : [ctx.swarmManager.getDrone(step.droneId as number)].filter(Boolean);
  for (const d of drones) {
    if (!d) continue;
    switch (step.action) {
      case 'arm':
        if (!d.ekf.isAligned()) d.ekf.bootstrapAligned();
        d.controller.forceArm();
        break;
      case 'hover': {
        const p = step.params?.position as number[];
        if (p) d.controller.setHover(new Float64Array(p), (step.params?.yaw as number) ?? 0);
        break;
      }
      case 'set-mission-plan': {
        const wps = step.params?.waypoints as { position: [number, number, number]; yaw: number; speed: number }[];
        if (wps) d.controller.setMissionPlan(wps);
        break;
      }
      case 'formation-enable':
        ctx.swarmManager.updateFormationConfig({ ...ctx.config.swarm.formation, enabled: true });
        return;
    }
  }
}

function runPatrol(distance: number, seed: number): DriftRun {
  const droneCount = 4;
  const speed = 2.0; // faster for test time
  const center: [number, number, number] = [5, 5, -2];

  const cfg = buildPatrolConfig({ droneCount, pattern: 'line', spacing: 3, distance });
  cfg.seed = seed;

  const waypointCount = Math.max(3, Math.ceil(distance / 20));
  const steps = buildPatrolMissionSteps({
    droneCount, distance, waypointCount, speed,
    returnToBase: true, center, spacing: 3, pattern: 'line',
  });

  const ctx = new SimContext(cfg);
  // Capture launch centroid (truth)
  let launchX = 0, launchY = 0, launchZ = 0;
  for (const d of ctx.swarmManager.drones) {
    launchX += d.state.position[0]; launchY += d.state.position[1]; launchZ += d.state.position[2];
  }
  launchX /= droneCount; launchY /= droneCount; launchZ /= droneCount;

  const duration = (2 * distance) / speed + 10; // out + back + settle
  const totalSteps = Math.round(duration / cfg.physicsDt);

  let stepIdx = 0;
  let maxDrift = 0;
  let sampleCount = 0;
  let estErrSqSum = 0;

  for (let i = 0; i < totalSteps; i++) {
    while (stepIdx < steps.length && steps[stepIdx].time <= ctx.simTime) {
      applyStep(ctx, steps[stepIdx]);
      stepIdx++;
    }
    ctx.stepOnce();
    // Sample drift/estError once per second
    if (i % Math.round(1 / cfg.physicsDt) === 0) {
      for (const d of ctx.swarmManager.drones) {
        if (d.destroyed) continue;
        const est = d.ekf.getEstimate();
        const dx = est.position[0] - d.state.position[0];
        const dy = est.position[1] - d.state.position[1];
        const dz = est.position[2] - d.state.position[2];
        const horiz = Math.sqrt(dx * dx + dy * dy);
        if (horiz > maxDrift) maxDrift = horiz;
        estErrSqSum += dx * dx + dy * dy + dz * dz;
        sampleCount++;
      }
    }
  }

  // End centroid
  let endX = 0, endY = 0, endZ = 0;
  for (const d of ctx.swarmManager.drones) {
    endX += d.state.position[0]; endY += d.state.position[1]; endZ += d.state.position[2];
  }
  endX /= droneCount; endY /= droneCount; endZ /= droneCount;
  const returnDrift = Math.sqrt((endX - launchX) ** 2 + (endY - launchY) ** 2 + (endZ - launchZ) ** 2);

  const leader = ctx.swarmManager.drones[0];
  const completed = leader.controller.getMissionProgress().done;
  const rmsEstError = sampleCount > 0 ? Math.sqrt(estErrSqSum / sampleCount) : 0;

  return { distance, seed, returnDrift, maxDrift, rmsEstError, completed };
}

describe('Drift budget experiment', () => {
  it('measures return drift vs distance (GPS-denied patrol)', () => {
    const distances = [20, 50, 100];
    const seeds = [42, 43, 44];
    const runs: DriftRun[] = [];

    for (const d of distances) {
      for (const s of seeds) {
        runs.push(runPatrol(d, s));
      }
    }

    // Print table
    console.log('\n=== Drift Budget Experiment ===');
    console.log('distance,seed,returnDrift,maxDrift,rmsEstError,completed');
    for (const r of runs) {
      console.log(`${r.distance},${r.seed},${r.returnDrift.toFixed(2)},${r.maxDrift.toFixed(2)},${r.rmsEstError.toFixed(3)},${r.completed}`);
    }

    // Aggregate stats per distance
    console.log('\n=== Summary ===');
    console.log('distance,mean_return_drift,mean_max_drift,mean_est_err,completion_rate');
    for (const d of distances) {
      const subset = runs.filter(r => r.distance === d);
      const meanReturn = subset.reduce((s, r) => s + r.returnDrift, 0) / subset.length;
      const meanMaxDrift = subset.reduce((s, r) => s + r.maxDrift, 0) / subset.length;
      const meanEstErr = subset.reduce((s, r) => s + r.rmsEstError, 0) / subset.length;
      const completionRate = subset.filter(r => r.completed).length / subset.length;
      console.log(`${d},${meanReturn.toFixed(2)},${meanMaxDrift.toFixed(2)},${meanEstErr.toFixed(3)},${completionRate.toFixed(2)}`);
    }

    // Sanity assertions
    for (const r of runs) {
      expect(Number.isFinite(r.returnDrift)).toBe(true);
      expect(r.returnDrift).toBeLessThan(r.distance * 0.5); // drift < 50% of distance
    }

    // Drift should generally grow with distance (loose check — might not be monotonic per seed)
    const meanAt20 = runs.filter(r => r.distance === 20).reduce((s, r) => s + r.returnDrift, 0) / seeds.length;
    const meanAt100 = runs.filter(r => r.distance === 100).reduce((s, r) => s + r.returnDrift, 0) / seeds.length;
    expect(meanAt100).toBeGreaterThan(meanAt20 * 0.5); // 100m drift at least half of 20m (very loose)
  }, 120000); // 2 min timeout for full sweep
});
