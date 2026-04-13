/**
 * Large swarm performance probe. Runs 5/10/15 drones for 5s sim and measures
 * wall-clock time. Reports per-drone cost. Not a hard pass/fail beyond
 * "does not crash and completes in reasonable time".
 */
import { describe, it, expect } from 'vitest';
import { SimContext } from '@sim/core/sim-context';
import { defaultSimConfig } from '@sim/core/config-defaults';
import { v3Create } from '@lib/math';

function makeCfg(n: number) {
  const cfg = defaultSimConfig();
  cfg.swarm.droneCount = n;
  cfg.swarm.initialPattern = 'circle';
  cfg.swarm.patternSpacing = 3;
  cfg.swarm.patternCenter = new Float64Array([5, 5, -2]);
  cfg.swarm.initialPositions = undefined;
  return cfg;
}

function armAll(ctx: SimContext) {
  for (const d of ctx.swarmManager.drones) {
    if (!d.ekf.isAligned()) d.ekf.bootstrapAligned();
    d.controller.forceArm();
    d.controller.setHover(v3Create(d.state.position[0], d.state.position[1], d.state.position[2]), 0);
  }
}

describe('Large swarm perf', () => {
  it('scales across 5/10/15 drones — reports timing', () => {
    const results: { n: number; ms: number; perDrone: number; rtf: number }[] = [];

    for (const n of [5, 10, 15]) {
      const cfg = makeCfg(n);
      const ctx = new SimContext(cfg);
      armAll(ctx);
      const simDurationSec = 5;
      const totalSteps = Math.round(simDurationSec / cfg.physicsDt);

      const startWall = performance.now();
      for (let i = 0; i < totalSteps; i++) ctx.stepOnce();
      const elapsed = performance.now() - startWall;
      const rtf = (simDurationSec * 1000) / elapsed;
      results.push({ n, ms: elapsed, perDrone: elapsed / n, rtf });

      // Assertions: no crashes, altitude bounded
      for (const d of ctx.swarmManager.drones) {
        expect(Number.isFinite(d.state.position[0])).toBe(true);
        expect(d.state.position[2]).toBeGreaterThan(-10);
        expect(d.state.position[2]).toBeLessThan(2);
      }
    }

    console.log('\n=== Swarm perf (5s sim each) ===');
    console.log('n_drones, wall_ms, ms_per_drone, rtf');
    for (const r of results) {
      console.log(`${r.n},${r.ms.toFixed(0)},${r.perDrone.toFixed(0)},${r.rtf.toFixed(2)}x`);
    }
  }, 180000);
});
