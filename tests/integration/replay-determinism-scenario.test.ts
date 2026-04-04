/**
 * Integration test: same scenario + same seed → identical metrics.
 */
import { describe, it, expect } from 'vitest';
import { ScenarioRunner } from '@sim/scenarios/scenario-runner';
import { hoverCalm } from '@sim/scenarios/scenario-presets';

describe('Replay determinism', () => {
  it('same seed produces identical metrics', () => {
    const runner = new ScenarioRunner();
    const scenario = { ...hoverCalm, duration: 2 };
    const r1 = runner.runSingle(scenario, 42);
    const r2 = runner.runSingle(scenario, 42);
    expect(r1.metrics.collisionCount).toBe(r2.metrics.collisionCount);
    expect(r1.metrics.minSeparation).toBe(r2.metrics.minSeparation);
  }, 30000);
});
