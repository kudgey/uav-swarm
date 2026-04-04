/**
 * Integration test: Monte Carlo batch run.
 */
import { describe, it, expect } from 'vitest';
import { ScenarioRunner } from '@sim/scenarios/scenario-runner';
import { mcHover } from '@sim/scenarios/scenario-presets';

describe('Batch Monte Carlo', () => {
  it('runs 3 seeds and produces finite results', () => {
    const runner = new ScenarioRunner();
    const scenario = { ...mcHover, duration: 1, monteCarloRanges: { ...mcHover.monteCarloRanges!, seeds: [100, 101, 102] } };
    const results = runner.runBatch(scenario);
    expect(results.length).toBe(3);
    for (const r of results) {
      expect(isFinite(r.metrics.realTimeFactor)).toBe(true);
      expect(r.metrics.collisionCount).toBeDefined();
    }
    // Different seeds should produce different RNG states
    // (metrics may still be similar for short runs)
  }, 60000);
});
