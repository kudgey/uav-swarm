/**
 * Integration test: hover-calm scenario via ScenarioRunner.
 */
import { describe, it, expect } from 'vitest';
import { ScenarioRunner } from '@sim/scenarios/scenario-runner';
import { hoverCalm } from '@sim/scenarios/scenario-presets';

describe('Scenario: hover-calm', () => {
  it('completes with finite metrics', () => {
    const runner = new ScenarioRunner();
    const scenario = { ...hoverCalm, duration: 3 }; // shorter for test
    const result = runner.runSingle(scenario, 42);
    // Single drone: minSeparation = Infinity (no pairs)
    expect(result.metrics.minSeparation).toBe(Infinity);
    expect(result.metrics.realTimeFactor).toBeGreaterThan(0);
    expect(result.duration).toBe(3);
  }, 30000);
});
