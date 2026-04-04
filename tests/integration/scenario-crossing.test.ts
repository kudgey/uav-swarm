/**
 * Integration test: crossing-2 scenario via ScenarioRunner.
 */
import { describe, it, expect } from 'vitest';
import { ScenarioRunner } from '@sim/scenarios/scenario-runner';
import { crossing2 } from '@sim/scenarios/scenario-presets';

describe('Scenario: crossing-2', () => {
  it('completes with safety metrics', () => {
    const runner = new ScenarioRunner();
    const scenario = { ...crossing2, duration: 3 };
    const result = runner.runSingle(scenario, 42);
    expect(result.metrics.collisionCount).toBeDefined();
    expect(isFinite(result.metrics.minSeparation)).toBe(true);
    expect(result.scenarioName).toBe('crossing-2');
  }, 30000);
});
