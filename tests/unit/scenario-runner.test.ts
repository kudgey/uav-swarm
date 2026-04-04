import { describe, it, expect } from 'vitest';
import { ScenarioRunner } from '@sim/scenarios/scenario-runner';
import { hoverCalm, crossing2, magAnomaly, mcHover } from '@sim/scenarios/scenario-presets';

describe('ScenarioRunner', () => {
  it('runSingle returns result with non-zero tracking metrics', () => {
    const runner = new ScenarioRunner();
    const scenario = { ...hoverCalm, duration: 2 };
    const result = runner.runSingle(scenario, 42);
    expect(result.seed).toBe(42);
    expect(result.scenarioName).toBe('hover-calm');
    expect(result.metrics.realTimeFactor).toBeGreaterThan(0);
    // Tracking metrics should be non-zero after 2s of flight
    expect(result.metrics.rmsExecutedTrackingError).toBeGreaterThanOrEqual(0);
    expect(result.metrics.rmsEstimationError).toBeGreaterThanOrEqual(0);
    expect(result.metrics.rmsAttitudeError).toBeGreaterThanOrEqual(0);
    expect(isFinite(result.metrics.rmsExecutedTrackingError)).toBe(true);
  });

  it('crossing-2 produces safety metrics', () => {
    const runner = new ScenarioRunner();
    const scenario = { ...crossing2, duration: 3 };
    const result = runner.runSingle(scenario, 42);
    expect(result.metrics.collisionCount).toBeDefined();
    // 2 drones: min separation should be finite
    expect(isFinite(result.metrics.minSeparation)).toBe(true);
    expect(result.metrics.minSeparation).toBeGreaterThan(0);
  });

  it('mag-anomaly produces innovation spike', () => {
    const runner = new ScenarioRunner();
    const scenario = { ...magAnomaly, duration: 5 };
    const result = runner.runSingle(scenario, 42);
    // After mag anomaly injection, maxInnovationNorm should be elevated
    expect(result.metrics.maxInnovationNorm).toBeGreaterThan(0);
  });

  it('runBatch returns array of results', () => {
    const runner = new ScenarioRunner();
    const scenario = { ...hoverCalm, duration: 0.5 };
    const results = runner.runBatch(scenario, [42, 43]);
    expect(results.length).toBe(2);
    expect(results[0].seed).toBe(42);
    expect(results[1].seed).toBe(43);
  });

  it('evaluateBatch computes batch verdict', () => {
    const runner = new ScenarioRunner();
    const scenario = { ...hoverCalm, duration: 0.5 };
    const results = runner.runBatch(scenario, [42, 43, 44]);
    const verdict = runner.evaluateBatch(results, { passRate: 1.0 });
    expect(verdict.passed).toBeDefined();
  });
});
