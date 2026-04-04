import { describe, it, expect } from 'vitest';
import { ALL_PRESETS } from '@sim/scenarios/scenario-presets';

describe('Scenario presets', () => {
  it('all presets have valid structure', () => {
    expect(ALL_PRESETS.length).toBeGreaterThanOrEqual(20);
    for (const p of ALL_PRESETS) {
      expect(p.name).toBeTruthy();
      expect(p.duration).toBeGreaterThan(0);
      expect(p.mission.steps.length).toBeGreaterThan(0);
      expect(p.config).toBeTruthy();
      expect(p.config.seed).toBeDefined();
    }
  });

  it('each preset has unique name', () => {
    const names = ALL_PRESETS.map(p => p.name);
    expect(new Set(names).size).toBe(names.length);
  });

  it('MC presets have seeds and batch criteria', () => {
    const mc = ALL_PRESETS.filter(p => p.monteCarloRanges);
    expect(mc.length).toBeGreaterThanOrEqual(2);
    for (const p of mc) {
      expect(p.monteCarloRanges!.seeds.length).toBeGreaterThan(0);
      expect(p.batchAcceptanceCriteria).toBeDefined();
    }
  });
});
