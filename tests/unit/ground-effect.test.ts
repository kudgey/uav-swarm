import { describe, it, expect } from 'vitest';
import { computeGroundEffectMultiplier } from '@sim/environment/ground-effect';

describe('Ground Effect (Cheeseman-Bennett)', () => {
  const Rprop = 0.12; // typical prop radius

  it('at altitude h >> R_prop: multiplier is exactly 1.0', () => {
    expect(computeGroundEffectMultiplier(10, Rprop)).toBe(1.0);
    expect(computeGroundEffectMultiplier(4 * Rprop + 0.01, Rprop)).toBe(1.0);
  });

  it('monotonicity: closer to ground means larger multiplier', () => {
    let prev = 1.0;
    for (let h = 4 * Rprop; h > Rprop * 0.5; h -= 0.01) {
      const m = computeGroundEffectMultiplier(h, Rprop);
      expect(m).toBeGreaterThanOrEqual(prev - 1e-10);
      prev = m;
    }
  });

  it('at h = R_prop: multiplier > 1', () => {
    const m = computeGroundEffectMultiplier(Rprop, Rprop);
    expect(m).toBeGreaterThan(1.0);
    expect(m).toBeLessThanOrEqual(1.5);
  });

  it('multiplier is always >= 1.0', () => {
    for (let h = 0.001; h < 1; h += 0.01) {
      expect(computeGroundEffectMultiplier(h, Rprop)).toBeGreaterThanOrEqual(1.0);
    }
  });

  it('h <= 0 returns 1.0 (disabled, no contact model)', () => {
    expect(computeGroundEffectMultiplier(0, Rprop)).toBe(1.0);
    expect(computeGroundEffectMultiplier(-1, Rprop)).toBe(1.0);
  });

  it('capped max is never exceeded', () => {
    expect(computeGroundEffectMultiplier(0.001, Rprop)).toBeLessThanOrEqual(1.5);
  });
});
