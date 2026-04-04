import { describe, it, expect } from 'vitest';
import { computeGroundEffectMultiplier, computeWallCeilingFactor } from '@sim/environment/ground-effect';

describe('Wall/ceiling effect', () => {
  it('no wall nearby: factor = 1.0', () => {
    expect(computeWallCeilingFactor(Infinity, Infinity, 0.1)).toBe(1.0);
  });

  it('wall close: factor > 1.0', () => {
    const factor = computeWallCeilingFactor(0.2, Infinity, 0.1);
    expect(factor).toBeGreaterThan(1.0);
  });

  it('ceiling close: factor > 1.0', () => {
    const factor = computeWallCeilingFactor(Infinity, 0.15, 0.1);
    expect(factor).toBeGreaterThan(1.0);
  });

  it('both wall and ceiling: combined factor > wall alone', () => {
    const wallOnly = computeWallCeilingFactor(0.2, Infinity, 0.1);
    const both = computeWallCeilingFactor(0.2, 0.15, 0.1);
    expect(both).toBeGreaterThan(wallOnly);
  });

  it('far from surfaces: factor = 1.0', () => {
    expect(computeWallCeilingFactor(10, 10, 0.1)).toBe(1.0);
  });

  it('clamped to max 1.5', () => {
    const factor = computeWallCeilingFactor(0.01, 0.01, 0.1);
    expect(factor).toBeLessThanOrEqual(1.5);
  });
});
