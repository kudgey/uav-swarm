/**
 * Integration test: Ground effect reduces thrust needed for hover.
 * Near-ground requires less motor speed than high altitude.
 */
import { describe, it, expect } from 'vitest';
import { computeGroundEffectMultiplier } from '@sim/environment/ground-effect';

describe('Ground effect hover', () => {
  it('near-ground multiplier > 1, high altitude = 1', () => {
    const propRadius = 0.12;
    const nearGround = computeGroundEffectMultiplier(0.3, propRadius); // 30cm
    const highAlt = computeGroundEffectMultiplier(5.0, propRadius);    // 5m

    expect(nearGround).toBeGreaterThan(1.0);
    expect(highAlt).toBe(1.0);
    expect(nearGround).toBeGreaterThan(highAlt);
  });
});
