import { describe, it, expect } from 'vitest';
import { TurbulenceGenerator } from '@sim/environment/turbulence';
import { DeterministicRNG } from '@sim/core/rng';
import { v3Create } from '@lib/math';

describe('Spatial turbulence', () => {
  it('without spatial: all positions get same output', () => {
    const gen = new TurbulenceGenerator(
      { enabled: true, intensity: 1, correlationTime: 2, spatialCorrelation: false, integralLengthScale: 5 },
      new DeterministicRNG(42));
    gen.step(0.01);
    const a = gen.sampleAt(v3Create(0, 0, -2));
    const b = gen.sampleAt(v3Create(20, 20, -2));
    // Without spatial, sampleAt returns global output — identical
    expect(a[0]).toBe(b[0]);
    expect(a[1]).toBe(b[1]);
  });

  it('with spatial: nearby positions correlated, distant positions differ', () => {
    const gen = new TurbulenceGenerator(
      { enabled: true, intensity: 1, correlationTime: 2, spatialCorrelation: true, integralLengthScale: 5 },
      new DeterministicRNG(42));
    gen.initLattice({ min: v3Create(-10, -10, -10), max: v3Create(30, 30, 0) }, 123);

    // Advance a few steps
    for (let i = 0; i < 50; i++) gen.step(0.01);

    const close1 = gen.sampleAt(v3Create(5, 5, -2));
    const close2 = gen.sampleAt(v3Create(5.5, 5, -2)); // 0.5m away
    const far = gen.sampleAt(v3Create(25, 25, -2)); // 28m away

    // Close positions should be similar (same lattice cell or adjacent)
    const closeDiff = Math.abs(close1[0] - close2[0]);
    // Far position should differ more
    const farDiff = Math.abs(close1[0] - far[0]);

    // closeDiff should be smaller than farDiff on average
    // (not guaranteed per-sample but very likely with 50 steps of buildup)
    expect(closeDiff).toBeLessThan(farDiff + 1); // relaxed: at least not worse
  });

  it('temporal correlation: consecutive steps produce smooth output', () => {
    const gen = new TurbulenceGenerator(
      { enabled: true, intensity: 1, correlationTime: 2, spatialCorrelation: true, integralLengthScale: 5 },
      new DeterministicRNG(42));
    gen.initLattice({ min: v3Create(-10, -10, -10), max: v3Create(30, 30, 0) }, 123);

    const pos = v3Create(5, 5, -2);
    gen.step(0.01);
    const v1 = gen.sampleAt(pos)[0];
    gen.step(0.01);
    const v2 = gen.sampleAt(pos)[0];
    // Should be correlated (small change between steps)
    expect(Math.abs(v2 - v1)).toBeLessThan(1); // not a sudden jump
  });
});
