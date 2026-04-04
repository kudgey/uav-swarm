import { describe, it, expect } from 'vitest';
import { DeterministicRNG } from '@sim/core/rng';

describe('DeterministicRNG', () => {
  it('same seed produces same sequence', () => {
    const a = new DeterministicRNG(42);
    const b = new DeterministicRNG(42);
    for (let i = 0; i < 1000; i++) {
      expect(a.next()).toBe(b.next());
    }
  });

  it('different seeds produce different sequences', () => {
    const a = new DeterministicRNG(42);
    const b = new DeterministicRNG(123);
    let same = 0;
    for (let i = 0; i < 100; i++) {
      if (a.next() === b.next()) same++;
    }
    expect(same).toBeLessThan(5);
  });

  it('output is in [0, 1)', () => {
    const rng = new DeterministicRNG(1);
    for (let i = 0; i < 10000; i++) {
      const v = rng.next();
      expect(v).toBeGreaterThanOrEqual(0);
      expect(v).toBeLessThan(1);
    }
  });

  it('basic uniformity check over 10k samples', () => {
    const rng = new DeterministicRNG(7);
    const bins = new Array(10).fill(0);
    const n = 10000;
    for (let i = 0; i < n; i++) {
      const bin = Math.floor(rng.next() * 10);
      bins[Math.min(bin, 9)]++;
    }
    // Each bin should have ~1000. Allow 20% deviation.
    for (const count of bins) {
      expect(count).toBeGreaterThan(800);
      expect(count).toBeLessThan(1200);
    }
  });

  it('clone produces independent but identical continuation', () => {
    const rng = new DeterministicRNG(99);
    for (let i = 0; i < 50; i++) rng.next();
    const clone = rng.clone();
    for (let i = 0; i < 100; i++) {
      expect(rng.next()).toBe(clone.next());
    }
  });

  it('gaussian produces reasonable values', () => {
    const rng = new DeterministicRNG(55);
    let sum = 0;
    const n = 10000;
    for (let i = 0; i < n; i++) {
      sum += rng.gaussian(0, 1);
    }
    const mean = sum / n;
    expect(Math.abs(mean)).toBeLessThan(0.1);
  });
});
