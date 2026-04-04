import { describe, it, expect } from 'vitest';
import { TurbulenceGenerator } from '@sim/environment/turbulence';
import { DeterministicRNG } from '@sim/core/rng';
import type { TurbulenceConfig } from '@sim/core/types';

function makeConfig(): TurbulenceConfig {
  return { enabled: true, intensity: 2.0, correlationTime: 1.0 };
}

describe('Turbulence (OU colored noise)', () => {
  it('seeded reproducibility: same seed produces same sequence', () => {
    const a = new TurbulenceGenerator(makeConfig(), new DeterministicRNG(42));
    const b = new TurbulenceGenerator(makeConfig(), new DeterministicRNG(42));
    const dt = 0.001;
    for (let i = 0; i < 100; i++) { a.step(dt); b.step(dt); }
    const outA = a.getOutput(), outB = b.getOutput();
    expect(outA[0]).toBe(outB[0]);
    expect(outA[1]).toBe(outB[1]);
    expect(outA[2]).toBe(outB[2]);
  });

  it('different seeds produce different sequences', () => {
    const a = new TurbulenceGenerator(makeConfig(), new DeterministicRNG(42));
    const b = new TurbulenceGenerator(makeConfig(), new DeterministicRNG(99));
    const dt = 0.001;
    for (let i = 0; i < 100; i++) { a.step(dt); b.step(dt); }
    const outA = a.getOutput(), outB = b.getOutput();
    expect(outA[0]).not.toBe(outB[0]);
  });

  it('NOT white noise: autocorrelation at lag 1 is significantly positive', () => {
    const turb = new TurbulenceGenerator(
      { enabled: true, intensity: 2.0, correlationTime: 0.5 },
      new DeterministicRNG(42),
    );
    const dt = 0.001;
    const values: number[] = [];
    for (let i = 0; i < 5000; i++) {
      turb.step(dt);
      values.push(turb.getOutput()[0]);
    }
    // Compute sample autocorrelation at lag 1
    const n = values.length;
    const mean = values.reduce((s, v) => s + v, 0) / n;
    let var0 = 0, cov1 = 0;
    for (let i = 0; i < n; i++) {
      var0 += (values[i] - mean) ** 2;
      if (i < n - 1) cov1 += (values[i] - mean) * (values[i + 1] - mean);
    }
    const r1 = cov1 / var0;
    // For OU with tau=0.5 and dt=0.001: decay = 1 - 0.001/0.5 = 0.998
    // So autocorrelation at lag 1 should be very high
    expect(r1).toBeGreaterThan(0.9);
  });

  it('output RMS approximately equals configured intensity', () => {
    const intensity = 3.0;
    const turb = new TurbulenceGenerator(
      { enabled: true, intensity, correlationTime: 0.5 },
      new DeterministicRNG(7),
    );
    const dt = 0.001;
    let sumSq = 0;
    const N = 50000;
    // Let it settle first
    for (let i = 0; i < 1000; i++) turb.step(dt);
    for (let i = 0; i < N; i++) {
      turb.step(dt);
      const v = turb.getOutput()[0];
      sumSq += v * v;
    }
    const rms = Math.sqrt(sumSq / N);
    expect(rms).toBeCloseTo(intensity, 0); // within ~20%
  });

  it('reset clears state', () => {
    const turb = new TurbulenceGenerator(makeConfig(), new DeterministicRNG(42));
    for (let i = 0; i < 100; i++) turb.step(0.001);
    turb.reset();
    const out = turb.getOutput();
    expect(out[0]).toBe(0);
    expect(out[1]).toBe(0);
    expect(out[2]).toBe(0);
  });
});
