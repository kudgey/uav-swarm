import { describe, it, expect } from 'vitest';
import { WindField } from '@sim/environment/wind';
import { v3Create } from '@lib/math';
import type { WindConfig } from '@sim/core/types';

function makeConfig(overrides?: Partial<WindConfig>): WindConfig {
  return {
    meanSpeed: 5,
    meanDirection: v3Create(1, 0, 0), // North
    shearEnabled: false,
    shearExponent: 0.14,
    gustEvents: [],
    ...overrides,
  };
}

describe('Wind Field', () => {
  it('mean wind has correct direction and magnitude', () => {
    const wind = new WindField(makeConfig({ meanSpeed: 5 }));
    const out = v3Create();
    wind.evaluate(out, v3Create(0, 0, -10), 0); // 10m altitude
    expect(out[0]).toBeCloseTo(5, 1); // North
    expect(out[1]).toBeCloseTo(0, 5);
    expect(out[2]).toBeCloseTo(0, 5);
  });

  it('altitude shear increases wind with height', () => {
    const wind = new WindField(makeConfig({ meanSpeed: 5, shearEnabled: true }));
    const lowOut = v3Create(), highOut = v3Create();
    wind.evaluate(lowOut, v3Create(0, 0, -2), 0);  // 2m
    wind.evaluate(highOut, v3Create(0, 0, -50), 0); // 50m
    expect(Math.abs(highOut[0])).toBeGreaterThan(Math.abs(lowOut[0]));
  });

  it('gust zero outside event window', () => {
    const wind = new WindField(makeConfig({
      meanSpeed: 0,
      gustEvents: [{ onsetTime: 5, duration: 2, peakAmplitude: v3Create(10, 0, 0) }],
    }));
    const out = v3Create();
    wind.evaluate(out, v3Create(0, 0, -5), 0); // before gust
    expect(out[0]).toBeCloseTo(0, 5);
    wind.evaluate(out, v3Create(0, 0, -5), 8); // after gust
    expect(out[0]).toBeCloseTo(0, 5);
  });

  it('gust peaks at end of duration (one-minus-cosine)', () => {
    const wind = new WindField(makeConfig({
      meanSpeed: 0,
      gustEvents: [{ onsetTime: 1, duration: 2, peakAmplitude: v3Create(10, 0, 0) }],
    }));
    const out = v3Create();
    // One-minus-cosine: 0.5*(1-cos(pi*t_local/T))
    // At t_local = T (end): 0.5*(1-cos(pi)) = 0.5*(1+1) = 1.0 → peak
    // At t_local = T/2 (mid): 0.5*(1-cos(pi/2)) = 0.5*(1-0) = 0.5
    wind.evaluate(out, v3Create(0, 0, -5), 2); // t=2, t_local=1 = T/2
    expect(out[0]).toBeCloseTo(5, 1); // 0.5 * 10 at midpoint
    wind.evaluate(out, v3Create(0, 0, -5), 3); // t=3, t_local=2 = T → full peak
    expect(out[0]).toBeCloseTo(10, 1);
  });

  it('multiple gusts superimpose', () => {
    const wind = new WindField(makeConfig({
      meanSpeed: 0,
      gustEvents: [
        { onsetTime: 0, duration: 2, peakAmplitude: v3Create(5, 0, 0) },
        { onsetTime: 0, duration: 2, peakAmplitude: v3Create(0, 3, 0) },
      ],
    }));
    const out = v3Create();
    wind.evaluate(out, v3Create(0, 0, -5), 1); // midpoint: 0.5 of each
    expect(out[0]).toBeCloseTo(2.5, 1);
    expect(out[1]).toBeCloseTo(1.5, 1);
  });
});
