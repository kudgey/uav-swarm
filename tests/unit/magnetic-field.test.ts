import { describe, it, expect } from 'vitest';
import { MagneticFieldModel } from '@sim/environment/magnetic-field';
import { v3Create } from '@lib/math';

describe('Magnetic Field', () => {
  it('at 90-deg inclination (pole): field is purely vertical (NED z)', () => {
    const model = new MagneticFieldModel({
      fieldStrength: 50e-6,
      inclination: Math.PI / 2,  // 90 degrees
      declination: 0,
      anomalies: [],
    });
    const out = v3Create();
    model.evaluate(out, v3Create(0, 0, 0));
    expect(Math.abs(out[0])).toBeLessThan(1e-10); // North ~0
    expect(Math.abs(out[1])).toBeLessThan(1e-10); // East ~0
    expect(out[2]).toBeCloseTo(50e-6, 8);          // Down = field strength
  });

  it('at 0-deg inclination (equator): field is purely horizontal', () => {
    const model = new MagneticFieldModel({
      fieldStrength: 50e-6,
      inclination: 0,
      declination: 0,
      anomalies: [],
    });
    const out = v3Create();
    model.evaluate(out, v3Create(0, 0, 0));
    expect(out[0]).toBeCloseTo(50e-6, 8);          // All in North
    expect(Math.abs(out[1])).toBeLessThan(1e-10);
    expect(Math.abs(out[2])).toBeLessThan(1e-10);
  });

  it('anomaly at zero distance: full strength', () => {
    const model = new MagneticFieldModel({
      fieldStrength: 50e-6,
      inclination: 0,
      declination: 0,
      anomalies: [{ position: v3Create(0, 0, 0), radius: 5, strength: v3Create(10e-6, 0, 0) }],
    });
    const out = v3Create();
    model.evaluate(out, v3Create(0, 0, 0));
    expect(out[0]).toBeCloseTo(60e-6, 8); // base + anomaly
  });

  it('anomaly at radius: zero perturbation', () => {
    const model = new MagneticFieldModel({
      fieldStrength: 50e-6,
      inclination: 0,
      declination: 0,
      anomalies: [{ position: v3Create(0, 0, 0), radius: 5, strength: v3Create(10e-6, 0, 0) }],
    });
    const out = v3Create();
    model.evaluate(out, v3Create(5, 0, 0)); // at edge of radius
    expect(out[0]).toBeCloseTo(50e-6, 8); // no anomaly effect
  });

  it('anomaly decays quadratically with distance', () => {
    const model = new MagneticFieldModel({
      fieldStrength: 0,
      inclination: 0, declination: 0,
      anomalies: [{ position: v3Create(0, 0, 0), radius: 10, strength: v3Create(1e-5, 0, 0) }],
    });
    const half = v3Create(), quarter = v3Create();
    model.evaluate(half, v3Create(5, 0, 0));     // 50% of radius
    model.evaluate(quarter, v3Create(2.5, 0, 0)); // 25% of radius
    // At 50%: (1-0.5)^2 = 0.25, at 25%: (1-0.25)^2 = 0.5625
    expect(half[0]).toBeCloseTo(1e-5 * 0.25, 8);
    expect(quarter[0]).toBeCloseTo(1e-5 * 0.5625, 8);
  });
});
