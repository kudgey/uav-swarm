import { describe, it, expect } from 'vitest';
import { MagneticFieldModel } from '@sim/environment/magnetic-field';
import { v3Create, v3Len } from '@lib/math';

describe('WMM magnetic field', () => {
  it('WMM disabled: uses fixed inclination/declination', () => {
    const model = new MagneticFieldModel({
      fieldStrength: 50e-6, inclination: 1.15, declination: 0.05,
      anomalies: [], wmmEnabled: false, latitude: 0, longitude: 0, wmmDate: 2025,
    });
    const out = v3Create();
    model.evaluate(out, v3Create(0, 0, 0));
    const mag = v3Len(out);
    expect(mag).toBeCloseTo(50e-6, 7);
  });

  it('WMM enabled at Moscow: field ~50-55 µT, strong inclination', () => {
    const model = new MagneticFieldModel({
      fieldStrength: 50e-6, inclination: 0, declination: 0,
      anomalies: [], wmmEnabled: true, latitude: 55.75, longitude: 37.62, wmmDate: 2025,
    });
    const out = v3Create();
    model.evaluate(out, v3Create(0, 0, 0));
    const mag = v3Len(out);
    // Moscow: ~54 µT total, ~72° inclination (dipole approx may differ)
    expect(mag).toBeGreaterThan(30e-6);
    expect(mag).toBeLessThan(80e-6);
    // Down component should be significant (high latitude)
    expect(Math.abs(out[2])).toBeGreaterThan(20e-6);
  });

  it('WMM at equator: lower total field, small inclination', () => {
    const model = new MagneticFieldModel({
      fieldStrength: 50e-6, inclination: 0, declination: 0,
      anomalies: [], wmmEnabled: true, latitude: 0, longitude: 0, wmmDate: 2025,
    });
    const out = v3Create();
    model.evaluate(out, v3Create(0, 0, 0));
    const mag = v3Len(out);
    // Equator: ~30 µT (dipole model), mostly horizontal
    expect(mag).toBeGreaterThan(15e-6);
    expect(mag).toBeLessThan(60e-6);
  });
});
