import { describe, it, expect } from 'vitest';
import { isaTemperature, isaPressure, isaDensity, isaPressureToAltitude } from '@sim/environment/atmosphere';

describe('ISA Atmosphere', () => {
  it('sea level returns standard values', () => {
    expect(isaTemperature(0)).toBeCloseTo(288.15, 2);
    expect(isaPressure(0)).toBeCloseTo(101325, 0);
    expect(isaDensity(0)).toBeCloseTo(1.225, 2);
  });

  it('1000m altitude matches ISA table', () => {
    expect(isaTemperature(1000)).toBeCloseTo(281.65, 1);
    expect(isaPressure(1000)).toBeCloseTo(89876, -1); // ~89.9 kPa
  });

  it('5000m altitude matches ISA table', () => {
    expect(isaTemperature(5000)).toBeCloseTo(255.65, 1);
    expect(isaPressure(5000)).toBeCloseTo(54020, -2); // ~54.0 kPa (within 50 Pa)
  });

  it('pressure-to-altitude roundtrip', () => {
    for (const h of [0, 100, 500, 1000, 3000, 5000]) {
      const p = isaPressure(h);
      const hBack = isaPressureToAltitude(p);
      expect(hBack).toBeCloseTo(h, 1);
    }
  });

  it('density and pressure decrease with altitude', () => {
    let prevP = Infinity, prevRho = Infinity;
    for (let h = 0; h <= 5000; h += 500) {
      const p = isaPressure(h);
      const rho = isaDensity(h);
      expect(p).toBeLessThan(prevP);
      expect(rho).toBeLessThan(prevRho);
      expect(rho).toBeGreaterThan(0);
      prevP = p; prevRho = rho;
    }
  });
});
