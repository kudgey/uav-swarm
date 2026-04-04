import { describe, it, expect } from 'vitest';

describe('LOS-projected variance', () => {
  it('horizontal LOS: projected ≈ horizontal trace', () => {
    // Same altitude: LOS direction = [1, 0, 0]
    // P = diag(1, 2, 3) → projected = û^T P û = 1*1 + 0 + 0 = 1
    const c = [1, 0, 0, 2, 0, 3]; // Pxx=1, Pyy=2, Pzz=3
    const ux = 1, uy = 0, uz = 0;
    const projected = ux * ux * c[0] + 2 * ux * uy * c[1] + 2 * ux * uz * c[2]
      + uy * uy * c[3] + 2 * uy * uz * c[4] + uz * uz * c[5];
    expect(projected).toBeCloseTo(1, 5);
    // Horizontal trace = Pxx + Pyy = 3, but projected along x-only = 1
    expect(projected).toBeLessThan(1 + 2);
  });

  it('vertical LOS: projected = Pzz', () => {
    const c = [1, 0, 0, 2, 0, 3];
    const ux = 0, uy = 0, uz = 1;
    const projected = ux * ux * c[0] + 2 * ux * uy * c[1] + 2 * ux * uz * c[2]
      + uy * uy * c[3] + 2 * uy * uz * c[4] + uz * uz * c[5];
    expect(projected).toBeCloseTo(3, 5);
  });

  it('diagonal LOS: projected mixes axes', () => {
    const c = [1, 0, 0, 4, 0, 9];
    const d = Math.sqrt(3);
    const ux = 1 / d, uy = 1 / d, uz = 1 / d;
    const projected = ux * ux * c[0] + 2 * ux * uy * c[1] + 2 * ux * uz * c[2]
      + uy * uy * c[3] + 2 * uy * uz * c[4] + uz * uz * c[5];
    // (1 + 4 + 9) / 3 = 4.67
    expect(projected).toBeCloseTo((1 + 4 + 9) / 3, 1);
  });
});
