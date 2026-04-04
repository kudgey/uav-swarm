import { describe, it, expect } from 'vitest';
import { GRAVITY_NED, GRAVITY_MPS2, xQuadRotorPositions, X_QUAD_ROTOR_DIRS } from '@sim/core/frames';

describe('Frame Conventions', () => {
  it('gravity points +z in NED (downward)', () => {
    expect(GRAVITY_NED[0]).toBe(0);
    expect(GRAVITY_NED[1]).toBe(0);
    expect(GRAVITY_NED[2]).toBeCloseTo(GRAVITY_MPS2);
    expect(GRAVITY_NED[2]).toBeGreaterThan(0);
  });

  it('X-quad has 4 rotors with alternating spin directions', () => {
    expect(X_QUAD_ROTOR_DIRS).toEqual([1, -1, 1, -1]);
    expect(X_QUAD_ROTOR_DIRS.length).toBe(4);
  });

  it('rotor positions are symmetric and at correct distance', () => {
    const arm = 0.2;
    const positions = xQuadRotorPositions(arm);
    expect(positions.length).toBe(4);
    const d = arm / Math.SQRT2;

    // Rotor 1: front-left [+d, -d, 0]
    expect(positions[0][0]).toBeCloseTo(d);
    expect(positions[0][1]).toBeCloseTo(-d);
    expect(positions[0][2]).toBeCloseTo(0);

    // Rotor 2: front-right [+d, +d, 0]
    expect(positions[1][0]).toBeCloseTo(d);
    expect(positions[1][1]).toBeCloseTo(d);

    // All at same distance from center
    for (const p of positions) {
      const dist = Math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
      expect(dist).toBeCloseTo(arm, 10);
    }
  });

  it('rotor z-positions are zero (in-plane)', () => {
    const positions = xQuadRotorPositions(0.17);
    for (const p of positions) {
      expect(p[2]).toBe(0);
    }
  });
});
