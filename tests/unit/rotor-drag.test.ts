import { describe, it, expect } from 'vitest';
import { computeBodyDrag } from '@sim/physics/drag';
import { defaultDroneParams } from '@sim/core/config-defaults';
import { v3Create } from '@lib/math';

describe('Rotor drag', () => {
  it('zero rotor drag coeffs: no rotor drag contribution', () => {
    const params = defaultDroneParams();
    const out = v3Create();
    computeBodyDrag(out, v3Create(5, 0, 0), params);
    const withoutRotor = out[0];
    // Default dragCoeffRotor = [0,0,0] → no rotor drag
    expect(withoutRotor).toBeCloseTo(-0.5, 1); // linear drag only: -0.1 * 5 = -0.5
  });

  it('non-zero rotor drag increases total drag', () => {
    const params = { ...defaultDroneParams(), dragCoeffRotor: v3Create(0.2, 0.2, 0.1) };
    const out = v3Create();
    computeBodyDrag(out, v3Create(5, 0, 0), params);
    // Linear: -0.1 * 5 = -0.5, Rotor: -0.2 * 5 = -1.0 → total: -1.5
    expect(out[0]).toBeCloseTo(-1.5, 1);
  });

  it('rotor drag is per-axis independent', () => {
    const params = { ...defaultDroneParams(), dragCoeffRotor: v3Create(0.3, 0.1, 0.05) };
    const out = v3Create();
    computeBodyDrag(out, v3Create(1, 1, 1), params);
    // X: linear(-0.1) + rotor(-0.3) = -0.4
    // Y: linear(-0.1) + rotor(-0.1) = -0.2
    // Z: linear(-0.15) + rotor(-0.05) = -0.2
    expect(out[0]).toBeCloseTo(-0.4, 2);
    expect(out[1]).toBeCloseTo(-0.2, 2);
    expect(out[2]).toBeCloseTo(-0.2, 2);
  });
});
