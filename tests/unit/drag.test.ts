import { describe, it, expect } from 'vitest';
import { computeBodyDrag } from '@sim/physics/drag';
import { v3Create } from '@lib/math';
import { defaultDroneParams } from '@sim/core/config-defaults';

describe('Body Drag', () => {
  const params = defaultDroneParams();

  it('drag opposes velocity (linear mode)', () => {
    const vRel = v3Create(5, 0, 0);
    const out = v3Create();
    computeBodyDrag(out, vRel, params);
    expect(out[0]).toBeLessThan(0); // opposes positive x velocity
    expect(out[1]).toBeCloseTo(0);
    expect(out[2]).toBeCloseTo(0);
  });

  it('zero velocity gives zero drag', () => {
    const vRel = v3Create(0, 0, 0);
    const out = v3Create();
    computeBodyDrag(out, vRel, params);
    expect(out[0]).toBeCloseTo(0);
    expect(out[1]).toBeCloseTo(0);
    expect(out[2]).toBeCloseTo(0);
  });

  it('quadratic drag scales with v^2', () => {
    const qParams = { ...params, dragMode: 'quadratic' as const, dragCoeffQuadratic: v3Create(0.5, 0.5, 0.5) };
    const v1 = v3Create(1, 0, 0);
    const v2 = v3Create(2, 0, 0);
    const out1 = v3Create(), out2 = v3Create();
    computeBodyDrag(out1, v1, qParams);
    computeBodyDrag(out2, v2, qParams);
    // |F2| / |F1| should be ~4 for quadratic
    expect(Math.abs(out2[0]) / Math.abs(out1[0])).toBeCloseTo(4, 1);
  });

  it('drag in each axis uses corresponding coefficient', () => {
    const vRel = v3Create(1, 1, 1);
    const out = v3Create();
    computeBodyDrag(out, vRel, params);
    // z-drag should be larger due to larger dragCoeff (0.15 vs 0.1)
    expect(Math.abs(out[2])).toBeGreaterThan(Math.abs(out[0]));
  });
});
