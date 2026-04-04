import { describe, it, expect } from 'vitest';
import { vioPositionUpdate, vioAttitudeUpdate, uwbRangeUpdate } from '@sim/estimation/ekf-updates';
import { v3Create } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import { m16Create, m16Identity, m16CheckDiagPositive } from '@sim/estimation/ekf-math';
import { defaultEKFConfig } from '@sim/core/config-defaults';
import type { EstimatedState } from '@sim/estimation/estimator-types';

function makeState(): EstimatedState {
  return {
    position: v3Create(5, 5, -2), velocity: v3Create(0, 0, 0),
    quaternion: qCreate(1, 0, 0, 0), gyroBias: v3Create(0, 0, 0),
    accelBias: v3Create(0, 0, 0), baroBias: 0,
    angularVelocity: v3Create(0, 0, 0), timestamp: 0,
  };
}

describe('EKF VIO/UWB Updates', () => {
  it('VIO position: perfect measurement gives near-zero innovation', () => {
    const state = makeState();
    const P = m16Create(); m16Identity(P);
    const config = defaultEKFConfig();
    const posMeas = v3Create(5, 5, -2); // exact match
    const rec = vioPositionUpdate(state, P, config, posMeas, 0.01, 0);
    expect(rec.innovationNorm).toBeLessThan(1e-10);
    expect(rec.gated).toBe(false);
  });

  it('VIO attitude: identity quat gives near-zero innovation', () => {
    const state = makeState();
    const P = m16Create(); m16Identity(P);
    const config = defaultEKFConfig();
    const quatMeas = qCreate(1, 0, 0, 0); // exact match
    const rec = vioAttitudeUpdate(state, P, config, quatMeas, 0.001, 0);
    expect(rec.innovationNorm).toBeLessThan(1e-10);
    expect(rec.gated).toBe(false);
  });

  it('UWB range: correct range gives near-zero innovation', () => {
    const state = makeState();
    const P = m16Create(); m16Identity(P);
    const config = defaultEKFConfig();
    const anchor = v3Create(0, 0, -0.5);
    const trueRange = Math.sqrt(25 + 25 + 2.25);
    const rec = uwbRangeUpdate(state, P, config, trueRange, anchor, false, 0);
    expect(rec.innovationNorm).toBeLessThan(0.01);
    expect(rec.gated).toBe(false);
  });

  it('VIO position update preserves SPD', () => {
    const state = makeState();
    const P = m16Create(); m16Identity(P);
    const config = defaultEKFConfig();
    vioPositionUpdate(state, P, config, v3Create(5.1, 5.1, -2.1), 0.01, 0);
    expect(m16CheckDiagPositive(P)).toBe(true);
  });

  it('UWB range update preserves SPD', () => {
    const state = makeState();
    const P = m16Create(); m16Identity(P);
    const config = defaultEKFConfig();
    uwbRangeUpdate(state, P, config, 7.2, v3Create(0, 0, -0.5), false, 0);
    expect(m16CheckDiagPositive(P)).toBe(true);
  });
});
