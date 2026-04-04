import { describe, it, expect } from 'vitest';
import { magUpdate, baroUpdate } from '@sim/estimation/ekf-updates';
import { v3Create } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import { m16Create, m16Identity, m16CheckDiagPositive } from '@sim/estimation/ekf-math';
import { defaultEKFConfig } from '@sim/core/config-defaults';
import type { EstimatedState } from '@sim/estimation/estimator-types';

function makeState(): EstimatedState {
  return {
    position: v3Create(0, 0, -2), velocity: v3Create(0, 0, 0),
    quaternion: qCreate(1, 0, 0, 0), gyroBias: v3Create(0, 0, 0),
    accelBias: v3Create(0, 0, 0), baroBias: 0,
    angularVelocity: v3Create(0, 0, 0), timestamp: 0,
  };
}

describe('EKF Updates', () => {
  it('perfect mag measurement gives near-zero innovation', () => {
    const state = makeState();
    const P = m16Create(); m16Identity(P);
    const config = defaultEKFConfig();
    // At identity quat, predicted body field = earth field
    const magMeas = v3Create(config.earthMagField[0], config.earthMagField[1], config.earthMagField[2]);
    const rec = magUpdate(state, P, config, magMeas, 0);
    expect(rec.innovationNorm).toBeLessThan(1e-10);
    expect(rec.gated).toBe(false);
  });

  it('mag gating rejects outlier', () => {
    const state = makeState();
    const P = m16Create(); m16Identity(P);
    const config = defaultEKFConfig();
    // Huge outlier
    const magMeas = v3Create(1, 1, 1); // way off from ~50 µT
    const rec = magUpdate(state, P, config, magMeas, 0);
    expect(rec.gated).toBe(true);
  });

  it('baro update preserves SPD covariance', () => {
    const state = makeState();
    const P = m16Create(); m16Identity(P);
    const config = defaultEKFConfig();
    // Measured pressure close to predicted
    const pMeas = 101300; // close to sea level
    baroUpdate(state, P, config, pMeas, 0);
    expect(m16CheckDiagPositive(P)).toBe(true);
  });
});
