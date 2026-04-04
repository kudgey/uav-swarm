import { describe, it, expect } from 'vitest';
import { computeInnerLoop } from '@sim/control/inner-loop';
import { v3Create } from '@lib/math';
import { qCreate, qFromAxisAngle } from '@sim/physics/quaternion';
import { defaultDroneParams, defaultControllerConfig } from '@sim/core/config-defaults';

describe('Inner Loop (Geometric SO(3))', () => {
  it('zero attitude error → zero torque', () => {
    const qEst = qCreate(1, 0, 0, 0);
    const qDes = qCreate(1, 0, 0, 0);
    const omega = v3Create(0, 0, 0);
    const params = defaultDroneParams();
    const config = defaultControllerConfig();

    const torque = computeInnerLoop(qEst, omega, qDes, params, config);
    expect(Math.abs(torque[0])).toBeLessThan(1e-8);
    expect(Math.abs(torque[1])).toBeLessThan(1e-8);
    expect(Math.abs(torque[2])).toBeLessThan(1e-8);
  });

  it('roll error produces correct-sign roll torque', () => {
    const qEst = qCreate();
    qFromAxisAngle(qEst, v3Create(1, 0, 0), 0.1); // 0.1 rad roll error
    const qDes = qCreate(1, 0, 0, 0); // identity = level
    const omega = v3Create(0, 0, 0);
    const params = defaultDroneParams();
    const config = defaultControllerConfig();

    const torque = computeInnerLoop(qEst, omega, qDes, params, config);
    // Roll error is positive → torque should be negative (restoring)
    expect(torque[0]).toBeLessThan(0);
  });

  it('pitch error produces pitch torque', () => {
    const qEst = qCreate();
    qFromAxisAngle(qEst, v3Create(0, 1, 0), 0.1); // 0.1 rad pitch
    const qDes = qCreate(1, 0, 0, 0);
    const omega = v3Create(0, 0, 0);
    const params = defaultDroneParams();
    const config = defaultControllerConfig();

    const torque = computeInnerLoop(qEst, omega, qDes, params, config);
    expect(torque[1]).toBeLessThan(0); // restoring
  });
});
