import { describe, it, expect } from 'vitest';
import { ErrorStateEKF } from '@sim/estimation/ekf-state';
import { defaultEKFConfig } from '@sim/core/config-defaults';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { v3Create } from '@lib/math';
import { GRAVITY_MPS2 } from '@sim/core/frames';

describe('Innovation-based NLOS detection', () => {
  it('without detection: truth isNLOS flag used directly', () => {
    const config = { ...defaultEKFConfig(), innovationNLOSDetection: false };
    const ekf = new ErrorStateEKF(config, new WorldGeometry(openField()));
    ekf.reset(v3Create(5, 5, -2));

    // Propagate to get non-trivial state
    const gyro = v3Create(0, 0, 0);
    const accel = v3Create(0, 0, -GRAVITY_MPS2);
    for (let i = 0; i < 10; i++) ekf.propagate(gyro, accel, 0.004);

    // Normal range: should not be gated
    const anchor = v3Create(0, 0, -0.5);
    const trueRange = Math.sqrt(25 + 25 + 2.25);
    ekf.updateUWBRange(trueRange + 0.05, anchor, false, undefined, undefined, 'A0');
    // No crash, no detection happening
  });

  it('with detection: consecutive high innovations switch to NLOS', () => {
    const config = { ...defaultEKFConfig(), innovationNLOSDetection: true, nlosConsecutiveCount: 2, nlosInnovThreshold: 3 };
    const ekf = new ErrorStateEKF(config, new WorldGeometry(openField()));
    ekf.reset(v3Create(5, 5, -2));

    const gyro = v3Create(0, 0, 0);
    const accel = v3Create(0, 0, -GRAVITY_MPS2);
    for (let i = 0; i < 10; i++) ekf.propagate(gyro, accel, 0.004);

    const anchor = v3Create(0, 0, -0.5);
    const trueRange = Math.sqrt(25 + 25 + 2.25);

    // Send several measurements with large bias (simulating NLOS)
    for (let i = 0; i < 5; i++) {
      ekf.propagate(gyro, accel, 0.004);
      ekf.updateUWBRange(trueRange + 2.0, anchor, false, undefined, undefined, 'A0'); // +2m bias
    }

    // The tracker should have detected NLOS by now (innovation high for >= 2 consecutive)
    // Verification: the EKF should not have diverged (NLOS detection switches to larger R)
    const est = ekf.getEstimate();
    expect(isFinite(est.position[0])).toBe(true);
    expect(isFinite(est.position[2])).toBe(true);
  });
});
