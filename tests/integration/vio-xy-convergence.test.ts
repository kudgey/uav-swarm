/**
 * Integration test: VIO provides XY anchor, covariance decreases.
 */
import { describe, it, expect } from 'vitest';
import { ErrorStateEKF } from '@sim/estimation/ekf-state';
import { defaultEKFConfig } from '@sim/core/config-defaults';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { v3Create } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import { m16Get } from '@sim/estimation/ekf-math';
import { GRAVITY_MPS2 } from '@sim/core/frames';

describe('VIO XY convergence', () => {
  it('horizontal covariance decreases with VIO updates', () => {
    const config = defaultEKFConfig();
    const ekf = new ErrorStateEKF(config, new WorldGeometry(openField()));
    ekf.reset(v3Create(5, 5, -2));

    const dt = 0.004;
    const gyro = v3Create(0, 0, 0);
    const accel = v3Create(0, 0, -GRAVITY_MPS2);

    // Phase 1: IMU only for 2s — covariance grows
    for (let i = 0; i < 500; i++) ekf.propagate(gyro, accel, dt);
    const P = ekf.getCovariance();
    const horizCovAfterIMU = Math.max(m16Get(P, 0, 0), m16Get(P, 1, 1));

    // Phase 2: Add VIO position updates at 30Hz for 3s
    for (let i = 0; i < 750; i++) {
      ekf.propagate(gyro, accel, dt);
      if (i % 8 === 0) { // ~30Hz
        ekf.updateVIOPosition(v3Create(5, 5, -2), config.vioPositionNoiseVar);
      }
    }
    const horizCovAfterVIO = Math.max(m16Get(P, 0, 0), m16Get(P, 1, 1));

    // Covariance should have decreased
    expect(horizCovAfterVIO).toBeLessThan(horizCovAfterIMU);

    // Health should now be 'healthy' for horizontal
    expect(ekf.getHealth().horizontal).toBe('healthy');
  });
});
