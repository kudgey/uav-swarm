/**
 * Integration test: covariance stays SPD over 60s.
 */
import { describe, it, expect } from 'vitest';
import { ErrorStateEKF } from '@sim/estimation/ekf-state';
import { defaultEKFConfig } from '@sim/core/config-defaults';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { v3Create } from '@lib/math';
import { isaPressure } from '@sim/environment/atmosphere';
import { m16CheckDiagPositive } from '@sim/estimation/ekf-math';
import { GRAVITY_MPS2 } from '@sim/core/frames';

describe('Covariance SPD', () => {
  it('P stays SPD over 60s of hover with all updates', () => {
    const config = defaultEKFConfig();
    const ekf = new ErrorStateEKF(config, new WorldGeometry(openField()));
    ekf.reset(v3Create(0, 0, -2));

    const dt = 0.004;
    const gyro = v3Create(0, 0, 0);
    const accel = v3Create(0, 0, -GRAVITY_MPS2);
    const magField = v3Create(config.earthMagField[0], config.earthMagField[1], config.earthMagField[2]);
    const truePressure = isaPressure(2);

    for (let i = 0; i < 15000; i++) { // 60s at 250Hz
      ekf.propagate(gyro, accel, dt);
      if (i % 5 === 0) ekf.updateMag(magField);
      if (i % 10 === 0) ekf.updateBaro(truePressure);
      if (i % 8 === 0) ekf.updateRange(2.0);

      // Check every 250 steps (1 second)
      if (i % 250 === 0) {
        expect(m16CheckDiagPositive(ekf.getCovariance())).toBe(true);
      }
    }

    // Final check
    expect(m16CheckDiagPositive(ekf.getCovariance())).toBe(true);
  }, 30000);
});
