/**
 * Integration test: baro + range fusion converges altitude.
 * open_field scene only (acceptance-validated).
 */
import { describe, it, expect } from 'vitest';
import { ErrorStateEKF } from '@sim/estimation/ekf-state';
import { defaultEKFConfig } from '@sim/core/config-defaults';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { v3Create } from '@lib/math';
import { isaPressure } from '@sim/environment/atmosphere';
import { GRAVITY_MPS2 } from '@sim/core/frames';

describe('Baro-range fusion', () => {
  it('altitude converges from 2m initial error within 5s', () => {
    const ekf = new ErrorStateEKF(defaultEKFConfig(), new WorldGeometry(openField()));
    // True altitude = 5m, init estimate at 3m (2m error)
    ekf.reset(v3Create(0, 0, -3)); // estimated at 3m

    const trueAlt = 5;
    const truePressure = isaPressure(trueAlt);
    const trueRange = trueAlt; // flat ground
    const dt = 0.004;
    const gyro = v3Create(0, 0, 0);
    const accel = v3Create(0, 0, -GRAVITY_MPS2);

    for (let i = 0; i < 1250; i++) { // 5 seconds at 250Hz
      ekf.propagate(gyro, accel, dt);

      // Baro at 25Hz (every 10th IMU step)
      if (i % 10 === 0) {
        ekf.updateBaro(truePressure);
      }
      // Range at 30Hz (every ~8th step)
      if (i % 8 === 0) {
        ekf.updateRange(trueRange);
      }
    }

    const est = ekf.getEstimate();
    const estAlt = -est.position[2];
    // Should have converged close to 5m
    expect(Math.abs(estAlt - trueAlt)).toBeLessThan(1.0);
  });
});
