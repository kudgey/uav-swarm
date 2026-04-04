/**
 * Integration test: UWB with 4 anchors bounds XY error.
 */
import { describe, it, expect } from 'vitest';
import { ErrorStateEKF } from '@sim/estimation/ekf-state';
import { defaultEKFConfig } from '@sim/core/config-defaults';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { v3Create } from '@lib/math';
import { m16Get } from '@sim/estimation/ekf-math';
import { GRAVITY_MPS2 } from '@sim/core/frames';

describe('UWB position hold', () => {
  it('4 anchors: XY covariance bounded over 10s', () => {
    const scene = openField();
    const config = defaultEKFConfig();
    const ekf = new ErrorStateEKF(config, new WorldGeometry(scene));
    ekf.reset(v3Create(5, 5, -2));

    const dt = 0.004;
    const gyro = v3Create(0, 0, 0);
    const accel = v3Create(0, 0, -GRAVITY_MPS2);
    const anchors = scene.uwbAnchors!;

    for (let i = 0; i < 2500; i++) { // 10s at 250Hz
      ekf.propagate(gyro, accel, dt);
      // UWB at 10Hz (every 25th IMU step)
      if (i % 25 === 0) {
        for (const anchor of anchors) {
          const dx = 5 - anchor.position[0];
          const dy = 5 - anchor.position[1];
          const dz = -2 - anchor.position[2];
          const trueRange = Math.sqrt(dx * dx + dy * dy + dz * dz);
          ekf.updateUWBRange(trueRange + 0.05, anchor.position, false); // small LOS bias
        }
      }
    }

    const P = ekf.getCovariance();
    const horizCov = Math.max(m16Get(P, 0, 0), m16Get(P, 1, 1));
    // With 4 anchors, horizontal covariance should be bounded
    expect(horizCov).toBeLessThan(1.0); // bounded, not growing
    expect(ekf.getHealth().horizontal).toBe('healthy');
  });
});
