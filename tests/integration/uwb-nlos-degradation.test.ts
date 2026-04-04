/**
 * Integration test: NLOS anchor degrades position accuracy.
 */
import { describe, it, expect } from 'vitest';
import { ErrorStateEKF } from '@sim/estimation/ekf-state';
import { defaultEKFConfig } from '@sim/core/config-defaults';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { v3Create } from '@lib/math';
import { GRAVITY_MPS2 } from '@sim/core/frames';

describe('UWB NLOS degradation', () => {
  it('NLOS anchor causes position estimate shift', () => {
    const scene = openField();
    const config = defaultEKFConfig();
    const anchors = scene.uwbAnchors!;

    // Run 1: all LOS
    const ekf1 = new ErrorStateEKF(config, new WorldGeometry(scene));
    ekf1.reset(v3Create(5, 5, -2));
    const dt = 0.004;
    const gyro = v3Create(0, 0, 0);
    const accel = v3Create(0, 0, -GRAVITY_MPS2);

    for (let i = 0; i < 1250; i++) {
      ekf1.propagate(gyro, accel, dt);
      if (i % 25 === 0) {
        for (const anchor of anchors) {
          const dx = 5 - anchor.position[0], dy = 5 - anchor.position[1], dz = -2 - anchor.position[2];
          const range = Math.sqrt(dx * dx + dy * dy + dz * dz) + 0.05;
          ekf1.updateUWBRange(range, anchor.position, false);
        }
      }
    }
    const pos1 = [...ekf1.getEstimate().position];

    // Run 2: one anchor NLOS with +0.5m bias
    const ekf2 = new ErrorStateEKF(config, new WorldGeometry(scene));
    ekf2.reset(v3Create(5, 5, -2));

    for (let i = 0; i < 1250; i++) {
      ekf2.propagate(gyro, accel, dt);
      if (i % 25 === 0) {
        for (let a = 0; a < anchors.length; a++) {
          const anchor = anchors[a];
          const dx = 5 - anchor.position[0], dy = 5 - anchor.position[1], dz = -2 - anchor.position[2];
          let range = Math.sqrt(dx * dx + dy * dy + dz * dz) + 0.05;
          const isNLOS = a === 0; // first anchor is NLOS
          if (isNLOS) range += 0.5; // positive NLOS bias
          ekf2.updateUWBRange(range, anchor.position, isNLOS);
        }
      }
    }
    const pos2 = [...ekf2.getEstimate().position];

    // Positions should differ measurably
    const diff = Math.sqrt(
      (pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2 + (pos1[2] - pos2[2]) ** 2);
    expect(diff).toBeGreaterThan(0.05); // measurable shift from NLOS bias
  });
});
