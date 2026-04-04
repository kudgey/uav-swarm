/**
 * Integration test: 2 drones crossing paths with ORCA → no collision.
 * Truth min separation must stay above SafetyConfig.minSeparation.
 */
import { describe, it, expect } from 'vitest';
import { SafetyLayer } from '@sim/safety/safety-layer';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { v3Create, v3Sub, v3Len } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import type { EstimatedState } from '@sim/estimation/estimator-types';
import type { GuidanceOutput } from '@sim/control/controller-types';
import type { SafetyConfig } from '@sim/safety/safety-types';
import type { NeighborEstimate } from '@sim/swarm/drone-instance';

describe('Crossing trajectories', () => {
  it('2 drones head-on: min separation > minSeparation policy', () => {
    const cfg: SafetyConfig = {
      enabled: true, minSeparation: 0.5, orcaRadius: 0.3,
      orcaTimeHorizon: 3, maxSpeed: 2, minAltitude: 0.3,
      noFlyZones: [], emergencyStopDistance: 0.2,
    };
    const layer = new SafetyLayer();
    const worldGeo = new WorldGeometry(openField());

    // Two drones approaching each other along x-axis
    const pos = [v3Create(0, 5, -2), v3Create(10, 5, -2)];
    const vel = [v3Create(1, 0, 0), v3Create(-1, 0, 0)];
    let minSepTruth = Infinity;

    for (let tick = 0; tick < 200; tick++) {
      const t = tick * 0.05; // 20Hz

      for (let i = 0; i < 2; i++) {
        const other = 1 - i;
        const est: EstimatedState = {
          position: v3Create(pos[i][0], pos[i][1], pos[i][2]),
          velocity: v3Create(vel[i][0], vel[i][1], vel[i][2]),
          quaternion: qCreate(1, 0, 0, 0), gyroBias: v3Create(0, 0, 0),
          accelBias: v3Create(0, 0, 0), baroBias: 0,
          angularVelocity: v3Create(0, 0, 0), timestamp: t,
        };
        const guidance: GuidanceOutput = {
          positionDes: v3Create(i === 0 ? 10 : 0, 5, -2),
          velocityDes: v3Create(vel[i][0], vel[i][1], vel[i][2]),
          accelerationDes: v3Create(0, 0, 0), yawDes: 0,
        };
        const neighbors = new Map<number, NeighborEstimate>();
        neighbors.set(other, {
          senderId: other, timestamp: t,
          position: v3Create(pos[other][0], pos[other][1], pos[other][2]),
          positionVariance: 0.01, velocity: v3Create(vel[other][0], vel[other][1], vel[other][2]),
          aligned: true, attitudeHealthy: true, inFormation: false,
        });

        const { safeGuidance } = layer.adjustGuidance(est, guidance, neighbors, worldGeo, cfg, t);

        // Simple integration: move toward safe velocity
        vel[i][0] = safeGuidance.velocityDes[0];
        vel[i][1] = safeGuidance.velocityDes[1];
        vel[i][2] = safeGuidance.velocityDes[2];
      }

      // Integrate positions
      for (let i = 0; i < 2; i++) {
        pos[i][0] += vel[i][0] * 0.05;
        pos[i][1] += vel[i][1] * 0.05;
        pos[i][2] += vel[i][2] * 0.05;
      }

      // Truth separation
      const diff = v3Create();
      v3Sub(diff, pos[0], pos[1]);
      const sep = v3Len(diff);
      if (sep < minSepTruth) minSepTruth = sep;
    }

    // Key acceptance: min separation must exceed policy threshold
    expect(minSepTruth).toBeGreaterThan(cfg.minSeparation);
  });
});
