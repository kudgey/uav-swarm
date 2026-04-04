import { describe, it, expect } from 'vitest';
import { SafetyLayer } from '@sim/safety/safety-layer';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { v3Create, v3Sub, v3Len } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import type { SafetyConfig } from '@sim/safety/safety-types';
import type { NeighborEstimate } from '@sim/swarm/drone-instance';

describe('Corridor passing', () => {
  it('2 drones pass in corridor: ORCA deflects, min sep > minSeparation', () => {
    const cfg: SafetyConfig = {
      enabled: true, minSeparation: 0.5, orcaRadius: 0.3,
      orcaTimeHorizon: 3, maxSpeed: 2, minAltitude: 0.3,
      noFlyZones: [], emergencyStopDistance: 0.2,
    };
    const layer = new SafetyLayer();
    const worldGeo = new WorldGeometry(openField());

    // Offset start: not exactly head-on, slight lateral offset
    const pos = [v3Create(2, 5, -2), v3Create(8, 5.3, -2)];
    const vel = [v3Create(1, 0, 0), v3Create(-1, 0, 0)];
    let minSep = Infinity;

    for (let tick = 0; tick < 150; tick++) {
      const t = tick * 0.05;
      for (let i = 0; i < 2; i++) {
        const other = 1 - i;
        const neighbors = new Map<number, NeighborEstimate>();
        neighbors.set(other, {
          senderId: other, timestamp: t,
          position: v3Create(pos[other][0], pos[other][1], pos[other][2]),
          positionVariance: 0.01, velocity: v3Create(vel[other][0], vel[other][1], vel[other][2]),
          aligned: true, attitudeHealthy: true, inFormation: false,
        });
        const { safeGuidance } = layer.adjustGuidance(
          { position: pos[i], velocity: vel[i], quaternion: qCreate(1, 0, 0, 0),
            gyroBias: v3Create(0, 0, 0), accelBias: v3Create(0, 0, 0), baroBias: 0,
            angularVelocity: v3Create(0, 0, 0), timestamp: t },
          { positionDes: v3Create(i === 0 ? 8 : 2, 5, -2), velocityDes: v3Create(vel[i][0], vel[i][1], 0),
            accelerationDes: v3Create(0, 0, 0), yawDes: 0 },
          neighbors, worldGeo, cfg, t);
        vel[i][0] = safeGuidance.velocityDes[0];
        vel[i][1] = safeGuidance.velocityDes[1];
      }
      for (let i = 0; i < 2; i++) {
        pos[i][0] += vel[i][0] * 0.05;
        pos[i][1] += vel[i][1] * 0.05;
      }
      const diff = v3Create(); v3Sub(diff, pos[0], pos[1]);
      const sep = v3Len(diff);
      if (sep < minSep) minSep = sep;
    }

    expect(minSep).toBeGreaterThan(cfg.minSeparation);
  });
});
