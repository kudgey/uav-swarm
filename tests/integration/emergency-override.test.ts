import { describe, it, expect } from 'vitest';
import { SafetyLayer } from '@sim/safety/safety-layer';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { v3Create } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import type { SafetyConfig } from '@sim/safety/safety-types';
import type { NeighborEstimate } from '@sim/swarm/drone-instance';

describe('Emergency override', () => {
  it('triggers level-hover when neighbor within emergencyStopDistance', () => {
    const cfg: SafetyConfig = {
      enabled: true, minSeparation: 0.5, orcaRadius: 0.3,
      orcaTimeHorizon: 3, maxSpeed: 5, minAltitude: 0.3,
      noFlyZones: [], emergencyStopDistance: 0.3,
    };
    const layer = new SafetyLayer();
    const neighbors = new Map<number, NeighborEstimate>();
    neighbors.set(1, {
      senderId: 1, timestamp: 0, position: v3Create(5.15, 5, -2), // 0.15m away < 0.3
      positionVariance: 0.01, velocity: v3Create(0, 0, 0),
      aligned: true, attitudeHealthy: true, inFormation: false,
    });

    const { safeGuidance, state } = layer.adjustGuidance(
      { position: v3Create(5, 5, -2), velocity: v3Create(1, 0, 0),
        quaternion: qCreate(1, 0, 0, 0), gyroBias: v3Create(0, 0, 0),
        accelBias: v3Create(0, 0, 0), baroBias: 0,
        angularVelocity: v3Create(0, 0, 0), timestamp: 0 },
      { positionDes: v3Create(10, 5, -2), velocityDes: v3Create(1, 0, 0),
        accelerationDes: v3Create(0, 0, 0), yawDes: 0 },
      neighbors, new WorldGeometry(openField()), cfg, 0);

    expect(state.reason).toBe('emergency');
    expect(safeGuidance.velocityDes[0]).toBe(0);
    expect(safeGuidance.velocityDes[1]).toBe(0);
  });
});
