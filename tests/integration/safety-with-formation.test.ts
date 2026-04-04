/**
 * Safety + formation: formation degrades but no collision.
 */
import { describe, it, expect } from 'vitest';
import { SafetyLayer } from '@sim/safety/safety-layer';
import { FormationManager } from '@sim/swarm/formation';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { v3Create, v3Sub, v3Len } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import type { SafetyConfig } from '@sim/safety/safety-types';
import type { NeighborEstimate } from '@sim/swarm/drone-instance';

describe('Safety with formation', () => {
  it('tight formation with safety: no collision', () => {
    const safetyCfg: SafetyConfig = {
      enabled: true, minSeparation: 0.5, orcaRadius: 0.3,
      orcaTimeHorizon: 3, maxSpeed: 2, minAltitude: 0.3,
      noFlyZones: [], emergencyStopDistance: 0.2,
    };
    const fm = new FormationManager({
      enabled: true, mode: 'consensus', topology: 'line', spacing: 0.8, // tight!
      leaderDroneId: 0, consensusGain: 0.6, leaderLossFallback: 'hover', offsetFrame: 'world',
    });
    fm.updateConfig(fm['config'], 3);

    const layer = new SafetyLayer();
    const worldGeo = new WorldGeometry(openField());
    const pos = [v3Create(4, 5, -2), v3Create(5, 5, -2), v3Create(6, 5, -2)];
    const vel = [v3Create(0, 0, 0), v3Create(0, 0, 0), v3Create(0, 0, 0)];
    let minSep = Infinity;

    for (let tick = 0; tick < 80; tick++) {
      const t = tick * 0.05;
      for (let i = 0; i < 3; i++) {
        const neighbors = new Map<number, NeighborEstimate>();
        for (let j = 0; j < 3; j++) {
          if (i === j) continue;
          neighbors.set(j, {
            senderId: j, timestamp: t,
            position: v3Create(pos[j][0], pos[j][1], pos[j][2]),
            positionVariance: 0.01, velocity: v3Create(vel[j][0], vel[j][1], vel[j][2]),
            aligned: true, attitudeHealthy: true, inFormation: true,
          });
        }

        const est = { position: pos[i], velocity: vel[i], quaternion: qCreate(1, 0, 0, 0),
          gyroBias: v3Create(0, 0, 0), accelBias: v3Create(0, 0, 0), baroBias: 0,
          angularVelocity: v3Create(0, 0, 0), timestamp: t };

        // Formation target
        const fmState = fm.getFormationTarget(i, est, neighbors, t, 2);
        const guidance = {
          positionDes: fmState.targetPosition,
          velocityDes: fmState.targetVelocity,
          accelerationDes: v3Create(0, 0, 0), yawDes: 0,
        };

        // Safety filter
        const { safeGuidance } = layer.adjustGuidance(est, guidance, neighbors, worldGeo, safetyCfg, t);
        const err = v3Create(
          safeGuidance.positionDes[0] - pos[i][0],
          safeGuidance.positionDes[1] - pos[i][1], 0);
        vel[i][0] = safeGuidance.velocityDes[0] + err[0] * 0.3;
        vel[i][1] = safeGuidance.velocityDes[1] + err[1] * 0.3;
      }
      for (let i = 0; i < 3; i++) {
        pos[i][0] += vel[i][0] * 0.05;
        pos[i][1] += vel[i][1] * 0.05;
      }
      for (let i = 0; i < 3; i++) for (let j = i + 1; j < 3; j++) {
        const diff = v3Create(); v3Sub(diff, pos[i], pos[j]);
        const sep = v3Len(diff);
        if (sep < minSep) minSep = sep;
      }
    }

    // Safety should prevent collisions even with tight formation spacing
    expect(minSep).toBeGreaterThan(safetyCfg.minSeparation - 0.1);
    expect(isFinite(pos[0][0])).toBe(true);
  });
});
