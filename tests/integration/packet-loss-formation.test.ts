/**
 * Integration test: consensus formation degrades under packet loss.
 * Formation quality worsens but doesn't diverge.
 */
import { describe, it, expect } from 'vitest';
import { FormationManager, computeTopologyOffsets } from '@sim/swarm/formation';
import { v3Create, v3Sub } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import type { EstimatedState } from '@sim/estimation/estimator-types';
import type { NeighborEstimate } from '@sim/swarm/drone-instance';
import { DeterministicRNG } from '@sim/core/rng';

describe('Packet loss formation', () => {
  it('50% packet loss degrades but doesnt diverge', () => {
    const fm = new FormationManager({
      enabled: true, mode: 'consensus', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.6, leaderLossFallback: 'hover', offsetFrame: 'world',
    });
    fm.updateConfig(fm['config'], 3);

    const rng = new DeterministicRNG(42);
    const positions = [v3Create(3, 0, -2), v3Create(6, 1, -2), v3Create(9, -1, -2)];

    for (let tick = 0; tick < 100; tick++) {
      for (let i = 0; i < 3; i++) {
        const neighbors = new Map<number, NeighborEstimate>();
        for (let j = 0; j < 3; j++) {
          if (i === j) continue;
          // 50% packet loss: sometimes neighbor data missing
          if (rng.next() < 0.5) continue;
          neighbors.set(j, {
            senderId: j, timestamp: tick * 0.1,
            position: v3Create(positions[j][0], positions[j][1], positions[j][2]),
            positionVariance: 0.1, velocity: v3Create(0, 0, 0),
            aligned: true, attitudeHealthy: true, inFormation: true,
          });
        }

        const est: EstimatedState = {
          position: v3Create(positions[i][0], positions[i][1], positions[i][2]),
          velocity: v3Create(0, 0, 0), quaternion: qCreate(1, 0, 0, 0),
          gyroBias: v3Create(0, 0, 0), accelBias: v3Create(0, 0, 0),
          baroBias: 0, angularVelocity: v3Create(0, 0, 0), timestamp: tick * 0.1,
        };

        const state = fm.getFormationTarget(i, est, neighbors, tick * 0.1, 2);
        if (state.active) {
          const err = v3Create();
          v3Sub(err, state.targetPosition, positions[i]);
          positions[i][0] += err[0] * 0.2;
          positions[i][1] += err[1] * 0.2;
          positions[i][2] += err[2] * 0.2;
        }
      }
    }

    // Positions should still be finite and roughly in a line
    for (const p of positions) {
      expect(isFinite(p[0])).toBe(true);
      expect(isFinite(p[1])).toBe(true);
    }
    // Spacing should be in a reasonable range (not perfect due to loss)
    const d01 = Math.abs(positions[1][0] - positions[0][0]);
    expect(d01).toBeGreaterThan(0.5);
    expect(d01).toBeLessThan(5);
  });
});
