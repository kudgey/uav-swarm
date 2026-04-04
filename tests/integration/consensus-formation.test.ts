/**
 * Integration test: 3-drone consensus formation.
 * All peers converge to correct line geometry.
 */
import { describe, it, expect } from 'vitest';
import { FormationManager, computeTopologyOffsets } from '@sim/swarm/formation';
import { v3Create, v3Sub } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import type { EstimatedState } from '@sim/estimation/estimator-types';
import type { NeighborEstimate } from '@sim/swarm/drone-instance';

describe('Consensus formation', () => {
  it('3 peers converge to line formation within 100 ticks', () => {
    const fm = new FormationManager({
      enabled: true, mode: 'consensus', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.6, leaderLossFallback: 'hover', offsetFrame: 'world',
    });
    fm.updateConfig(fm['config'], 3);

    const offsets = computeTopologyOffsets(3, 2, 'line');
    // Start all at different random-ish positions
    const positions = [v3Create(3, 0, -2), v3Create(6, 1, -2), v3Create(9, -1, -2)];

    for (let tick = 0; tick < 100; tick++) {
      for (let i = 0; i < 3; i++) {
        const neighbors = new Map<number, NeighborEstimate>();
        for (let j = 0; j < 3; j++) {
          if (i === j) continue;
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
          positions[i][0] += err[0] * 0.3;
          positions[i][1] += err[1] * 0.3;
          positions[i][2] += err[2] * 0.3;
        }
      }
    }

    // Check relative spacing: drone 0 to 1 and 1 to 2 should be ~2m apart in X
    const d01 = Math.abs(positions[1][0] - positions[0][0]);
    const d12 = Math.abs(positions[2][0] - positions[1][0]);
    expect(d01).toBeCloseTo(2, 0);
    expect(d12).toBeCloseTo(2, 0);
    // Y should have converged near 0
    for (const p of positions) {
      expect(Math.abs(p[1])).toBeLessThan(0.5);
    }
  });
});
