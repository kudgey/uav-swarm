/**
 * Integration test: 2-drone leader-follower formation.
 * Leader hovers, follower tracks 2m offset via communicated estimate.
 */
import { describe, it, expect } from 'vitest';
import { FormationManager, computeTopologyOffsets } from '@sim/swarm/formation';
import { v3Create, v3Len, v3Sub } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import type { EstimatedState } from '@sim/estimation/estimator-types';
import type { NeighborEstimate } from '@sim/swarm/drone-instance';

describe('Leader-follower formation', () => {
  it('follower converges to desired offset over 50 ticks', () => {
    const fm = new FormationManager({
      enabled: true, mode: 'leader-follower', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'hover', offsetFrame: 'world',
    });
    fm.updateConfig(fm['config'], 2);

    const offsets = computeTopologyOffsets(2, 2, 'line');
    const leaderPos = v3Create(5, 5, -2);
    const followerPos = v3Create(5, 5, -2); // starts at same position

    for (let tick = 0; tick < 50; tick++) {
      const neighbors = new Map<number, NeighborEstimate>();
      neighbors.set(0, {
        senderId: 0, timestamp: tick * 0.1,
        position: v3Create(leaderPos[0], leaderPos[1], leaderPos[2]),
        positionVariance: 0.1, velocity: v3Create(0, 0, 0),
        aligned: true, attitudeHealthy: true, inFormation: true,
      });

      const est: EstimatedState = {
        position: v3Create(followerPos[0], followerPos[1], followerPos[2]),
        velocity: v3Create(0, 0, 0), quaternion: qCreate(1, 0, 0, 0),
        gyroBias: v3Create(0, 0, 0), accelBias: v3Create(0, 0, 0),
        baroBias: 0, angularVelocity: v3Create(0, 0, 0), timestamp: tick * 0.1,
      };

      const state = fm.getFormationTarget(1, est, neighbors, tick * 0.1, 2);
      expect(state.active).toBe(true);

      // Simulate simple proportional movement toward target
      const err = v3Create();
      v3Sub(err, state.targetPosition, followerPos);
      followerPos[0] += err[0] * 0.3;
      followerPos[1] += err[1] * 0.3;
      followerPos[2] += err[2] * 0.3;
    }

    // Check that follower is near its desired offset from leader
    const desiredX = leaderPos[0] + offsets.offsets[1][0] - offsets.offsets[0][0];
    const errorX = Math.abs(followerPos[0] - desiredX);
    expect(errorX).toBeLessThan(0.1);
  });
});
