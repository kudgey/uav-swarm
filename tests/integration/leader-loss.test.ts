/**
 * Integration test: leader loss via comm suppression.
 * 2 drones, leader's neighborEstimate expires → follower falls back to hover.
 */
import { describe, it, expect } from 'vitest';
import { FormationManager } from '@sim/swarm/formation';
import { v3Create } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import type { EstimatedState } from '@sim/estimation/estimator-types';
import type { NeighborEstimate } from '@sim/swarm/drone-instance';

describe('Leader loss', () => {
  it('follower detects leader loss when estimate expires', () => {
    const fm = new FormationManager({
      enabled: true, mode: 'leader-follower', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'hover', offsetFrame: 'world',
    });
    fm.updateConfig(fm['config'], 2);

    const est: EstimatedState = {
      position: v3Create(7, 5, -2), velocity: v3Create(0, 0, 0),
      quaternion: qCreate(1, 0, 0, 0), gyroBias: v3Create(0, 0, 0),
      accelBias: v3Create(0, 0, 0), baroBias: 0,
      angularVelocity: v3Create(0, 0, 0), timestamp: 10,
    };

    // Leader estimate is fresh at t=5
    const neighbors = new Map<number, NeighborEstimate>();
    neighbors.set(0, {
      senderId: 0, timestamp: 5, position: v3Create(5, 5, -2),
      positionVariance: 0.1, velocity: v3Create(0, 0, 0),
      aligned: true, attitudeHealthy: true, inFormation: true,
    });

    // At t=6 (1s after estimate): still valid (timeout=2s)
    let state = fm.getFormationTarget(1, est, neighbors, 6, 2);
    expect(state.referenceHealthy).toBe(true);

    // At t=8 (3s after estimate): expired (timeout=2s)
    state = fm.getFormationTarget(1, est, neighbors, 8, 2);
    expect(state.referenceHealthy).toBe(false);
    expect(state.active).toBe(false);
  });

  it('follower detects leader exit via inFormation=false', () => {
    const fm = new FormationManager({
      enabled: true, mode: 'leader-follower', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'hover', offsetFrame: 'world',
    });
    fm.updateConfig(fm['config'], 2);

    const est: EstimatedState = {
      position: v3Create(7, 5, -2), velocity: v3Create(0, 0, 0),
      quaternion: qCreate(1, 0, 0, 0), gyroBias: v3Create(0, 0, 0),
      accelBias: v3Create(0, 0, 0), baroBias: 0,
      angularVelocity: v3Create(0, 0, 0), timestamp: 5,
    };

    const neighbors = new Map<number, NeighborEstimate>();
    neighbors.set(0, {
      senderId: 0, timestamp: 5, position: v3Create(5, 5, -2),
      positionVariance: 0.1, velocity: v3Create(0, 0, 0),
      aligned: true, attitudeHealthy: true, inFormation: false, // leader exited formation
    });

    const state = fm.getFormationTarget(1, est, neighbors, 5.1, 2);
    expect(state.referenceHealthy).toBe(false);
  });
});
