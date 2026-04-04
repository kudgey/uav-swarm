import { describe, it, expect } from 'vitest';
import { FormationManager } from '@sim/swarm/formation';
import { v3Create } from '@lib/math';
import { qCreate, qFromAxisAngle } from '@sim/physics/quaternion';
import type { EstimatedState } from '@sim/estimation/estimator-types';
import type { NeighborEstimate } from '@sim/swarm/drone-instance';

describe('Heading-aligned formation', () => {
  it('world mode: leader rotation does not affect follower target', () => {
    const fm = new FormationManager({
      enabled: true, mode: 'leader-follower', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'hover', offsetFrame: 'world',
    });
    fm.updateConfig(fm['config'], 2);

    const leaderQuat = qCreate();
    qFromAxisAngle(leaderQuat, v3Create(0, 0, 1), Math.PI / 2); // 90° yaw

    const neighbors = new Map<number, NeighborEstimate>();
    neighbors.set(0, {
      senderId: 0, timestamp: 0, position: v3Create(5, 5, -2),
      positionVariance: 0.1, velocity: v3Create(0, 0, 0),
      quaternion: leaderQuat,
      aligned: true, attitudeHealthy: true, inFormation: true,
    });

    const est: EstimatedState = {
      position: v3Create(7, 5, -2), velocity: v3Create(0, 0, 0),
      quaternion: qCreate(1, 0, 0, 0), gyroBias: v3Create(0, 0, 0),
      accelBias: v3Create(0, 0, 0), baroBias: 0,
      angularVelocity: v3Create(0, 0, 0), timestamp: 0,
    };

    const state = fm.getFormationTarget(1, est, neighbors, 0, 2);
    // World mode: target should be along world X axis regardless of leader yaw
    expect(state.targetPosition[0]).not.toBeCloseTo(5, 0); // offset in X
    expect(state.targetPosition[1]).toBeCloseTo(5, 0); // no Y offset
  });

  it('heading mode: leader 90° yaw rotates follower offset', () => {
    const fm = new FormationManager({
      enabled: true, mode: 'leader-follower', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'hover', offsetFrame: 'heading',
    });
    fm.updateConfig(fm['config'], 2);

    const leaderQuat = qCreate();
    qFromAxisAngle(leaderQuat, v3Create(0, 0, 1), Math.PI / 2); // 90° yaw (NED: around down axis)

    const neighbors = new Map<number, NeighborEstimate>();
    neighbors.set(0, {
      senderId: 0, timestamp: 0, position: v3Create(5, 5, -2),
      positionVariance: 0.1, velocity: v3Create(0, 0, 0),
      quaternion: leaderQuat,
      aligned: true, attitudeHealthy: true, inFormation: true,
    });

    const est: EstimatedState = {
      position: v3Create(5, 7, -2), velocity: v3Create(0, 0, 0),
      quaternion: qCreate(1, 0, 0, 0), gyroBias: v3Create(0, 0, 0),
      accelBias: v3Create(0, 0, 0), baroBias: 0,
      angularVelocity: v3Create(0, 0, 0), timestamp: 0,
    };

    const state = fm.getFormationTarget(1, est, neighbors, 0, 2);
    // Heading mode: line topology offset (along X) rotated by 90° → should appear along Y
    // Exact value depends on topology offsets, but Y component should be non-zero
    expect(state.active).toBe(true);
    // The rotated offset should differ from world-mode
    expect(Math.abs(state.targetPosition[1] - 5)).toBeGreaterThan(0.5);
  });
});
