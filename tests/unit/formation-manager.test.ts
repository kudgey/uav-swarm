import { describe, it, expect } from 'vitest';
import { FormationManager } from '@sim/swarm/formation';
import { v3Create } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import type { EstimatedState } from '@sim/estimation/estimator-types';
import type { NeighborEstimate } from '@sim/swarm/drone-instance';

function makeEst(x: number, y: number, z: number): EstimatedState {
  return {
    position: v3Create(x, y, z), velocity: v3Create(0, 0, 0),
    quaternion: qCreate(1, 0, 0, 0), gyroBias: v3Create(0, 0, 0),
    accelBias: v3Create(0, 0, 0), baroBias: 0,
    angularVelocity: v3Create(0, 0, 0), timestamp: 0,
  };
}

function makeNeighbor(id: number, x: number, y: number, z: number, inForm = true): NeighborEstimate {
  return {
    senderId: id, timestamp: 0, position: v3Create(x, y, z),
    positionVariance: 0.1, velocity: v3Create(0, 0, 0),
    aligned: true, attitudeHealthy: true, inFormation: inForm,
  };
}

describe('FormationManager', () => {
  it('leader-follower: leader gets role=leader', () => {
    const fm = new FormationManager({
      enabled: true, mode: 'leader-follower', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'hover', offsetFrame: 'world',
    });
    fm.updateConfig(fm['config'], 2);

    const state = fm.getFormationTarget(0, makeEst(0, 0, -2), new Map(), 0, 2);
    expect(state.role).toBe('leader');
    expect(state.active).toBe(true);
  });

  it('leader-follower: follower computes target from leader estimate', () => {
    const fm = new FormationManager({
      enabled: true, mode: 'leader-follower', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'hover', offsetFrame: 'world',
    });
    fm.updateConfig(fm['config'], 2);

    const neighbors = new Map<number, NeighborEstimate>();
    neighbors.set(0, makeNeighbor(0, 5, 5, -2));

    const state = fm.getFormationTarget(1, makeEst(5, 5, -2), neighbors, 0, 2);
    expect(state.role).toBe('follower');
    expect(state.active).toBe(true);
    expect(state.referenceHealthy).toBe(true);
    // Target should be offset from leader
    expect(Math.abs(state.targetPosition[0] - 5)).toBeGreaterThan(0.5);
  });

  it('leader-follower: leader loss detected when inFormation=false', () => {
    const fm = new FormationManager({
      enabled: true, mode: 'leader-follower', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'hover', offsetFrame: 'world',
    });
    fm.updateConfig(fm['config'], 2);

    const neighbors = new Map<number, NeighborEstimate>();
    neighbors.set(0, makeNeighbor(0, 5, 5, -2, false)); // inFormation=false

    const state = fm.getFormationTarget(1, makeEst(5, 5, -2), neighbors, 0, 2);
    expect(state.referenceHealthy).toBe(false);
    expect(state.active).toBe(false);
  });

  it('consensus: peer computes centroid-based target', () => {
    const fm = new FormationManager({
      enabled: true, mode: 'consensus', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.8, leaderLossFallback: 'hover', offsetFrame: 'world',
    });
    fm.updateConfig(fm['config'], 3);

    const neighbors = new Map<number, NeighborEstimate>();
    neighbors.set(1, makeNeighbor(1, 5, 0, -2));
    neighbors.set(2, makeNeighbor(2, 7, 0, -2));

    const state = fm.getFormationTarget(0, makeEst(3, 0, -2), neighbors, 0, 2);
    expect(state.role).toBe('peer');
    expect(state.active).toBe(true);
    expect(state.referenceHealthy).toBe(true);
  });

  it('consensus: no neighbors → detach', () => {
    const fm = new FormationManager({
      enabled: true, mode: 'consensus', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.8, leaderLossFallback: 'hover', offsetFrame: 'world',
    });
    fm.updateConfig(fm['config'], 2);

    const state = fm.getFormationTarget(0, makeEst(3, 0, -2), new Map(), 0, 2);
    expect(state.referenceHealthy).toBe(false);
    expect(state.active).toBe(false);
  });
});
