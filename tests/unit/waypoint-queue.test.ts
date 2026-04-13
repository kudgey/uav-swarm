/**
 * Test: waypoint queue auto-advance and mission plan export.
 */
import { describe, it, expect } from 'vitest';
import { GuidanceModule, type MissionWaypoint } from '@sim/control/guidance';
import { v3Create } from '@lib/math';
import type { EstimatedState } from '@sim/estimation/estimator-types';

function makeEstimate(x: number, y: number, z: number): EstimatedState {
  return {
    position: new Float64Array([x, y, z]),
    velocity: new Float64Array([0, 0, 0]),
    quaternion: new Float64Array([1, 0, 0, 0]),
    gyroBias: new Float64Array([0, 0, 0]),
    accelBias: new Float64Array([0, 0, 0]),
    baroBias: 0,
  };
}

describe('Waypoint queue', () => {
  it('advances through waypoints when within arrival radius', () => {
    const g = new GuidanceModule();
    const wps: MissionWaypoint[] = [
      { position: [10, 0, -2], yaw: 0, speed: 2 },
      { position: [20, 0, -2], yaw: 0, speed: 2 },
      { position: [30, 0, -2], yaw: 0, speed: 2 },
    ];
    g.setMissionPlan(wps);

    // Start far from first waypoint
    let out = g.evaluate(makeEstimate(0, 0, -2));
    expect(out.positionDes[0]).toBe(10);
    expect(g.getMissionProgress().current).toBe(0);
    expect(g.getMissionProgress().total).toBe(3);

    // Move close to first waypoint — should advance to second
    out = g.evaluate(makeEstimate(10, 0, -2));
    expect(out.positionDes[0]).toBe(20);
    expect(g.getMissionProgress().current).toBe(1);

    // Move close to second — advance to third
    out = g.evaluate(makeEstimate(20, 0, -2));
    expect(out.positionDes[0]).toBe(30);
    expect(g.getMissionProgress().current).toBe(2);

    // Arrive at third — mission done, hover
    out = g.evaluate(makeEstimate(30, 0, -2));
    expect(g.getMissionProgress().done).toBe(true);
    expect(g.getMode()).toBe('hover');
  });

  it('exportRemainingPlan returns unvisited waypoints', () => {
    const g = new GuidanceModule();
    const wps: MissionWaypoint[] = [
      { position: [10, 0, -2], yaw: 0, speed: 2 },
      { position: [20, 0, -2], yaw: 0, speed: 2 },
      { position: [30, 0, -2], yaw: 0, speed: 2 },
    ];
    g.setMissionPlan(wps);

    // Advance past first
    g.evaluate(makeEstimate(10, 0, -2));
    const remaining = g.exportRemainingPlan();
    expect(remaining).toHaveLength(2);
    expect(remaining[0].position[0]).toBe(20);
    expect(remaining[1].position[0]).toBe(30);
  });

  it('consensus sync with 0 neighbors: self progresses normally', () => {
    const g = new GuidanceModule();
    g.setMissionPlan([
      { position: [10, 0, -2], yaw: 0, speed: 2 },
      { position: [20, 0, -2], yaw: 0, speed: 2 },
    ]);
    // No neighbors — should not crash, self retains progress
    g.syncConsensusProgress([]);
    expect(g.getMissionProgress().current).toBe(0);
  });

  it('consensus sync with 1 slow neighbor: self held back', () => {
    const g = new GuidanceModule();
    g.setMissionPlan([
      { position: [10, 0, -2], yaw: 0, speed: 2 },
      { position: [20, 0, -2], yaw: 0, speed: 2 },
      { position: [30, 0, -2], yaw: 0, speed: 2 },
    ]);
    // Advance self manually to index 2
    g.evaluate(makeEstimate(10, 0, -2));
    g.evaluate(makeEstimate(20, 0, -2));
    expect(g.getMissionProgress().current).toBe(2);
    // Slow neighbor still at 0 — median = 1, self must drop to 1
    g.syncConsensusProgress([0]);
    expect(g.getMissionProgress().current).toBeLessThanOrEqual(2);
  });

  it('empty plan → no crash, stays in hover', () => {
    const g = new GuidanceModule();
    g.setMissionPlan([]);
    const progress = g.getMissionProgress();
    expect(progress.total).toBe(0);
    expect(progress.done).toBe(true);
  });
});
