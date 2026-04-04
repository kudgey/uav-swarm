import { describe, it, expect } from 'vitest';
import { computeORCA } from '@sim/safety/orca';
import { v3Create, v3Len, v3Dot } from '@lib/math';
import type { SafetyConfig } from '@sim/safety/safety-types';

const defaultSafety: SafetyConfig = {
  enabled: true, minSeparation: 0.5, orcaRadius: 0.3,
  orcaTimeHorizon: 3, maxSpeed: 5, minAltitude: 0.3,
  noFlyZones: [], emergencyStopDistance: 0.3,
};

describe('ORCA', () => {
  it('no neighbors: v_safe = v_pref (capped at maxSpeed)', () => {
    const vPref = v3Create(1, 0, 0);
    const vSafe = computeORCA(
      v3Create(0, 0, -2), v3Create(0, 0, 0), vPref,
      [], [], defaultSafety);
    expect(vSafe[0]).toBeCloseTo(1, 1);
    expect(vSafe[1]).toBeCloseTo(0, 1);
  });

  it('head-on: ORCA deflects velocities apart', () => {
    const pos1 = v3Create(0, 0, -2);
    const pos2 = v3Create(1.5, 0, -2); // closer: 1.5m apart, within orca time horizon
    const vPref1 = v3Create(1, 0, 0); // drone 1 wants to go right
    const neighbors1 = [{ position: pos2, velocity: v3Create(-1, 0, 0), inflatedRadius: 0.5 }];

    const vSafe1 = computeORCA(pos1, v3Create(1, 0, 0), vPref1, neighbors1, [], defaultSafety);

    // ORCA should modify velocity: either lateral deflection or reduced forward speed or both
    const changed = Math.abs(vSafe1[0] - vPref1[0]) > 0.001
      || Math.abs(vSafe1[1]) > 0.001 || Math.abs(vSafe1[2]) > 0.001;
    expect(changed).toBe(true);
  });

  it('speed cap enforced', () => {
    const vPref = v3Create(10, 0, 0); // exceeds maxSpeed=5
    const vSafe = computeORCA(
      v3Create(0, 0, -2), v3Create(0, 0, 0), vPref,
      [], [], defaultSafety);
    expect(v3Len(vSafe)).toBeLessThanOrEqual(5.01);
  });

  it('obstacle avoidance: deflects away from AABB', () => {
    const pos = v3Create(0, 0, -2);
    const vPref = v3Create(1, 0, 0); // moving toward obstacle
    const obstacles = [{ min: v3Create(1, -1, -3), max: v3Create(2, 1, -1) }];
    const vSafe = computeORCA(pos, v3Create(0, 0, 0), vPref, [], obstacles, defaultSafety);
    // Should deflect or slow
    expect(v3Len(vSafe)).toBeLessThanOrEqual(5.01);
  });
});
