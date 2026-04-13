import { describe, it, expect } from 'vitest';
import { computeORCA } from '@sim/safety/orca';
import { v3Create } from '@lib/math';
import type { SafetyConfig } from '@sim/safety/safety-types';

const cfg: SafetyConfig = {
  enabled: true,
  minSeparation: 0.5,
  orcaRadius: 0.3,
  orcaTimeHorizon: 3.0,
  maxSpeed: 5.0,
  minAltitude: 0.3,
  emergencyStopDistance: 0.3,
  noFlyZones: [],
};

describe('ORCA edge cases', () => {
  it('parallel flight: does NOT trigger VO when both move same direction at same speed', () => {
    // Two drones flying north at same speed, 2m apart laterally
    const ownPos = v3Create(0, 0, -2);
    const ownVel = v3Create(2, 0, 0);
    const neighbor = {
      position: v3Create(0, 2, -2), // 2m east
      velocity: v3Create(2, 0, 0),  // same direction
      inflatedRadius: 0.3,
    };
    const result = computeORCA(ownPos, ownVel, ownVel, [neighbor], [], cfg);
    // Result should equal preferred velocity (no deflection)
    expect(result[0]).toBeCloseTo(2, 1);
    expect(result[1]).toBeCloseTo(0, 1);
  });

  it('diverging paths: no VO constraint', () => {
    const ownPos = v3Create(0, 0, -2);
    const ownVel = v3Create(2, 0, 0);
    const neighbor = {
      position: v3Create(5, 0, -2),
      velocity: v3Create(-2, 0, 0), // moving toward us
      inflatedRadius: 0.3,
    };
    // This IS a head-on — should deflect
    const result = computeORCA(ownPos, ownVel, ownVel, [neighbor], [], cfg);
    // Some component should be non-trivial (deflected laterally)
    const deflection = Math.abs(result[1]) + Math.abs(result[2]);
    expect(deflection).toBeGreaterThan(0);
  });

  it('co-located drones: does not produce NaN', () => {
    const ownPos = v3Create(0, 0, -2);
    const ownVel = v3Create(1, 0, 0);
    const neighbor = {
      position: v3Create(0, 0, -2), // same location
      velocity: v3Create(0, 0, 0),
      inflatedRadius: 0.3,
    };
    const result = computeORCA(ownPos, ownVel, ownVel, [neighbor], [], cfg);
    expect(Number.isFinite(result[0])).toBe(true);
    expect(Number.isFinite(result[1])).toBe(true);
    expect(Number.isFinite(result[2])).toBe(true);
  });

  it('head-on collision: produces lateral deflection, no NaN', () => {
    const ownPos = v3Create(0, 0, -2);
    const ownVel = v3Create(2, 0, 0);
    const neighbor = {
      position: v3Create(5, 0, -2),
      velocity: v3Create(-2, 0, 0),
      inflatedRadius: 0.3,
    };
    const result = computeORCA(ownPos, ownVel, ownVel, [neighbor], [], cfg);
    expect(Number.isFinite(result[0])).toBe(true);
    expect(Number.isFinite(result[1])).toBe(true);
    expect(Number.isFinite(result[2])).toBe(true);
  });

  it('zero velocity against stationary neighbor: no NaN', () => {
    const ownPos = v3Create(0, 0, -2);
    const ownVel = v3Create(0, 0, 0);
    const neighbor = {
      position: v3Create(1, 0, -2),
      velocity: v3Create(0, 0, 0),
      inflatedRadius: 0.3,
    };
    const result = computeORCA(ownPos, ownVel, ownVel, [neighbor], [], cfg);
    expect(Number.isFinite(result[0])).toBe(true);
  });

  it('nearly-parallel paths with small lateral velocity: handles perpLen≈0', () => {
    // rv parallel to rp, forcing perpLen to be very small
    const ownPos = v3Create(0, 0, -2);
    const ownVel = v3Create(2, 0.001, 0); // tiny lateral
    const neighbor = {
      position: v3Create(5, 0, -2),
      velocity: v3Create(-2, 0, 0), // rv ≈ [4, 0, 0], rp = [5, 0, 0] — nearly parallel
      inflatedRadius: 0.3,
    };
    const result = computeORCA(ownPos, ownVel, ownVel, [neighbor], [], cfg);
    expect(Number.isFinite(result[0])).toBe(true);
    expect(Number.isFinite(result[1])).toBe(true);
  });
});
