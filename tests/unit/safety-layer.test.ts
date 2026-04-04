import { describe, it, expect } from 'vitest';
import { SafetyLayer } from '@sim/safety/safety-layer';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { v3Create } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import type { EstimatedState } from '@sim/estimation/estimator-types';
import type { GuidanceOutput } from '@sim/control/controller-types';
import type { SafetyConfig } from '@sim/safety/safety-types';

const safetyCfg: SafetyConfig = {
  enabled: true, minSeparation: 0.5, orcaRadius: 0.3,
  orcaTimeHorizon: 3, maxSpeed: 2, minAltitude: 0.5,
  noFlyZones: [], emergencyStopDistance: 0.2,
};

function makeEst(x: number, y: number, z: number): EstimatedState {
  return { position: v3Create(x, y, z), velocity: v3Create(0, 0, 0),
    quaternion: qCreate(1, 0, 0, 0), gyroBias: v3Create(0, 0, 0),
    accelBias: v3Create(0, 0, 0), baroBias: 0,
    angularVelocity: v3Create(0, 0, 0), timestamp: 0 };
}

function makeGuidance(vx = 0, vy = 0, vz = 0): GuidanceOutput {
  return { positionDes: v3Create(5, 5, -2), velocityDes: v3Create(vx, vy, vz),
    accelerationDes: v3Create(0, 0, 0), yawDes: 0 };
}

describe('SafetyLayer', () => {
  it('speed cap clamps fast velocity', () => {
    const layer = new SafetyLayer();
    const guidance = makeGuidance(10, 0, 0); // way over maxSpeed=2
    const { safeGuidance, state } = layer.adjustGuidance(
      makeEst(5, 5, -2), guidance, new Map(),
      new WorldGeometry(openField()), safetyCfg, 0);
    const speed = Math.sqrt(safeGuidance.velocityDes[0] ** 2 + safeGuidance.velocityDes[1] ** 2);
    expect(speed).toBeLessThanOrEqual(2.01);
    expect(state.active).toBe(true);
  });

  it('altitude floor prevents descent below minAltitude', () => {
    const layer = new SafetyLayer();
    const guidance = makeGuidance(0, 0, 1); // descending
    const { safeGuidance, events } = layer.adjustGuidance(
      makeEst(5, 5, -0.2), guidance, new Map(), // altitude = 0.2m < 0.5m min
      new WorldGeometry(openField()), safetyCfg, 0);
    expect(safeGuidance.velocityDes[2]).toBeLessThanOrEqual(0); // no further descent
    expect(events.some(e => e.type === 'altitude-clamp')).toBe(true);
  });

  it('emergency stop when neighbor very close', () => {
    const layer = new SafetyLayer();
    const guidance = makeGuidance(1, 0, 0);
    const neighbors = new Map();
    neighbors.set(1, {
      senderId: 1, timestamp: 0, position: v3Create(5.1, 5, -2),
      positionVariance: 0.01, velocity: v3Create(0, 0, 0),
      aligned: true, attitudeHealthy: true, inFormation: false,
    });
    const { safeGuidance, state, events } = layer.adjustGuidance(
      makeEst(5, 5, -2), guidance, neighbors,
      new WorldGeometry(openField()), safetyCfg, 0);
    expect(state.reason).toBe('emergency');
    expect(safeGuidance.velocityDes[0]).toBe(0);
    expect(events.some(e => e.type === 'emergency-stop')).toBe(true);
  });

  it('no neighbors, no constraints: guidance unchanged', () => {
    const layer = new SafetyLayer();
    const guidance = makeGuidance(1, 0, 0);
    const { safeGuidance, state } = layer.adjustGuidance(
      makeEst(5, 5, -2), guidance, new Map(),
      new WorldGeometry(openField()), safetyCfg, 0);
    expect(safeGuidance.velocityDes[0]).toBeCloseTo(1, 1);
    expect(state.active).toBe(false);
  });
});
