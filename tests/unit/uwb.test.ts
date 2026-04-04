import { describe, it, expect } from 'vitest';
import { readUWB } from '@sim/sensors/uwb';
import { createDroneState } from '@sim/physics/state';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField, indoorBasic } from '@sim/environment/scene-presets';
import { DeterministicRNG } from '@sim/core/rng';
import { v3Create } from '@lib/math';
import type { UWBSensorConfig, UWBAnchorDef } from '@sim/core/types';

const defaultUWBCfg: UWBSensorConfig = {
  enabled: true, rate: 10, losNoiseStdDev: 0.05, losBias: 0.05,
  nlosBiasBase: 0.5, nlosBiasStdDev: 0.3, nlosNoiseStdDev: 0.15,
  packetLossProbability: 0, minRange: 0.2, maxRange: 40,
};

describe('UWB Sensor', () => {
  it('LOS: small positive bias', () => {
    const state = createDroneState(4);
    state.position[0] = 5; state.position[1] = 5; state.position[2] = -2;
    const anchors: UWBAnchorDef[] = [{ id: 'A0', position: v3Create(0, 0, -0.5) }];
    const worldGeo = new WorldGeometry(openField());

    let totalBias = 0;
    const N = 200;
    const trueRange = Math.sqrt(25 + 25 + 2.25);
    for (let i = 0; i < N; i++) {
      const results = readUWB(state, anchors, defaultUWBCfg, worldGeo, new DeterministicRNG(i), 0);
      if (results[0].valid) {
        totalBias += results[0].measurement.range - trueRange;
      }
    }
    const meanBias = totalBias / N;
    expect(meanBias).toBeGreaterThan(0); // positive bias
    expect(meanBias).toBeLessThan(0.2);  // small
  });

  it('NLOS: asymmetric positive bias (always > LOS bias)', () => {
    // Use indoor scene with obstacle blocking LOS
    const scene = indoorBasic();
    const worldGeo = new WorldGeometry(scene);
    const state = createDroneState(4);
    state.position[0] = 5; state.position[1] = 5; state.position[2] = -1.5;
    // Place anchor behind a wall (outside the room)
    const anchors: UWBAnchorDef[] = [{ id: 'A', position: v3Create(-2, 5, -1.5) }];

    let nlosCount = 0;
    let totalBias = 0;
    const trueRange = 7;
    for (let i = 0; i < 200; i++) {
      const results = readUWB(state, anchors, defaultUWBCfg, worldGeo, new DeterministicRNG(i), 0);
      if (results[0].valid && results[0].measurement.isNLOS) {
        nlosCount++;
        totalBias += results[0].measurement.range - trueRange;
      }
    }
    if (nlosCount > 0) {
      const meanBias = totalBias / nlosCount;
      expect(meanBias).toBeGreaterThan(0.3); // significantly positive NLOS bias
    }
  });

  it('packet loss produces invalid outputs', () => {
    const state = createDroneState(4);
    state.position[0] = 5; state.position[1] = 5; state.position[2] = -2;
    const anchors: UWBAnchorDef[] = [{ id: 'A0', position: v3Create(0, 0, -0.5) }];
    const worldGeo = new WorldGeometry(openField());
    const cfg = { ...defaultUWBCfg, packetLossProbability: 0.5 };

    let invalid = 0;
    for (let i = 0; i < 200; i++) {
      const results = readUWB(state, anchors, cfg, worldGeo, new DeterministicRNG(i), 0);
      if (!results[0].valid) invalid++;
    }
    expect(invalid).toBeGreaterThan(60);
    expect(invalid).toBeLessThan(140);
  });

  it('range beyond maxRange: invalid', () => {
    const state = createDroneState(4);
    state.position[2] = -2;
    const anchors: UWBAnchorDef[] = [{ id: 'A', position: v3Create(100, 0, -0.5) }];
    const worldGeo = new WorldGeometry(openField());
    const cfg = { ...defaultUWBCfg, maxRange: 40 };
    const results = readUWB(state, anchors, cfg, worldGeo, new DeterministicRNG(42), 0);
    expect(results[0].valid).toBe(false);
  });
});
