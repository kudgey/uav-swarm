import { describe, it, expect } from 'vitest';
import { readUWB } from '@sim/sensors/uwb';
import { DeterministicRNG } from '@sim/core/rng';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { defaultSimConfig } from '@sim/core/config-defaults';
import { createDroneState } from '@sim/physics/state';

describe('UWB multipath', () => {
  it('adds positive extra delay in NLOS with multipath probability', () => {
    const cfg = defaultSimConfig();
    cfg.sensors.uwb!.enabled = true;
    cfg.sensors.uwb!.multipathProbability = 1.0; // force multipath
    cfg.sensors.uwb!.multipathExtraDelayMean = 2.0;
    cfg.sensors.uwb!.multipathExtraDelayStd = 0.1;
    cfg.sensors.uwb!.nlosBiasBase = 0.5;
    cfg.sensors.uwb!.nlosBiasStdDev = 0.01;
    cfg.sensors.uwb!.losNoiseStdDev = 0.001;
    cfg.sensors.uwb!.nlosNoiseStdDev = 0.001;
    cfg.sensors.uwb!.packetLossProbability = 0;

    // Scene with obstacle blocking LOS
    cfg.environment.scene.obstacles = [{ min: [2, -1, -3], max: [3, 1, 0] }];
    cfg.environment.scene.uwbAnchors = [{ id: 'A0', position: [5, 0, -1] }];

    const worldGeo = new WorldGeometry(cfg.environment.scene);
    const state = createDroneState(4);
    state.position[0] = 0; state.position[1] = 0; state.position[2] = -1;

    const anchors = cfg.environment.scene.uwbAnchors!;
    const trueRange = Math.sqrt(25 + 0 + 0); // = 5

    // Sample many times
    const rng = new DeterministicRNG(42);
    let withMultipathMean = 0;
    const N = 50;
    for (let i = 0; i < N; i++) {
      const results = readUWB(state, anchors, cfg.sensors.uwb!, worldGeo, rng, i);
      withMultipathMean += results[0].measurement.range - trueRange;
    }
    withMultipathMean /= N;

    // With multipath at P=1, extraDelay≈2.0, nlosBiasBase=0.5 → mean error ~2.5m
    expect(withMultipathMean).toBeGreaterThan(1.5);
    expect(withMultipathMean).toBeLessThan(4.0);

    // Now with multipath off, error should be much smaller
    cfg.sensors.uwb!.multipathProbability = 0;
    const rng2 = new DeterministicRNG(42);
    let noMultipathMean = 0;
    for (let i = 0; i < N; i++) {
      const results = readUWB(state, anchors, cfg.sensors.uwb!, worldGeo, rng2, i);
      noMultipathMean += results[0].measurement.range - trueRange;
    }
    noMultipathMean /= N;

    // Without multipath: only NLOS bias ~0.5m
    expect(noMultipathMean).toBeLessThan(withMultipathMean);
    expect(noMultipathMean).toBeGreaterThan(0); // still has positive NLOS bias
  });

  it('NLOS with P=0.5 multipath: distribution is bimodal (two peaks)', () => {
    const cfg = defaultSimConfig();
    cfg.sensors.uwb!.enabled = true;
    cfg.sensors.uwb!.multipathProbability = 0.5;
    cfg.sensors.uwb!.multipathExtraDelayMean = 3.0; // big echo separation
    cfg.sensors.uwb!.multipathExtraDelayStd = 0.2;
    cfg.sensors.uwb!.nlosBiasBase = 0.3;
    cfg.sensors.uwb!.nlosBiasStdDev = 0.1;
    cfg.sensors.uwb!.nlosNoiseStdDev = 0.05;
    cfg.sensors.uwb!.packetLossProbability = 0;
    cfg.environment.scene.obstacles = [{ min: [2, -1, -3], max: [3, 1, 0] }];
    cfg.environment.scene.uwbAnchors = [{ id: 'A0', position: [5, 0, -1] }];

    const worldGeo = new WorldGeometry(cfg.environment.scene);
    const state = createDroneState(4);
    state.position[0] = 0; state.position[1] = 0; state.position[2] = -1;

    const rng = new DeterministicRNG(42);
    const errors: number[] = [];
    const N = 500;
    for (let i = 0; i < N; i++) {
      const results = readUWB(state, cfg.environment.scene.uwbAnchors!, cfg.sensors.uwb!, worldGeo, rng, i);
      errors.push(results[0].measurement.range - 5);
    }
    // Two modes: low (~0.3) and high (~3.3)
    const lowMode = errors.filter(e => e < 1.5).length;
    const highMode = errors.filter(e => e > 2.5).length;
    // Expect both groups substantial (at least 30% each for P=0.5)
    expect(lowMode / N).toBeGreaterThan(0.3);
    expect(highMode / N).toBeGreaterThan(0.3);
  });

  it('LOS measurements unaffected by multipath config', () => {
    const cfg = defaultSimConfig();
    cfg.sensors.uwb!.enabled = true;
    cfg.sensors.uwb!.multipathProbability = 1.0;
    cfg.sensors.uwb!.multipathExtraDelayMean = 10.0; // huge
    cfg.sensors.uwb!.losNoiseStdDev = 0.001;
    cfg.sensors.uwb!.packetLossProbability = 0;
    cfg.environment.scene.obstacles = []; // clear LOS
    cfg.environment.scene.uwbAnchors = [{ id: 'A0', position: [5, 0, -1] }];

    const worldGeo = new WorldGeometry(cfg.environment.scene);
    const state = createDroneState(4);
    state.position[0] = 0; state.position[1] = 0; state.position[2] = -1;

    const rng = new DeterministicRNG(42);
    const results = readUWB(state, cfg.environment.scene.uwbAnchors!, cfg.sensors.uwb!, worldGeo, rng, 0);
    expect(results[0].measurement.isNLOS).toBe(false);
    // Range should be close to true (5) + losBias (0.05) + small noise
    expect(Math.abs(results[0].measurement.range - 5)).toBeLessThan(0.5);
  });
});
