import { describe, it, expect } from 'vitest';
import { readUWB } from '@sim/sensors/uwb';
import { createDroneState } from '@sim/physics/state';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { DeterministicRNG } from '@sim/core/rng';
import { v3Create } from '@lib/math';
import type { UWBSensorConfig } from '@sim/core/types';

describe('Inter-drone UWB', () => {
  it('range between two drones matches distance ± noise', () => {
    const state = createDroneState(4);
    state.position[0] = 0; state.position[1] = 0; state.position[2] = -2;

    const otherDrone = { id: 'drone-1', position: v3Create(5, 0, -2) };
    const trueRange = 5.0;

    const cfg: UWBSensorConfig = {
      enabled: true, rate: 10, losNoiseStdDev: 0.05, losBias: 0.05,
      nlosBiasBase: 0.5, nlosBiasStdDev: 0.3, nlosNoiseStdDev: 0.15,
      packetLossProbability: 0, minRange: 0.2, maxRange: 40,
    };

    const worldGeo = new WorldGeometry(openField());
    let totalError = 0;
    const N = 100;

    for (let i = 0; i < N; i++) {
      const results = readUWB(state, [otherDrone], cfg, worldGeo, new DeterministicRNG(i), 0);
      expect(results.length).toBe(1);
      expect(results[0].valid).toBe(true);
      totalError += Math.abs(results[0].measurement.range - trueRange);
    }

    const meanError = totalError / N;
    // Mean error should be around losBias + noise (~0.05 + 0.04 ≈ 0.09)
    expect(meanError).toBeGreaterThan(0.01);
    expect(meanError).toBeLessThan(0.3);
  });
});
