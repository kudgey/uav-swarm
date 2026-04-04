import { describe, it, expect } from 'vitest';
import { createDroneInstance } from '@sim/swarm/drone-instance';
import { defaultDroneParams, defaultSensorSuiteConfig } from '@sim/core/config-defaults';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { DeterministicRNG } from '@sim/core/rng';
import { v3Create } from '@lib/math';

describe('DroneInstance', () => {
  it('creates with correct initial position', () => {
    const pos = v3Create(5, 3, -2);
    const d = createDroneInstance(0, pos, defaultDroneParams(), defaultSensorSuiteConfig(),
      new WorldGeometry(openField()), new DeterministicRNG(42));
    expect(d.state.position[0]).toBe(5);
    expect(d.state.position[1]).toBe(3);
    expect(d.state.position[2]).toBe(-2);
    expect(d.id).toBe(0);
  });

  it('two instances have independent state', () => {
    const rng = new DeterministicRNG(42);
    const wg = new WorldGeometry(openField());
    const params = defaultDroneParams();
    const sc = defaultSensorSuiteConfig();
    for (let i = 0; i < 64; i++) rng.nextU32();
    const d0 = createDroneInstance(0, v3Create(0, 0, -1), params, sc, wg, rng.clone());
    for (let i = 0; i < 64; i++) rng.nextU32();
    const d1 = createDroneInstance(1, v3Create(5, 5, -2), params, sc, wg, rng.clone());

    d0.state.position[0] = 99;
    expect(d1.state.position[0]).toBe(5); // independent
  });

  it('has empty neighborEstimates initially', () => {
    const d = createDroneInstance(0, v3Create(0, 0, 0), defaultDroneParams(),
      defaultSensorSuiteConfig(), new WorldGeometry(openField()), new DeterministicRNG(1));
    expect(d.neighborEstimates.size).toBe(0);
  });
});
