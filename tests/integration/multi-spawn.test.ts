import { describe, it, expect } from 'vitest';
import { SwarmManager } from '@sim/swarm/swarm-manager';
import { defaultDroneParams, defaultSensorSuiteConfig, defaultCommConfig, defaultFormationConfig, defaultSafetyConfig, defaultEKFConfig } from '@sim/core/config-defaults';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { DeterministicRNG } from '@sim/core/rng';
import { v3Create } from '@lib/math';

describe('Multi-drone spawn', () => {
  it('4 drones at different positions', () => {
    const positions = [
      v3Create(0, 0, -2), v3Create(5, 0, -2),
      v3Create(0, 5, -2), v3Create(5, 5, -2),
    ];
    const rng = new DeterministicRNG(42);
    for (let i = 0; i < 64; i++) rng.nextU32(); // env fork
    rng.clone();
    const sm = new SwarmManager(positions, defaultDroneParams(), defaultSensorSuiteConfig(), defaultEKFConfig(),
      defaultCommConfig(), defaultFormationConfig(), { ...defaultSafetyConfig(), noFlyZones: [] }, new WorldGeometry(openField()), rng);

    expect(sm.count).toBe(4);
    expect(sm.drones[0].state.position[0]).toBe(0);
    expect(sm.drones[1].state.position[0]).toBe(5);
    expect(sm.drones[2].state.position[1]).toBe(5);
    expect(sm.drones[3].id).toBe(3);
  });
});
