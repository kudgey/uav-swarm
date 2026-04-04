import { describe, it, expect } from 'vitest';
import { SwarmManager } from '@sim/swarm/swarm-manager';
import { defaultDroneParams, defaultSensorSuiteConfig, defaultEKFConfig, defaultCommConfig, defaultFormationConfig, defaultSafetyConfig } from '@sim/core/config-defaults';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { DeterministicRNG } from '@sim/core/rng';
import { v3Create } from '@lib/math';

describe('Isolated state', () => {
  it('modifying drone 0 does not affect drone 1', () => {
    const positions = [v3Create(0, 0, -2), v3Create(5, 5, -2)];
    const rng = new DeterministicRNG(42);
    for (let i = 0; i < 64; i++) rng.nextU32();
    rng.clone();
    const sm = new SwarmManager(positions, defaultDroneParams(), defaultSensorSuiteConfig(), defaultEKFConfig(),
      defaultCommConfig(), defaultFormationConfig(), { ...defaultSafetyConfig(), noFlyZones: [] }, new WorldGeometry(openField()), rng);

    const before1 = sm.drones[1].state.position[0];
    sm.drones[0].state.position[0] = 99;
    expect(sm.drones[1].state.position[0]).toBe(before1);

    sm.drones[0].state.velocity[2] = -5;
    expect(sm.drones[1].state.velocity[2]).toBe(0);
  });
});
