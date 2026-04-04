import { describe, it, expect } from 'vitest';
import { SwarmManager } from '@sim/swarm/swarm-manager';
import { EnvironmentManager } from '@sim/environment/environment-manager';
import { defaultDroneParams, defaultSensorSuiteConfig, defaultEKFConfig, defaultCommConfig, defaultFormationConfig, defaultSafetyConfig, defaultEnvironmentConfig } from '@sim/core/config-defaults';
import { DeterministicRNG } from '@sim/core/rng';
import { v3Create } from '@lib/math';

describe('Performance multi-drone', () => {
  it('10 drones, 1000 physics steps complete in < 5s wall time', () => {
    const rng = new DeterministicRNG(42);
    for (let i = 0; i < 64; i++) rng.nextU32();
    const envRng = rng.clone();
    const params = defaultDroneParams();
    const env = new EnvironmentManager(defaultEnvironmentConfig(), envRng, params.armLength);

    const positions = Array.from({ length: 10 }, (_, i) =>
      v3Create(i * 2, 0, -2));
    const sm = new SwarmManager(positions, params, defaultSensorSuiteConfig(), defaultEKFConfig(),
      defaultCommConfig(), defaultFormationConfig(), { ...defaultSafetyConfig(), noFlyZones: [] }, env.getWorldGeometry(), rng);

    const start = performance.now();
    for (let t = 0; t < 1000; t++) {
      sm.stepPhysics(env, 0.001, t * 0.001);
      sm.stepBiases(0.001);
      if (t % 4 === 0) sm.stepSensorIMU(t * 0.001);
      if (t % 4 === 0) sm.stepEKFPropagate(0.004);
    }
    const elapsed = performance.now() - start;

    expect(elapsed).toBeLessThan(5000);
    // Verify all drones still have finite state
    for (const d of sm.drones) {
      expect(isFinite(d.state.position[0])).toBe(true);
      expect(isFinite(d.state.position[2])).toBe(true);
    }
  }, 10000);
});
