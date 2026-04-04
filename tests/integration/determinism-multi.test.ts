import { describe, it, expect } from 'vitest';
import { SwarmManager } from '@sim/swarm/swarm-manager';
import { EnvironmentManager } from '@sim/environment/environment-manager';
import { defaultDroneParams, defaultSensorSuiteConfig, defaultEKFConfig, defaultCommConfig, defaultFormationConfig, defaultSafetyConfig, defaultEnvironmentConfig } from '@sim/core/config-defaults';
import { DeterministicRNG } from '@sim/core/rng';
import { v3Create } from '@lib/math';

describe('Multi-drone determinism', () => {
  it('same seed + same N → identical states after 500 physics steps', () => {
    function run(seed: number) {
      const rng = new DeterministicRNG(seed);
      for (let i = 0; i < 64; i++) rng.nextU32();
      const envRng = rng.clone();
      const envCfg = defaultEnvironmentConfig();
      const params = defaultDroneParams();
      const env = new EnvironmentManager(envCfg, envRng, params.armLength);
      const positions = [v3Create(0, 0, -2), v3Create(5, 0, -2), v3Create(0, 5, -2)];
      const sm = new SwarmManager(positions, params, defaultSensorSuiteConfig(), defaultEKFConfig(),
        defaultCommConfig(), defaultFormationConfig(), { ...defaultSafetyConfig(), noFlyZones: [] }, env.getWorldGeometry(), rng);

      for (let t = 0; t < 500; t++) {
        sm.stepPhysics(env, 0.001, t * 0.001);
        sm.stepBiases(0.001);
      }
      return sm.drones.map(d => [...d.state.position]);
    }

    const r1 = run(42);
    const r2 = run(42);
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        expect(r1[i][j]).toBe(r2[i][j]);
      }
    }
  });
});
