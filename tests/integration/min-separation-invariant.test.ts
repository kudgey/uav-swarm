/**
 * KEY ACCEPTANCE TEST: min separation invariant with real closed-loop avoidance.
 * 2 drones armed and flying toward each other via waypoints.
 * ORCA must prevent truth min separation from dropping below minSeparation.
 */
import { describe, it, expect } from 'vitest';
import { SwarmManager } from '@sim/swarm/swarm-manager';
import { EnvironmentManager } from '@sim/environment/environment-manager';
import { defaultDroneParams, defaultSensorSuiteConfig, defaultEKFConfig, defaultCommConfig,
  defaultFormationConfig, defaultSafetyConfig, defaultEnvironmentConfig } from '@sim/core/config-defaults';
import { DeterministicRNG } from '@sim/core/rng';
import { v3Create, v3Sub, v3Len } from '@lib/math';
import { computeSwarmSafetyMetrics } from '@sim/safety/safety-metrics';
import { GRAVITY_MPS2 } from '@sim/core/frames';

describe('Min separation invariant (real closed-loop)', () => {
  it('2 drones with converging waypoints: ORCA maintains min separation', () => {
    const rng = new DeterministicRNG(42);
    for (let i = 0; i < 64; i++) rng.nextU32();
    const envRng = rng.clone();
    const params = defaultDroneParams();
    const env = new EnvironmentManager(defaultEnvironmentConfig(), envRng, params.armLength);

    // 2 drones 8m apart, will command to fly toward each other
    const positions = [v3Create(1, 5, -2), v3Create(9, 5, -2)];
    const safetyCfg = { ...defaultSafetyConfig(), noFlyZones: [], enabled: true, minSeparation: 0.5 };

    const sm = new SwarmManager(positions, params, defaultSensorSuiteConfig(), defaultEKFConfig(),
      defaultCommConfig(), defaultFormationConfig(), safetyCfg,
      env.getWorldGeometry(), rng);

    const dt = 0.001;
    let globalMinSep = Infinity;

    // Phase 1: Run 500ms for EKF to align while hovering at trim
    for (let tick = 0; tick < 500; tick++) {
      const t = tick * dt;
      sm.stepPhysics(env, dt, t);
      sm.stepBiases(dt);
      if (tick % 4 === 0) {
        sm.stepSensorIMU(t);
        sm.stepEKFPropagate(dt * 4);
      }
      if (tick % 20 === 0) { sm.stepSensorMag(t); sm.stepEKFMag(); }
      if (tick % 40 === 0) { sm.stepSensorBaro(t); sm.stepEKFBaro(); }
    }

    // Arm all drones, set hover at initial positions
    for (const d of sm.drones) {
      // Force armed for test (bypass stationarity check)
      (d.controller as unknown as { _armState: string })._armState = 'armed';
      d.controller.setHover(d.ekf.getEstimate().position, 0);
    }

    // Phase 2: Run 500ms to stabilize hover
    for (let tick = 500; tick < 1000; tick++) {
      const t = tick * dt;
      sm.stepPhysics(env, dt, t);
      sm.stepBiases(dt);
      if (tick % 4 === 0) {
        sm.stepSensorIMU(t);
        sm.stepEKFPropagate(dt * 4);
      }
      if (tick % 20 === 0) { sm.stepSensorMag(t); sm.stepEKFMag(); }
      if (tick % 40 === 0) { sm.stepSensorBaro(t); sm.stepEKFBaro(); }
      if (tick % 4 === 0) {
        sm.stepSafetyGuidance(t);
        sm.stepControllers(t);
      }
    }

    // Phase 3: Command drones toward each other
    sm.drones[0].controller.setWaypoint(v3Create(9, 5, -2), 0, 2); // drone 0 → right
    sm.drones[1].controller.setWaypoint(v3Create(1, 5, -2), 0, 2); // drone 1 → left

    // Run 3000 ticks (3 seconds) of converging flight with safety
    for (let tick = 1000; tick < 4000; tick++) {
      const t = tick * dt;
      sm.stepPhysics(env, dt, t);
      sm.stepBiases(dt);
      if (tick % 4 === 0) {
        sm.stepSensorIMU(t);
        sm.stepEKFPropagate(dt * 4);
      }
      if (tick % 20 === 0) { sm.stepSensorMag(t); sm.stepEKFMag(); }
      if (tick % 40 === 0) { sm.stepSensorBaro(t); sm.stepEKFBaro(); }
      if (tick % 4 === 0) {
        // Comm for neighbor awareness
        sm.commSystem.updateNeighborGraph(sm.getPositionsMap());
        sm.stepEstimatePublish(t);
        sm.commSystem.deliver(t);
        sm.processDeliveredMessages(t);

        sm.stepSafetyGuidance(t);
        sm.stepControllers(t);
      }

      // Measure truth min separation at 10Hz
      if (tick % 100 === 0) {
        const metrics = computeSwarmSafetyMetrics(sm.drones, safetyCfg);
        if (metrics.minSeparation < globalMinSep) globalMinSep = metrics.minSeparation;
      }
    }

    // KEY ACCEPTANCE: drones tried to fly through each other, but ORCA prevented it
    expect(globalMinSep).toBeGreaterThan(safetyCfg.minSeparation - 0.1); // small tolerance for discrete integration
    expect(globalMinSep).toBeLessThan(8); // they actually moved closer (not stuck at 8m)
    expect(isFinite(sm.drones[0].state.position[0])).toBe(true);
    expect(isFinite(sm.drones[1].state.position[0])).toBe(true);
  }, 30000);
});
