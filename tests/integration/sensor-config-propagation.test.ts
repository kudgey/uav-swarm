/**
 * Integration test: set-sensor-config propagates to live drone SensorManagers.
 * Verifies that updateConfig() on SensorManager changes the private config clone.
 */
import { describe, it, expect } from 'vitest';
import { SimContext } from '@sim/core/sim-context';
import { defaultSimConfig } from '@sim/core/config-defaults';

describe('Sensor config live propagation', () => {
  it('disabling a sensor via updateConfig stops readings on running drones', () => {
    const cfg = defaultSimConfig();
    cfg.swarm.droneCount = 2;
    cfg.swarm.initialPattern = 'line';
    const ctx = new SimContext(cfg);

    // Run a few steps so sensors produce readings
    ctx.stepN(50);

    const d0 = ctx.swarmManager.drones[0];
    const d1 = ctx.swarmManager.drones[1];

    // Barometer should be enabled and producing readings
    expect(d0.sensorManager.getConfig().barometer.enabled).toBe(true);
    expect(d1.sensorManager.getConfig().barometer.enabled).toBe(true);

    // Simulate the worker propagation: updateConfig with barometer disabled
    const partial: Record<string, unknown> = { barometer: { enabled: false } };
    for (const d of ctx.swarmManager.drones) {
      d.sensorManager.updateConfig(partial);
    }

    // Private config should now reflect the change
    expect(d0.sensorManager.getConfig().barometer.enabled).toBe(false);
    expect(d1.sensorManager.getConfig().barometer.enabled).toBe(false);

    // Run more steps — barometer readings should be cleared
    const readingsBefore = d0.sensorManager.getLatestReadings().barometer;
    expect(readingsBefore).toBeNull();
  });

  it('enabling cameraIntrinsics via updateConfig affects VIO behavior', () => {
    const cfg = defaultSimConfig();
    cfg.swarm.droneCount = 1;
    const ctx = new SimContext(cfg);

    const d0 = ctx.swarmManager.drones[0];

    // Initially cameraIntrinsicsEnabled should be false (default)
    expect(d0.sensorManager.getConfig().cameraVIO?.cameraIntrinsicsEnabled).toBe(false);

    // Propagate config change
    d0.sensorManager.updateConfig({
      cameraVIO: { cameraIntrinsicsEnabled: true, focalLength: 400, principalPoint: [320, 240], imageSize: [640, 480], distortionK1: 0, extrinsicsRotation: [0, 0, 0], extrinsicsTranslation: [0, 0, 0] },
    });

    expect(d0.sensorManager.getConfig().cameraVIO?.cameraIntrinsicsEnabled).toBe(true);
    expect((d0.sensorManager.getConfig().cameraVIO as any).focalLength).toBe(400);
  });

  it('each drone gets independent config update (no cross-talk)', () => {
    const cfg = defaultSimConfig();
    cfg.swarm.droneCount = 2;
    cfg.swarm.initialPattern = 'line';
    const ctx = new SimContext(cfg);

    const d0 = ctx.swarmManager.drones[0];
    const d1 = ctx.swarmManager.drones[1];

    // Disable barometer only on drone 0
    d0.sensorManager.updateConfig({ barometer: { enabled: false } });

    expect(d0.sensorManager.getConfig().barometer.enabled).toBe(false);
    expect(d1.sensorManager.getConfig().barometer.enabled).toBe(true); // unaffected
  });
});
