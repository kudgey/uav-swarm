import { describe, expect, it } from 'vitest';
import {
  buildMissionConfig,
  buildVisualMissionSteps,
  computePatternPositions,
} from '@ui/panels/mission-panel';

describe('Mission panel planners', () => {
  it('computes distinct spawn positions for multi-drone patterns', () => {
    const positions = computePatternPositions(3, 'line', 2, [5, 5, -2]);
    expect(positions).toEqual([[3, 5, -2], [5, 5, -2], [7, 5, -2]]);
  });

  it('formation-transit: uses set-mission-plan with out+back waypoints', () => {
    const steps = buildVisualMissionSteps({
      type: 'formation-transit', droneCount: 3, pattern: 'line', spacing: 2,
      distance: 10, speed: 1.5, wind: 0,
    });
    expect(steps.some(s => s.action === 'formation-enable')).toBe(true);
    const planStep = steps.find(s => s.action === 'set-mission-plan');
    expect(planStep).toBeDefined();
    const wps = planStep?.params?.waypoints as { position: number[] }[];
    expect(wps).toHaveLength(2); // out + back
    expect(wps[0].position[0]).toBe(15); // center[0] + distance
    expect(wps[1].position[0]).toBe(5);  // back to center
  });

  it('formation-reconfig: includes kill-drone step', () => {
    const steps = buildVisualMissionSteps({
      type: 'formation-reconfig', droneCount: 5, pattern: 'circle', spacing: 2,
      distance: 20, speed: 1.5, wind: 0,
    });
    expect(steps.some(s => s.action === 'formation-enable')).toBe(true);
    expect(steps.some(s => s.action === 'set-mission-plan')).toBe(true);
    const kill = steps.find(s => s.action === 'kill-drone');
    expect(kill).toBeDefined();
    expect(kill?.droneId).toBe(2);
  });

  it('leader-loss: kills drone 0 and has mission plan', () => {
    const steps = buildVisualMissionSteps({
      type: 'leader-loss', droneCount: 5, pattern: 'line', spacing: 2,
      distance: 20, speed: 1.5, wind: 0,
    });
    expect(steps.some(s => s.action === 'set-mission-plan' && s.droneId === 0)).toBe(true);
    expect(steps.some(s => s.action === 'kill-drone' && s.droneId === 0)).toBe(true);
  });

  it('multi-loss: kills two drones at different times', () => {
    const steps = buildVisualMissionSteps({
      type: 'multi-loss', droneCount: 6, pattern: 'circle', spacing: 2,
      distance: 20, speed: 1.5, wind: 0,
    });
    const kills = steps.filter(s => s.action === 'kill-drone');
    expect(kills).toHaveLength(2);
    expect(kills[0].droneId).toBe(3);
    expect(kills[1].droneId).toBe(4);
    expect(kills[1].time).toBeGreaterThan(kills[0].time);
  });

  it('long-range-patrol: uses shared builder with return', () => {
    const steps = buildVisualMissionSteps({
      type: 'long-range-patrol', droneCount: 4, pattern: 'line', spacing: 2,
      distance: 60, speed: 1.5, wind: 0,
    });
    const plan = steps.find(s => s.action === 'set-mission-plan');
    expect(plan).toBeDefined();
    const wps = plan?.params?.waypoints as unknown[];
    expect(wps.length).toBeGreaterThanOrEqual(6); // 3+ out + 3+ return
  });

  it('config has GPS-denied aiding for long-range-patrol', () => {
    const cfg = buildMissionConfig(5, 'line', 2, { type: 'long-range-patrol', distance: 100 });
    expect(cfg.sensors.cameraVIO?.enabled).toBe(true);
    expect(cfg.sensors.cameraVIO?.vioDriftEnabled).toBe(true);
    expect(cfg.environment.scene.uwbAnchors).toEqual([]);
  });

  // ── Consensus (leaderless) tests ──

  it('consensus-transit: every drone gets offset-adjusted waypoints + formation-enable', () => {
    const steps = buildVisualMissionSteps({
      type: 'consensus-transit', droneCount: 4, pattern: 'line', spacing: 2,
      distance: 10, speed: 1.5, wind: 0,
    });
    // Formation-enable IS present (needed for consensus sync)
    expect(steps.some(s => s.action === 'formation-enable')).toBe(true);
    // Every drone gets its own mission plan (offset-adjusted)
    const planSteps = steps.filter(s => s.action === 'set-mission-plan');
    expect(planSteps).toHaveLength(4);
    // Each drone's waypoints should be different (different offsets)
    const wps0 = (planSteps[0].params?.waypoints as { position: number[] }[]);
    const wps1 = (planSteps[1].params?.waypoints as { position: number[] }[]);
    expect(wps0[0].position[0]).not.toBeCloseTo(wps1[0].position[0], 1);
  });

  it('consensus-loss: has per-drone plans + kill-drone + formation-enable', () => {
    const steps = buildVisualMissionSteps({
      type: 'consensus-loss', droneCount: 5, pattern: 'circle', spacing: 2,
      distance: 20, speed: 1.5, wind: 0,
    });
    expect(steps.some(s => s.action === 'formation-enable')).toBe(true);
    expect(steps.filter(s => s.action === 'set-mission-plan')).toHaveLength(5);
    expect(steps.some(s => s.action === 'kill-drone' && s.droneId === 2)).toBe(true);
  });

  it('consensus-patrol: config uses consensus mode + GPS-denied', () => {
    const cfg = buildMissionConfig(5, 'line', 2, { type: 'consensus-patrol', distance: 100 });
    expect(cfg.swarm.formation.mode).toBe('consensus');
    expect(cfg.sensors.cameraVIO?.vioDriftEnabled).toBe(true);
    expect(cfg.environment.scene.uwbAnchors).toEqual([]);
  });
});
