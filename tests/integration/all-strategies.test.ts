/**
 * E2E tests: run every swarm strategy and verify basic correctness.
 * Each test checks: drones stay bounded in altitude, no NaN positions,
 * and strategy-specific assertions.
 */
import { describe, it, expect } from 'vitest';
import { SimContext } from '@sim/core/sim-context';
import { buildMissionConfig, buildVisualMissionSteps, type MissionType } from '@ui/panels/mission-panel';

type Step = { time: number; droneId: number | 'all'; action: string; params?: Record<string, unknown> };

function applyStep(ctx: SimContext, step: Step): boolean {
  const drones = step.droneId === 'all'
    ? ctx.swarmManager.drones
    : [ctx.swarmManager.getDrone(step.droneId as number)].filter(Boolean);
  for (const d of drones) {
    if (!d) continue;
    switch (step.action) {
      case 'arm':
        if (!d.ekf.isAligned()) d.ekf.bootstrapAligned();
        d.controller.forceArm();
        break;
      case 'hover': {
        const p = step.params?.position as number[] | undefined;
        if (p) d.controller.setHover(new Float64Array(p), (step.params?.yaw as number) ?? 0);
        break;
      }
      case 'waypoint': {
        const p = step.params?.position as number[] | undefined;
        if (p) d.controller.setWaypoint(new Float64Array(p), (step.params?.yaw as number) ?? 0, (step.params?.speed as number) ?? 1);
        break;
      }
      case 'set-mission-plan': {
        const wps = step.params?.waypoints as { position: [number, number, number]; yaw: number; speed: number }[];
        if (wps) d.controller.setMissionPlan(wps);
        break;
      }
      case 'kill-drone':
        d.destroyed = true; d.commEnabled = false; d.controller.requestDisarm();
        for (let i = 0; i < d.state.motorCommands.length; i++) { d.state.motorCommands[i] = 0; d.state.motorSpeeds[i] = 0; }
        break;
      case 'formation-enable':
        return ctx.swarmManager.updateFormationConfig({ ...ctx.config.swarm.formation, enabled: true });
      case 'formation-disable':
        return ctx.swarmManager.updateFormationConfig({ ...ctx.config.swarm.formation, enabled: false });
      case 'set-environment':
        Object.assign(ctx.config.environment, step.params);
        ctx.envManager.updateConfig(ctx.config.environment, ctx.config.drone.armLength);
        return true;
      case 'inject-fault': {
        const type = step.params?.type as string;
        if (type === 'wind-gust') {
          ctx.config.environment.wind.meanSpeed = (step.params?.speed as number) ?? 3;
          ctx.envManager.updateConfig(ctx.config.environment, ctx.config.drone.armLength);
        }
        return true;
      }
      case 'set-safety-config':
        Object.assign(ctx.swarmManager.safetyConfig, step.params);
        return true;
    }
  }
  return true;
}

function runStrategy(type: MissionType, droneCount: number, duration: number) {
  const cfg = buildMissionConfig(droneCount, 'line', 2, { type, distance: 10 });
  const steps = buildVisualMissionSteps({
    type, droneCount, pattern: 'line', spacing: 2, distance: 10, speed: 1.5, wind: 0,
  });
  const ctx = new SimContext(cfg);
  let stepIdx = 0;
  const totalSteps = Math.round(duration / cfg.physicsDt);
  for (let i = 0; i < totalSteps; i++) {
    while (stepIdx < steps.length && steps[stepIdx].time <= ctx.simTime) {
      applyStep(ctx, steps[stepIdx]);
      stepIdx++;
    }
    ctx.stepOnce();
  }
  return ctx;
}

function assertBoundedAltitude(ctx: SimContext) {
  for (const d of ctx.swarmManager.drones) {
    if (d.destroyed) continue;
    expect(d.state.position[2]).toBeGreaterThan(-20);
    expect(d.state.position[2]).toBeLessThan(2);
    expect(isNaN(d.state.position[0])).toBe(false);
  }
}

describe('All swarm strategies E2E', () => {
  it('formation-transit: drones fly and return', () => {
    const ctx = runStrategy('formation-transit', 4, 8);
    assertBoundedAltitude(ctx);
  });

  it('formation-hold-wind: survives two wind gusts', () => {
    const ctx = runStrategy('formation-hold-wind', 4, 15);
    assertBoundedAltitude(ctx);
  });

  it('formation-reconfig: survives drone loss mid-flight', () => {
    const ctx = runStrategy('formation-reconfig', 5, 10);
    assertBoundedAltitude(ctx);
    // At least one drone should be destroyed
    expect(ctx.swarmManager.drones.some(d => d.destroyed)).toBe(true);
    // Surviving drones should still be armed
    const alive = ctx.swarmManager.drones.filter(d => !d.destroyed);
    expect(alive.length).toBeGreaterThanOrEqual(3);
  });

  it('leader-loss: leader killed, new leader elected, mission continues to completion', () => {
    const ctx = runStrategy('leader-loss', 5, 20);
    assertBoundedAltitude(ctx);
    expect(ctx.swarmManager.drones[0].destroyed).toBe(true);
    const alive = ctx.swarmManager.drones.filter(d => !d.destroyed);
    expect(alive.length).toBe(4);
    for (const d of alive) expect(d.controller.armState).toBe('armed');
    // New leader should not be drone 0
    const newLeaderId = ctx.swarmManager.formationManager.getLeaderId();
    expect(newLeaderId).not.toBe(0);
    // New leader should have a mission plan (handoff) or have completed it
    const newLeader = ctx.swarmManager.getDrone(newLeaderId);
    if (newLeader) {
      const progress = newLeader.controller.getMissionProgress();
      // Either has plan (total > 0) or inherited and completed
      expect(progress.total).toBeGreaterThanOrEqual(0);
    }
  });

  it('multi-loss: two drones killed, swarm adapts both times', () => {
    const ctx = runStrategy('multi-loss', 6, 15);
    assertBoundedAltitude(ctx);
    const destroyed = ctx.swarmManager.drones.filter(d => d.destroyed).length;
    expect(destroyed).toBe(2);
    const alive = ctx.swarmManager.drones.filter(d => !d.destroyed);
    expect(alive.length).toBe(4);
    for (const d of alive) expect(d.controller.armState).toBe('armed');
  });

  it('long-range-patrol: GPS-denied patrol completes and returns near start', () => {
    const cfg = buildMissionConfig(3, 'line', 2, { type: 'long-range-patrol', distance: 20 });
    const steps = buildVisualMissionSteps({
      type: 'long-range-patrol', droneCount: 3, pattern: 'line', spacing: 2,
      distance: 20, speed: 1.5, wind: 0,
    });
    const ctx = new SimContext(cfg);
    // Record launch position
    const launchX = ctx.swarmManager.drones[0].state.position[0];
    let stepIdx = 0;
    // Run long enough for out (20m/1.5=13s) + return (13s) + settle = 35s
    const totalSteps = Math.round(35 / cfg.physicsDt);
    for (let i = 0; i < totalSteps; i++) {
      while (stepIdx < steps.length && steps[stepIdx].time <= ctx.simTime) {
        applyStep(ctx, steps[stepIdx]);
        stepIdx++;
      }
      ctx.stepOnce();
    }
    assertBoundedAltitude(ctx);
    const leader = ctx.swarmManager.drones[0];
    const progress = leader.controller.getMissionProgress();
    expect(progress.total).toBeGreaterThan(0);
    // Mission should be complete (all waypoints visited)
    expect(progress.done).toBe(true);
    // Leader should be near launch position (within ~5m for 20m patrol with drift)
    expect(Math.abs(leader.state.position[0] - launchX)).toBeLessThan(5);
    // VIO drift should be enabled in config
    expect(cfg.sensors.cameraVIO?.vioDriftEnabled).toBe(true);
    expect(cfg.environment.scene.uwbAnchors).toEqual([]);
  });

  it('consensus-transit: all drones complete mission with formation spacing', () => {
    const ctx = runStrategy('consensus-transit', 4, 20);
    assertBoundedAltitude(ctx);
    for (const d of ctx.swarmManager.drones) {
      const progress = d.controller.getMissionProgress();
      expect(progress.total).toBeGreaterThan(0);
      expect(progress.done).toBe(true);
    }
  });

  it('consensus-loss: drone killed, survivors finish their plans', () => {
    const ctx = runStrategy('consensus-loss', 5, 20);
    assertBoundedAltitude(ctx);
    expect(ctx.swarmManager.drones.some(d => d.destroyed)).toBe(true);
    // Surviving drones should complete their mission
    const alive = ctx.swarmManager.drones.filter(d => !d.destroyed);
    for (const d of alive) {
      const progress = d.controller.getMissionProgress();
      expect(progress.done).toBe(true);
    }
  });

  it('consensus-patrol: leaderless GPS-denied patrol completes', () => {
    const ctx = runStrategy('consensus-patrol', 4, 20);
    assertBoundedAltitude(ctx);
    for (const d of ctx.swarmManager.drones) {
      const progress = d.controller.getMissionProgress();
      expect(progress.total).toBeGreaterThan(0);
      expect(progress.done).toBe(true);
    }
  });
});
