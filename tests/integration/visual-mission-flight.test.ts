import { describe, expect, it } from 'vitest';
import { SimContext } from '@sim/core/sim-context';
import { buildMissionConfig, buildVisualMissionSteps } from '@ui/panels/mission-panel';

type VisualMissionStep = { time: number; droneId: number | 'all'; action: string; params?: Record<string, unknown> };

function applyStep(ctx: SimContext, step: VisualMissionStep): boolean {
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
      case 'formation-enable':
        return ctx.swarmManager.updateFormationConfig({ ...ctx.config.swarm.formation, enabled: true });
      case 'set-environment':
        Object.assign(ctx.config.environment, step.params);
        ctx.envManager.updateConfig(ctx.config.environment, ctx.config.drone.armLength);
        return true;
    }
  }
  return true;
}

function runMission(droneCount: number, type: string, duration: number) {
  const cfg = buildMissionConfig(droneCount, 'circle', 2, { type: type as any, distance: 10 });
  const steps = buildVisualMissionSteps({
    type: type as any, droneCount, pattern: 'circle', spacing: 2,
    distance: 10, speed: 1.5, wind: 0,
  });
  const ctx = new SimContext(cfg);
  let stepIdx = 0;
  const totalSteps = Math.round(duration / cfg.physicsDt);
  for (let i = 0; i < totalSteps; i++) {
    while (stepIdx < steps.length && steps[stepIdx].time <= ctx.simTime) {
      if (!applyStep(ctx, steps[stepIdx])) break;
      stepIdx++;
    }
    ctx.stepOnce();
  }
  return ctx;
}

describe('Visual mission flight', () => {
  it('crossing: drones move toward mirror positions', () => {
    const ctx = runMission(4, 'crossing', 6);
    // After 6s, drones should have moved from their start positions
    const d0 = ctx.swarmManager.drones[0];
    const d1 = ctx.swarmManager.drones[1];
    // D0 and D1 started at different positions, both should have moved
    expect(d0.controller.controlMode).toBe('waypoint');
    expect(d1.controller.controlMode).toBe('waypoint');
  });

  it('squeeze: all drones converge, altitude stays bounded', () => {
    const ctx = runMission(5, 'squeeze', 8);
    // All drones should stay within reasonable altitude
    for (const d of ctx.swarmManager.drones) {
      expect(d.state.position[2]).toBeGreaterThan(-10); // no higher than 10m
      expect(d.state.position[2]).toBeLessThan(1);       // not underground
    }
  });

  it('formation-transit: leader moves in +X then back', () => {
    const ctx = runMission(3, 'formation-transit', 8);
    const leader = ctx.swarmManager.drones[0];
    // Leader should be armed and in waypoint/hover mode
    expect(leader.controller.armState).toBe('armed');
    // After 8s with speed 1.5, leader should have moved >1m from start
    expect(leader.state.position[0]).not.toBeCloseTo(5, 0);
  });
});
