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
      case 'set-mission-plan': {
        const wps = step.params?.waypoints as { position: [number, number, number]; yaw: number; speed: number }[];
        if (wps) d.controller.setMissionPlan(wps);
        break;
      }
      case 'formation-enable': {
        const ok = ctx.swarmManager.updateFormationConfig({ ...ctx.config.swarm.formation, enabled: true });
        if (!ok) console.log('[TEST] formation-enable FAILED, mode:', ctx.config.swarm.formation.mode, 'drones armed:', ctx.swarmManager.drones.map(d => d.controller.armState));
        return ok;
      }
      case 'set-environment':
        Object.assign(ctx.config.environment, step.params);
        ctx.envManager.updateConfig(ctx.config.environment, ctx.config.drone.armLength);
        return true;
    }
  }
  return true;
}

function runMission(droneCount: number, type: string, duration: number) {
  const cfg = buildMissionConfig(droneCount, 'line', 2, { type: type as any, distance: 10 });
  const steps = buildVisualMissionSteps({
    type: type as any, droneCount, pattern: 'line', spacing: 2,
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
  it('formation-transit: leader moves via mission plan', () => {
    const ctx = runMission(3, 'formation-transit', 8);
    const leader = ctx.swarmManager.drones[0];
    expect(leader.controller.armState).toBe('armed');
    // After 8s with speed 1.5, leader should have moved
    expect(leader.state.position[0]).not.toBeCloseTo(5, 0);
  });

  it('all drones stay within reasonable altitude', () => {
    const ctx = runMission(4, 'formation-transit', 6);
    for (const d of ctx.swarmManager.drones) {
      expect(d.state.position[2]).toBeGreaterThan(-15);
      expect(d.state.position[2]).toBeLessThan(1);
    }
  });

  it('consensus-transit: drones complete mission and maintain relative spacing', () => {
    const ctx = runMission(3, 'consensus-transit', 20);

    // All drones should have completed the mission
    for (const d of ctx.swarmManager.drones) {
      const progress = d.controller.getMissionProgress();
      expect(progress.done).toBe(true);
    }

    // After return, drones should be near starting positions
    const positions = ctx.swarmManager.drones.map(d => d.state.position[0]);
    expect(positions[0]).toBeCloseTo(3, 0);
    expect(positions[2]).toBeCloseTo(7, 0);

    // Altitude stayed bounded
    for (const d of ctx.swarmManager.drones) {
      expect(d.state.position[2]).toBeGreaterThan(-5);
      expect(d.state.position[2]).toBeLessThan(1);
    }

    // Relative spacing approximately maintained (started with spacing=2)
    const sorted = [...positions].sort((a, b) => a - b);
    for (let i = 1; i < sorted.length; i++) {
      const gap = sorted[i] - sorted[i - 1];
      expect(gap).toBeGreaterThan(0.5);
      expect(gap).toBeLessThan(5);
    }
  });

  it('consensus-transit: inFormation flag is true during flight', () => {
    // Run partway through mission (before completion)
    const ctx = runMission(3, 'consensus-transit', 6);

    // At least some drones should report inFormation via consensus correction
    const inFormationCount = ctx.swarmManager.drones.filter(d =>
      d.controller.inConsensusFormation
    ).length;
    expect(inFormationCount).toBeGreaterThan(0);
  });
});
