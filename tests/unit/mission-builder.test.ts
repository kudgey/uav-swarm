/**
 * Test: shared mission builder generates correct routes and steps.
 */
import { describe, it, expect } from 'vitest';
import { buildPatrolRoute, buildPatrolMissionSteps } from '@sim/missions/mission-builder';

describe('Mission builder', () => {
  it('buildPatrolRoute: generates outbound waypoints at correct positions', () => {
    const route = buildPatrolRoute({
      distance: 100, waypointCount: 5, speed: 2, returnToBase: false,
      center: [0, 0, -2],
    });
    expect(route).toHaveLength(5);
    expect(route[0].position[0]).toBeCloseTo(20);  // 100/5 = 20m intervals
    expect(route[4].position[0]).toBeCloseTo(100);
    // All at same altitude
    for (const wp of route) expect(wp.position[2]).toBe(-2);
  });

  it('buildPatrolRoute: return adds reverse waypoints back to origin', () => {
    const route = buildPatrolRoute({
      distance: 60, waypointCount: 3, speed: 1.5, returnToBase: true,
      center: [5, 5, -2],
    });
    // 3 outbound + 3 return = 6
    expect(route).toHaveLength(6);
    // Last waypoint should be back at center
    expect(route[5].position[0]).toBeCloseTo(5);
    expect(route[5].position[1]).toBeCloseTo(5);
  });

  it('buildPatrolMissionSteps: includes arm, hover, formation, mission plan', () => {
    const steps = buildPatrolMissionSteps({
      droneCount: 3, distance: 50, waypointCount: 3, speed: 1.5,
      returnToBase: true, center: [5, 5, -2], spacing: 2, pattern: 'line',
    });
    expect(steps.some(s => s.action === 'arm')).toBe(true);
    expect(steps.filter(s => s.action === 'hover')).toHaveLength(3);
    expect(steps.some(s => s.action === 'formation-enable')).toBe(true);
    expect(steps.some(s => s.action === 'set-mission-plan')).toBe(true);
    // Mission plan should have waypoints
    const planStep = steps.find(s => s.action === 'set-mission-plan');
    const wps = planStep?.params?.waypoints as unknown[];
    expect(wps.length).toBe(6); // 3 out + 3 return
  });

  it('buildPatrolMissionSteps: optional kill-drone step', () => {
    const steps = buildPatrolMissionSteps({
      droneCount: 5, distance: 50, waypointCount: 3, speed: 1.5,
      returnToBase: false, center: [5, 5, -2], spacing: 2, pattern: 'circle',
      killDroneId: 2, killTime: 10,
    });
    const killStep = steps.find(s => s.action === 'kill-drone');
    expect(killStep).toBeDefined();
    expect(killStep?.droneId).toBe(2);
    expect(killStep?.time).toBe(10);
  });
});
