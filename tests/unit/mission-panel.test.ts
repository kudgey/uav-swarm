import { describe, expect, it } from 'vitest';
import {
  buildMissionConfig,
  buildVisualMissionSteps,
  computePatternPositions,
} from '@ui/panels/mission-panel';

describe('Mission panel planners', () => {
  it('computes distinct spawn positions for multi-drone patterns', () => {
    const positions = computePatternPositions(3, 'line', 2, [5, 5, -2]);
    expect(positions).toEqual([
      [3, 5, -2],
      [5, 5, -2],
      [7, 5, -2],
    ]);
  });

  it('crossing: every drone gets a mirror waypoint through center', () => {
    const steps = buildVisualMissionSteps({
      type: 'crossing', droneCount: 4, pattern: 'line', spacing: 2,
      distance: 10, speed: 1.5, wind: 0,
    });
    const wps = steps.filter(s => s.action === 'waypoint');
    expect(wps).toHaveLength(4);
    // Each drone's waypoint should be the mirror of its hover position
    const hovers = steps.filter(s => s.action === 'hover');
    for (let i = 0; i < 4; i++) {
      const hp = hovers[i].params?.position as number[];
      const wp = wps[i].params?.position as number[];
      // Mirror through center [5,5]: wp = 2*5 - hp
      expect(wp[0]).toBeCloseTo(10 - hp[0]);
      expect(wp[1]).toBeCloseTo(10 - hp[1]);
      expect(wp[2]).toBe(hp[2]); // same altitude
    }
  });

  it('head-on: two groups with opposing waypoints', () => {
    const steps = buildVisualMissionSteps({
      type: 'head-on', droneCount: 4, pattern: 'line', spacing: 2,
      distance: 10, speed: 1.5, wind: 0,
    });
    const wps = steps.filter(s => s.action === 'waypoint');
    expect(wps).toHaveLength(4);
    // First 2 drones go right (positive x), last 2 go left (negative x)
    const group_a = wps.filter(s => (s.droneId as number) < 2);
    const group_b = wps.filter(s => (s.droneId as number) >= 2);
    expect((group_a[0].params?.position as number[])[0]).toBeGreaterThan(5);
    expect((group_b[0].params?.position as number[])[0]).toBeLessThan(5);
  });

  it('squeeze: all drones converge to center', () => {
    const steps = buildVisualMissionSteps({
      type: 'squeeze', droneCount: 5, pattern: 'circle', spacing: 3,
      distance: 10, speed: 1, wind: 0,
    });
    const wps = steps.filter(s => s.action === 'waypoint');
    expect(wps).toHaveLength(5);
    // All targets = center
    for (const wp of wps) {
      const p = wp.params?.position as number[];
      expect(p[0]).toBe(5);
      expect(p[1]).toBe(5);
    }
  });

  it('scatter-regroup: scatter phase then regroup to original positions', () => {
    const steps = buildVisualMissionSteps({
      type: 'scatter-regroup', droneCount: 4, pattern: 'circle', spacing: 2,
      distance: 10, speed: 1.5, wind: 0,
    });
    const wps = steps.filter(s => s.action === 'waypoint');
    // 4 scatter + 4 regroup = 8
    expect(wps).toHaveLength(8);
    // Regroup targets should match hover positions
    const hovers = steps.filter(s => s.action === 'hover');
    const regroupWps = wps.slice(4);
    for (let i = 0; i < 4; i++) {
      const hp = hovers[i].params?.position as number[];
      const rp = regroupWps[i].params?.position as number[];
      expect(rp[0]).toBeCloseTo(hp[0]);
      expect(rp[1]).toBeCloseTo(hp[1]);
    }
  });

  it('formation-transit: formation enable + out and back waypoints', () => {
    const steps = buildVisualMissionSteps({
      type: 'formation-transit', droneCount: 3, pattern: 'line', spacing: 2,
      distance: 10, speed: 1.5, wind: 0,
    });
    expect(steps.some(s => s.action === 'formation-enable')).toBe(true);
    const wps = steps.filter(s => s.action === 'waypoint');
    expect(wps).toHaveLength(2); // out + back
    expect((wps[0].params?.position as number[])[0]).toBe(15); // center + distance
    expect((wps[1].params?.position as number[])[0]).toBe(5); // back to center
  });

  it('config has GPS-denied aiding and expanded scene bounds', () => {
    const cfg = buildMissionConfig(5, 'line', 2, { type: 'crossing', distance: 20 });
    expect(cfg.sensors.cameraVIO?.enabled).toBe(true);
    expect(cfg.sensors.uwb?.enabled).toBe(true);
    expect(cfg.swarm.formation.offsetFrame).toBe('heading');
    expect(cfg.environment.scene.sceneBounds).toBeDefined();
  });
});
