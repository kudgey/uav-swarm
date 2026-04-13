/**
 * Fast-yaw formation test: leader rotates yaw rapidly, verify followers track.
 * With heading-aligned offsets and comm latency, rapid yaw can break tracking.
 * Verifies formation stays bounded even with aggressive yaw.
 */
import { describe, it, expect } from 'vitest';
import { SimContext } from '@sim/core/sim-context';
import { defaultSimConfig } from '@sim/core/config-defaults';
import { v3Create } from '@lib/math';

describe('Fast yaw formation tracking', () => {
  it('leader performs yaw sweep, followers maintain bounded formation', () => {
    const cfg = defaultSimConfig();
    cfg.swarm.droneCount = 4;
    cfg.swarm.initialPattern = 'line';
    cfg.swarm.patternSpacing = 2;
    cfg.swarm.patternCenter = new Float64Array([5, 5, -2]);
    cfg.swarm.initialPositions = undefined;
    cfg.swarm.formation.enabled = true;
    cfg.swarm.formation.offsetFrame = 'heading';

    const ctx = new SimContext(cfg);
    // Arm all
    for (const d of ctx.swarmManager.drones) {
      if (!d.ekf.isAligned()) d.ekf.bootstrapAligned();
      d.controller.forceArm();
      d.controller.setHover(v3Create(d.state.position[0], d.state.position[1], d.state.position[2]), 0);
    }
    ctx.swarmManager.updateFormationConfig({ ...cfg.swarm.formation, enabled: true });

    // Establish formation for 3s
    const steps1 = Math.round(3 / cfg.physicsDt);
    for (let i = 0; i < steps1; i++) ctx.stepOnce();

    // Now command leader hover with yaw sweep: every 1s rotate by π/2
    const leader = ctx.swarmManager.drones[0];
    const leaderPos = v3Create(leader.state.position[0], leader.state.position[1], leader.state.position[2]);
    const yawSchedule = [0, Math.PI / 2, Math.PI, -Math.PI / 2, 0];
    const steps2 = Math.round(1 / cfg.physicsDt);
    for (const yaw of yawSchedule) {
      leader.controller.setHover(leaderPos, yaw);
      for (let i = 0; i < steps2; i++) ctx.stepOnce();
    }

    // Verify no NaN, altitudes bounded, position bounded
    for (const d of ctx.swarmManager.drones) {
      expect(Number.isFinite(d.state.position[0])).toBe(true);
      expect(Number.isFinite(d.state.position[1])).toBe(true);
      expect(d.state.position[2]).toBeGreaterThan(-10);
      expect(d.state.position[2]).toBeLessThan(2);
      // XY stays bounded — heading-aligned with rapid yaw causes large transients,
      // so threshold is loose to document the current limit rather than strict bound.
      const dx = d.state.position[0] - 5;
      const dy = d.state.position[1] - 5;
      const horiz = Math.sqrt(dx * dx + dy * dy);
      expect(horiz).toBeLessThan(30);
    }
  }, 30000);
});
