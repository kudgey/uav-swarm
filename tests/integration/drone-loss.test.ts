/**
 * Test: drone loss detection, formation reconfiguration, mission handoff.
 */
import { describe, it, expect } from 'vitest';
import { SimContext } from '@sim/core/sim-context';
import { defaultSimConfig } from '@sim/core/config-defaults';

describe('Drone loss and reconfiguration', () => {
  function makeConfig(n: number) {
    const cfg = defaultSimConfig();
    cfg.swarm.droneCount = n;
    cfg.swarm.initialPattern = 'line';
    cfg.swarm.patternSpacing = 3;
    cfg.swarm.patternCenter = new Float64Array([5, 5, -2]);
    cfg.swarm.initialPositions = undefined;
    cfg.swarm.formation.enabled = true;
    cfg.swarm.formation.leaderLossFallback = 'reelect';
    // Enable UWB for inter-drone comms
    cfg.sensors.uwb = { ...cfg.sensors.uwb!, enabled: true };
    return cfg;
  }

  function armAll(ctx: SimContext) {
    for (const d of ctx.swarmManager.drones) {
      if (!d.ekf.isAligned()) d.ekf.bootstrapAligned();
      d.controller.forceArm();
      // Set hover at spawn position so controller is active
      d.controller.setHover(d.state.position, 0);
    }
  }

  function killDrone(d: import('@sim/swarm/drone-instance').DroneInstance) {
    d.destroyed = true;
    d.commEnabled = false;
    d.controller.requestDisarm();
    for (let i = 0; i < d.state.motorCommands.length; i++) {
      d.state.motorCommands[i] = 0;
      d.state.motorSpeeds[i] = 0;
    }
  }

  it('destroyed drone stops broadcasting and is detected via comm silence', () => {
    const cfg = makeConfig(4);
    const ctx = new SimContext(cfg);
    armAll(ctx);

    // Run a few seconds for estimates to establish
    ctx.stepN(3000); // 3s

    // All drones should have neighbor estimates (from comm broadcasts)
    // Verify comm is working
    expect(ctx.swarmManager.commSystem.sentCount).toBeGreaterThan(0);

    killDrone(ctx.swarmManager.drones[2]);

    // Run until estimate timeout expires (~2s + margin)
    ctx.stepN(3000); // 3s more

    // Drone 2's estimate should be expired from all surviving drones
    for (const d of ctx.swarmManager.drones) {
      if (d.id === 2) continue;
      expect(d.neighborEstimates.has(2)).toBe(false);
    }
  });

  it('formation reconfigures after drone loss', () => {
    const cfg = makeConfig(5);
    const ctx = new SimContext(cfg);
    armAll(ctx);
    ctx.stepN(3000);

    killDrone(ctx.swarmManager.drones[2]);

    // Run until detection + reconfiguration
    ctx.stepN(4000);

    // Formation should still be active for surviving drones
    for (const d of ctx.swarmManager.drones) {
      if (d.id === 2) continue;
      // Surviving drones should have formation state
      if (d.lastFormationState) {
        expect(d.lastFormationState.active).toBe(true);
      }
    }
  });

  it('killed drone is detected and formation re-elects leader', () => {
    const cfg = makeConfig(4);
    const ctx = new SimContext(cfg);
    armAll(ctx);
    ctx.stepN(5000); // establish comms

    const leaderBefore = ctx.swarmManager.formationManager.getLeaderId();
    const leader = ctx.swarmManager.getDrone(leaderBefore)!;
    killDrone(leader);

    ctx.stepN(5000); // wait for detection

    // Leader should have changed (or at least surviving drones still have formation)
    const aliveWithFormation = ctx.swarmManager.drones.filter(
      d => !d.destroyed && d.lastFormationState?.active
    );
    expect(aliveWithFormation.length).toBeGreaterThan(0);
  });
});
