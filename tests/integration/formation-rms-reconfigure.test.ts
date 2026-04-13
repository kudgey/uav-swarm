/**
 * Verify formation RMS uses correct offsetMap after drone loss + reconfigure.
 * After reconfigure, surviving drones [0, 1, 3, 4] map to slots [0, 1, 2, 3].
 * Formation RMS must use remapped offsets, not old dense-index offsets.
 */
import { describe, it, expect } from 'vitest';
import { FormationManager } from '@sim/swarm/formation';
import { computeTruthFormationRMS } from '@sim/swarm/formation-metrics';
import type { DroneInstance } from '@sim/swarm/drone-instance';

function mockDrone(id: number, x: number, y: number, z: number, destroyed = false): DroneInstance {
  return {
    id,
    state: {
      position: new Float64Array([x, y, z]),
      velocity: new Float64Array([0, 0, 0]),
      quaternion: new Float64Array([1, 0, 0, 0]),
      angularVelocity: new Float64Array([0, 0, 0]),
      motorSpeeds: new Float64Array(4),
      motorCommands: new Float64Array(4),
    },
    destroyed,
  } as unknown as DroneInstance;
}

describe('Formation RMS after reconfigure', () => {
  it('5-drone line at spacing 2, perfect placement → RMS = 0', () => {
    const fm = new FormationManager({
      enabled: true, mode: 'leader-follower', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'reelect', offsetFrame: 'world',
    });
    fm.updateConfig({
      enabled: true, mode: 'leader-follower', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'reelect', offsetFrame: 'world',
    }, 5);

    // Line offsets for 5 drones at spacing 2: [-4, -2, 0, 2, 4]
    const drones = [0, 1, 2, 3, 4].map(id => {
      const x = (id - 2) * 2; // match line offset
      return mockDrone(id, x, 0, -2);
    });
    const rms = computeTruthFormationRMS(drones, (id) => fm.getOffset(id), 0);
    expect(rms).toBeLessThan(0.01);
  });

  it('After reconfigure (drone 2 destroyed), surviving drones at new slots → low RMS', () => {
    const fm = new FormationManager({
      enabled: true, mode: 'leader-follower', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'reelect', offsetFrame: 'world',
    });
    fm.updateConfig({
      enabled: true, mode: 'leader-follower', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'reelect', offsetFrame: 'world',
    }, 5);

    // Drone 2 destroyed. Reconfigure for active [0, 1, 3, 4] — new topology 4 drones spacing 2.
    // New offsets: [-3, -1, 1, 3]. Active IDs sorted: [0, 1, 3, 4].
    // Slot map: 0→0 (-3), 1→1 (-1), 3→2 (+1), 4→3 (+3)
    fm.reconfigureForActive([0, 1, 3, 4]);

    // Place drones at their remapped offsets (leader=0 at x=-3, plus leader pos offset)
    const leaderX = 0; // arbitrary leader truth x
    const drones = [
      mockDrone(0, leaderX, 0, -2),                 // leader, offset[0] = -3 from centroid
      mockDrone(1, leaderX + 2, 0, -2),             // offset[-1] - (-3) = +2
      mockDrone(2, 99, 99, 99, true),               // destroyed — should be skipped
      mockDrone(3, leaderX + 4, 0, -2),             // offset[+1] - (-3) = +4
      mockDrone(4, leaderX + 6, 0, -2),             // offset[+3] - (-3) = +6
    ];
    const rms = computeTruthFormationRMS(drones, (id) => fm.getOffset(id), 0);
    expect(rms).toBeLessThan(0.5);
  });

  it('After reconfigure with sparse gap, RMS reflects actual relative errors only', () => {
    const fm = new FormationManager({
      enabled: true, mode: 'leader-follower', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'reelect', offsetFrame: 'world',
    });
    fm.updateConfig({
      enabled: true, mode: 'leader-follower', topology: 'line', spacing: 2,
      leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'reelect', offsetFrame: 'world',
    }, 5);
    fm.reconfigureForActive([0, 1, 3, 4]);

    // Drones misplaced by known small amounts after reconfigure
    const drones = [
      mockDrone(0, 0.1, 0, -2),
      mockDrone(1, 2.1, 0, -2),
      mockDrone(2, 99, 99, 99, true),
      mockDrone(3, 4.1, 0, -2),
      mockDrone(4, 6.1, 0, -2),
    ];
    const rms = computeTruthFormationRMS(drones, (id) => fm.getOffset(id), 0);
    expect(Number.isFinite(rms)).toBe(true);
    expect(rms).toBeLessThan(1.0);
  });
});
