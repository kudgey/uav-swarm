/**
 * Formation configuration and state types.
 * STATUS: experimental
 *
 * Does NOT import DroneState — formation control uses only estimates.
 */

import type { Vec3 } from '@sim/core/types';

export interface FormationConfig {
  enabled: boolean;
  mode: 'leader-follower' | 'consensus';
  topology: 'line' | 'grid' | 'circle';
  spacing: number;                     // m between drones
  leaderDroneId: number;               // for leader-follower only
  consensusGain: number;               // 0-1, weight on neighbor centroid (consensus mode)
  leaderLossFallback: 'hover' | 'reelect';
  offsetFrame: 'world' | 'heading';   // Phase 11: heading-aligned offsets
}

export interface FormationState {
  active: boolean;
  role: 'leader' | 'follower' | 'peer';
  targetPosition: Vec3;                // world-frame target
  targetVelocity: Vec3;                // reference velocity for feedforward
  yawDes: number;                      // frozen at entry time
  estimateError: Vec3;                 // estimate-based (controller-visible)
  referenceHealthy: boolean;
  neighborCount: number;
}

export interface FormationTopology {
  offsets: Vec3[];                     // per-drone world-frame offsets from centroid
}
