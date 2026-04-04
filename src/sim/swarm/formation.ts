/**
 * FormationManager: computes formation targets from estimated/communicated data.
 * STATUS: experimental
 *
 * Does NOT import DroneState — operates only on EstimatedState + NeighborEstimate.
 * Truth-based metrics live in formation-metrics.ts.
 */

import { v3Create, v3Add, v3Sub, v3Scale, v3Len } from '@lib/math';
import type { Vec3, Quat } from '@sim/core/types';
import type { EstimatedState } from '@sim/estimation/estimator-types';
import type { NeighborEstimate } from './drone-instance';
import type { FormationConfig, FormationState, FormationTopology } from './formation-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('formation-manager', 'experimental',
  'Leader-follower + consensus formation, world-frame offsets, no bearing-only');

// Pre-allocated scratch
const _target = v3Create();
const _refVel = v3Create();
const _centroid = v3Create();
const _errEst = v3Create();
const _diff = v3Create();

/**
 * Extract yaw from NED quaternion [w,x,y,z] and rotate a vector around the down axis only.
 * This avoids projecting horizontal offsets into the vertical when the drone is tilted.
 */
function rotateByYawOnly(out: Vec3, q: Quat | Float64Array, v: Vec3): void {
  // Yaw from quaternion: atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
  const qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  const yaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  // Rotate x,y around z-axis (NED down), keep z unchanged
  out[0] = cy * v[0] - sy * v[1];
  out[1] = sy * v[0] + cy * v[1];
  out[2] = v[2];
}

export class FormationManager {
  private config: FormationConfig;
  private topology: FormationTopology = { offsets: [] };
  private droneCount = 0;
  private electedLeaderId: number | null = null;
  /** Maps droneId → slot index for sparse active drone sets. */
  private offsetMap: Map<number, number> | null = null;

  constructor(config: FormationConfig) {
    this.config = config;
  }

  updateConfig(config: FormationConfig, droneCount: number): void {
    this.config = config;
    this.droneCount = droneCount;
    this.topology = computeTopologyOffsets(droneCount, config.spacing, config.topology);
    this.electedLeaderId = null;
    this.offsetMap = null; // full reset — dense mapping
  }

  /**
   * Reconfigure formation for a sparse set of active drones (after drone loss).
   * Recomputes topology for N-1 and builds offsetMap for ID → slot remapping.
   */
  reconfigureForActive(activeIds: number[], lostLeaderId?: number): void {
    const sorted = [...activeIds].sort((a, b) => a - b);
    this.droneCount = sorted.length;
    this.topology = computeTopologyOffsets(sorted.length, this.config.spacing, this.config.topology);
    this.offsetMap = new Map();
    for (let i = 0; i < sorted.length; i++) this.offsetMap.set(sorted[i], i);
    // Re-elect leader if the old one was lost
    if (lostLeaderId !== undefined && this.getLeaderId() === lostLeaderId) {
      this.electedLeaderId = sorted[0];
    }
  }

  getOffset(droneId: number): Vec3 {
    if (this.offsetMap) {
      const slot = this.offsetMap.get(droneId);
      if (slot === undefined) return v3Create(0, 0, 0); // unmapped (destroyed)
      return this.topology.offsets[slot] ?? v3Create(0, 0, 0);
    }
    return this.topology.offsets[droneId] ?? v3Create(0, 0, 0);
  }

  getLeaderId(): number {
    return this.electedLeaderId ?? this.config.leaderDroneId;
  }

  /**
   * Compute formation target for a single drone.
   * Uses only estimates and communicated data — no truth state.
   */
  getFormationTarget(
    droneId: number,
    ownEstimate: EstimatedState,
    neighborEstimates: Map<number, NeighborEstimate>,
    simTime: number,
    estimateTimeout: number,
  ): FormationState {
    const inactive: FormationState = {
      active: false, role: 'peer',
      targetPosition: v3Create(ownEstimate.position[0], ownEstimate.position[1], ownEstimate.position[2]),
      targetVelocity: v3Create(0, 0, 0), yawDes: 0,
      estimateError: v3Create(0, 0, 0), referenceHealthy: false, neighborCount: 0,
    };

    if (!this.config.enabled) return inactive;

    if (this.config.mode === 'leader-follower') {
      return this.leaderFollowerTarget(droneId, ownEstimate, neighborEstimates, simTime, estimateTimeout);
    } else {
      return this.consensusTarget(droneId, ownEstimate, neighborEstimates, simTime, estimateTimeout);
    }
  }

  private leaderFollowerTarget(
    droneId: number,
    ownEstimate: EstimatedState,
    neighborEstimates: Map<number, NeighborEstimate>,
    simTime: number,
    estimateTimeout: number,
  ): FormationState {
    const leaderId = this.getLeaderId();

    // Am I the leader?
    if (droneId === leaderId) {
      return {
        active: true, role: 'leader',
        targetPosition: v3Create(ownEstimate.position[0], ownEstimate.position[1], ownEstimate.position[2]),
        targetVelocity: v3Create(0, 0, 0), yawDes: 0,
        estimateError: v3Create(0, 0, 0),
        referenceHealthy: true, neighborCount: neighborEstimates.size,
      };
    }

    // I'm a follower — look up leader estimate
    const leaderEst = neighborEstimates.get(leaderId);
    if (!leaderEst || !leaderEst.inFormation || !leaderEst.aligned || !leaderEst.attitudeHealthy) {
      // Leader lost — check reelection
      if (this.config.leaderLossFallback === 'reelect') {
        const newLeader = this.electLeader(droneId, neighborEstimates, simTime, estimateTimeout);
        if (newLeader !== null && newLeader !== leaderId) {
          this.electedLeaderId = newLeader;
          if (newLeader === droneId) {
            // I am the new leader — snapshot hover
            return {
              active: true, role: 'leader',
              targetPosition: v3Create(ownEstimate.position[0], ownEstimate.position[1], ownEstimate.position[2]),
              targetVelocity: v3Create(0, 0, 0), yawDes: 0,
              estimateError: v3Create(0, 0, 0),
              referenceHealthy: true, neighborCount: neighborEstimates.size,
            };
          }
          // Try following new leader
          const newLeaderEst = neighborEstimates.get(newLeader);
          if (newLeaderEst && newLeaderEst.inFormation) {
            return this.buildFollowerTarget(droneId, newLeader, newLeaderEst, ownEstimate);
          }
        }
      }
      // Fallback: hover
      return {
        active: false, role: 'follower',
        targetPosition: v3Create(ownEstimate.position[0], ownEstimate.position[1], ownEstimate.position[2]),
        targetVelocity: v3Create(0, 0, 0), yawDes: 0,
        estimateError: v3Create(0, 0, 0),
        referenceHealthy: false, neighborCount: 0,
      };
    }

    // Check staleness
    if (simTime - leaderEst.timestamp > estimateTimeout) {
      return {
        active: false, role: 'follower',
        targetPosition: v3Create(ownEstimate.position[0], ownEstimate.position[1], ownEstimate.position[2]),
        targetVelocity: v3Create(0, 0, 0), yawDes: 0,
        estimateError: v3Create(0, 0, 0),
        referenceHealthy: false, neighborCount: 0,
      };
    }

    return this.buildFollowerTarget(droneId, leaderId, leaderEst, ownEstimate);
  }

  private buildFollowerTarget(
    droneId: number, leaderId: number, leaderEst: NeighborEstimate, ownEstimate: EstimatedState,
  ): FormationState {
    const myOffset = this.getOffset(droneId);
    const leaderOffset = this.getOffset(leaderId);

    // Relative offset
    const relOff = v3Create(myOffset[0] - leaderOffset[0], myOffset[1] - leaderOffset[1], myOffset[2] - leaderOffset[2]);

    // If heading-aligned and leader quaternion available, rotate offset by leader yaw only
    if (this.config.offsetFrame === 'heading' && leaderEst.quaternion) {
      const rotated = v3Create();
      rotateByYawOnly(rotated, leaderEst.quaternion, relOff);
      _target[0] = leaderEst.position[0] + rotated[0];
      _target[1] = leaderEst.position[1] + rotated[1];
      _target[2] = leaderEst.position[2] + rotated[2];
    } else {
      // World-frame offsets (default)
      _target[0] = leaderEst.position[0] + relOff[0];
      _target[1] = leaderEst.position[1] + relOff[1];
      _target[2] = leaderEst.position[2] + relOff[2];
    }

    // Feedforward only horizontal velocity — altitude is controlled by position target.
    // Copying leader's z velocity through comm delay creates positive feedback → divergence.
    _refVel[0] = leaderEst.velocity[0];
    _refVel[1] = leaderEst.velocity[1];
    _refVel[2] = 0;

    // Estimate-based error
    _errEst[0] = ownEstimate.position[0] - _target[0];
    _errEst[1] = ownEstimate.position[1] - _target[1];
    _errEst[2] = ownEstimate.position[2] - _target[2];

    return {
      active: true, role: 'follower',
      targetPosition: v3Create(_target[0], _target[1], _target[2]),
      targetVelocity: v3Create(_refVel[0], _refVel[1], _refVel[2]),
      yawDes: 0,
      estimateError: v3Create(_errEst[0], _errEst[1], _errEst[2]),
      referenceHealthy: true,
      neighborCount: 1,
    };
  }

  private consensusTarget(
    droneId: number,
    ownEstimate: EstimatedState,
    neighborEstimates: Map<number, NeighborEstimate>,
    simTime: number,
    estimateTimeout: number,
  ): FormationState {
    const myOffset = this.getOffset(droneId);
    const K = this.config.consensusGain;

    // Compute centroid from healthy, inFormation neighbors
    let count = 0;
    _centroid[0] = 0; _centroid[1] = 0; _centroid[2] = 0;
    _refVel[0] = 0; _refVel[1] = 0; _refVel[2] = 0;

    for (const [nid, est] of neighborEstimates) {
      if (!est.inFormation || !est.aligned || !est.attitudeHealthy) continue;
      if (simTime - est.timestamp > estimateTimeout) continue;

      const nOffset = this.getOffset(nid);
      // c_j = p_j - R(q_j)*o_j (recover centroid, heading-aware if enabled)
      let offX = nOffset[0], offY = nOffset[1], offZ = nOffset[2];
      if (this.config.offsetFrame === 'heading' && est.quaternion) {
        const rotOff = v3Create();
        rotateByYawOnly(rotOff, est.quaternion, nOffset);
        offX = rotOff[0]; offY = rotOff[1]; offZ = rotOff[2];
      }
      _centroid[0] += est.position[0] - offX;
      _centroid[1] += est.position[1] - offY;
      _centroid[2] += est.position[2] - offZ;
      _refVel[0] += est.velocity[0];
      _refVel[1] += est.velocity[1];
      // No z velocity feedforward — altitude controlled by position
      count++;
    }

    if (count === 0) {
      // No healthy neighbors — detach, hover
      return {
        active: false, role: 'peer',
        targetPosition: v3Create(ownEstimate.position[0], ownEstimate.position[1], ownEstimate.position[2]),
        targetVelocity: v3Create(0, 0, 0), yawDes: 0,
        estimateError: v3Create(0, 0, 0),
        referenceHealthy: false, neighborCount: 0,
      };
    }

    _centroid[0] /= count; _centroid[1] /= count; _centroid[2] /= count;
    _refVel[0] /= count; _refVel[1] /= count; _refVel[2] /= count;

    // target_i = (1 - K) * p_i + K * (centroid + R(q_own)*o_i)
    let myOffX = myOffset[0], myOffY = myOffset[1], myOffZ = myOffset[2];
    if (this.config.offsetFrame === 'heading') {
      const rotOff = v3Create();
      rotateByYawOnly(rotOff, ownEstimate.quaternion, myOffset);
      myOffX = rotOff[0]; myOffY = rotOff[1]; myOffZ = rotOff[2];
    }
    _target[0] = (1 - K) * ownEstimate.position[0] + K * (_centroid[0] + myOffX);
    _target[1] = (1 - K) * ownEstimate.position[1] + K * (_centroid[1] + myOffY);
    _target[2] = (1 - K) * ownEstimate.position[2] + K * (_centroid[2] + myOffZ);

    _errEst[0] = ownEstimate.position[0] - (_centroid[0] + myOffset[0]);
    _errEst[1] = ownEstimate.position[1] - (_centroid[1] + myOffset[1]);
    _errEst[2] = ownEstimate.position[2] - (_centroid[2] + myOffset[2]);

    return {
      active: true, role: 'peer',
      targetPosition: v3Create(_target[0], _target[1], _target[2]),
      targetVelocity: v3Create(_refVel[0], _refVel[1], _refVel[2]),
      yawDes: 0,
      estimateError: v3Create(_errEst[0], _errEst[1], _errEst[2]),
      referenceHealthy: true,
      neighborCount: count,
    };
  }

  /**
   * Elect new leader: lowest ID in {self} ∪ {healthy inFormation neighbors}.
   */
  private electLeader(
    selfId: number,
    neighborEstimates: Map<number, NeighborEstimate>,
    simTime: number,
    estimateTimeout: number,
  ): number | null {
    let bestId = selfId; // self is always a candidate
    for (const [nid, est] of neighborEstimates) {
      if (!est.inFormation || !est.aligned || !est.attitudeHealthy) continue;
      if (simTime - est.timestamp > estimateTimeout) continue;
      if (nid < bestId) bestId = nid;
    }
    return bestId;
  }
}

// ── Topology preset computation ──

export function computeTopologyOffsets(
  droneCount: number, spacing: number, topology: 'line' | 'grid' | 'circle',
): FormationTopology {
  const offsets: Vec3[] = [];

  if (topology === 'line') {
    for (let i = 0; i < droneCount; i++) {
      offsets.push(v3Create((i - (droneCount - 1) / 2) * spacing, 0, 0));
    }
  } else if (topology === 'grid') {
    const cols = Math.ceil(Math.sqrt(droneCount));
    for (let i = 0; i < droneCount; i++) {
      const row = Math.floor(i / cols);
      const col = i % cols;
      offsets.push(v3Create(
        (col - (cols - 1) / 2) * spacing,
        (row - (Math.ceil(droneCount / cols) - 1) / 2) * spacing,
        0));
    }
  } else { // circle
    for (let i = 0; i < droneCount; i++) {
      const angle = (2 * Math.PI * i) / droneCount;
      const r = droneCount === 1 ? 0 : spacing;
      offsets.push(v3Create(r * Math.cos(angle), r * Math.sin(angle), 0));
    }
  }

  return { offsets };
}
