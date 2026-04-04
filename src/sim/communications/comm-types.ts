/**
 * Communication system types.
 */

import type { Vec3 } from '@sim/core/types';

export interface CommConfig {
  range: number;              // m, max communication distance
  latencyMean: number;        // s
  latencyJitter: number;      // s (std dev)
  dropRate: number;           // 0-1
  bandwidth: number;          // messages per second per link
  estimatePublishRate: number; // Hz, how often each drone broadcasts its estimate
  maneuverAccelStdDev: number; // m/s², for inter-drone UWB variance inflation
  estimateTimeout: number;    // s, expire stale neighbor estimates
  losProjectedVariance: boolean; // Phase 11: send P_3x3, LOS-projected R_eff
}

export interface CommMessage {
  senderId: number;
  receiverId: number;         // -1 = broadcast to all neighbors
  sendTime: number;
  deliveryTime: number;       // sendTime + latency + jitter
  type: 'state-estimate';
  payload: DroneStateEstimatePayload;
}

export interface DroneStateEstimatePayload {
  senderId: number;
  timestamp: number;          // when estimate was produced
  position: Vec3;
  positionVariance: number;   // Pxx + Pyy (horizontal trace, for safety/ORCA)
  positionCov3x3?: [number, number, number, number, number, number]; // upper triangle: Pxx,Pxy,Pxz,Pyy,Pyz,Pzz (Phase 11)
  velocity: Vec3;
  quaternion?: [number, number, number, number]; // Phase 11: for heading-aligned formation
  aligned: boolean;
  attitudeHealthy: boolean;
  inFormation: boolean;
  missionProgress?: number; // consensus: current waypoint index for leaderless sync
}
