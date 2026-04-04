/**
 * Controller types: commands, config, state, failsafe.
 */

import type { Vec3, Quat } from '@sim/core/types';

export interface AttitudeCommand {
  roll: number;     // rad
  pitch: number;    // rad
  yaw: number;      // rad
  thrust: number;   // N (total)
}

export interface RateCommand {
  rollRate: number;  // rad/s
  pitchRate: number; // rad/s
  yawRate: number;   // rad/s
  thrust: number;    // N
}

export interface MotorCommand {
  speeds: Float64Array;  // rad/s per rotor
}

// ── Guidance ──

export type GuidanceMode = 'hover' | 'waypoint';

export interface GuidanceOutput {
  positionDes: Vec3;
  velocityDes: Vec3;
  accelerationDes: Vec3;
  yawDes: number;
}

// ── Controller config ──

export interface ControllerConfig {
  // Outer loop gains
  kpPos: Vec3;        // position proportional [x, y, z]
  kdPos: Vec3;        // position derivative (velocity damping)
  // Inner loop gains
  krAtt: Vec3;        // attitude proportional [roll, pitch, yaw]
  kwRate: Vec3;       // rate damping [p, q, r]
  // Limits
  maxTiltRad: number;
  maxYawRate: number; // rad/s
  maxThrust: number;  // N
  minThrust: number;  // N
  // Failsafe
  kwFailsafe: Vec3;   // rate damping gains for failsafe mode
  hoverThrustFraction: number; // fraction of mg for hover-trim in failsafe
  // Stationarity thresholds for alignment
  stationaryGyroThreshold: number;  // rad/s
  stationaryAccelThreshold: number; // relative to g
}

// ── Arm and control mode ──

export type ArmState = 'disarmed' | 'aligning' | 'armed';
export type ControlMode = 'manual' | 'hover' | 'waypoint' | 'formation';

export interface FailsafeState {
  active: boolean;
  reason: string;
  mode: 'none' | 'rate-damped-descent' | 'emergency-descent';
}
