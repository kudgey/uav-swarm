/**
 * Estimator type definitions.
 * EstimatedState is the contract between EKF and controller — NO DroneState import.
 */

import type { Vec3, Quat } from '@sim/core/types';

export interface EstimatedState {
  position: Vec3;           // NED world frame
  velocity: Vec3;           // NED world frame
  quaternion: Quat;         // body-to-world, Hamilton scalar-first
  gyroBias: Vec3;           // rad/s
  accelBias: Vec3;          // m/s^2
  baroBias: number;         // Pa
  angularVelocity: Vec3;    // body frame (from bias-corrected IMU)
  timestamp: number;
}

export interface EKFConfig {
  // Process noise power spectral densities
  gyroNoisePSD: number;        // (rad/s)^2/Hz
  accelNoisePSD: number;       // (m/s^2)^2/Hz
  gyroBiasRWPSD: number;       // (rad/s^2)^2/Hz
  accelBiasRWPSD: number;      // (m/s^3)^2/Hz
  baroBiasRWPSD: number;       // Pa^2/Hz

  // Measurement noise variances (per-axis)
  magNoiseVar: number;         // T^2
  baroNoiseVar: number;        // Pa^2
  rangeNoiseVar: number;       // m^2
  flowNoiseVar: number;        // (rad/s)^2

  // Initial covariance diagonal
  initPosVar: number;          // m^2
  initVelVar: number;          // (m/s)^2
  initAttVar: number;          // rad^2
  initGyroBiasVar: number;     // (rad/s)^2
  initAccelBiasVar: number;    // (m/s^2)^2
  initBaroBiasVar: number;     // Pa^2

  // Expected earth magnetic field (world frame NED)
  earthMagField: Vec3;

  // Innovation gating (chi-squared thresholds)
  magGateThreshold: number;    // chi2(3, 0.99) ≈ 11.34
  rangeGateThreshold: number;  // 3-sigma ≈ 9
  flowGateThreshold: number;   // chi2(2, 0.99) ≈ 9.21

  // Alignment thresholds
  alignAttVarThreshold: number;   // rad^2
  alignAltVarThreshold: number;   // m^2
  alignMinUpdates: number;        // min mag+baro updates before aligned

  // VIO noise variances
  vioPositionNoiseVar: number;    // m^2
  vioAttitudeNoiseVar: number;    // rad^2
  vioGateThreshold: number;       // chi2(3, 0.99)

  // UWB noise variances
  uwbLosNoiseVar: number;         // m^2
  uwbNlosNoiseVar: number;        // m^2 (larger for NLOS)
  uwbGateThreshold: number;       // chi2(1)

  // Phase 11: Innovation-based NLOS detection
  innovationNLOSDetection: boolean; // enable per-anchor innovation tracking
  nlosInnovThreshold: number;       // normalized innovation χ² threshold for NLOS
  nlosConsecutiveCount: number;     // consecutive high innovations before NLOS declared

  // Health thresholds (per-subsystem)
  attHealthThreshold: number;     // rad^2, attitude P diagonal max
  vertHealthThreshold: number;    // m^2, vertical P diagonal
  horizHealthThreshold: number;   // m^2, achievable with VIO/UWB
}

export type EKFSubsystemHealth = 'healthy' | 'degraded' | 'lost';

export interface EKFHealth {
  attitude: EKFSubsystemHealth;
  vertical: EKFSubsystemHealth;
  horizontal: EKFSubsystemHealth; // always 'degraded' or 'lost' without XY anchor
  aligned: boolean;
  attCovMax: number;
  vertCov: number;
  horizCovMax: number;
  updateCount: number;
}

export interface InnovationRecord {
  timestamp: number;
  source: 'mag' | 'baro' | 'range' | 'flow' | 'vio-pos' | 'vio-att' | 'uwb';
  innovationNorm: number;
  gated: boolean;   // was the update rejected by innovation gating
}

export type ArmState = 'disarmed' | 'aligning' | 'armed';
