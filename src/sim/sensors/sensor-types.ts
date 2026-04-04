/**
 * Common sensor interfaces and output types.
 */

import type { Vec3, Quat } from '@sim/core/types';

export interface SensorOutput<T> {
  timestamp: number;
  valid: boolean;
  measurement: T;
}

export interface ImuMeasurement {
  gyro: Vec3;     // rad/s, body frame
  accel: Vec3;    // m/s^2, body frame (specific force)
}

export interface MagMeasurement {
  field: Vec3;    // T, body frame (distorted)
}

export interface BaroMeasurement {
  pressure: number;    // Pa
  altitude: number;    // m (derived from pressure)
}

export interface RangeMeasurement {
  range: number;       // m
}

export interface FlowMeasurement {
  flowX: number;  // rad/s
  flowY: number;  // rad/s
}

export interface VIOMeasurement {
  position: Vec3;       // NED world frame
  quaternion: Quat;     // body-to-world, Hamilton scalar-first
  featureQuality: number; // 0-1, reported quality metric
}

export interface UWBMeasurement {
  anchorId: string;
  range: number;        // m
  isNLOS: boolean;       // truth flag for telemetry (NOT used by EKF for estimation)
}

/** All latest sensor readings bundled for telemetry. */
export interface SensorReadings {
  imu: SensorOutput<ImuMeasurement> | null;
  magnetometer: SensorOutput<MagMeasurement> | null;
  barometer: SensorOutput<BaroMeasurement> | null;
  rangefinder: SensorOutput<RangeMeasurement> | null;
  opticalFlow: SensorOutput<FlowMeasurement> | null;
  cameraVIO: SensorOutput<VIOMeasurement> | null;
  uwb: SensorOutput<UWBMeasurement>[] | null;
}
