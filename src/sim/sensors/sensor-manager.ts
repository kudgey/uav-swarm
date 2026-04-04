/**
 * Sensor manager: orchestrates all sensors, manages bias state and timing.
 */

import type { DroneState, DroneParams, EnvironmentOutput, SensorSuiteConfig, Vec3 } from '@sim/core/types';
import type { DeterministicRNG } from '@sim/core/rng';
import type { WorldGeometry } from '@sim/environment/world-geometry';
import { v3Create } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import { createSensorBiasState, stepBiasRandomWalks, resetSensorBiasState } from './sensor-bias-state';
import { createVIODriftState, type VIODriftState } from './camera-vio';
import type { SensorBiasState } from './sensor-bias-state';
import type { SensorReadings } from './sensor-types';
import { readImu } from './imu';
import { readMagnetometer } from './magnetometer';
import { readBarometer } from './barometer';
import { readRangefinder } from './rangefinder';
import { readOpticalFlow } from './optical-flow';
import { readCameraVIO } from './camera-vio';
import { readUWB } from './uwb';

export class SensorManager {
  private config: SensorSuiteConfig;
  private rng: DeterministicRNG;
  private worldGeo: WorldGeometry;
  private params: DroneParams;
  biasState: SensorBiasState;
  private latest: SensorReadings;
  private vioDriftState: VIODriftState;

  constructor(
    config: SensorSuiteConfig,
    rng: DeterministicRNG,
    worldGeo: WorldGeometry,
    params: DroneParams,
    initialPosition?: Vec3,
  ) {
    this.config = config;
    this.rng = rng;
    this.worldGeo = worldGeo;
    this.params = params;
    this.biasState = createSensorBiasState();
    resetSensorBiasState(this.biasState, config);
    this.vioDriftState = createVIODriftState(initialPosition ?? v3Create());
    this.latest = {
      imu: {
        timestamp: 0, valid: false,
        measurement: { gyro: v3Create(), accel: v3Create() },
      },
      magnetometer: {
        timestamp: 0, valid: false,
        measurement: { field: v3Create() },
      },
      barometer: {
        timestamp: 0, valid: false,
        measurement: { pressure: 0, altitude: 0 },
      },
      rangefinder: {
        timestamp: 0, valid: false,
        measurement: { range: 0 },
      },
      opticalFlow: {
        timestamp: 0, valid: false,
        measurement: { flowX: 0, flowY: 0 },
      },
      cameraVIO: {
        timestamp: 0, valid: false,
        measurement: { position: v3Create(), quaternion: qCreate(), featureQuality: 0 },
      },
      uwb: null,
    };
  }

  /** Step bias random walks. Call once per physics tick. */
  stepBiases(dt: number): void {
    stepBiasRandomWalks(this.biasState, dt, this.rng, this.config);
  }

  readImu(state: DroneState, env: EnvironmentOutput, simTime: number): void {
    const out = readImu(state, this.params, env, this.biasState, this.config.imu, this.rng, simTime);
    if (!this.latest.imu) {
      this.latest.imu = { timestamp: 0, valid: false, measurement: { gyro: v3Create(), accel: v3Create() } };
    }
    this.latest.imu.timestamp = out.timestamp;
    this.latest.imu.valid = out.valid;
    this.latest.imu.measurement.gyro.set(out.measurement.gyro);
    this.latest.imu.measurement.accel.set(out.measurement.accel);
  }

  readMag(state: DroneState, env: EnvironmentOutput, simTime: number): void {
    const out = readMagnetometer(state, this.params, env, this.config.magnetometer, this.rng, simTime);
    if (!this.latest.magnetometer) {
      this.latest.magnetometer = { timestamp: 0, valid: false, measurement: { field: v3Create() } };
    }
    this.latest.magnetometer.timestamp = out.timestamp;
    this.latest.magnetometer.valid = out.valid;
    this.latest.magnetometer.measurement.field.set(out.measurement.field);
  }

  readBaro(state: DroneState, env: EnvironmentOutput, simTime: number): void {
    this.latest.barometer = readBarometer(state, env, this.biasState, this.config.barometer, this.rng, simTime);
  }

  readRange(state: DroneState, simTime: number): void {
    this.latest.rangefinder = readRangefinder(state, this.config.rangefinder, this.rng, this.worldGeo, simTime);
  }

  readFlow(state: DroneState, env: EnvironmentOutput, simTime: number): void {
    this.latest.opticalFlow = readOpticalFlow(state, env, this.config.opticalFlow, this.rng, simTime);
  }

  readCamera(state: DroneState, env: EnvironmentOutput, simTime: number): void {
    if (this.config.cameraVIO?.enabled) {
      const out = readCameraVIO(state, env, this.config.cameraVIO, this.rng, simTime, this.worldGeo, this.vioDriftState);
      if (!this.latest.cameraVIO) {
        this.latest.cameraVIO = {
          timestamp: 0, valid: false,
          measurement: { position: v3Create(), quaternion: qCreate(), featureQuality: 0 },
        };
      }
      this.latest.cameraVIO.timestamp = out.timestamp;
      this.latest.cameraVIO.valid = out.valid;
      this.latest.cameraVIO.measurement.position.set(out.measurement.position);
      this.latest.cameraVIO.measurement.quaternion.set(out.measurement.quaternion);
      this.latest.cameraVIO.measurement.featureQuality = out.measurement.featureQuality;
    }
  }

  readUWB(state: DroneState, anchors: import('@sim/core/types').UWBAnchorDef[], simTime: number): void {
    if (this.config.uwb?.enabled) {
      this.latest.uwb = readUWB(state, anchors, this.config.uwb, this.worldGeo, this.rng, simTime);
    }
  }

  /**
   * Read UWB with infrastructure anchors + inter-drone targets (truth positions).
   * Scene anchors and other drones are passed explicitly — no hidden config field.
   */
  readUWBWithDrones(
    state: DroneState,
    sceneAnchors: import('@sim/core/types').UWBAnchorDef[],
    otherDrones: { id: string; position: import('@sim/core/types').Vec3 }[],
    simTime: number,
  ): void {
    if (!this.config.uwb?.enabled) return;
    const combined = [
      ...sceneAnchors.map(a => ({ id: a.id, position: a.position })),
      ...otherDrones,
    ];
    this.latest.uwb = readUWB(state, combined as import('@sim/core/types').UWBAnchorDef[], this.config.uwb, this.worldGeo, this.rng, simTime);
  }

  /** Update sensor config with partial changes. Propagates to this drone's private config. */
  updateConfig(partial: Record<string, unknown>): void {
    for (const key of Object.keys(partial)) {
      if (key in this.config) {
        Object.assign((this.config as unknown as Record<string, Record<string, unknown>>)[key], partial[key]);
      }
    }
    this.clearDisabledReadings();
  }

  getConfig(): SensorSuiteConfig { return this.config; }

  getLatestReadings(): SensorReadings {
    return this.latest;
  }

  /** Clear cached readings for disabled sensors. Prevents stale data after config change. */
  clearDisabledReadings(): void {
    if (!this.config.imu.enabled) this.latest.imu = null;
    if (!this.config.magnetometer.enabled) this.latest.magnetometer = null;
    if (!this.config.barometer.enabled) this.latest.barometer = null;
    if (!this.config.rangefinder.enabled) this.latest.rangefinder = null;
    if (!this.config.opticalFlow.enabled) this.latest.opticalFlow = null;
    if (!this.config.cameraVIO?.enabled) this.latest.cameraVIO = null;
    if (!this.config.uwb?.enabled) this.latest.uwb = null;
  }

  reset(rng: DeterministicRNG, initialPosition?: Vec3): void {
    this.rng = rng;
    this.biasState = createSensorBiasState();
    resetSensorBiasState(this.biasState, this.config);
    this.vioDriftState = createVIODriftState(initialPosition ?? v3Create());
    this.latest = {
      imu: {
        timestamp: 0, valid: false,
        measurement: { gyro: v3Create(), accel: v3Create() },
      },
      magnetometer: {
        timestamp: 0, valid: false,
        measurement: { field: v3Create() },
      },
      barometer: {
        timestamp: 0, valid: false,
        measurement: { pressure: 0, altitude: 0 },
      },
      rangefinder: {
        timestamp: 0, valid: false,
        measurement: { range: 0 },
      },
      opticalFlow: {
        timestamp: 0, valid: false,
        measurement: { flowX: 0, flowY: 0 },
      },
      cameraVIO: {
        timestamp: 0, valid: false,
        measurement: { position: v3Create(), quaternion: qCreate(), featureQuality: 0 },
      },
      uwb: null,
    };
  }
}
