/**
 * SimContext: reusable simulation session.
 * Encapsulates all sim state (scheduler, environment, swarm, telemetry).
 * Used by both the interactive worker and the ScenarioRunner.
 */

import { DeterministicScheduler } from './scheduler';
import { DeterministicRNG } from './rng';
import { createDefaultEnv } from './config-defaults';
import { TelemetryCollector } from '@sim/metrics/telemetry';
import { EnvironmentManager } from '@sim/environment/environment-manager';
import { SwarmManager } from '@sim/swarm/swarm-manager';
import { computeSwarmSafetyMetrics } from '@sim/safety/safety-metrics';
import type { SimConfig, Vec3 } from './types';
import type { SwarmSnapshot, DroneSnapshot, SensorSnapshot, EnvironmentSnapshot, EstimateSnapshot } from '@worker/worker-protocol';
import { v3Create } from '@lib/math';
import { registerSubsystem } from './status-labels';

registerSubsystem('sim-context', 'experimental', 'Reusable simulation session');

export class SimContext {
  readonly config: SimConfig;
  readonly scheduler: DeterministicScheduler;
  readonly envManager: EnvironmentManager;
  readonly swarmManager: SwarmManager;
  readonly telemetry: TelemetryCollector;

  constructor(config: SimConfig) {
    this.config = config;
    const rng = new DeterministicRNG(config.seed);
    this.scheduler = new DeterministicScheduler(config.physicsDt);
    this.telemetry = new TelemetryCollector();

    // Environment RNG fork
    for (let i = 0; i < 64; i++) rng.nextU32();
    const envRng = rng.clone();
    this.envManager = new EnvironmentManager(config.environment, envRng, config.drone.armLength);

    // Expand positions
    const positions = SimContext.expandPositions(config);

    // SwarmManager
    this.swarmManager = new SwarmManager(
      positions, config.drone, config.sensors, config.estimation,
      config.swarm.communication, config.swarm.formation,
      { ...config.swarm.safety, noFlyZones: (config.swarm.safety.noFlyZones ?? []).map(z => ({
        min: new Float64Array(z.min), max: new Float64Array(z.max),
      })) },
      this.envManager.getWorldGeometry(), rng,
    );

    this.setupTasks();
  }

  stepOnce(): void { this.scheduler.step(); }

  stepN(n: number): void {
    for (let i = 0; i < n; i++) this.scheduler.step();
  }

  get simTime(): number { return this.scheduler.simTime; }
  get stepCount(): number { return this.scheduler.stepCount; }

  private setupTasks(): void {
    const sc = this.config.sensors;
    const sm = this.swarmManager;
    const env = this.envManager;
    const cfg = this.config;

    // Physics
    this.scheduler.addTask('physics-all', cfg.physicsDt, 1, (dt, simTime) => {
      sm.stepPhysics(env, dt, simTime);
    });
    this.scheduler.addTask('bias-all', cfg.physicsDt, 2, (dt) => { sm.stepBiases(dt); });

    // Sensors + EKF
    if (sc.imu.enabled) {
      this.scheduler.addTask('sensors-imu-all', 1 / sc.imu.rate, 10, (_dt, simTime) => { sm.stepSensorIMU(simTime); });
      this.scheduler.addTask('ekf-propagate-all', 1 / sc.imu.rate, 11, (dt) => { sm.stepEKFPropagate(dt); });
    }
    if (sc.magnetometer.enabled) {
      this.scheduler.addTask('sensors-mag-all', 1 / sc.magnetometer.rate, 20, (_dt, simTime) => { sm.stepSensorMag(simTime); });
      this.scheduler.addTask('ekf-mag-all', 1 / sc.magnetometer.rate, 21, () => { sm.stepEKFMag(); });
    }
    if (sc.barometer.enabled) {
      this.scheduler.addTask('sensors-baro-all', 1 / sc.barometer.rate, 30, (_dt, simTime) => { sm.stepSensorBaro(simTime); });
      this.scheduler.addTask('ekf-baro-all', 1 / sc.barometer.rate, 31, () => { sm.stepEKFBaro(); });
    }
    if (sc.rangefinder.enabled) {
      this.scheduler.addTask('sensors-range-all', 1 / sc.rangefinder.rate, 40, (_dt, simTime) => { sm.stepSensorRange(simTime); });
      this.scheduler.addTask('ekf-range-all', 1 / sc.rangefinder.rate, 41, () => { sm.stepEKFRange(); });
    }
    if (sc.opticalFlow.enabled) {
      this.scheduler.addTask('sensors-flow-all', 1 / sc.opticalFlow.rate, 50, (_dt, simTime) => { sm.stepSensorFlow(simTime); });
      this.scheduler.addTask('ekf-flow-all', 1 / sc.opticalFlow.rate, 51, () => { sm.stepEKFFlow(); });
    }

    // Estimate publication + comm
    const pubRate = cfg.swarm.communication.estimatePublishRate;
    this.scheduler.addTask('estimate-publish-all', 1 / pubRate, 54, (_dt, simTime) => { sm.stepEstimatePublish(simTime); });
    this.scheduler.addTask('comm-step', cfg.physicsDt, 55, (_dt, simTime) => {
      sm.commSystem.updateNeighborGraph(sm.getPositionsMap());
      sm.commSystem.deliver(simTime);
      sm.processDeliveredMessages(simTime);
    });

    // VIO + UWB
    if (sc.cameraVIO?.enabled) {
      this.scheduler.addTask('sensors-camera-all', 1 / sc.cameraVIO.rate, 56, (_dt, simTime) => { sm.stepSensorCamera(simTime); });
      this.scheduler.addTask('ekf-vio-pos-all', 1 / sc.cameraVIO.rate, 57, () => { sm.stepEKFVIOPos(); });
      this.scheduler.addTask('ekf-vio-att-all', 1 / sc.cameraVIO.rate, 58, () => { sm.stepEKFVIOAtt(); });
    }
    if (sc.uwb?.enabled) {
      this.scheduler.addTask('sensors-uwb-all', 1 / sc.uwb.rate, 59, (_dt, simTime) => {
        sm.stepSensorUWB(cfg.environment.scene.uwbAnchors ?? [], simTime);
      });
      this.scheduler.addTask('ekf-uwb-all', 1 / sc.uwb.rate, 60, (_dt, simTime) => {
        sm.stepEKFUWB(cfg.environment.scene.uwbAnchors ?? [], simTime);
      });
    }

    // Formation + safety + controller
    this.scheduler.addTask('formation-step', 1 / cfg.commandRate, 62, (_dt, simTime) => { sm.stepFormation(simTime); });
    this.scheduler.addTask('safety-guidance', 1 / cfg.commandRate, 64, (_dt, simTime) => { sm.stepSafetyGuidance(simTime); });
    this.scheduler.addTask('controller-all', 1 / cfg.commandRate, 65, (_dt, simTime) => { sm.stepControllers(simTime); });

    // Telemetry
    this.scheduler.addTask('telemetry', 1 / cfg.telemetryRate, 70, (_dt, simTime) => {
      const d = sm.drones[0];
      if (d) this.telemetry.record(d.state, simTime, cfg.drone.kT);
    });
  }

  static expandPositions(cfg: SimConfig): Vec3[] {
    const sw = cfg.swarm;
    if (sw.initialPattern === 'explicit' && sw.initialPositions) {
      return sw.initialPositions.map(p => v3Create(p[0], p[1], p[2]));
    }
    const positions: Vec3[] = [];
    const c = sw.patternCenter;
    const s = sw.patternSpacing;
    const n = sw.droneCount;
    if (sw.initialPattern === 'line') {
      for (let i = 0; i < n; i++) positions.push(v3Create(c[0] + (i - (n - 1) / 2) * s, c[1], c[2]));
    } else if (sw.initialPattern === 'grid') {
      const cols = Math.ceil(Math.sqrt(n));
      for (let i = 0; i < n; i++) {
        const row = Math.floor(i / cols), col = i % cols;
        positions.push(v3Create(c[0] + (col - (cols - 1) / 2) * s, c[1] + (row - (cols - 1) / 2) * s, c[2]));
      }
    } else if (sw.initialPattern === 'circle') {
      for (let i = 0; i < n; i++) {
        const angle = (2 * Math.PI * i) / n;
        const r = n === 1 ? 0 : s;
        positions.push(v3Create(c[0] + r * Math.cos(angle), c[1] + r * Math.sin(angle), c[2]));
      }
    } else {
      positions.push(v3Create(0, 0, 0));
    }
    return positions;
  }
}
