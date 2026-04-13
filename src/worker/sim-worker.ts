/**
 * Simulation Web Worker.
 * Phase 10: uses SimContext for both interactive and batch modes.
 */

import { SimContext } from '@sim/core/sim-context';
import { z } from 'zod';
import { simConfigSchema } from '@sim/core/config-schema';
import { defaultSimConfig } from '@sim/core/config-defaults';
import { computeSwarmSafetyMetrics } from '@sim/safety/safety-metrics';
import { computeTruthFormationRMS } from '@sim/swarm/formation-metrics';
import { ScenarioRunner } from '@sim/scenarios/scenario-runner';
import { type DroneAccum, newAccum, sampleDroneMetrics, rms, meanOf, maxOf } from '@sim/metrics/mission-accum';
import type { SimConfig } from '@sim/core/types';
import type {
  WorkerCommand, SwarmSnapshot, DroneSnapshot, SensorSnapshot,
  EnvironmentSnapshot, EstimateSnapshot, WorkerEvent, SubsystemStatusEntry,
  VisualMissionStep,
} from './worker-protocol';
import { getAllSubsystems } from '@sim/core/status-labels';

// Import registrations
import '@sim/sensors/sensors';
import '@sim/estimation/estimation';
import '@sim/control/guidance';
import '@sim/control/outer-loop';
import '@sim/control/inner-loop';
import '@sim/control/controller';
import '@sim/swarm/drone-instance';
import '@sim/swarm/swarm-manager';
import '@sim/communications/comm-system';
import '@sim/safety/safety';
import '@sim/scenarios/scenarios';
import '@sim/core/sim-context';

// ── Module state ──

let ctx: SimContext | null = null;
let config: SimConfig = defaultSimConfig();
let selectedDroneId = 0;

let running = false;
let accumulator = 0;
let lastWallTime = 0;
let timeScale = config.timeScale;
let maxSubSteps = config.maxSubSteps;

// ── Visual mission state ──
let missionActive = false;
let missionSteps: VisualMissionStep[] = [];
let missionStepIdx = 0;
let missionDuration = 0;
let missionStartStep = 0;
let missionWallStart = 0;

// ── Mission metrics accumulators ──
let missionAccums: DroneAccum[] = [];
let missionSampleInterval = 0;
let lastMissionSampleStep = 0;
let missionLaunchCentroid = [0, 0, 0];

// ── Flight log recorder ──
const LOG_RATE_HZ = 30;
let logLines: string[] = [];
let lastLogStep = 0;

function initFlightLog(): void {
  logLines = [
    'time,droneId,truthX,truthY,truthZ,estX,estY,estZ,velX,velY,velZ,' +
    'qW,qX,qY,qZ,mode,armed,destroyed,motor0,motor1,motor2,motor3,' +
    'featureQuality,commEnabled,neighborCount,formationActive'
  ];
  lastLogStep = 0;
}

function sampleFlightLog(): void {
  if (!ctx) return;
  const logInterval = Math.round(1 / (config.physicsDt * LOG_RATE_HZ));
  if (ctx.stepCount - lastLogStep < logInterval) return;
  lastLogStep = ctx.stepCount;

  const t = ctx.simTime.toFixed(3);
  for (const d of ctx.swarmManager.drones) {
    const p = d.state.position;
    const v = d.state.velocity;
    const q = d.state.quaternion;
    const est = d.ekf.getEstimate();
    const readings = d.sensorManager.getLatestReadings();
    const fq = readings.cameraVIO?.valid ? readings.cameraVIO.measurement.featureQuality : 0;
    logLines.push(
      `${t},${d.id},` +
      `${p[0].toFixed(4)},${p[1].toFixed(4)},${p[2].toFixed(4)},` +
      `${est.position[0].toFixed(4)},${est.position[1].toFixed(4)},${est.position[2].toFixed(4)},` +
      `${v[0].toFixed(4)},${v[1].toFixed(4)},${v[2].toFixed(4)},` +
      `${q[0].toFixed(4)},${q[1].toFixed(4)},${q[2].toFixed(4)},${q[3].toFixed(4)},` +
      `${d.controller.controlMode},${d.controller.armState},${d.destroyed},` +
      `${d.state.motorSpeeds[0].toFixed(1)},${d.state.motorSpeeds[1].toFixed(1)},${d.state.motorSpeeds[2].toFixed(1)},${d.state.motorSpeeds[3].toFixed(1)},` +
      `${fq.toFixed(3)},${d.commEnabled},${d.neighborEstimates.size},` +
      `${d.lastFormationState?.active ?? false}`
    );
  }
}

/**
 * Validate full config against schema after a partial merge.
 * Converts Float64Array fields to plain arrays for Zod compatibility.
 * Returns error message or null if valid.
 */
function validateConfigAfterMerge(): string | null {
  try {
    // Deep-clone config converting Float64Arrays to plain arrays for Zod
    const plain = JSON.parse(JSON.stringify(config));
    const result = simConfigSchema.safeParse(plain);
    if (!result.success) {
      return result.error.issues.map(i => `${i.path.join('.')}: ${i.message}`).join('; ');
    }
    return null;
  } catch (e) {
    return String(e);
  }
}

// ── Snapshot building ──

function postSubsystemStatus(): void {
  const entries: SubsystemStatusEntry[] = [];
  for (const [id, entry] of getAllSubsystems()) {
    entries.push({ id, status: entry.status, description: entry.description, demoOnly: entry.demoOnly });
  }
  post({ type: 'subsystem-status', entries });
}

function buildDroneSnapshot(d: import('@sim/swarm/drone-instance').DroneInstance): DroneSnapshot {
  const s = d.state;
  let T = 0;
  for (let i = 0; i < config.drone.numRotors; i++) T += config.drone.kT * s.motorSpeeds[i] * s.motorSpeeds[i];

  const readings = d.sensorManager.getLatestReadings();
  const sensorSnap: SensorSnapshot = {
    imu: null, mag: null, baro: null, range: null, flow: null, vio: null, uwb: null,
  };
  if (readings.imu?.valid) { const m = readings.imu.measurement; sensorSnap.imu = { valid: true, gyro: [m.gyro[0], m.gyro[1], m.gyro[2]], accel: [m.accel[0], m.accel[1], m.accel[2]] }; }
  if (readings.magnetometer?.valid) { const m = readings.magnetometer.measurement; sensorSnap.mag = { valid: true, field: [m.field[0], m.field[1], m.field[2]] }; }
  if (readings.barometer?.valid) { const m = readings.barometer.measurement; sensorSnap.baro = { valid: true, pressure: m.pressure, altitude: m.altitude }; }
  if (readings.rangefinder?.valid) sensorSnap.range = { valid: true, range: readings.rangefinder.measurement.range };
  if (readings.opticalFlow?.valid) { const m = readings.opticalFlow.measurement; sensorSnap.flow = { valid: true, flowX: m.flowX, flowY: m.flowY }; }
  if (readings.cameraVIO) { const m = readings.cameraVIO.measurement; sensorSnap.vio = { valid: readings.cameraVIO.valid, position: [m.position[0], m.position[1], m.position[2]], featureQuality: m.featureQuality }; }
  if (readings.uwb) { sensorSnap.uwb = readings.uwb.map(r => ({ anchorId: r.measurement.anchorId, range: r.measurement.range, valid: r.valid, isNLOS: r.measurement.isNLOS })); }

  const windSpeed = Math.sqrt(d.envOutput.wind[0] ** 2 + d.envOutput.wind[1] ** 2 + d.envOutput.wind[2] ** 2);
  const envSnap: EnvironmentSnapshot = { windSpeed, heightAboveGround: d.envOutput.heightAboveGround, groundEffectMultiplier: d.envOutput.groundEffectMultiplier, surfaceTextureQuality: d.envOutput.surfaceTextureQuality };

  const est = d.ekf.getEstimate(); const h = d.ekf.getHealth();
  const estimateSnap: EstimateSnapshot = {
    position: [est.position[0], est.position[1], est.position[2]],
    velocity: [est.velocity[0], est.velocity[1], est.velocity[2]],
    quaternion: [est.quaternion[0], est.quaternion[1], est.quaternion[2], est.quaternion[3]],
    gyroBias: [est.gyroBias[0], est.gyroBias[1], est.gyroBias[2]],
    accelBias: [est.accelBias[0], est.accelBias[1], est.accelBias[2]],
    baroBias: est.baroBias,
    health: { attitudeHealthy: h.attitude === 'healthy', verticalHealthy: h.vertical === 'healthy', aligned: h.aligned, attCovMax: h.attCovMax, vertCov: h.vertCov, horizCovMax: h.horizCovMax },
  };

  return {
    id: d.id,
    position: [s.position[0], s.position[1], s.position[2]],
    velocity: [s.velocity[0], s.velocity[1], s.velocity[2]],
    quaternion: [s.quaternion[0], s.quaternion[1], s.quaternion[2], s.quaternion[3]],
    angularVelocity: [s.angularVelocity[0], s.angularVelocity[1], s.angularVelocity[2]],
    motorSpeeds: [s.motorSpeeds[0], s.motorSpeeds[1], s.motorSpeeds[2], s.motorSpeeds[3]],
    motorCommands: [s.motorCommands[0], s.motorCommands[1], s.motorCommands[2], s.motorCommands[3]],
    totalThrust: T, sensors: sensorSnap, environment: envSnap,
    armState: d.controller.armState, controlMode: d.controller.controlMode,
    estimate: estimateSnap, commEnabled: d.commEnabled,
    formationState: d.lastFormationState ? { active: d.lastFormationState.active, role: d.lastFormationState.role, referenceHealthy: d.lastFormationState.referenceHealthy, neighborCount: d.lastFormationState.neighborCount } : undefined,
    safetyState: d.lastSafetyState ? { active: d.lastSafetyState.active, reason: d.lastSafetyState.reason, constraintCount: d.lastSafetyState.constraintCount, minSeparationCurrent: d.lastSafetyState.minSeparationCurrent } : undefined,
  };
}

function buildSnapshot(): SwarmSnapshot {
  if (!ctx) {
    const df = { enabled: false, mode: 'leader-follower', topology: 'line', spacing: 2, leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'hover', offsetFrame: 'world' };
    const dm = { minSeparation: Infinity, collisionCount: 0, avgSeparation: Infinity, safetyOverrideCount: 0, emergencyStopCount: 0 };
    const ds = { enabled: true, minSeparation: 0.5, orcaRadius: 0.3, orcaTimeHorizon: 3, maxSpeed: 5, minAltitude: 0.3, emergencyStopDistance: 0.3 };
    return { simTime: 0, stepCount: 0, selectedDroneId: 0, drones: [], commLinks: [], formation: df, safetyMetrics: dm, safety: ds, safetyEvents: [] };
  }
  const sm = ctx.swarmManager;
  const fmCfg = sm.formationManager['config'];
  return {
    simTime: ctx.simTime, stepCount: ctx.stepCount, selectedDroneId,
    drones: sm.drones.map(buildDroneSnapshot),
    commLinks: sm.commSystem.getLinks().map(l => ({ ...l, active: true })),
    formation: { enabled: fmCfg.enabled, mode: fmCfg.mode, topology: fmCfg.topology, spacing: fmCfg.spacing, leaderDroneId: fmCfg.leaderDroneId, consensusGain: fmCfg.consensusGain, leaderLossFallback: fmCfg.leaderLossFallback, offsetFrame: fmCfg.offsetFrame },
    safetyMetrics: computeSwarmSafetyMetrics(sm.drones, sm.safetyConfig),
    safety: { enabled: sm.safetyConfig.enabled, minSeparation: sm.safetyConfig.minSeparation, orcaRadius: sm.safetyConfig.orcaRadius, orcaTimeHorizon: sm.safetyConfig.orcaTimeHorizon, maxSpeed: sm.safetyConfig.maxSpeed, minAltitude: sm.safetyConfig.minAltitude, emergencyStopDistance: sm.safetyConfig.emergencyStopDistance },
    safetyEvents: sm.safetyEventLog.getRecent(10),
  };
}

function post(event: WorkerEvent): void { self.postMessage(event); }

// ── Frame loop ──

function scheduleNextFrame(): void { setTimeout(onFrame, 0); }

function onFrame(): void {
  if (!running || !ctx) return;
  const now = performance.now();
  const wallDt = (now - lastWallTime) / 1000;
  lastWallTime = now;
  accumulator += wallDt * timeScale;
  let steps = Math.floor(accumulator / config.physicsDt);
  steps = Math.min(steps, maxSubSteps);
  accumulator -= steps * config.physicsDt;
  for (let i = 0; i < steps; i++) {
    // Execute mission steps at the right sim time
    if (missionActive) {
      const missionTime = ctx.simTime - missionStartStep * config.physicsDt;
      while (missionStepIdx < missionSteps.length && missionSteps[missionStepIdx].time <= missionTime) {
        applyVisualMissionStep(missionSteps[missionStepIdx]);
        missionStepIdx++;
      }
      sampleFlightLog();
      // Sample metrics at controller rate
      if (ctx.stepCount - lastMissionSampleStep >= missionSampleInterval) {
        lastMissionSampleStep = ctx.stepCount;
        const sm = ctx.swarmManager;
        for (let di = 0; di < sm.drones.length; di++) {
          const d = sm.drones[di];
          if (d.destroyed) continue;
          const est = d.ekf.getEstimate();
          const guide = d.controller.evaluateGuidanceOnly(est);
          const safe = d.safeGuidance;
          const r = d.sensorManager.getLatestReadings();
          const innovs = d.ekf.getInnovationLogger().getRecent(1);
          sampleDroneMetrics(
            missionAccums[di],
            d.state.position, est.position, est.velocity, d.state.velocity,
            d.state.quaternion, est.quaternion,
            guide.positionDes, safe?.positionDes,
            d.state.motorSpeeds, config.drone.motorOmegaMax, config.drone.numRotors,
            ctx.simTime,
            {
              flowValid: r.opticalFlow?.valid ?? false,
              vioValid: r.cameraVIO?.valid ?? false,
              innovGated: innovs.length > 0 && innovs[0].gated,
              innovNorm: innovs.length > 0 ? innovs[0].innovationNorm : 0,
              hasInnov: innovs.length > 0,
            },
          );
        }
      }
      // Check if mission is done
      if (missionTime >= missionDuration) {
        missionActive = false;
        running = false;
        const snap = buildSnapshot();
        post({ type: 'state', snapshot: snap });
        // Send flight log CSV
        post({ type: 'mission-log', csv: logLines.join('\n') });
        // Aggregate real metrics
        const sm = ctx.swarmManager;
        const fmEnabled = sm.formationManager['config'].enabled;
        let formationRMS = NaN;
        if (fmEnabled) {
          try { formationRMS = computeTruthFormationRMS(sm.drones, (id) => sm.formationManager.getOffset(id), sm.formationManager.getLeaderId()); } catch {}
        }
        // Return drift: distance from launch centroid
        let endCx = 0, endCy = 0, endCz = 0, aliveN = 0;
        for (const d of sm.drones) { if (!d.destroyed) { endCx += d.state.position[0]; endCy += d.state.position[1]; endCz += d.state.position[2]; aliveN++; } }
        if (aliveN > 0) { endCx /= aliveN; endCy /= aliveN; endCz /= aliveN; }
        const returnDrift = Math.sqrt((endCx - missionLaunchCentroid[0]) ** 2 + (endCy - missionLaunchCentroid[1]) ** 2 + (endCz - missionLaunchCentroid[2]) ** 2);

        post({
          type: 'mission-complete',
          metrics: {
            collisionCount: snap.safetyMetrics.collisionCount,
            minSeparation: snap.safetyMetrics.minSeparation,
            avgSeparation: snap.safetyMetrics.avgSeparation,
            safetyOverrideCount: snap.safetyMetrics.safetyOverrideCount,
            emergencyStopCount: snap.safetyMetrics.emergencyStopCount,
            rmsExecutedTrackingError: meanOf(missionAccums.map(a => rms(a.executedTrackingSqSum, a.sampleCount))),
            rmsEstimationError: meanOf(missionAccums.map(a => rms(a.estimationSqSum, a.sampleCount))),
            rmsAltitudeError: meanOf(missionAccums.map(a => rms(a.altitudeSqSum, a.sampleCount))),
            maxHorizontalDrift: maxOf(missionAccums.map(a => a.maxHorizDrift)),
            formationRMS,
            returnDrift,
            packetDeliveryRate: sm.commSystem.sentCount > 0
              ? sm.commSystem.deliveredCount / sm.commSystem.sentCount : 1,
            realTimeFactor: missionDuration / Math.max(0.001, (performance.now() - missionWallStart) / 1000),
          },
        });
        return;
      }
    }
    ctx.stepOnce();
  }
  post({ type: 'state', snapshot: buildSnapshot() });
  if (ctx.stepCount % Math.round(1 / config.physicsDt) === 0) {
    const batch = ctx.telemetry.buildBatch();
    if (batch) post({ type: 'telemetry', batch });
  }
  scheduleNextFrame();
}

function applyVisualMissionStep(step: VisualMissionStep): boolean {
  if (!ctx) return false;
  const drones = step.droneId === 'all'
    ? ctx.swarmManager.drones
    : [ctx.swarmManager.getDrone(step.droneId as number)].filter(Boolean);

  for (const d of drones) {
    if (!d) continue;
    switch (step.action) {
      case 'arm':
        if (!d.ekf.isAligned()) d.ekf.bootstrapAligned();
        d.controller.forceArm();
        break;
      case 'hover': {
        const p = step.params?.position as number[];
        if (p) d.controller.setHover(new Float64Array(p), (step.params?.yaw as number) ?? 0);
        break;
      }
      case 'waypoint': {
        const p = step.params?.position as number[];
        if (p) d.controller.setWaypoint(new Float64Array(p), (step.params?.yaw as number) ?? 0, (step.params?.speed as number) ?? 1);
        break;
      }
      case 'set-drone-comm':
        d.commEnabled = (step.params?.commEnabled as boolean) ?? true;
        break;
      case 'formation-enable':
        return ctx.swarmManager.updateFormationConfig({ ...config.swarm.formation, enabled: true });
      case 'formation-disable':
        return ctx.swarmManager.updateFormationConfig({ ...config.swarm.formation, enabled: false });
      case 'set-environment': {
        Object.assign(config.environment, step.params);
        ctx.envManager.updateConfig(config.environment, config.drone.armLength);
        return true;
      }
      case 'set-sensor-config': {
        const partial = step.params as Record<string, unknown>;
        for (const key of Object.keys(partial)) {
          const cfg = d.sensorManager.getConfig() as unknown as Record<string, Record<string, unknown>>;
          if (key in cfg) Object.assign(cfg[key], partial[key]);
        }
        d.sensorManager.clearDisabledReadings();
        break;
      }
      case 'set-safety-config':
        Object.assign(ctx.swarmManager.safetyConfig, step.params);
        return true;
      case 'set-mission-plan': {
        const wps = step.params?.waypoints as { position: [number, number, number]; yaw: number; speed: number }[];
        if (wps) d.controller.setMissionPlan(wps);
        break;
      }
      case 'kill-drone':
        d.destroyed = true;
        d.commEnabled = false;
        d.controller.requestDisarm();
        for (let i = 0; i < d.state.motorCommands.length; i++) {
          d.state.motorCommands[i] = 0;
          d.state.motorSpeeds[i] = 0;
        }
        break;
      case 'inject-fault': {
        const type = step.params?.type as string;
        if (type === 'wind-gust') {
          config.environment.wind.meanSpeed = (step.params?.speed as number) ?? 3;
          ctx.envManager.updateConfig(config.environment, config.drone.armLength);
        } else if (type === 'sensor-dropout') {
          const rate = (step.params?.rate as number) ?? 0.5;
          const cfg = d.sensorManager.getConfig();
          if (cfg.magnetometer) cfg.magnetometer.dropoutProbability = rate;
          if (cfg.rangefinder) cfg.rangefinder.dropoutProbability = rate;
          if (cfg.opticalFlow) cfg.opticalFlow.dropoutProbability = rate;
          d.sensorManager.clearDisabledReadings();
        }
        break;
      }
    }
  }
  return true;
}

function initSim(cfg: SimConfig): void {
  config = cfg;
  timeScale = cfg.timeScale;
  maxSubSteps = cfg.maxSubSteps;
  accumulator = 0;
  selectedDroneId = 0;
  ctx = new SimContext(cfg);
}

// ── Message handler ──

self.onmessage = (e: MessageEvent) => {
  const msg = e.data as WorkerCommand;
  switch (msg.type) {
    case 'init':
      initSim(msg.config);
      post({ type: 'ready' });
      postSubsystemStatus();
      break;

    case 'start':
      if (!running) { running = true; lastWallTime = performance.now(); accumulator = 0; scheduleNextFrame(); }
      break;

    case 'pause':
      running = false;
      post({ type: 'status', running: false, simTime: ctx?.simTime ?? 0, stepCount: ctx?.stepCount ?? 0, rtFactor: 0 });
      break;

    case 'reset':
      running = false;
      config.seed = msg.seed ?? config.seed;
      initSim(config);
      post({ type: 'ready' });
      postSubsystemStatus();
      break;

    case 'step':
      if (ctx) { const count = msg.count ?? 1; ctx.stepN(count); post({ type: 'state', snapshot: buildSnapshot() }); }
      break;

    case 'set-timescale': timeScale = msg.value; break;

    case 'update-params':
    case 'set-drone-params': {
      if (!ctx) break;
      const snapshot = JSON.stringify(config.drone);
      const p = msg.params;
      for (const key of Object.keys(p)) {
        const val = p[key];
        if (key === 'dragCoeffRotorX') config.drone.dragCoeffRotor[0] = val as number;
        else if (key === 'dragCoeffRotorY') config.drone.dragCoeffRotor[1] = val as number;
        else if (key === 'dragCoeffRotorZ') config.drone.dragCoeffRotor[2] = val as number;
        else if (key in config.drone) (config.drone as unknown as Record<string, unknown>)[key] = val;
      }
      const err = validateConfigAfterMerge();
      if (err) { Object.assign(config.drone, JSON.parse(snapshot)); post({ type: 'error', message: `Drone params rejected: ${err}` }); }
      break;
    }

    case 'set-estimation-config': {
      if (!ctx) break;
      const snapshot = JSON.stringify(config.estimation);
      Object.assign(config.estimation, msg.config);
      const err = validateConfigAfterMerge();
      if (err) { Object.assign(config.estimation, JSON.parse(snapshot)); post({ type: 'error', message: `EKF config rejected: ${err}` }); }
      break;
    }

    case 'set-comm-config': {
      if (!ctx) break;
      const snapshot = JSON.stringify(config.swarm.communication);
      Object.assign(config.swarm.communication, msg.config);
      const err = validateConfigAfterMerge();
      if (err) {
        Object.assign(config.swarm.communication, JSON.parse(snapshot));
        post({ type: 'error', message: `Comm config rejected: ${err}` });
      } else {
        // Also update live CommSystem
        const commSys = ctx.swarmManager.commSystem as unknown as { config: Record<string, unknown> };
        Object.assign(commSys.config, msg.config);
      }
      break;
    }

    case 'set-motor-commands': {
      const d = ctx?.swarmManager.getDrone(msg.droneId);
      if (d) { for (let i = 0; i < config.drone.numRotors && i < msg.commands.length; i++) d.state.motorCommands[i] = msg.commands[i]; }
      break;
    }

    case 'set-environment-config': {
      if (!ctx) break;
      const snapshot = JSON.stringify(config.environment);
      Object.assign(config.environment, msg.config);
      const err = validateConfigAfterMerge();
      if (err) {
        Object.assign(config.environment, JSON.parse(snapshot));
        post({ type: 'error', message: `Environment config rejected: ${err}` });
      } else {
        ctx.envManager.updateConfig(config.environment, config.drone.armLength);
      }
      break;
    }

    case 'set-sensor-config': {
      if (!ctx) break;
      const snapshot = JSON.stringify(config.sensors);
      const partial = msg.config as Record<string, unknown>;
      for (const key of Object.keys(partial)) {
        if (key in config.sensors) Object.assign((config.sensors as unknown as Record<string, Record<string, unknown>>)[key], partial[key]);
      }
      const err = validateConfigAfterMerge();
      if (err) {
        Object.assign(config.sensors, JSON.parse(snapshot));
        post({ type: 'error', message: `Sensor config rejected: ${err}` });
      } else {
        // Propagate to each drone's private SensorManager config
        for (const d of ctx.swarmManager.drones) d.sensorManager.updateConfig(partial);
      }
      break;
    }

    case 'set-controller-mode': {
      const d = ctx?.swarmManager.getDrone(msg.droneId);
      if (d) {
        if (msg.mode === 'manual') d.controller.requestDisarm();
        else if (msg.mode === 'hover' && msg.params?.position) { const p = msg.params.position; d.controller.setHover(new Float64Array([p[0], p[1], p[2]]), msg.params.yaw ?? 0); }
        else if (msg.mode === 'waypoint' && msg.params?.position) { const p = msg.params.position; d.controller.setWaypoint(new Float64Array([p[0], p[1], p[2]]), msg.params.yaw ?? 0, msg.params.speed ?? 1.0); }
      }
      break;
    }

    case 'select-drone': selectedDroneId = msg.droneId; break;

    case 'set-formation-config':
      if (ctx) { const ok = ctx.swarmManager.updateFormationConfig(msg.config); if (!ok) post({ type: 'error', message: 'Formation preconditions not met' }); }
      break;

    case 'set-drone-comm': {
      const d = ctx?.swarmManager.getDrone(msg.droneId);
      if (d) d.commEnabled = msg.commEnabled;
      break;
    }

    case 'set-safety-config':
      if (ctx) Object.assign(ctx.swarmManager.safetyConfig, msg.config);
      break;

    case 'run-scenario': {
      const wasRunning = running; running = false;
      const runner = new ScenarioRunner();
      const result = runner.runSingle(msg.scenario, msg.scenario.config.seed);
      post({ type: 'scenario-result', result });
      if (wasRunning && ctx) { running = true; lastWallTime = performance.now(); accumulator = 0; scheduleNextFrame(); }
      break;
    }

    case 'run-batch': {
      const wasRunning = running; running = false;
      const runner = new ScenarioRunner();
      const results = runner.runBatch(msg.scenario, msg.seeds);
      post({ type: 'batch-complete', results });
      if (wasRunning && ctx) { running = true; lastWallTime = performance.now(); accumulator = 0; scheduleNextFrame(); }
      break;
    }

    case 'run-visual-mission': {
      if (!ctx) break;
      missionSteps = [...msg.steps].sort((a, b) => a.time - b.time);
      missionStepIdx = 0;
      missionDuration = msg.duration;
      missionStartStep = ctx.stepCount;
      missionActive = true;
      missionWallStart = performance.now();
      initFlightLog();
      // Init metrics accumulators
      missionAccums = ctx.swarmManager.drones.map(() => newAccum());
      missionSampleInterval = Math.round(1 / (config.physicsDt * config.commandRate));
      lastMissionSampleStep = 0;
      // Record launch centroid for return drift
      let cx = 0, cy = 0, cz = 0;
      for (const d of ctx.swarmManager.drones) { cx += d.state.position[0]; cy += d.state.position[1]; cz += d.state.position[2]; }
      const n = ctx.swarmManager.drones.length;
      missionLaunchCentroid = [cx / n, cy / n, cz / n];
      // Start the sim loop
      running = true;
      lastWallTime = performance.now();
      accumulator = 0;
      scheduleNextFrame();
      break;
    }

    case 'stop-visual-mission': {
      missionActive = false;
      running = false;
      break;
    }
  }
};

// No auto-init.
