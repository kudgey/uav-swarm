/**
 * ScenarioRunner: executes scenarios using SimContext.
 * STATUS: experimental
 */

import { SimContext } from '@sim/core/sim-context';
import { DeterministicRNG } from '@sim/core/rng';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import { computeSwarmSafetyMetrics } from '@sim/safety/safety-metrics';
import { computeTruthFormationRMS } from '@sim/swarm/formation-metrics';
import { v3Len, v3Sub, v3Create } from '@lib/math';
import type { SimConfig } from '@sim/core/types';
import type {
  ScenarioDefinition, ScenarioRunResult, ScenarioMetrics,
  MissionStep, AcceptanceCriteria, BatchAcceptanceCriteria,
  CpuTimeBySubsystem,
} from './scenario-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('scenario-runner', 'experimental', 'Scenario executor with full metrics');

const _diff = v3Create();

// ── Per-drone accumulators ──
interface DroneAccum {
  missionTrackingSqSum: number; executedTrackingSqSum: number;
  estimationSqSum: number; altitudeSqSum: number; maxAltErr: number;
  attitudeSqSum: number; velocitySqSum: number;
  maxHorizDrift: number; saturatedTicks: number;
  flowValidCount: number; vioValidCount: number;
  innovGatedCount: number; innovTotalCount: number;
  maxInnovNorm: number; sampleCount: number;
  uwbRangeSqSum: number; uwbRangeCount: number;
  biasSqSum: number;
  driftSamples: { t: number; d: number }[];
}

function newAccum(): DroneAccum {
  return { missionTrackingSqSum: 0, executedTrackingSqSum: 0, estimationSqSum: 0,
    altitudeSqSum: 0, maxAltErr: 0, attitudeSqSum: 0, velocitySqSum: 0,
    maxHorizDrift: 0, saturatedTicks: 0, flowValidCount: 0, vioValidCount: 0,
    innovGatedCount: 0, innovTotalCount: 0, maxInnovNorm: 0, sampleCount: 0,
    uwbRangeSqSum: 0, uwbRangeCount: 0, biasSqSum: 0, driftSamples: [] };
}

// ── Recovery tracking ──
interface RecoveryTracker {
  triggered: boolean;
  belowSince: number;    // simTime when error first dropped below threshold
  recoveredAt: number;   // NaN until recovered
}


export class ScenarioRunner {
  runSingle(scenario: ScenarioDefinition, seed: number): ScenarioRunResult {
    const config = JSON.parse(JSON.stringify(scenario.config)) as SimConfig;
    config.seed = seed;

    const ctx = new SimContext(config);
    const stepsPerSecond = Math.round(1 / config.physicsDt);
    const totalSteps = Math.round(scenario.duration * stepsPerSecond);
    const sortedMissions = [...scenario.mission.steps].sort((a, b) => a.time - b.time);
    let missionIdx = 0;

    // Pre-fill hover trim
    const omegaTrim = Math.sqrt(config.drone.mass * GRAVITY_MPS2 / (config.drone.numRotors * config.drone.kT));
    for (const d of ctx.swarmManager.drones) {
      for (let i = 0; i < config.drone.numRotors; i++) {
        d.state.motorSpeeds[i] = omegaTrim; d.state.motorCommands[i] = omegaTrim;
      }
    }

    // Per-drone accumulators
    const accums = ctx.swarmManager.drones.map(() => newAccum());
    const controllerRate = Math.round(stepsPerSecond / config.commandRate);
    const cpuTime: CpuTimeBySubsystem = { physics: 0, sensors: 0, estimation: 0, communication: 0, safety: 0, control: 0, total: 0 };

    // Per-task CPU timing via scheduler callback
    ctx.scheduler.onTaskTiming = (taskId: string, durationMs: number) => {
      if (taskId.startsWith('physics') || taskId.startsWith('bias')) cpuTime.physics += durationMs;
      else if (taskId.startsWith('sensors')) cpuTime.sensors += durationMs;
      else if (taskId.startsWith('ekf')) cpuTime.estimation += durationMs;
      else if (taskId.startsWith('comm') || taskId.startsWith('estimate-publish')) cpuTime.communication += durationMs;
      else if (taskId.startsWith('safety')) cpuTime.safety += durationMs;
      else if (taskId.startsWith('controller') || taskId.startsWith('formation')) cpuTime.control += durationMs;
    };

    const startWall = performance.now();

    // Recovery tracking
    const recovery: RecoveryTracker = { triggered: false, belowSince: NaN, recoveredAt: NaN };
    const recSpec = scenario.recoverySpec;

    for (let step = 0; step < totalSteps; step++) {
      const simTime = step * config.physicsDt;

      while (missionIdx < sortedMissions.length && sortedMissions[missionIdx].time <= simTime) {
        this.applyMissionStep(ctx, sortedMissions[missionIdx]);
        missionIdx++;
      }

      // Instrumented step (scheduler fires all tasks in priority order)
      ctx.stepOnce();

      // Sample metrics at controller rate
      if (step % controllerRate === 0) {
        this.sampleMetrics(ctx, accums, simTime);

        // Recovery tracking
        if (recSpec && simTime >= recSpec.triggerTime && isNaN(recovery.recoveredAt)) {
          recovery.triggered = true;
          const errorNow = this.getRecoveryMetric(ctx, recSpec.metric);
          if (errorNow < recSpec.threshold) {
            if (isNaN(recovery.belowSince)) recovery.belowSince = simTime;
            else if (simTime - recovery.belowSince >= recSpec.holdTime) recovery.recoveredAt = recovery.belowSince - recSpec.triggerTime;
          } else {
            recovery.belowSince = NaN;
          }
        }
      }
    }

    const wallTime = (performance.now() - startWall) / 1000;
    cpuTime.total = wallTime * 1000;
    const metrics = this.aggregateMetrics(ctx, accums, scenario, wallTime, cpuTime, recovery);
    const { passed, failedCriteria } = this.evaluate(metrics, scenario.acceptanceCriteria);

    return { seed, scenarioName: scenario.name, metrics, passed, failedCriteria, duration: scenario.duration };
  }

  private getRecoveryMetric(ctx: SimContext, metric: string): number {
    const sm = ctx.swarmManager;
    if (metric === 'positionErrorNorm') {
      let maxErr = 0;
      for (const d of sm.drones) {
        const est = d.ekf.getEstimate();
        const mRef = d.controller.evaluateGuidanceOnly(est);
        const exRef = d.safeGuidance ?? mRef;
        v3Sub(_diff, exRef.positionDes, d.state.position);
        const err = v3Len(_diff);
        if (err > maxErr) maxErr = err;
      }
      return maxErr;
    }
    if (metric === 'altitudeError') {
      let maxErr = 0;
      for (const d of sm.drones) {
        const est = d.ekf.getEstimate();
        const mRef = d.controller.evaluateGuidanceOnly(est);
        const exRef = d.safeGuidance ?? mRef;
        const err = Math.abs(-d.state.position[2] - (-exRef.positionDes[2]));
        if (err > maxErr) maxErr = err;
      }
      return maxErr;
    }
    if (metric === 'formationRMS') {
      return computeTruthFormationRMS(sm.drones, (id) => sm.formationManager.getOffset(id), sm.formationManager.getLeaderId());
    }
    return Infinity;
  }

  runBatch(scenario: ScenarioDefinition, seeds?: number[]): ScenarioRunResult[] {
    const seedList = seeds ?? scenario.monteCarloRanges?.seeds ?? [scenario.config.seed];
    const results: ScenarioRunResult[] = [];

    for (const seed of seedList) {
      const modifiedScenario = this.applyMCParams(scenario, seed);
      results.push(this.runSingle(modifiedScenario, seed));
    }

    return results;
  }

  evaluateBatch(results: ScenarioRunResult[], criteria?: BatchAcceptanceCriteria): { passed: boolean; failedCriteria: string[] } {
    if (!criteria) return { passed: true, failedCriteria: [] };
    const failed: string[] = [];
    const passRate = criteria.passRate ?? 1.0;
    const passedCount = results.filter(r => r.passed).length;
    if (passedCount / results.length < passRate) failed.push(`passRate (${(passedCount / results.length).toFixed(2)} < ${passRate})`);

    const drifts = results.map(r => r.metrics.maxHorizontalDrift);
    if (criteria.meanMaxHorizontalDrift !== undefined) {
      const mean = drifts.reduce((s, v) => s + v, 0) / drifts.length;
      if (mean > criteria.meanMaxHorizontalDrift) failed.push('meanMaxHorizontalDrift');
    }
    if (criteria.meanFormationRMS !== undefined) {
      const vals = results.map(r => r.metrics.formationRMS).filter(v => isFinite(v));
      if (vals.length > 0) {
        const m = vals.reduce((s, v) => s + v, 0) / vals.length;
        if (m > criteria.meanFormationRMS) failed.push('meanFormationRMS');
      }
    }
    if (criteria.p95MaxHorizontalDrift !== undefined) {
      const sorted = [...drifts].sort((a, b) => a - b);
      const p95Idx = Math.min(sorted.length - 1, Math.floor(sorted.length * 0.95));
      if (sorted[p95Idx] > criteria.p95MaxHorizontalDrift) failed.push('p95MaxHorizontalDrift');
    }
    if (criteria.meanRmsExecutedTrackingError !== undefined) {
      const m = results.reduce((s, r) => s + r.metrics.rmsExecutedTrackingError, 0) / results.length;
      if (m > criteria.meanRmsExecutedTrackingError) failed.push('meanRmsExecutedTrackingError');
    }
    if (criteria.meanRmsAltitudeError !== undefined) {
      const m = results.reduce((s, r) => s + r.metrics.rmsAltitudeError, 0) / results.length;
      if (m > criteria.meanRmsAltitudeError) failed.push('meanRmsAltitudeError');
    }

    return { passed: failed.length === 0, failedCriteria: failed };
  }

  // ── Metrics sampling ──

  private sampleMetrics(ctx: SimContext, accums: DroneAccum[], simTime: number): void {
    const sm = ctx.swarmManager;
    for (let i = 0; i < sm.drones.length; i++) {
      const d = sm.drones[i];
      const a = accums[i];
      const truth = d.state.position;
      const est = d.ekf.getEstimate();
      const estPos = est.position;

      // Mission and executed references
      const missionRef = d.controller.evaluateGuidanceOnly(est);
      const executedRef = d.safeGuidance ?? missionRef;

      // Tracking errors
      v3Sub(_diff, missionRef.positionDes, truth);
      a.missionTrackingSqSum += v3Len(_diff) ** 2;
      v3Sub(_diff, executedRef.positionDes, truth);
      a.executedTrackingSqSum += v3Len(_diff) ** 2;

      // Estimation error
      v3Sub(_diff, estPos, truth);
      a.estimationSqSum += v3Len(_diff) ** 2;

      // Horizontal drift
      const horizDrift = Math.sqrt((_diff[0]) ** 2 + (_diff[1]) ** 2);
      if (horizDrift > a.maxHorizDrift) a.maxHorizDrift = horizDrift;

      // Altitude error
      const altErr = Math.abs(-truth[2] - (-executedRef.positionDes[2]));
      a.altitudeSqSum += altErr ** 2;
      if (altErr > a.maxAltErr) a.maxAltErr = altErr;

      // Attitude error (simplified: use quaternion w component)
      const qEst = est.quaternion;
      const qTruth = d.state.quaternion;
      const dotQ = Math.abs(qEst[0] * qTruth[0] + qEst[1] * qTruth[1] + qEst[2] * qTruth[2] + qEst[3] * qTruth[3]);
      const attErr = 2 * Math.acos(Math.min(1, dotQ));
      a.attitudeSqSum += attErr ** 2;

      // Velocity error
      v3Sub(_diff, est.velocity, d.state.velocity);
      a.velocitySqSum += v3Len(_diff) ** 2;

      // Saturation
      for (let m = 0; m < ctx.config.drone.numRotors; m++) {
        if (d.state.motorSpeeds[m] >= ctx.config.drone.motorOmegaMax * 0.99) { a.saturatedTicks++; break; }
      }

      // Sensor validity
      const readings = d.sensorManager.getLatestReadings();
      if (readings.opticalFlow?.valid) a.flowValidCount++;
      if (readings.cameraVIO?.valid) a.vioValidCount++;

      // Innovation
      const innovs = d.ekf.getInnovationLogger().getRecent(1);
      if (innovs.length > 0) {
        a.innovTotalCount++;
        if (innovs[0].gated) a.innovGatedCount++;
        if (innovs[0].innovationNorm > a.maxInnovNorm) a.maxInnovNorm = innovs[0].innovationNorm;
      }

      // UWB range error: compare each UWB measurement vs truth distance
      if (readings.uwb) {
        for (const r of readings.uwb) {
          if (!r.valid) continue;
          // Find truth distance to anchor/drone
          const anchors = ctx.config.environment.scene.uwbAnchors ?? [];
          const anchor = anchors.find(an => an.id === r.measurement.anchorId);
          if (anchor) {
            const dx = truth[0] - anchor.position[0], dy = truth[1] - anchor.position[1], dz = truth[2] - anchor.position[2];
            const truthDist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            a.uwbRangeSqSum += (r.measurement.range - truthDist) ** 2;
            a.uwbRangeCount++;
          } else {
            // Inter-drone: parse drone-N
            const match = r.measurement.anchorId.match(/^drone-(\d+)$/);
            if (match) {
              const neighbor = sm.getDrone(parseInt(match[1], 10));
              if (neighbor) {
                v3Sub(_diff, truth, neighbor.state.position);
                const truthDist = v3Len(_diff);
                a.uwbRangeSqSum += (r.measurement.range - truthDist) ** 2;
                a.uwbRangeCount++;
              }
            }
          }
        }
      }

      // Bias estimation error: compare estimated bias vs truth bias from sensor bias state
      const biasState = (d.sensorManager as unknown as { biasState: { gyroBias: Float64Array; accelBias: Float64Array } }).biasState;
      if (biasState) {
        const estBg = est.gyroBias; const estBa = est.accelBias;
        const bgErr = Math.sqrt((estBg[0] - biasState.gyroBias[0]) ** 2 + (estBg[1] - biasState.gyroBias[1]) ** 2 + (estBg[2] - biasState.gyroBias[2]) ** 2);
        const baErr = Math.sqrt((estBa[0] - biasState.accelBias[0]) ** 2 + (estBa[1] - biasState.accelBias[1]) ** 2 + (estBa[2] - biasState.accelBias[2]) ** 2);
        a.biasSqSum += (bgErr + baErr) ** 2;
      }

      // Drift rate samples (every 10th sample to keep array small)
      if (a.sampleCount % 10 === 0) {
        a.driftSamples.push({ t: simTime, d: horizDrift });
      }

      a.sampleCount++;
    }
  }

  private aggregateMetrics(ctx: SimContext, accums: DroneAccum[], scenario: ScenarioDefinition, wallTime: number, cpuTime: CpuTimeBySubsystem, recovery: RecoveryTracker): ScenarioMetrics {
    const sm = ctx.swarmManager;
    const safetyMetrics = computeSwarmSafetyMetrics(sm.drones, sm.safetyConfig);
    const n = accums.length;

    const rms = (sqSum: number, count: number) => count > 0 ? Math.sqrt(sqSum / count) : 0;
    const mean = (vals: number[]) => vals.length > 0 ? vals.reduce((s, v) => s + v, 0) / vals.length : 0;
    const max = (vals: number[]) => vals.length > 0 ? Math.max(...vals) : 0;

    return {
      collisionCount: safetyMetrics.collisionCount,
      minSeparation: safetyMetrics.minSeparation,
      rmsMissionTrackingError: mean(accums.map(a => rms(a.missionTrackingSqSum, a.sampleCount))),
      rmsExecutedTrackingError: mean(accums.map(a => rms(a.executedTrackingSqSum, a.sampleCount))),
      rmsEstimationError: mean(accums.map(a => rms(a.estimationSqSum, a.sampleCount))),
      maxHorizontalDrift: max(accums.map(a => a.maxHorizDrift)),
      xyDriftRate: (() => {
        // Linear fit of drift vs time from sampled data
        const allSamples = accums.flatMap(a => a.driftSamples);
        if (allSamples.length < 2) return 0;
        const n2 = allSamples.length;
        const sumT = allSamples.reduce((s, p) => s + p.t, 0);
        const sumD = allSamples.reduce((s, p) => s + p.d, 0);
        const sumTD = allSamples.reduce((s, p) => s + p.t * p.d, 0);
        const sumT2 = allSamples.reduce((s, p) => s + p.t * p.t, 0);
        const denom = n2 * sumT2 - sumT * sumT;
        return denom > 0 ? (n2 * sumTD - sumT * sumD) / denom : 0;
      })(),
      rmsAltitudeError: mean(accums.map(a => rms(a.altitudeSqSum, a.sampleCount))),
      maxAltitudeError: max(accums.map(a => a.maxAltErr)),
      saturationTimeFraction: max(accums.map(a => a.sampleCount > 0 ? a.saturatedTicks / a.sampleCount : 0)),
      packetDeliveryRate: sm.commSystem.sentCount > 0
        ? sm.commSystem.deliveredCount / sm.commSystem.sentCount : 1,
      avgCommLatency: sm.commSystem.deliveredCount > 0
        ? sm.commSystem.totalLatency / sm.commSystem.deliveredCount : 0,
      formationRMS: sm.formationManager['config'].enabled
        ? computeTruthFormationRMS(sm.drones, (id) => sm.formationManager.getOffset(id), sm.formationManager.getLeaderId())
        : NaN,
      innovationGatedFraction: mean(accums.map(a => a.innovTotalCount > 0 ? a.innovGatedCount / a.innovTotalCount : 0)),
      flowValidFraction: mean(accums.map(a => a.sampleCount > 0 ? a.flowValidCount / a.sampleCount : 0)),
      vioValidFraction: mean(accums.map(a => a.sampleCount > 0 ? a.vioValidCount / a.sampleCount : 0)),
      uwbRangeRmse: (() => {
        let totalSqSum = 0; let totalCount = 0;
        for (const a of accums) { totalSqSum += a.uwbRangeSqSum; totalCount += a.uwbRangeCount; }
        return totalCount > 0 ? Math.sqrt(totalSqSum / totalCount) : 0;
      })(),
      neighborEstimateError: (() => {
        let totalErr = 0; let count = 0;
        for (const d of sm.drones) {
          for (const [nid, est] of d.neighborEstimates) {
            const neighbor = sm.getDrone(nid);
            if (!neighbor) continue;
            v3Sub(_diff, est.position, neighbor.state.position);
            totalErr += v3Len(_diff);
            count++;
          }
        }
        return count > 0 ? totalErr / count : 0;
      })(),
      maxInnovationNorm: max(accums.map(a => a.maxInnovNorm)),
      rmsAttitudeError: mean(accums.map(a => rms(a.attitudeSqSum, a.sampleCount))),
      rmsVelocityError: mean(accums.map(a => rms(a.velocitySqSum, a.sampleCount))),
      rmsBiasEstimationError: mean(accums.map(a => rms(a.biasSqSum, a.sampleCount))),
      realTimeFactor: scenario.duration / Math.max(0.001, wallTime),
      cpuTimeBySubsystem: cpuTime,
      recoveryTime: isNaN(recovery.recoveredAt) ? NaN : recovery.recoveredAt,
    };
  }

  // ── Mission step execution ──

  private applyMissionStep(ctx: SimContext, step: MissionStep): void {
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
        case 'set-sensor-config': {
          // Per-drone sensor config mutation
          const partial = step.params as Record<string, unknown>;
          for (const key of Object.keys(partial)) {
            const cfg = d.sensorManager.getConfig() as unknown as Record<string, Record<string, unknown>>;
            if (key in cfg) Object.assign(cfg[key], partial[key]);
          }
          d.sensorManager.clearDisabledReadings();
          break;
        }
        case 'set-environment': {
          const partial = step.params as Record<string, unknown>;
          Object.assign(ctx.config.environment, partial);
          ctx.envManager.updateConfig(ctx.config.environment, ctx.config.drone.armLength);
          return; // global, don't iterate per-drone
        }
        case 'set-safety-config': {
          Object.assign(ctx.swarmManager.safetyConfig, step.params);
          return;
        }
        case 'formation-enable':
          ctx.swarmManager.updateFormationConfig({ ...ctx.config.swarm.formation, enabled: true });
          return;
        case 'formation-disable':
          ctx.swarmManager.updateFormationConfig({ ...ctx.config.swarm.formation, enabled: false });
          return;
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
        case 'inject-fault':
          this.applyFault(ctx, step.params ?? {}, d);
          break;
      }
    }
  }

  private applyFault(ctx: SimContext, params: Record<string, unknown>, drone?: import('@sim/swarm/drone-instance').DroneInstance): void {
    const type = params.type as string;
    switch (type) {
      case 'wind-gust':
        ctx.config.environment.wind.meanSpeed = (params.speed as number) ?? 3;
        ctx.envManager.updateConfig(ctx.config.environment, ctx.config.drone.armLength);
        break;
      case 'comm-delay-spike':
        if (params.dropRate !== undefined)
          (ctx.swarmManager.commSystem as unknown as { config: { dropRate: number } }).config.dropRate = params.dropRate as number;
        if (params.latency !== undefined)
          (ctx.swarmManager.commSystem as unknown as { config: { latencyMean: number } }).config.latencyMean = params.latency as number;
        break;
      case 'mag-anomaly':
        // Add a strong local anomaly near the drone
        ctx.config.environment.magneticField.anomalies.push({
          position: new Float64Array([5, 5, -2]),
          radius: 10,
          strength: new Float64Array([50e-6, 50e-6, 0]),
        });
        ctx.envManager.updateConfig(ctx.config.environment, ctx.config.drone.armLength);
        break;
      case 'sensor-dropout':
        if (drone) {
          const cfg = drone.sensorManager.getConfig();
          if (cfg.magnetometer) cfg.magnetometer.dropoutProbability = (params.rate as number) ?? 0.5;
          if (cfg.rangefinder) cfg.rangefinder.dropoutProbability = (params.rate as number) ?? 0.5;
          if (cfg.opticalFlow) cfg.opticalFlow.dropoutProbability = (params.rate as number) ?? 0.5;
          drone.sensorManager.clearDisabledReadings();
        }
        break;
      case 'uwb-nlos':
        // Increase UWB packet loss as proxy for NLOS degradation
        if (ctx.config.sensors.uwb) {
          ctx.config.sensors.uwb.packetLossProbability = (params.probability as number) ?? 0.5;
        }
        break;
    }
  }

  // ── MC parameter sampling ──

  private applyMCParams(scenario: ScenarioDefinition, seed: number): ScenarioDefinition {
    const ranges = scenario.monteCarloRanges;
    if (!ranges) return scenario;

    const paramRng = new DeterministicRNG(seed + 1000000);
    const config = JSON.parse(JSON.stringify(scenario.config)) as SimConfig;

    if (ranges.imuBiasScale) {
      const scale = ranges.imuBiasScale[0] + paramRng.next() * (ranges.imuBiasScale[1] - ranges.imuBiasScale[0]);
      config.sensors.imu.gyroBiasRW *= scale;
      config.sensors.imu.accelBiasRW *= scale;
    }
    if (ranges.windSpeed) {
      config.environment.wind.meanSpeed = ranges.windSpeed[0] + paramRng.next() * (ranges.windSpeed[1] - ranges.windSpeed[0]);
    }
    if (ranges.sensorDropoutRate) {
      const rate = ranges.sensorDropoutRate[0] + paramRng.next() * (ranges.sensorDropoutRate[1] - ranges.sensorDropoutRate[0]);
      config.sensors.magnetometer.dropoutProbability = rate;
      config.sensors.rangefinder.dropoutProbability = rate;
      config.sensors.opticalFlow.dropoutProbability = rate;
      if (config.sensors.cameraVIO) config.sensors.cameraVIO.dropoutProbability = rate;
    }
    if (ranges.commLatency) {
      config.swarm.communication.latencyMean = ranges.commLatency[0] + paramRng.next() * (ranges.commLatency[1] - ranges.commLatency[0]);
    }

    return { ...scenario, config };
  }

  // ── Acceptance evaluation ──

  private evaluate(metrics: ScenarioMetrics, criteria?: AcceptanceCriteria): { passed: boolean; failedCriteria: string[] } {
    if (!criteria) return { passed: true, failedCriteria: [] };
    const f: string[] = [];
    const c = criteria;
    const m = metrics;

    if (c.maxCollisionCount !== undefined && m.collisionCount > c.maxCollisionCount) f.push('maxCollisionCount');
    if (c.minMinSeparation !== undefined && m.minSeparation < c.minMinSeparation) f.push('minMinSeparation');
    if (c.maxRmsExecutedTrackingError !== undefined && m.rmsExecutedTrackingError > c.maxRmsExecutedTrackingError) f.push('maxRmsExecutedTrackingError');
    if (c.maxRmsEstimationError !== undefined && m.rmsEstimationError > c.maxRmsEstimationError) f.push('maxRmsEstimationError');
    if (c.maxHorizontalDrift !== undefined && m.maxHorizontalDrift > c.maxHorizontalDrift) f.push('maxHorizontalDrift');
    if (c.maxRmsAltitudeError !== undefined && m.rmsAltitudeError > c.maxRmsAltitudeError) f.push('maxRmsAltitudeError');
    if (c.maxMaxAltitudeError !== undefined && m.maxAltitudeError > c.maxMaxAltitudeError) f.push('maxMaxAltitudeError');
    if (c.maxSaturationTimeFraction !== undefined && m.saturationTimeFraction > c.maxSaturationTimeFraction) f.push('maxSaturationTimeFraction');
    if (c.maxFormationRMS !== undefined && isFinite(m.formationRMS) && m.formationRMS > c.maxFormationRMS) f.push('maxFormationRMS');
    if (c.maxInnovationGatedFraction !== undefined && m.innovationGatedFraction > c.maxInnovationGatedFraction) f.push('maxInnovationGatedFraction');
    if (c.minFlowValidFraction !== undefined && m.flowValidFraction < c.minFlowValidFraction) f.push('minFlowValidFraction');
    if (c.maxFlowValidFraction !== undefined && m.flowValidFraction > c.maxFlowValidFraction) f.push('maxFlowValidFraction');
    if (c.minVioValidFraction !== undefined && m.vioValidFraction < c.minVioValidFraction) f.push('minVioValidFraction');
    if (c.maxVioValidFraction !== undefined && m.vioValidFraction > c.maxVioValidFraction) f.push('maxVioValidFraction');
    // maxRecoveryTime: NaN means no recovery happened → fail (not pass)
    if (c.maxRecoveryTime !== undefined && (!isFinite(m.recoveryTime) || m.recoveryTime > c.maxRecoveryTime)) f.push('maxRecoveryTime');
    if (c.minPacketDeliveryRate !== undefined && m.packetDeliveryRate < c.minPacketDeliveryRate) f.push('minPacketDeliveryRate');
    if (c.maxUwbRangeRmse !== undefined && m.uwbRangeRmse > c.maxUwbRangeRmse) f.push('maxUwbRangeRmse');
    if (c.minUwbRangeRmse !== undefined && m.uwbRangeRmse < c.minUwbRangeRmse) f.push('minUwbRangeRmse');
    if (c.minMaxInnovationNorm !== undefined && m.maxInnovationNorm < c.minMaxInnovationNorm) f.push('minMaxInnovationNorm');
    if (c.minInnovationGatedFraction !== undefined && m.innovationGatedFraction < c.minInnovationGatedFraction) f.push('minInnovationGatedFraction');
    // Previously missing criteria
    if (c.maxXYDriftRate !== undefined && m.xyDriftRate > c.maxXYDriftRate) f.push('maxXYDriftRate');
    if (c.maxNeighborEstimateError !== undefined && m.neighborEstimateError > c.maxNeighborEstimateError) f.push('maxNeighborEstimateError');
    if (c.minNeighborEstimateError !== undefined && m.neighborEstimateError < c.minNeighborEstimateError) f.push('minNeighborEstimateError');

    return { passed: f.length === 0, failedCriteria: f };
  }
}
