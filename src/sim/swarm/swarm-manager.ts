/**
 * SwarmManager: manages N DroneInstances with compound step methods.
 * STATUS: experimental
 *
 * Single-drone backward compatible (droneCount=1, origin spawn).
 * Turbulence globally correlated (simplified): all drones receive same
 * turbulence perturbation per tick. Spatial decorrelation deferred to Phase 11.
 */

import { v3Create, v3Len } from '@lib/math';
import { rk4Step } from '@sim/physics/integrator';
import { stepBattery } from '@sim/actuators/battery';
import { createDroneInstance, type DroneInstance, type NeighborEstimate } from './drone-instance';
import { CommunicationSystem } from '@sim/communications/comm-system';
import type { CommConfig, DroneStateEstimatePayload } from '@sim/communications/comm-types';
import { EnvironmentManager } from '@sim/environment/environment-manager';
import { m16Get } from '@sim/estimation/ekf-math';
import { SafetyLayer } from '@sim/safety/safety-layer';
import { SafetyEventLog } from '@sim/safety/safety-metrics';
import type { SafetyConfig } from '@sim/safety/safety-types';
import type { DeterministicRNG } from '@sim/core/rng';
import type { DroneParams, SensorSuiteConfig, Vec3, FormationConfigType } from '@sim/core/types';
import type { WorldGeometry } from '@sim/environment/world-geometry';
import { FormationManager } from './formation';
import type { FormationConfig } from './formation-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('swarm-manager', 'experimental',
  'Multi-drone orchestration with compound step methods');

export class SwarmManager {
  readonly drones: DroneInstance[] = [];
  readonly commSystem: CommunicationSystem;
  private droneParams: DroneParams;
  private commConfig: CommConfig;
  private _positionsMap: Map<number, Vec3> = new Map();
  readonly formationManager: FormationManager;
  readonly safetyLayer: SafetyLayer = new SafetyLayer();
  readonly safetyEventLog: SafetyEventLog = new SafetyEventLog();
  safetyConfig: SafetyConfig;
  private worldGeo!: WorldGeometry;
  private lostDroneIds: Set<number> = new Set();
  /** Last time each drone was seen in any neighbor's estimates. */
  private lastSeenTime: Map<number, number> = new Map();

  constructor(
    positions: Vec3[],
    droneParams: DroneParams,
    sensorConfig: SensorSuiteConfig,
    estimationConfig: import('@sim/estimation/estimator-types').EKFConfig,
    commConfig: CommConfig,
    formationConfig: FormationConfigType,
    safetyConfig: SafetyConfig,
    worldGeo: WorldGeometry,
    masterRng: DeterministicRNG,
  ) {
    this.droneParams = droneParams;
    this.commConfig = commConfig;
    this.safetyConfig = safetyConfig;
    this.worldGeo = worldGeo;

    // Fork independent RNG per drone
    for (let i = 0; i < positions.length; i++) {
      for (let k = 0; k < 64; k++) masterRng.nextU32();
      const droneRng = masterRng.clone();
      this.drones.push(createDroneInstance(
        i, positions[i], droneParams, sensorConfig, estimationConfig, worldGeo, droneRng,
      ));
    }

    // Comm system gets its own RNG fork
    for (let k = 0; k < 64; k++) masterRng.nextU32();
    const commRng = masterRng.clone();
    this.commSystem = new CommunicationSystem(commConfig, commRng, worldGeo);

    // Formation manager
    this.formationManager = new FormationManager({
      enabled: formationConfig.enabled,
      mode: formationConfig.mode,
      topology: formationConfig.topology,
      spacing: formationConfig.spacing,
      leaderDroneId: formationConfig.leaderDroneId,
      consensusGain: formationConfig.consensusGain,
      leaderLossFallback: formationConfig.leaderLossFallback,
      offsetFrame: formationConfig.offsetFrame ?? 'world',
    });
    this.formationManager.updateConfig(this.formationManager['config'], positions.length);
  }

  get count(): number { return this.drones.length; }

  getDrone(id: number): DroneInstance | undefined {
    return this.drones.find(d => d.id === id);
  }

  /** Get truth positions map for comm/UWB (keyed by drone id). */
  getPositionsMap(): Map<number, Vec3> {
    this._positionsMap.clear();
    for (const d of this.drones) {
      this._positionsMap.set(d.id, d.state.position);
    }
    return this._positionsMap;
  }

  // ── Compound step methods ──

  /** Physics: turbulence once, then per-drone env sample + downwash + battery + RK4. */
  stepPhysics(envManager: EnvironmentManager, dt: number, simTime: number): void {
    envManager.stepTemporal(dt);
    // First pass: sample environment for each drone
    for (const d of this.drones) {
      envManager.sampleAt(d.envOutput, d.state.position, simTime);
    }
    // Second pass: compute downwash (needs all drone positions)
    envManager.applyDownwash(this.drones);
    // Third pass: step battery + compose into downwashMultiplier
    const batteryCfg = this.droneParams.battery;
    for (const d of this.drones) {
      if (!batteryCfg.enabled || d.destroyed) { d.batteryThrustMult = 1.0; continue; }
      // Total thrust at current motor speeds (before multiplier)
      let T = 0;
      for (let i = 0; i < d.state.motorSpeeds.length; i++) {
        T += this.droneParams.kT * d.state.motorSpeeds[i] * d.state.motorSpeeds[i];
      }
      d.batteryThrustMult = stepBattery(d.battery, batteryCfg, T, dt);
      // Compose into downwash multiplier (rigid-body multiplies by downwash)
      d.envOutput.downwashMultiplier *= d.batteryThrustMult;
    }
    // Fourth pass: integrate physics
    for (const d of this.drones) {
      rk4Step(d.state, this.droneParams, d.envOutput, dt, d.rk4Scratch);
    }
  }

  /** Sensor bias random walks for all drones. */
  stepBiases(dt: number): void {
    for (const d of this.drones) { if (!d.destroyed) d.sensorManager.stepBiases(dt); }
  }

  /** Read a specific sensor type for all drones. */
  stepSensorIMU(simTime: number): void {
    for (const d of this.drones) { if (!d.destroyed) d.sensorManager.readImu(d.state, d.envOutput, simTime); }
  }
  stepSensorMag(simTime: number): void {
    for (const d of this.drones) { if (!d.destroyed) d.sensorManager.readMag(d.state, d.envOutput, simTime); }
  }
  stepSensorBaro(simTime: number): void {
    for (const d of this.drones) { if (!d.destroyed) d.sensorManager.readBaro(d.state, d.envOutput, simTime); }
  }
  stepSensorRange(simTime: number): void {
    for (const d of this.drones) { if (!d.destroyed) d.sensorManager.readRange(d.state, simTime); }
  }
  stepSensorFlow(simTime: number): void {
    for (const d of this.drones) { if (!d.destroyed) d.sensorManager.readFlow(d.state, d.envOutput, simTime); }
  }
  stepSensorCamera(simTime: number): void {
    for (const d of this.drones) { if (!d.destroyed) d.sensorManager.readCamera(d.state, d.envOutput, simTime); }
  }
  /**
   * Read UWB sensors for all drones: infrastructure anchors + inter-drone truth positions.
   * Scene anchors passed explicitly so infrastructure aiding is not dropped.
   */
  stepSensorUWB(sceneAnchors: import('@sim/core/types').UWBAnchorDef[], simTime: number): void {
    for (const d of this.drones) {
      if (d.destroyed) continue;
      // Build inter-drone truth targets (exclude destroyed drones)
      const others: { id: string; position: Vec3 }[] = [];
      for (const o of this.drones) {
        if (o.id !== d.id && !o.destroyed) {
          others.push({ id: `drone-${o.id}`, position: o.state.position });
        }
      }
      d.sensorManager.readUWBWithDrones(d.state, sceneAnchors, others, simTime);
    }
  }

  /** EKF propagation for all drones. */
  stepEKFPropagate(dt: number): void {
    for (const d of this.drones) {
      if (d.destroyed) continue;
      const r = d.sensorManager.getLatestReadings();
      if (r.imu?.valid) {
        d.ekf.propagate(r.imu.measurement.gyro, r.imu.measurement.accel, dt);
      }
    }
  }

  /** EKF mag update for all drones. */
  stepEKFMag(): void {
    for (const d of this.drones) {
      if (d.destroyed) continue;
      const r = d.sensorManager.getLatestReadings();
      if (r.magnetometer?.valid) d.ekf.updateMag(r.magnetometer.measurement.field);
    }
  }

  stepEKFBaro(): void {
    for (const d of this.drones) {
      if (d.destroyed) continue;
      const r = d.sensorManager.getLatestReadings();
      if (r.barometer?.valid) d.ekf.updateBaro(r.barometer.measurement.pressure);
    }
  }

  stepEKFRange(): void {
    for (const d of this.drones) {
      if (d.destroyed) continue;
      const r = d.sensorManager.getLatestReadings();
      if (r.rangefinder?.valid) d.ekf.updateRange(r.rangefinder.measurement.range);
    }
  }

  stepEKFFlow(): void {
    for (const d of this.drones) {
      if (d.destroyed) continue;
      const r = d.sensorManager.getLatestReadings();
      if (r.opticalFlow?.valid) {
        const hEst = Math.max(0, -d.ekf.getEstimate().position[2]);
        d.ekf.updateFlow(r.opticalFlow.measurement.flowX, r.opticalFlow.measurement.flowY, hEst);
      }
    }
  }

  stepEKFVIOPos(): void {
    for (const d of this.drones) {
      if (d.destroyed) continue;
      const r = d.sensorManager.getLatestReadings();
      if (r.cameraVIO?.valid) {
        const fq = r.cameraVIO.measurement.featureQuality;
        const noiseVar = fq > 0 ? 0.01 / (fq * fq) : 1.0;
        d.ekf.updateVIOPosition(r.cameraVIO.measurement.position, noiseVar);
      }
    }
  }

  stepEKFVIOAtt(): void {
    for (const d of this.drones) {
      if (d.destroyed) continue;
      const r = d.sensorManager.getLatestReadings();
      if (r.cameraVIO?.valid) {
        const fq = r.cameraVIO.measurement.featureQuality;
        const noiseVar = fq > 0 ? 0.0004 / (fq * fq) : 0.04;
        d.ekf.updateVIOAttitude(r.cameraVIO.measurement.quaternion, noiseVar);
      }
    }
  }

  /** EKF UWB update — infrastructure + inter-drone with variance inflation. */
  stepEKFUWB(sceneAnchors: { id: string; position: Vec3 }[], simTime: number): void {
    for (const d of this.drones) {
      const r = d.sensorManager.getLatestReadings();
      if (!r.uwb) continue;

      for (const meas of r.uwb) {
        if (!meas.valid) continue;

        // Infrastructure anchor
        const anchor = sceneAnchors.find(a => a.id === meas.measurement.anchorId);
        if (anchor) {
          d.ekf.updateUWBRange(meas.measurement.range, anchor.position, meas.measurement.isNLOS, undefined, undefined, meas.measurement.anchorId);
          continue;
        }

        // Inter-drone: look up communicated neighbor estimate
        const match = meas.measurement.anchorId.match(/^drone-(\d+)$/);
        if (!match) continue;
        const neighborId = parseInt(match[1], 10);
        const neighborEst = d.neighborEstimates.get(neighborId);
        if (!neighborEst || !neighborEst.aligned || !neighborEst.attitudeHealthy) continue;

        // Check staleness
        const latency = simTime - neighborEst.timestamp;
        if (latency > this.commConfig.estimateTimeout) continue;

        // Extrapolate neighbor position
        const pPred = v3Create(
          neighborEst.position[0] + neighborEst.velocity[0] * latency,
          neighborEst.position[1] + neighborEst.velocity[1] * latency,
          neighborEst.position[2] + neighborEst.velocity[2] * latency,
        );

        // R_eff = base + neighbor variance + maneuver uncertainty
        const baseR = meas.measurement.isNLOS ? 0.09 : 0.01;
        const maneuverVar = (this.commConfig.maneuverAccelStdDev * latency * latency) ** 2;

        // LOS-projected variance (Phase 11) or scalar fallback
        let neighborVar = neighborEst.positionVariance;
        if (this.commConfig.losProjectedVariance && neighborEst.positionCov3x3) {
          const est = d.ekf.getEstimate();
          const dx = pPred[0] - est.position[0], dy = pPred[1] - est.position[1], dz = pPred[2] - est.position[2];
          const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
          if (dist > 0.01) {
            const ux = dx / dist, uy = dy / dist, uz = dz / dist;
            const c = neighborEst.positionCov3x3; // [Pxx, Pxy, Pxz, Pyy, Pyz, Pzz]
            // σ²_los = û^T P û
            neighborVar = ux * ux * c[0] + 2 * ux * uy * c[1] + 2 * ux * uz * c[2]
              + uy * uy * c[3] + 2 * uy * uz * c[4] + uz * uz * c[5];
          }
        }
        const Reff = baseR + neighborVar + maneuverVar;

        d.ekf.updateUWBRange(meas.measurement.range, pPred, meas.measurement.isNLOS, undefined, Reff);
      }
    }
  }

  /**
   * Detect drones lost via comm silence (no truth leakage).
   * A drone is "lost" when ALL surviving drones have expired its neighbor estimate.
   */
  private detectAndHandleLostDrones(simTime: number): void {
    // Update last-seen timestamps for all drones that appear in any neighbor's estimates
    for (const d of this.drones) {
      for (const nid of d.neighborEstimates.keys()) {
        this.lastSeenTime.set(nid, simTime);
      }
    }

    // Only consider drones that were reliably seen (last seen > 1s ago means established)
    const minEstablishTime = 2.0; // must have been seen at least this long ago to be "established"

    for (const [candidateId, lastSeen] of this.lastSeenTime) {
      if (this.lostDroneIds.has(candidateId)) continue;
      // Don't trigger until the drone was established (seen for a while)
      if (simTime - lastSeen < this.commConfig.estimateTimeout) continue; // still within timeout — not lost yet

      // Also require the drone was seen at least minEstablishTime before now
      if (lastSeen < minEstablishTime) continue; // only seen briefly at start, ignore

      // Confirmed: was established, now missing from ALL for > timeout
      let missingFromAll = true;
      for (const d of this.drones) {
        if (d.id === candidateId || this.lostDroneIds.has(d.id)) continue;
        if (d.neighborEstimates.has(candidateId)) { missingFromAll = false; break; }
      }

      if (!missingFromAll) continue;

      // Confirmed lost via comm silence
      const oldLeaderId = this.formationManager.getLeaderId();
      this.lostDroneIds.add(candidateId);
      const activeIds = this.drones.filter(d => !this.lostDroneIds.has(d.id)).map(d => d.id);
      if (activeIds.length < 2) continue; // can't form with <2

      // Reconfigure formation
      this.formationManager.reconfigureForActive(
        activeIds,
        candidateId === oldLeaderId ? oldLeaderId : undefined,
      );

      // Mission plan handoff: if lost drone was leader, transfer remaining waypoints
      if (candidateId === oldLeaderId) {
        const newLeaderId = this.formationManager.getLeaderId();
        const oldLeaderDrone = this.getDrone(candidateId);
        const remaining = oldLeaderDrone?.controller.exportRemainingPlan() ?? [];
        if (remaining.length > 0 && newLeaderId !== candidateId) {
          const newLeaderDrone = this.getDrone(newLeaderId);
          if (newLeaderDrone) newLeaderDrone.controller.setMissionPlan(remaining);
        }
      }
    }
  }

  /** Formation step: compute targets for all drones from communicated data. */
  stepFormation(simTime: number): void {
    const fm = this.formationManager;
    if (!fm['config'].enabled) return;

    // Detect drone loss via comm silence and reconfigure formation
    this.detectAndHandleLostDrones(simTime);

    const isConsensus = fm['config'].mode === 'consensus';

    for (const d of this.drones) {
      if (d.destroyed) continue;

      // Consensus mode: sync progress + compute consensus correction velocity
      if (isConsensus) {
        const neighborProgresses: number[] = [];
        for (const [, est] of d.neighborEstimates) {
          if (est.missionProgress !== undefined) neighborProgresses.push(est.missionProgress);
        }
        if (neighborProgresses.length > 0) {
          d.controller.syncConsensusProgress(neighborProgresses);
        }

        // If drone has an active mission plan: compute consensus correction
        // (pulls drone toward correct formation position while it flies its waypoints)
        const progress = d.controller.getMissionProgress();
        if (progress.total > 0 && !progress.done) {
          const est = d.ekf.getEstimate();
          const state = fm.getFormationTarget(
            d.id, est, d.neighborEstimates, simTime, this.commConfig.estimateTimeout);
          d.lastFormationState = state;

          if (state.active && state.neighborCount > 0) {
            // Consensus correction: velocity toward formation target position
            // Clamped to avoid large transients during initial convergence
            const K = fm['config'].consensusGain;
            const MAX_CORRECTION = 1.0; // m/s max correction velocity
            let cx = K * (state.targetPosition[0] - est.position[0]);
            let cy = K * (state.targetPosition[1] - est.position[1]);
            const mag = Math.sqrt(cx * cx + cy * cy);
            if (mag > MAX_CORRECTION) {
              cx *= MAX_CORRECTION / mag;
              cy *= MAX_CORRECTION / mag;
            }
            const correction = v3Create(cx, cy, 0);
            d.controller.setConsensusCorrection(correction);
          } else {
            d.controller.clearConsensusCorrection();
          }
          continue; // don't call setFormationTarget — drone stays in waypoint mode
          continue;
        } else {
          // Mission done or no plan: clear correction and just hover
          d.controller.clearConsensusCorrection();
          continue; // don't fall through to leader-follower setFormationTarget
        }
      }

      const est = d.ekf.getEstimate();
      const state = fm.getFormationTarget(
        d.id, est, d.neighborEstimates, simTime, this.commConfig.estimateTimeout);

      d.lastFormationState = state;

      if (state.active && state.role === 'leader' && d.controller.controlMode === 'formation') {
        d.controller.exitFormation();
        d.controller.setHover(est.position, 0);
      } else if (state.active && state.role !== 'leader') {
        d.controller.setFormationTarget(state.targetPosition, state.targetVelocity, state.yawDes);
      } else if (!state.active && d.controller.controlMode === 'formation') {
        d.controller.exitFormation();
      }
    }
  }

  /** Update formation config at runtime (hot-apply). */
  updateFormationConfig(cfg: FormationConfigType): boolean {
    // Check preconditions
    if (cfg.enabled) {
      if (cfg.mode === 'leader-follower') {
        const leader = this.getDrone(cfg.leaderDroneId);
        if (!leader || leader.controller.armState !== 'armed') return false;
        if (leader.controller.controlMode !== 'hover' && leader.controller.controlMode !== 'waypoint') return false;
      }
      for (const d of this.drones) {
        if (d.controller.armState !== 'armed') return false;
      }
    }

    // Deactivate current formation if disabling or mode switching
    if (!cfg.enabled || cfg.mode !== this.formationManager['config'].mode) {
      for (const d of this.drones) {
        if (d.controller.controlMode === 'formation') d.controller.exitFormation();
      }
    }

    this.formationManager.updateConfig({
      enabled: cfg.enabled, mode: cfg.mode, topology: cfg.topology,
      spacing: cfg.spacing, leaderDroneId: cfg.leaderDroneId,
      consensusGain: cfg.consensusGain, leaderLossFallback: cfg.leaderLossFallback,
      offsetFrame: cfg.offsetFrame ?? 'world',
    }, this.drones.length);
    return true;
  }

  /** Publish state estimates to comm system (health-gated + comm-enabled). */
  stepEstimatePublish(simTime: number): void {
    for (const d of this.drones) {
      if (d.destroyed || !d.commEnabled) continue;
      if (!d.ekf.isAligned()) continue;
      if (d.ekf.getHealth().attitude !== 'healthy') continue;

      const est = d.ekf.getEstimate();
      const P = d.ekf.getCovariance();
      const payload: DroneStateEstimatePayload = {
        senderId: d.id,
        timestamp: simTime,
        position: v3Create(est.position[0], est.position[1], est.position[2]),
        positionVariance: m16Get(P, 0, 0) + m16Get(P, 1, 1),
        positionCov3x3: this.commConfig.losProjectedVariance
          ? [m16Get(P, 0, 0), m16Get(P, 0, 1), m16Get(P, 0, 2), m16Get(P, 1, 1), m16Get(P, 1, 2), m16Get(P, 2, 2)]
          : undefined,
        velocity: v3Create(est.velocity[0], est.velocity[1], est.velocity[2]),
        quaternion: [est.quaternion[0], est.quaternion[1], est.quaternion[2], est.quaternion[3]],
        aligned: true,
        attitudeHealthy: true,
        inFormation: this.formationManager['config'].enabled && (
          d.controller.controlMode === 'formation' ||
          d.controller.inConsensusFormation ||
          // Consensus mode with mission plan: always report inFormation to bootstrap consensus
          (this.formationManager['config'].mode === 'consensus' &&
           d.controller.getMissionProgress().total > 0) ||
          (this.formationManager['config'].mode === 'leader-follower' &&
           d.id === this.formationManager.getLeaderId() &&
           (d.controller.controlMode === 'hover' || d.controller.controlMode === 'waypoint'))
        ),
        missionProgress: d.controller.getMissionProgress().current,
      };

      this.commSystem.send({
        senderId: d.id,
        receiverId: -1, // broadcast
        sendTime: simTime,
        type: 'state-estimate',
        payload,
      });
    }
  }

  /** Process delivered comm messages → update neighborEstimates. */
  processDeliveredMessages(simTime: number): void {
    for (const d of this.drones) {
      const msgs = this.commSystem.getMessages(d.id);
      for (const msg of msgs) {
        if (msg.type === 'state-estimate') {
          const p = msg.payload;
          if (!p.aligned || !p.attitudeHealthy) continue;
          d.neighborEstimates.set(p.senderId, {
            senderId: p.senderId,
            timestamp: p.timestamp,
            position: p.position,
            positionVariance: p.positionVariance,
            positionCov3x3: p.positionCov3x3 ? new Float64Array(p.positionCov3x3) : undefined,
            velocity: p.velocity,
            quaternion: p.quaternion ? new Float64Array(p.quaternion) : undefined,
            aligned: p.aligned,
            attitudeHealthy: p.attitudeHealthy,
            inFormation: p.inFormation,
            missionProgress: p.missionProgress,
          });
        }
      }

      // Expire old estimates
      for (const [nid, est] of d.neighborEstimates) {
        if (simTime - est.timestamp > this.commConfig.estimateTimeout) {
          d.neighborEstimates.delete(nid);
        }
      }
    }
  }

  /** Pre-controller safety: compute safeGuidance for each drone. */
  stepSafetyGuidance(simTime: number): void {
    for (const d of this.drones) {
      d.safeGuidance = undefined;
      d.lastSafetyState = null;
      if (d.destroyed) continue;

      if (!this.safetyConfig.enabled) continue;
      if (d.controller.failsafeState.active) continue;

      const est = d.ekf.getEstimate();
      const guidance = d.controller.evaluateGuidanceOnly(est);
      const result = this.safetyLayer.adjustGuidance(
        est, guidance, d.neighborEstimates,
        this.worldGeo, this.safetyConfig, simTime);
      d.safeGuidance = result.safeGuidance;
      d.lastSafetyState = result.state;
      // Record safety events
      for (const evt of result.events) {
        evt.droneId = d.id;
        this.safetyEventLog.record(evt);
      }
    }
  }

  /** Controller step for all drones. Passes safeGuidance if available. */
  stepControllers(simTime: number): void {
    for (const d of this.drones) {
      if (d.destroyed) continue;
      const est = d.ekf.getEstimate();
      const health = d.ekf.getHealth();
      const r = d.sensorManager.getLatestReadings();
      const imuValid = r.imu?.valid ?? false;
      const rawGyro = imuValid ? r.imu!.measurement.gyro : null;
      const rawAccel = imuValid ? r.imu!.measurement.accel : null;

      const cmds = d.controller.update(est, health, imuValid, rawGyro, rawAccel, simTime, d.safeGuidance);

      if (d.controller.controlMode !== 'manual') {
        for (let i = 0; i < this.droneParams.numRotors; i++) {
          d.state.motorCommands[i] = cmds[i];
        }
      }
    }
  }
}
