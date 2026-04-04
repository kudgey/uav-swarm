/**
 * Error-State Extended Kalman Filter.
 * STATUS: experimental
 *
 * State vector (16 DOF): [δp(3), δv(3), δθ(3), δbg(3), δba(3), δbbaro(1)]
 * No absolute XY anchor — horizontal covariance grows by design.
 * Does NOT import DroneState — reads only sensor measurements.
 */

import { v3Create } from '@lib/math';
import { qCreate, qIdentity } from '@sim/physics/quaternion';
import type { Vec3, Quat } from '@sim/core/types';
import type { EstimatedState, EKFConfig, EKFHealth, InnovationRecord, ArmState } from './estimator-types';
import type { WorldGeometry } from '@sim/environment/world-geometry';
import { m16Create, m16Identity, m16CheckDiagPositive, m16Get, m16Set, N } from './ekf-math';
import { propagateNominal, buildPhi, buildQ, propagateCovariance } from './ekf-propagation';
import {
  magUpdate, baroUpdate, rangeUpdate, flowUpdate, resetRangeGating,
  vioPositionUpdate, vioAttitudeUpdate, uwbRangeUpdate,
} from './ekf-updates';
import { InnovationLogger } from './innovation-logger';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('error-state-ekf', 'experimental',
  'ESKF 16-state, no XY anchor, innovation gating on mag/range');

export class ErrorStateEKF {
  private nominal: EstimatedState;
  private P: Float64Array;        // 16×16 covariance
  private config: EKFConfig;
  private worldGeo: WorldGeometry;
  private logger: InnovationLogger;
  private health: EKFHealth;
  private updateCount = 0;
  /** Count of alignment-relevant updates (mag + baro only). */
  private alignUpdateCount = 0;

  // Propagation scratch
  private _Phi = m16Create();
  private _Q = m16Create();
  private _propScratch1 = m16Create();
  private _propScratch2 = m16Create();

  // Phase 11: per-anchor NLOS tracker
  private anchorNLOSTracker: Map<string, { consecutiveHigh: number; estimatedNLOS: boolean }> = new Map();

  constructor(config: EKFConfig, worldGeo: WorldGeometry) {
    this.config = config;
    this.worldGeo = worldGeo;
    this.logger = new InnovationLogger();

    // Initialize nominal state
    this.nominal = {
      position: v3Create(0, 0, 0),
      velocity: v3Create(0, 0, 0),
      quaternion: qCreate(1, 0, 0, 0),
      gyroBias: v3Create(0, 0, 0),
      accelBias: v3Create(0, 0, 0),
      baroBias: 0,
      angularVelocity: v3Create(0, 0, 0),
      timestamp: 0,
    };

    // Initialize covariance
    this.P = m16Create();
    this.initCovariance();

    this.health = {
      attitude: 'degraded',
      vertical: 'degraded',
      horizontal: 'degraded',
      aligned: false,
      attCovMax: config.initAttVar,
      vertCov: config.initPosVar,
      horizCovMax: config.initPosVar,
      updateCount: 0,
    };
  }

  private initCovariance(): void {
    this.P.fill(0);
    const c = this.config;
    // Position
    for (let i = 0; i < 3; i++) this.P[i * N + i] = c.initPosVar;
    // Velocity
    for (let i = 3; i < 6; i++) this.P[i * N + i] = c.initVelVar;
    // Attitude
    for (let i = 6; i < 9; i++) this.P[i * N + i] = c.initAttVar;
    // Gyro bias
    for (let i = 9; i < 12; i++) this.P[i * N + i] = c.initGyroBiasVar;
    // Accel bias
    for (let i = 12; i < 15; i++) this.P[i * N + i] = c.initAccelBiasVar;
    // Baro bias
    this.P[15 * N + 15] = c.initBaroBiasVar;
  }

  // ── Propagation ──

  propagate(gyroMeas: Vec3, accelMeas: Vec3, dt: number): void {
    // Build Φ before propagating nominal (uses current state)
    buildPhi(this._Phi, this.nominal, gyroMeas, accelMeas, dt);
    buildQ(this._Q, this.config, dt);

    // Propagate nominal state
    propagateNominal(this.nominal, gyroMeas, accelMeas, dt);

    // Propagate covariance
    propagateCovariance(this.P, this._Phi, this._Q, this._propScratch1, this._propScratch2);

    this.updateHealth();
  }

  // ── Measurement updates ──

  updateMag(magField: Vec3): void {
    const rec = magUpdate(this.nominal, this.P, this.config, magField, this.nominal.timestamp);
    this.logger.record(rec);
    if (!rec.gated) { this.updateCount++; this.alignUpdateCount++; }
    this.updateHealth();
  }

  updateBaro(pressure: number): void {
    const rec = baroUpdate(this.nominal, this.P, this.config, pressure, this.nominal.timestamp);
    this.logger.record(rec);
    if (!rec.gated) { this.updateCount++; this.alignUpdateCount++; }
    this.updateHealth();
  }

  updateRange(rangeDist: number): void {
    const rec = rangeUpdate(
      this.nominal, this.P, this.config, rangeDist,
      this.worldGeo, this.nominal.timestamp,
    );
    this.logger.record(rec);
    if (!rec.gated) this.updateCount++;
    this.updateHealth();
  }

  updateFlow(flowX: number, flowY: number, heightEst: number): void {
    const rec = flowUpdate(
      this.nominal, this.P, this.config, flowX, flowY,
      heightEst, this.nominal.timestamp,
    );
    this.logger.record(rec);
    if (!rec.gated) this.updateCount++;
    this.updateHealth();
  }

  // ── VIO + UWB updates (Phase 6) ──

  updateVIOPosition(position: Vec3, noiseVar: number): void {
    const rec = vioPositionUpdate(this.nominal, this.P, this.config,
      position, noiseVar, this.nominal.timestamp);
    this.logger.record(rec);
    if (!rec.gated) this.updateCount++;
    this.updateHealth();
  }

  updateVIOAttitude(quat: Quat, noiseVar: number): void {
    const rec = vioAttitudeUpdate(this.nominal, this.P, this.config,
      quat, noiseVar, this.nominal.timestamp);
    this.logger.record(rec);
    if (!rec.gated) this.updateCount++;
    this.updateHealth();
  }

  updateUWBRange(range: number, anchorPos: Vec3, isNLOS: boolean, timestamp?: number, noiseVarOverride?: number, anchorId?: string): void {
    // Phase 11: innovation-based NLOS detection
    let effectiveNLOS = isNLOS;
    if (this.config.innovationNLOSDetection && anchorId) {
      let tracker = this.anchorNLOSTracker.get(anchorId);
      if (!tracker) { tracker = { consecutiveHigh: 0, estimatedNLOS: false }; this.anchorNLOSTracker.set(anchorId, tracker); }
      effectiveNLOS = tracker.estimatedNLOS;
    }

    const rec = uwbRangeUpdate(this.nominal, this.P, this.config,
      range, anchorPos, effectiveNLOS, timestamp ?? this.nominal.timestamp, noiseVarOverride);

    // Update per-anchor tracker from innovation
    if (this.config.innovationNLOSDetection && anchorId && !rec.gated) {
      const tracker = this.anchorNLOSTracker.get(anchorId)!;
      // Compute normalized innovation (already in rec.innovationNorm but we need χ² = innov²/S)
      // Approximate: use innovationNorm² as proxy (S ≈ R for scalar updates)
      const chi2 = rec.innovationNorm * rec.innovationNorm / (effectiveNLOS ? this.config.uwbNlosNoiseVar : this.config.uwbLosNoiseVar);
      if (chi2 > this.config.nlosInnovThreshold) {
        tracker.consecutiveHigh++;
        if (tracker.consecutiveHigh >= this.config.nlosConsecutiveCount) tracker.estimatedNLOS = true;
      } else {
        tracker.consecutiveHigh = 0;
        tracker.estimatedNLOS = false;
      }
    }

    this.logger.record(rec);
    if (!rec.gated) this.updateCount++;
    this.updateHealth();
  }

  // ── Health ──

  private updateHealth(): void {
    const attMax = Math.max(m16Get(this.P, 6, 6), m16Get(this.P, 7, 7), m16Get(this.P, 8, 8));
    const vertCov = m16Get(this.P, 2, 2);
    const horizMax = Math.max(m16Get(this.P, 0, 0), m16Get(this.P, 1, 1));

    this.health.attCovMax = attMax;
    this.health.vertCov = vertCov;
    this.health.horizCovMax = horizMax;
    this.health.updateCount = this.updateCount;

    this.health.attitude = attMax < this.config.attHealthThreshold ? 'healthy' : 'degraded';
    this.health.vertical = vertCov < this.config.vertHealthThreshold ? 'healthy' : 'degraded';
    // Horizontal health: now checkable when VIO/UWB provides XY anchor
    this.health.horizontal = horizMax < this.config.horizHealthThreshold ? 'healthy' : 'degraded';

    // Alignment check
    this.health.aligned = (
      attMax < this.config.alignAttVarThreshold &&
      vertCov < this.config.alignAltVarThreshold &&
      this.alignUpdateCount >= this.config.alignMinUpdates
    );
  }

  // ── Accessors ──

  getEstimate(): EstimatedState {
    return this.nominal;
  }

  getHealth(): EKFHealth {
    return this.health;
  }

  getCovariance(): Float64Array {
    return this.P;
  }

  getInnovationLogger(): InnovationLogger {
    return this.logger;
  }

  isAligned(): boolean {
    return this.health.aligned;
  }

  /**
   * Bootstrap the EKF into a preflight-aligned state for scripted mission starts.
   * This is used only by deterministic scenario/visual mission tooling so the
   * controller can start from the known spawn pose without waiting on a separate
   * pre-arm alignment sequence.
   */
  bootstrapAligned(): void {
    const attVar = Math.min(this.config.alignAttVarThreshold * 0.25, this.config.attHealthThreshold * 0.25);
    const vertVar = Math.min(this.config.alignAltVarThreshold * 0.25, this.config.vertHealthThreshold * 0.25);
    const horizVar = this.config.horizHealthThreshold * 0.25;

    for (let i = 6; i < 9; i++) m16Set(this.P, i, i, attVar);
    m16Set(this.P, 2, 2, vertVar);
    m16Set(this.P, 0, 0, horizVar);
    m16Set(this.P, 1, 1, horizVar);
    this.alignUpdateCount = Math.max(this.alignUpdateCount, this.config.alignMinUpdates);
    this.updateHealth();
  }

  reset(initialPosition?: Vec3, initialQuat?: Quat): void {
    if (initialPosition) {
      this.nominal.position[0] = initialPosition[0];
      this.nominal.position[1] = initialPosition[1];
      this.nominal.position[2] = initialPosition[2];
    } else {
      this.nominal.position[0] = 0; this.nominal.position[1] = 0; this.nominal.position[2] = 0;
    }
    this.nominal.velocity[0] = 0; this.nominal.velocity[1] = 0; this.nominal.velocity[2] = 0;
    if (initialQuat) {
      this.nominal.quaternion[0] = initialQuat[0]; this.nominal.quaternion[1] = initialQuat[1];
      this.nominal.quaternion[2] = initialQuat[2]; this.nominal.quaternion[3] = initialQuat[3];
    } else {
      qIdentity(this.nominal.quaternion);
    }
    this.nominal.gyroBias[0] = 0; this.nominal.gyroBias[1] = 0; this.nominal.gyroBias[2] = 0;
    this.nominal.accelBias[0] = 0; this.nominal.accelBias[1] = 0; this.nominal.accelBias[2] = 0;
    this.nominal.baroBias = 0;
    this.nominal.angularVelocity[0] = 0; this.nominal.angularVelocity[1] = 0; this.nominal.angularVelocity[2] = 0;
    this.nominal.timestamp = 0;

    this.initCovariance();
    this.updateCount = 0;
    this.alignUpdateCount = 0;
    this.logger.reset();
    resetRangeGating();
    this.anchorNLOSTracker.clear();
    this.updateHealth();
  }
}
