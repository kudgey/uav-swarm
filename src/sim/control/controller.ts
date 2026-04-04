/**
 * Cascaded controller: guidance → outer loop → inner loop → mixer.
 * STATUS: experimental
 *
 * Arm state: disarmed → aligning → armed (independent of control mode).
 * Control mode: manual | hover | waypoint.
 * Failsafe: rate-damped descent (attitude lost), emergency descent (IMU lost).
 */

import { v3Create } from '@lib/math';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import { allocate, createMixer, type MixerState } from '@sim/actuators/mixer';
import { GuidanceModule } from './guidance';
import { computeOuterLoop } from './outer-loop';
import { computeInnerLoop, computeRateDamping } from './inner-loop';
import type { Vec3, DroneParams } from '@sim/core/types';
import type { EstimatedState, EKFHealth } from '@sim/estimation/estimator-types';
import type { ControllerConfig, ArmState, ControlMode, FailsafeState } from './controller-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('controller', 'experimental', 'Cascaded PD + geometric SO(3), short-horizon');

export class CascadedController {
  private guidance: GuidanceModule;
  private config: ControllerConfig;
  private params: DroneParams;
  private mixer: MixerState;
  private motorCommands: Float64Array;

  private _armState: ArmState = 'disarmed';
  private _controlMode: ControlMode = 'manual';
  private _pendingMode: ControlMode = 'manual';
  private failsafe: FailsafeState = { active: false, reason: '', mode: 'none' };

  constructor(config: ControllerConfig, params: DroneParams) {
    this.config = config;
    this.params = params;
    this.guidance = new GuidanceModule();
    this.mixer = createMixer(params);
    this.motorCommands = new Float64Array(params.numRotors);
  }

  get armState(): ArmState { return this._armState; }
  get controlMode(): ControlMode { return this._controlMode; }
  get failsafeState(): FailsafeState { return this.failsafe; }

  /**
   * Request arming. Transitions to 'aligning' if currently disarmed.
   */
  requestArm(): void {
    if (this._armState === 'disarmed') {
      this._armState = 'aligning';
    }
  }

  /**
   * Scripted scenario/mission arming path.
   * Bypasses the alignment gate so deterministic validation missions can start
   * from an already initialized simulation state without waiting on EKF pre-arm logic.
   */
  forceArm(): void {
    this._armState = 'armed';
    if (this._pendingMode !== 'manual') {
      this._controlMode = this._pendingMode;
    }
  }

  requestDisarm(): void {
    this._armState = 'disarmed';
    this._controlMode = 'manual';
    this.failsafe = { active: false, reason: '', mode: 'none' };
  }

  setControlMode(mode: ControlMode): void {
    if (mode !== 'manual' && this._armState !== 'armed') {
      return; // Refuse hover/waypoint if not armed
    }
    this._controlMode = mode;
  }

  /**
   * Set hover target. Sets guidance but only switches to hover mode when armed.
   * If not armed, stores as pending mode for when alignment completes.
   */
  setHover(position: Vec3, yaw: number): void {
    this.guidance.setHover(position, yaw);
    this._pendingMode = 'hover';
    if (this._armState === 'armed') {
      this._controlMode = 'hover';
    }
    // If disarmed, start aligning
    if (this._armState === 'disarmed') {
      this._armState = 'aligning';
    }
  }

  setWaypoint(target: Vec3, yaw: number, speed: number): void {
    this.guidance.setWaypoint(target, yaw, speed);
    this._pendingMode = 'waypoint';
    if (this._armState === 'armed') {
      this._controlMode = 'waypoint';
    }
    if (this._armState === 'disarmed') {
      this._armState = 'aligning';
    }
  }

  /** Load a multi-waypoint mission plan with auto-advance. */
  setMissionPlan(waypoints: import('./guidance').MissionWaypoint[]): void {
    this.guidance.setMissionPlan(waypoints);
    this._pendingMode = 'waypoint';
    if (this._armState === 'armed') {
      this._controlMode = 'waypoint';
    }
    if (this._armState === 'disarmed') {
      this._armState = 'aligning';
    }
  }

  getMissionProgress(): { current: number; total: number; done: boolean } {
    return this.guidance.getMissionProgress();
  }

  exportRemainingPlan(): import('./guidance').MissionWaypoint[] {
    return this.guidance.exportRemainingPlan();
  }

  /** Set dynamic formation target (called each tick by formation manager). */
  setFormationTarget(targetPos: Vec3, targetVel: Vec3, yaw: number): void {
    if (this._armState !== 'armed') return;
    this.guidance.setFormation(targetPos, targetVel, yaw);
    this._controlMode = 'formation';
  }

  /** Exit formation mode, revert to hover at current formation target. */
  exitFormation(): void {
    if (this._controlMode === 'formation') {
      this.guidance.exitFormation();
      this._controlMode = 'hover';
    }
  }

  /** Evaluate guidance without running outer/inner loop. Source of v_pref for safety layer. */
  evaluateGuidanceOnly(estimate: EstimatedState): import('./controller-types').GuidanceOutput {
    return this.guidance.evaluate(estimate);
  }

  /**
   * Main update. Called at controller rate (250 Hz).
   * If safeGuidance provided, uses it instead of guidance.evaluate().
   * Returns motor commands. Controller reads ONLY from estimate, never truth state.
   */
  update(
    estimate: EstimatedState,
    health: EKFHealth,
    imuValid: boolean,
    rawGyro: Vec3 | null,
    rawAccel: Vec3 | null,
    simTime: number,
    safeGuidance?: import('./controller-types').GuidanceOutput,
  ): Float64Array {
    // Check alignment
    if (this._armState === 'aligning') {
      if (this.checkStationarity(estimate, rawGyro, rawAccel) && health.aligned) {
        this._armState = 'armed';
        // Apply pending mode now that we're armed
        if (this._pendingMode !== 'manual') {
          this._controlMode = this._pendingMode;
        }
      }
    }

    // Manual mode or not armed: don't touch motor commands (manual passthrough)
    if (this._controlMode === 'manual' || this._armState !== 'armed') {
      return this.motorCommands;
    }

    // Check failsafe conditions (per-subsystem health)
    if (!imuValid) {
      return this.emergencyDescent();
    }
    if (health.attitude !== 'healthy' && rawGyro) {
      return this.rateDampedDescent(rawGyro);
    }

    // Clear failsafe if healthy
    this.failsafe = { active: false, reason: '', mode: 'none' };

    // Normal control path — use safeGuidance if safety layer provided it
    const ref = safeGuidance ?? this.guidance.evaluate(estimate);
    const outer = computeOuterLoop(estimate, ref, this.params, this.config);
    const torque = computeInnerLoop(
      estimate.quaternion, estimate.angularVelocity,
      outer.desiredQuat, this.params, this.config,
    );

    allocate(this.motorCommands, outer.thrust, torque[0], torque[1], torque[2],
      this.mixer, this.params);

    return this.motorCommands;
  }

  private checkStationarity(
    _estimate: EstimatedState,
    rawGyro: Vec3 | null,
    rawAccel: Vec3 | null,
  ): boolean {
    if (!rawGyro || !rawAccel) return false;

    // |ω| < threshold (low angular rate)
    const gyroNorm = Math.sqrt(rawGyro[0] ** 2 + rawGyro[1] ** 2 + rawGyro[2] ** 2);
    if (gyroNorm > this.config.stationaryGyroThreshold) return false;

    // ||a|| ≈ g ± threshold (accel magnitude near gravity, attitude-independent)
    const accelNorm = Math.sqrt(rawAccel[0] ** 2 + rawAccel[1] ** 2 + rawAccel[2] ** 2);
    if (Math.abs(accelNorm - GRAVITY_MPS2) > this.config.stationaryAccelThreshold) return false;

    return true;
  }

  private emergencyDescent(): Float64Array {
    this.failsafe = { active: true, reason: 'IMU invalid', mode: 'emergency-descent' };
    const hoverThrust = this.params.mass * GRAVITY_MPS2 * 0.6;
    allocate(this.motorCommands, hoverThrust, 0, 0, 0, this.mixer, this.params);
    return this.motorCommands;
  }

  private rateDampedDescent(rawGyro: Vec3): Float64Array {
    this.failsafe = { active: true, reason: 'Attitude health lost', mode: 'rate-damped-descent' };
    const hoverThrust = this.params.mass * GRAVITY_MPS2 * this.config.hoverThrustFraction;
    const torque = computeRateDamping(rawGyro, this.config);
    allocate(this.motorCommands, hoverThrust, torque[0], torque[1], torque[2],
      this.mixer, this.params);
    return this.motorCommands;
  }

  getMotorCommands(): Float64Array {
    return this.motorCommands;
  }

  reset(): void {
    this._armState = 'disarmed';
    this._controlMode = 'manual';
    this.failsafe = { active: false, reason: '', mode: 'none' };
    this.motorCommands.fill(0);
  }
}
