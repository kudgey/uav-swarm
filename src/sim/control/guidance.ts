/**
 * Guidance module: generates trajectory references.
 * STATUS: experimental
 * Modes: hover (hold position+yaw), waypoint (go-to with speed limit),
 *        mission (multi-waypoint queue with auto-advance).
 */

import { v3Create, v3Sub, v3Len, v3Scale, v3Normalize } from '@lib/math';
import type { Vec3 } from '@sim/core/types';
import type { EstimatedState } from '@sim/estimation/estimator-types';
import type { GuidanceMode, GuidanceOutput } from './controller-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('guidance', 'experimental', 'Hover + waypoint + mission queue guidance');

const _diff = v3Create();
const _dir = v3Create();

export interface MissionWaypoint {
  position: [number, number, number];
  yaw: number;
  speed: number;
}

export class GuidanceModule {
  private mode: GuidanceMode = 'hover';
  private hoverPos: Vec3 = v3Create();
  private hoverYaw = 0;
  private waypointTarget: Vec3 = v3Create();
  private waypointYaw = 0;
  private waypointSpeed = 1.0;
  // Formation: dynamic reference updated each tick
  private formationTarget: Vec3 = v3Create();
  private formationVel: Vec3 = v3Create();
  private formationYaw = 0;

  // Mission plan: ordered waypoint queue
  private missionQueue: MissionWaypoint[] = [];
  private missionIndex = 0;
  private static readonly ARRIVAL_RADIUS = 1.0; // m

  private output: GuidanceOutput = {
    positionDes: v3Create(),
    velocityDes: v3Create(),
    accelerationDes: v3Create(),
    yawDes: 0,
  };

  setHover(position: Vec3, yaw: number): void {
    this.mode = 'hover';
    this.hoverPos[0] = position[0];
    this.hoverPos[1] = position[1];
    this.hoverPos[2] = position[2];
    this.hoverYaw = yaw;
    this.clearMissionPlan();
  }

  setWaypoint(target: Vec3, yaw: number, speed: number): void {
    this.mode = 'waypoint';
    this.waypointTarget[0] = target[0];
    this.waypointTarget[1] = target[1];
    this.waypointTarget[2] = target[2];
    this.waypointYaw = yaw;
    this.waypointSpeed = speed;
    this.clearMissionPlan();
  }

  private clearMissionPlan(): void {
    this.missionQueue = [];
    this.missionIndex = 0;
  }

  /** Load a multi-waypoint mission plan. Auto-advances through queue. */
  setMissionPlan(waypoints: MissionWaypoint[]): void {
    this.missionQueue = waypoints.slice();
    this.missionIndex = 0;
    if (waypoints.length > 0) {
      const wp = waypoints[0];
      this.waypointTarget[0] = wp.position[0];
      this.waypointTarget[1] = wp.position[1];
      this.waypointTarget[2] = wp.position[2];
      this.waypointYaw = wp.yaw;
      this.waypointSpeed = wp.speed;
      this.mode = 'waypoint';
    }
  }

  /** Get mission progress. */
  getMissionProgress(): { current: number; total: number; done: boolean } {
    return {
      current: this.missionIndex,
      total: this.missionQueue.length,
      done: this.missionQueue.length === 0 || this.missionIndex >= this.missionQueue.length,
    };
  }

  /** Export remaining (unvisited) waypoints for handoff to a new leader. */
  exportRemainingPlan(): MissionWaypoint[] {
    if (this.missionQueue.length === 0) return [];
    return this.missionQueue.slice(this.missionIndex);
  }

  /** Set dynamic formation target (called each tick by formation manager). */
  setFormation(targetPos: Vec3, targetVel: Vec3, yaw: number): void {
    this.mode = 'hover'; // reuse hover evaluate logic with dynamic target
    this.formationTarget[0] = targetPos[0];
    this.formationTarget[1] = targetPos[1];
    this.formationTarget[2] = targetPos[2];
    this.formationVel[0] = targetVel[0];
    this.formationVel[1] = targetVel[1];
    this.formationVel[2] = targetVel[2];
    this.formationYaw = yaw;
    this._isFormation = true;
  }

  /** Exit formation mode back to hover at current target. */
  exitFormation(): void {
    this._isFormation = false;
    this.hoverPos[0] = this.formationTarget[0];
    this.hoverPos[1] = this.formationTarget[1];
    this.hoverPos[2] = this.formationTarget[2];
    this.hoverYaw = this.formationYaw;
    this.mode = 'hover';
  }

  private _isFormation = false;
  get isFormation(): boolean { return this._isFormation; }

  getMode(): GuidanceMode { return this.mode; }

  evaluate(estimate: EstimatedState): GuidanceOutput {
    const out = this.output;

    if (this._isFormation) {
      out.positionDes[0] = this.formationTarget[0];
      out.positionDes[1] = this.formationTarget[1];
      out.positionDes[2] = this.formationTarget[2];
      out.velocityDes[0] = this.formationVel[0];
      out.velocityDes[1] = this.formationVel[1];
      out.velocityDes[2] = this.formationVel[2];
      out.accelerationDes[0] = 0; out.accelerationDes[1] = 0; out.accelerationDes[2] = 0;
      out.yawDes = this.formationYaw;
    } else if (this.mode === 'hover') {
      out.positionDes[0] = this.hoverPos[0];
      out.positionDes[1] = this.hoverPos[1];
      out.positionDes[2] = this.hoverPos[2];
      out.velocityDes[0] = 0; out.velocityDes[1] = 0; out.velocityDes[2] = 0;
      out.accelerationDes[0] = 0; out.accelerationDes[1] = 0; out.accelerationDes[2] = 0;
      out.yawDes = this.hoverYaw;
    } else {
      // Waypoint mode: velocity toward current waypoint target
      v3Sub(_diff, this.waypointTarget, estimate.position);
      const dist = v3Len(_diff);

      // Mission queue: auto-advance when within arrival radius
      if (this.missionQueue.length > 0 && dist < GuidanceModule.ARRIVAL_RADIUS) {
        this.missionIndex++;
        if (this.missionIndex < this.missionQueue.length) {
          const wp = this.missionQueue[this.missionIndex];
          this.waypointTarget[0] = wp.position[0];
          this.waypointTarget[1] = wp.position[1];
          this.waypointTarget[2] = wp.position[2];
          this.waypointYaw = wp.yaw;
          this.waypointSpeed = wp.speed;
          // Recompute diff for new target
          v3Sub(_diff, this.waypointTarget, estimate.position);
        } else {
          // Mission complete: hover at last waypoint
          this.hoverPos[0] = this.waypointTarget[0];
          this.hoverPos[1] = this.waypointTarget[1];
          this.hoverPos[2] = this.waypointTarget[2];
          this.hoverYaw = this.waypointYaw;
          this.mode = 'hover';
          out.positionDes[0] = this.hoverPos[0];
          out.positionDes[1] = this.hoverPos[1];
          out.positionDes[2] = this.hoverPos[2];
          out.velocityDes[0] = 0; out.velocityDes[1] = 0; out.velocityDes[2] = 0;
          out.accelerationDes[0] = 0; out.accelerationDes[1] = 0; out.accelerationDes[2] = 0;
          out.yawDes = this.hoverYaw;
          return out;
        }
      }

      const newDist = v3Len(_diff);
      out.positionDes[0] = this.waypointTarget[0];
      out.positionDes[1] = this.waypointTarget[1];
      out.positionDes[2] = this.waypointTarget[2];

      if (newDist > 0.1) {
        v3Normalize(_dir, _diff);
        const speed = Math.min(this.waypointSpeed, newDist);
        v3Scale(out.velocityDes, _dir, speed);
      } else {
        out.velocityDes[0] = 0; out.velocityDes[1] = 0; out.velocityDes[2] = 0;
      }
      out.accelerationDes[0] = 0; out.accelerationDes[1] = 0; out.accelerationDes[2] = 0;
      out.yawDes = this.waypointYaw;
    }

    return out;
  }
}
