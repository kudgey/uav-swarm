/**
 * Safety layer: pre-controller guidance filter.
 * STATUS: experimental
 *
 * All safety operates at guidance-reference level.
 * No motor-level intervention.
 */

import { v3Create, v3Len, v3Sub, clamp } from '@lib/math';
import { computeORCA } from './orca';
import type { Vec3, ObstacleDef } from '@sim/core/types';
import type { EstimatedState } from '@sim/estimation/estimator-types';
import type { GuidanceOutput } from '@sim/control/controller-types';
import type { NeighborEstimate } from '@sim/swarm/drone-instance';
import type { SafetyConfig, SafetyState, SafetyEvent } from './safety-types';
import type { WorldGeometry } from '@sim/environment/world-geometry';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('safety-layer', 'experimental',
  'Pre-controller ORCA + hard constraints, guidance-level only');

const _diff = v3Create();

export class SafetyLayer {
  /**
   * Modify guidance reference for safety. Returns safe guidance + state + events.
   * All safety decisions produce a modified GuidanceOutput — no motor commands touched.
   */
  adjustGuidance(
    ownEstimate: EstimatedState,
    guidanceOutput: GuidanceOutput,
    neighborEstimates: Map<number, NeighborEstimate>,
    worldGeo: WorldGeometry,
    config: SafetyConfig,
    simTime: number,
  ): { safeGuidance: GuidanceOutput; state: SafetyState; events: SafetyEvent[] } {
    const events: SafetyEvent[] = [];
    let reason = 'none';
    let constraintCount = 0;
    let collisionPredicted = false;
    let active = false;

    // Copy guidance for modification
    const safe: GuidanceOutput = {
      positionDes: v3Create(guidanceOutput.positionDes[0], guidanceOutput.positionDes[1], guidanceOutput.positionDes[2]),
      velocityDes: v3Create(guidanceOutput.velocityDes[0], guidanceOutput.velocityDes[1], guidanceOutput.velocityDes[2]),
      accelerationDes: v3Create(guidanceOutput.accelerationDes[0], guidanceOutput.accelerationDes[1], guidanceOutput.accelerationDes[2]),
      yawDes: guidanceOutput.yawDes,
    };

    // Compute min separation from neighbor estimates
    let minSep = Infinity;
    const orcaNeighbors: { position: Vec3; velocity: Vec3; inflatedRadius: number }[] = [];

    for (const [_nid, est] of neighborEstimates) {
      if (!est.aligned || !est.attitudeHealthy) continue;
      const latency = simTime - est.timestamp;
      if (latency > 5) continue; // too stale

      // Extrapolate neighbor position
      const pPred = v3Create(
        est.position[0] + est.velocity[0] * latency,
        est.position[1] + est.velocity[1] * latency,
        est.position[2] + est.velocity[2] * latency,
      );

      v3Sub(_diff, pPred, ownEstimate.position);
      const dist = v3Len(_diff);
      if (dist < minSep) minSep = dist;

      // Inflate radius by uncertainty
      const radiusInflated = config.orcaRadius + Math.sqrt(Math.max(0, est.positionVariance))
        + 2.0 * latency * latency; // maneuver uncertainty

      orcaNeighbors.push({ position: pPred, velocity: est.velocity, inflatedRadius: radiusInflated });
    }

    // 1. Emergency override: any neighbor too close → level-hover
    if (minSep < config.emergencyStopDistance) {
      safe.positionDes[0] = ownEstimate.position[0];
      safe.positionDes[1] = ownEstimate.position[1];
      safe.positionDes[2] = ownEstimate.position[2];
      safe.velocityDes[0] = 0; safe.velocityDes[1] = 0; safe.velocityDes[2] = 0;
      safe.accelerationDes[0] = 0; safe.accelerationDes[1] = 0; safe.accelerationDes[2] = 0;
      events.push({ timestamp: simTime, droneId: -1, type: 'emergency-stop', detail: `minSep=${minSep.toFixed(2)}` });
      return {
        safeGuidance: safe,
        state: { active: true, reason: 'emergency', constraintCount: 0,
          minSeparationCurrent: minSep, collisionPredicted: true },
        events,
      };
    }

    // 2. ORCA: project preferred velocity into safe set
    if (orcaNeighbors.length > 0) {
      const vPref = safe.velocityDes;
      const vSafe = computeORCA(
        ownEstimate.position, ownEstimate.velocity, vPref,
        orcaNeighbors, worldGeo.getObstacles(),
        config,
      );

      // Check if ORCA modified the velocity
      const dv = Math.sqrt(
        (vSafe[0] - vPref[0]) ** 2 + (vSafe[1] - vPref[1]) ** 2 + (vSafe[2] - vPref[2]) ** 2);
      if (dv > 0.01) {
        safe.velocityDes[0] = vSafe[0];
        safe.velocityDes[1] = vSafe[1];
        safe.velocityDes[2] = vSafe[2];
        active = true;
        reason = 'orca';
        constraintCount = orcaNeighbors.length;
        events.push({ timestamp: simTime, droneId: -1, type: 'orca-override', detail: `dv=${dv.toFixed(2)}` });
      }

      if (minSep < config.minSeparation * 2) collisionPredicted = true;
    }

    // 3. Speed cap
    const speed = v3Len(safe.velocityDes);
    if (speed > config.maxSpeed) {
      const scale = config.maxSpeed / speed;
      safe.velocityDes[0] *= scale; safe.velocityDes[1] *= scale; safe.velocityDes[2] *= scale;
      active = true;
      if (reason === 'none') reason = 'speed';
      events.push({ timestamp: simTime, droneId: -1, type: 'speed-clamp', detail: `speed=${speed.toFixed(1)}` });
    }

    // 4. Altitude floor (NED: z more negative = higher)
    const estAltitude = -ownEstimate.position[2];
    if (estAltitude < config.minAltitude) {
      // Push position target up
      safe.positionDes[2] = Math.min(safe.positionDes[2], -config.minAltitude);
      if (safe.velocityDes[2] > 0) safe.velocityDes[2] = 0; // don't descend further
      active = true;
      if (reason === 'none') reason = 'altitude';
      events.push({ timestamp: simTime, droneId: -1, type: 'altitude-clamp', detail: `alt=${estAltitude.toFixed(2)}` });
    }

    // 5. No-fly zones
    for (const zone of config.noFlyZones) {
      const inside = ownEstimate.position[0] >= zone.min[0] && ownEstimate.position[0] <= zone.max[0]
        && ownEstimate.position[1] >= zone.min[1] && ownEstimate.position[1] <= zone.max[1]
        && ownEstimate.position[2] >= zone.min[2] && ownEstimate.position[2] <= zone.max[2];
      if (inside) {
        // Push position target out of zone (toward nearest face)
        safe.positionDes[0] = ownEstimate.position[0] < (zone.min[0] + zone.max[0]) / 2
          ? zone.min[0] - 0.5 : zone.max[0] + 0.5;
        safe.velocityDes[0] = 0; safe.velocityDes[1] = 0; safe.velocityDes[2] = 0;
        active = true;
        if (reason === 'none') reason = 'nofly';
        events.push({ timestamp: simTime, droneId: -1, type: 'nofly-clamp', detail: 'inside zone' });
        break;
      }
    }

    return {
      safeGuidance: safe,
      state: { active, reason, constraintCount, minSeparationCurrent: minSep, collisionPredicted },
      events,
    };
  }
}
