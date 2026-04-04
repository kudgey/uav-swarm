/**
 * Simplified 3D ORCA (Optimal Reciprocal Collision Avoidance).
 * STATUS: simplified — best-effort avoidance, not formal closed-loop guarantee.
 *
 * Uses extrapolated stale neighbors + inflated radii.
 * Validated by truth-based min-separation metrics.
 */

import { v3Create, v3Sub, v3Add, v3Scale, v3Dot, v3Len, v3Normalize, v3Cross, clamp } from '@lib/math';
import type { Vec3, ObstacleDef } from '@sim/core/types';
import type { SafetyConfig } from './safety-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('orca', 'simplified',
  'Best-effort 3D ORCA, extrapolated neighbors, inflated radii, no formal guarantee');

const _relPos = v3Create();
const _relVel = v3Create();
const _w = v3Create();
const _u = v3Create();

interface ORCAPlane {
  point: Vec3;
  normal: Vec3;
}

/**
 * Compute ORCA-safe velocity.
 */
export function computeORCA(
  ownPosition: Vec3,
  ownVelocity: Vec3,
  preferredVelocity: Vec3,
  neighbors: { position: Vec3; velocity: Vec3; inflatedRadius: number }[],
  obstacles: ObstacleDef[],
  config: SafetyConfig,
): Vec3 {
  const planes: ORCAPlane[] = [];
  const invTau = 1 / config.orcaTimeHorizon;

  for (const nb of neighbors) {
    // Relative position and velocity
    const rpx = nb.position[0] - ownPosition[0];
    const rpy = nb.position[1] - ownPosition[1];
    const rpz = nb.position[2] - ownPosition[2];
    const dist = Math.sqrt(rpx * rpx + rpy * rpy + rpz * rpz);
    const totalR = config.orcaRadius + nb.inflatedRadius;

    if (dist < 0.001) continue;

    const rvx = ownVelocity[0] - nb.velocity[0];
    const rvy = ownVelocity[1] - nb.velocity[1];
    const rvz = ownVelocity[2] - nb.velocity[2];

    if (dist > totalR) {
      // Time-to-collision: positive if approaching
      const invDist0 = 1 / dist;
      // rp points from own toward neighbor. rv = ownVel - nbVel.
      // If rv · rp_hat > 0, own is moving toward neighbor → closing.
      const closingRate = (rpx * invDist0 * rvx + rpy * invDist0 * rvy + rpz * invDist0 * rvz);
      const ttc = closingRate > 0 ? (dist - totalR) / closingRate : Infinity;

      if (ttc < config.orcaTimeHorizon) {
        // Will collide within time horizon — create repulsive half-plane
        const invDist = 1 / dist;
        // Normal: away from neighbor (relative position direction, negated for "push away")
        const nx = -rpx * invDist;
        const ny = -rpy * invDist;
        const nz = -rpz * invDist;

        // Required separation velocity: stronger when closer in time
        const urgency = 1 - ttc / config.orcaTimeHorizon; // 0 = far, 1 = imminent
        const pushSpeed = urgency * config.maxSpeed * 0.5 + 0.1;

        // Half-plane: velocity must have component >= pushSpeed in normal direction
        planes.push({
          point: v3Create(
            nx * pushSpeed,
            ny * pushSpeed,
            nz * pushSpeed),
          normal: v3Create(nx, ny, nz),
        });
      }
    } else {
      // Already overlapping: strong push apart
      const invDist = 1 / dist;
      // Normal pointing away from neighbor
      const nx = -rpx * invDist;
      const ny = -rpy * invDist;
      const nz = -rpz * invDist;
      const pushMag = (totalR - dist) * invTau + 0.5;

      planes.push({
        point: v3Create(
          ownVelocity[0] + nx * pushMag,
          ownVelocity[1] + ny * pushMag,
          ownVelocity[2] + nz * pushMag),
        normal: v3Create(nx, ny, nz),
      });
    }
  }

  // Static obstacle planes
  for (const obs of obstacles) {
    const cx = clamp(ownPosition[0], obs.min[0], obs.max[0]);
    const cy = clamp(ownPosition[1], obs.min[1], obs.max[1]);
    const cz = clamp(ownPosition[2], obs.min[2], obs.max[2]);
    const dx = ownPosition[0] - cx, dy = ownPosition[1] - cy, dz = ownPosition[2] - cz;
    const d = Math.sqrt(dx * dx + dy * dy + dz * dz);
    if (d < config.orcaRadius + config.minSeparation && d > 0.001) {
      const invD = 1 / d;
      planes.push({
        point: v3Create(0, 0, 0),
        normal: v3Create(dx * invD, dy * invD, dz * invD),
      });
    }
  }

  return projectToPlanes(preferredVelocity, planes, config.maxSpeed);
}

function projectToPlanes(v: Vec3, planes: ORCAPlane[], maxSpeed: number): Vec3 {
  const result = v3Create(v[0], v[1], v[2]);

  for (let pass = 0; pass < 5; pass++) {
    for (const plane of planes) {
      const dot = (result[0] - plane.point[0]) * plane.normal[0]
                + (result[1] - plane.point[1]) * plane.normal[1]
                + (result[2] - plane.point[2]) * plane.normal[2];
      if (dot < 0) {
        result[0] -= dot * plane.normal[0];
        result[1] -= dot * plane.normal[1];
        result[2] -= dot * plane.normal[2];
      }
    }
  }

  const speed = v3Len(result);
  if (speed > maxSpeed) {
    const s = maxSpeed / speed;
    result[0] *= s; result[1] *= s; result[2] *= s;
  }
  return result;
}
