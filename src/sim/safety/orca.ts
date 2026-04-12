/**
 * 3D ORCA (Optimal Reciprocal Collision Avoidance), proper velocity-obstacle formulation.
 * STATUS: experimental — truncated cone VO, TTC-gated to avoid false positives on parallel paths.
 *
 * Algorithm (per neighbor):
 *   Let tau = time horizon. Relative position rp, relative velocity rv.
 *   Combined radius R = r_self + r_neighbor.
 *   First: TTC check. If not approaching or ttc > tau, skip (no VO constraint).
 *   Otherwise construct VO truncation: circle at rp/tau with radius R/tau.
 *   Find u = vector from rv to closest point on VO boundary.
 *   ORCA half-plane: normal = u/|u|, plane through ownVel + 0.5*u (reciprocal).
 *
 * Already-overlapping case (dist < R): emergency push apart.
 */

import { v3Create, v3Len, clamp } from '@lib/math';
import type { Vec3, ObstacleDef } from '@sim/core/types';
import type { SafetyConfig } from './safety-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('orca', 'experimental',
  '3D ORCA with truncated-cone VO, reciprocal half-planes');

interface ORCAPlane {
  point: Vec3;
  normal: Vec3;
}

/**
 * Compute ORCA-safe velocity using true velocity obstacles.
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
  const tau = config.orcaTimeHorizon;
  const invTau = 1 / tau;

  for (const nb of neighbors) {
    // Relative position (from self to neighbor)
    const rpx = nb.position[0] - ownPosition[0];
    const rpy = nb.position[1] - ownPosition[1];
    const rpz = nb.position[2] - ownPosition[2];
    const distSq = rpx * rpx + rpy * rpy + rpz * rpz;
    const dist = Math.sqrt(distSq);
    const R = config.orcaRadius + nb.inflatedRadius;

    if (dist < 0.001) continue;

    // Relative velocity
    const rvx = ownVelocity[0] - nb.velocity[0];
    const rvy = ownVelocity[1] - nb.velocity[1];
    const rvz = ownVelocity[2] - nb.velocity[2];

    let ux: number, uy: number, uz: number;

    if (distSq > R * R) {
      // TTC pre-check: only build VO if actually approaching within horizon
      const closingRate = -(rpx * rvx + rpy * rvy + rpz * rvz) / dist; // positive if moving apart
      // rv = ownV - nbV; rp points self→nb. If rp·rv > 0, approaching.
      const dotRpRv = rpx * rvx + rpy * rvy + rpz * rvz;
      if (dotRpRv <= 0) continue; // moving apart or stationary relative → no threat
      // Distance to collision: (dist - R) along closing direction, closing speed = dotRpRv/dist
      const closingSpeed = dotRpRv / dist;
      const ttc = (dist - R) / closingSpeed;
      if (ttc > tau) continue; // won't collide within horizon
      void closingRate; // suppress unused
      // Not yet overlapping — use truncated VO cone
      // Offset: vector from rv to center of truncation circle (rp/tau)
      const wx = rvx - rpx * invTau;
      const wy = rvy - rpy * invTau;
      const wz = rvz - rpz * invTau;
      const wLenSq = wx * wx + wy * wy + wz * wz;
      const wLen = Math.sqrt(wLenSq);
      const truncRadius = R * invTau;

      // Dot product: w · rp (negative if rv is "past" the truncation side)
      const dotWRP = wx * rpx + wy * rpy + wz * rpz;

      if (dotWRP < 0 && wLenSq > truncRadius * truncRadius) {
        // rv is on the truncation cap side AND outside cap → project to truncation cap circle
        const invWLen = 1 / wLen;
        const ulen = truncRadius - wLen;
        ux = -ulen * wx * invWLen;
        uy = -ulen * wy * invWLen;
        uz = -ulen * wz * invWLen;
      } else {
        // Project onto VO cone (tangent).
        // The VO cone apex is at origin of velocity space (relative vel frame),
        // axis along rp, half-angle theta where sin(theta) = R/dist.
        // Project rv onto the cone surface.
        const legSq = distSq - R * R;
        const leg = Math.sqrt(Math.max(0, legSq));
        // Cross product: n = normalized component of rv perpendicular to rp
        const invDist = 1 / dist;
        const rpHatX = rpx * invDist;
        const rpHatY = rpy * invDist;
        const rpHatZ = rpz * invDist;
        // rv projected onto rp direction
        const rvDotRp = rvx * rpHatX + rvy * rpHatY + rvz * rpHatZ;
        // Perpendicular component of rv (in VO-meaningful sense)
        const perpX = rvx - rvDotRp * rpHatX;
        const perpY = rvy - rvDotRp * rpHatY;
        const perpZ = rvz - rvDotRp * rpHatZ;
        const perpLen = Math.sqrt(perpX * perpX + perpY * perpY + perpZ * perpZ);
        if (perpLen < 1e-9) {
          // rv parallel to rp: push sideways (pick arbitrary perpendicular)
          // Use small lateral push away from rp
          ux = -rpHatY * 0.1;
          uy = rpHatX * 0.1;
          uz = 0;
        } else {
          // Closest point on cone boundary to rv in the (rp, perp) plane:
          // cone tangent vector in this plane = (leg*rpHat + R*perpHat) / dist
          const invPerpLen = 1 / perpLen;
          const phX = perpX * invPerpLen;
          const phY = perpY * invPerpLen;
          const phZ = perpZ * invPerpLen;
          // Tangent direction
          const tanX = (leg * rpHatX + R * phX) * invDist;
          const tanY = (leg * rpHatY + R * phY) * invDist;
          const tanZ = (leg * rpHatZ + R * phZ) * invDist;
          // Distance from rv to cone = component of rv along cone normal
          // Normal to cone surface (pointing outward from apex axis) = (-R*rpHat + leg*perpHat) / dist
          const nX = (-R * rpHatX + leg * phX) * invDist;
          const nY = (-R * rpHatY + leg * phY) * invDist;
          const nZ = (-R * rpHatZ + leg * phZ) * invDist;
          const rvDotN = rvx * nX + rvy * nY + rvz * nZ;
          if (rvDotN >= 0) {
            // rv is already outside VO cone — no collision on this path
            continue;
          }
          // u points from rv to nearest cone boundary point (along -n)
          ux = -rvDotN * nX;
          uy = -rvDotN * nY;
          uz = -rvDotN * nZ;
          // Use tan to suppress lint warning
          void tanX; void tanY; void tanZ;
        }
      }
    } else {
      // Already overlapping — emergency push apart
      const invDist = 1 / dist;
      const pushMag = (R - dist) + 0.3;
      ux = -rpx * invDist * pushMag - rvx;
      uy = -rpy * invDist * pushMag - rvy;
      uz = -rpz * invDist * pushMag - rvz;
    }

    // Build ORCA half-plane: normal = u/|u|, plane through ownVel + 0.5*u (reciprocal)
    const uLen = Math.sqrt(ux * ux + uy * uy + uz * uz);
    if (uLen < 1e-9) continue;
    const invU = 1 / uLen;
    const nx = ux * invU;
    const ny = uy * invU;
    const nz = uz * invU;
    planes.push({
      point: v3Create(
        ownVelocity[0] + 0.5 * ux,
        ownVelocity[1] + 0.5 * uy,
        ownVelocity[2] + 0.5 * uz,
      ),
      normal: v3Create(nx, ny, nz),
    });
  }

  // Static obstacle planes (non-reciprocal — obstacles don't move)
  for (const obs of obstacles) {
    const cx = clamp(ownPosition[0], obs.min[0], obs.max[0]);
    const cy = clamp(ownPosition[1], obs.min[1], obs.max[1]);
    const cz = clamp(ownPosition[2], obs.min[2], obs.max[2]);
    const dx = ownPosition[0] - cx;
    const dy = ownPosition[1] - cy;
    const dz = ownPosition[2] - cz;
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
  // Iterative half-plane projection (multiple passes to resolve conflicts)
  for (let pass = 0; pass < 5; pass++) {
    for (const plane of planes) {
      // Signed distance from point to plane (toward forbidden side is negative)
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
