/**
 * World geometry: floor plane, AABB obstacles, ray intersection, LOS checks.
 * STATUS: experimental
 *
 * Floor at z_NED = 0. Height above ground = -position_z (when floor is at z=0).
 * Obstacles are axis-aligned bounding boxes in NED coordinates.
 */

import { v3Create } from '@lib/math';
import type { Vec3, SceneConfig, ObstacleDef } from '@sim/core/types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('world-geometry', 'experimental', 'Floor + AABB obstacles with raycast');

// Pre-allocated scratch for raycast normal
const _hitNormal = v3Create();

export interface RaycastResult {
  hit: boolean;
  distance: number;
  /** Outward surface normal at hit point (NED world frame). Only valid when hit=true. */
  normal: Vec3;
}

export class WorldGeometry {
  private obstacles: ObstacleDef[];
  private floorZ = 0;  // NED z of floor surface
  private defaultTextureQuality: number;
  private sceneBounds: { min: Vec3; max: Vec3 } | null = null;

  constructor(config: SceneConfig) {
    this.obstacles = config.obstacles;
    this.defaultTextureQuality = config.surfaceTextureQuality;
    if (config.sceneBounds) {
      this.sceneBounds = {
        min: v3Create(config.sceneBounds.min[0], config.sceneBounds.min[1], config.sceneBounds.min[2]),
        max: v3Create(config.sceneBounds.max[0], config.sceneBounds.max[1], config.sceneBounds.max[2]),
      };
    }
  }

  getObstacles(): ObstacleDef[] { return this.obstacles; }

  /** Get scene operational bounds (NED). Falls back to obstacle AABB + margin or default 30x30x10. */
  getBounds(): { min: Vec3; max: Vec3 } {
    if (this.sceneBounds) return this.sceneBounds;
    if (this.obstacles.length > 0) {
      const min = v3Create(Infinity, Infinity, Infinity);
      const max = v3Create(-Infinity, -Infinity, -Infinity);
      for (const o of this.obstacles) {
        for (let i = 0; i < 3; i++) {
          if (o.min[i] < min[i]) min[i] = o.min[i];
          if (o.max[i] > max[i]) max[i] = o.max[i];
        }
      }
      // Add margin
      for (let i = 0; i < 3; i++) { min[i] -= 5; max[i] += 5; }
      return { min, max };
    }
    return { min: v3Create(-15, -15, -10), max: v3Create(15, 15, 0) };
  }

  updateConfig(config: SceneConfig): void {
    this.obstacles = config.obstacles;
    this.defaultTextureQuality = config.surfaceTextureQuality;
  }

  /**
   * Height above ground at a position.
   * Considers floor (z=0) and tops of obstacles below the drone.
   * Returns max(0, height) — clamped, never negative.
   */
  getHeightAboveGround(position: Vec3): number {
    const droneZ = position[2]; // NED z (positive = down)
    let groundZ = this.floorZ;  // NED z of nearest surface below (most negative = highest in NED)

    for (const obs of this.obstacles) {
      // Check if drone is above this obstacle (in XY projection)
      if (position[0] >= obs.min[0] && position[0] <= obs.max[0] &&
          position[1] >= obs.min[1] && position[1] <= obs.max[1]) {
        // Obstacle top is at min z (most negative z = highest point in NED)
        const topZ = obs.min[2];
        if (topZ < groundZ && topZ <= droneZ) {
          // This obstacle top is higher than current ground AND below the drone
          // In NED: "higher" means more negative z, but topZ < groundZ means
          // actually... let me be careful.
          // In NED: z increases downward. So a surface at z=-3 (ceiling) is
          // ABOVE a surface at z=0 (floor). The "top" of an obstacle is its
          // most negative z value (obs.min[2]).
          // The drone at position z is ABOVE a surface at z_surface if
          // position_z < z_surface (drone has smaller z = higher up).
          // Wait — no: in NED, smaller z = higher altitude.
          // Drone at z=-2 is above floor at z=0 because -2 < 0.
          // The obstacle top at z=-1 is between them.
          // The drone is above the obstacle top if drone_z < obstacle_top_z = -1?
          // No: drone at z=-2 has MORE negative z = higher than obstacle top at z=-1.
          // So drone_z < obs_top_z means drone is above the obstacle.
          // Ground below drone: the highest surface whose z > drone_z (in NED).
          // Floor is at z=0 (> drone_z=-2 for a flying drone). Good.
          // Obstacle top at z=-1 (> drone_z=-2). Also below the drone. Good.
          // We want the nearest surface below: the one with the smallest z > drone_z.
        }
      }
    }

    // Rewrite with clearer logic:
    // In NED, the drone is at z=droneZ. A surface at z=surfZ is below the drone
    // if surfZ > droneZ (larger z = further down = below). But that doesn't make sense
    // for a flying drone above the floor.
    //
    // Actually: In NED, z increases DOWN. Drone at z=-2 is 2m above z=0 floor.
    // The floor at z=0 is BELOW the drone because 0 > -2.
    // Height above ground = floor_z - drone_z = 0 - (-2) = 2m. Correct.
    //
    // For an obstacle with top at z=-1 (1m above floor), if drone is at z=-2:
    // The obstacle top is below the drone (because -1 > -2).
    // Height above this surface = obs_top_z - drone_z = -1 - (-2) = 1m.
    //
    // We want the CLOSEST surface below: the one that gives the smallest positive height.
    // That's the surface with the smallest (surfZ - droneZ) where surfZ > droneZ.
    // Wait: surfZ - droneZ = (-1) - (-2) = 1. For floor: 0 - (-2) = 2. So obstacle at 1m is closer.
    // We want the minimum positive (surfZ - droneZ). This is correct.

    // Reset: find minimum height above any surface
    let minHeight = this.floorZ - droneZ; // floor height

    for (const obs of this.obstacles) {
      // Check XY overlap
      if (position[0] >= obs.min[0] && position[0] <= obs.max[0] &&
          position[1] >= obs.min[1] && position[1] <= obs.max[1]) {
        const topZ = obs.min[2]; // In NED AABB, min[2] is the most negative z = highest point
        const hAbove = topZ - droneZ;
        if (hAbove > 0 && hAbove < minHeight) {
          minHeight = hAbove;
        }
      }
    }

    return Math.max(0, minHeight);
  }

  /**
   * Surface texture quality at the ground below the drone.
   */
  getSurfaceTextureQuality(position: Vec3): number {
    const droneZ = position[2];

    // Check if drone is above any obstacle
    for (const obs of this.obstacles) {
      if (position[0] >= obs.min[0] && position[0] <= obs.max[0] &&
          position[1] >= obs.min[1] && position[1] <= obs.max[1]) {
        const topZ = obs.min[2];
        if (topZ - droneZ > 0) {
          // Drone is above this obstacle
          return obs.textureQuality ?? this.defaultTextureQuality;
        }
      }
    }

    return this.defaultTextureQuality;
  }

  /**
   * Generic ray-scene intersection. Tests against floor and all obstacle AABBs.
   * @param origin  Ray origin (NED)
   * @param direction  Ray direction (NED, does not need to be unit length)
   * @returns Hit result with distance (0 = miss)
   */
  raycast(origin: Vec3, direction: Vec3): RaycastResult {
    let closestT = Infinity;
    // Track which surface was hit: -1=floor, >=0=obstacle index
    let hitSource = -2;  // -2 = no hit
    let hitAxis = 2;     // axis of the hit face normal (0=x, 1=y, 2=z)
    let hitSign = -1;    // sign of the outward normal along hitAxis

    // Test floor plane (z = 0 in NED). Floor normal points upward: [0, 0, -1] in NED.
    if (Math.abs(direction[2]) > 1e-10) {
      const t = (this.floorZ - origin[2]) / direction[2];
      if (t > 0 && t < closestT) {
        closestT = t;
        hitSource = -1;
        hitAxis = 2;
        hitSign = -1; // outward normal is [0, 0, -1] (upward in NED)
      }
    }

    // Test obstacle AABBs
    for (let oi = 0; oi < this.obstacles.length; oi++) {
      const obs = this.obstacles[oi];
      const result = this.rayAABBIntersectDetailed(origin, direction, obs.min, obs.max);
      if (result.t > 0 && result.t < closestT) {
        closestT = result.t;
        hitSource = oi;
        hitAxis = result.axis;
        hitSign = result.sign;
      }
    }

    if (hitSource === -2) {
      return { hit: false, distance: 0, normal: _hitNormal };
    }

    // Build normal vector into pre-allocated scratch
    _hitNormal[0] = 0; _hitNormal[1] = 0; _hitNormal[2] = 0;
    _hitNormal[hitAxis] = hitSign;

    // Distance is the length along the direction vector
    const dx = direction[0] * closestT;
    const dy = direction[1] * closestT;
    const dz = direction[2] * closestT;
    const actualDist = Math.sqrt(dx * dx + dy * dy + dz * dz);
    return { hit: true, distance: actualDist, normal: _hitNormal };
  }

  /** Line-of-sight check between two points. */
  hasLineOfSight(from: Vec3, to: Vec3): boolean {
    const dx = to[0] - from[0];
    const dy = to[1] - from[1];
    const dz = to[2] - from[2];
    const len = Math.sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-10) return true;

    const dir = new Float64Array([dx / len, dy / len, dz / len]);
    const result = this.raycast(from, dir);
    return !result.hit || result.distance >= len - 1e-6;
  }

  /** Check if point is inside any obstacle. */
  isInsideObstacle(position: Vec3): boolean {
    for (const obs of this.obstacles) {
      if (position[0] >= obs.min[0] && position[0] <= obs.max[0] &&
          position[1] >= obs.min[1] && position[1] <= obs.max[1] &&
          position[2] >= obs.min[2] && position[2] <= obs.max[2]) {
        return true;
      }
    }
    return false;
  }

  /**
   * Ray-AABB intersection with face info (slab method).
   * Returns parametric t, hit axis (0=x, 1=y, 2=z), and outward normal sign.
   */
  private rayAABBIntersectDetailed(
    origin: Vec3, dir: Vec3, aabbMin: Vec3, aabbMax: Vec3,
  ): { t: number; axis: number; sign: number } {
    let tmin = -Infinity;
    let tmax = Infinity;
    let entryAxis = 0;
    let entrySign = 1;

    for (let i = 0; i < 3; i++) {
      if (Math.abs(dir[i]) < 1e-15) {
        if (origin[i] < aabbMin[i] || origin[i] > aabbMax[i])
          return { t: -1, axis: 0, sign: 1 };
      } else {
        const invD = 1 / dir[i];
        let t1 = (aabbMin[i] - origin[i]) * invD;
        let t2 = (aabbMax[i] - origin[i]) * invD;
        // t1 = near face, t2 = far face along this axis
        let nearSign = -1; // normal points toward min face (-axis direction)
        if (t1 > t2) {
          const tmp = t1; t1 = t2; t2 = tmp;
          nearSign = 1; // swapped, so normal points toward max face (+axis direction)
        }
        if (t1 > tmin) {
          tmin = t1;
          entryAxis = i;
          entrySign = nearSign; // outward normal of the entry face
        }
        tmax = Math.min(tmax, t2);
        if (tmin > tmax) return { t: -1, axis: 0, sign: 1 };
      }
    }

    const t = tmin > 0 ? tmin : (tmax > 0 ? tmax : -1);
    return { t, axis: entryAxis, sign: entrySign };
  }
}
