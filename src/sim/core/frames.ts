/**
 * Coordinate frame conventions for the UAV swarm simulator.
 *
 * This file is the SINGLE SOURCE OF TRUTH for all frame definitions.
 * Every module that deals with coordinates MUST import from here.
 *
 * ── World Frame: NED (North-East-Down) ──
 *   x = North
 *   y = East
 *   z = Down
 *   Gravity vector: [0, 0, +9.80665] (positive z is down)
 *
 * ── Body Frame: FRD (Forward-Right-Down) ──
 *   x = Forward
 *   y = Right
 *   z = Down
 *   At identity quaternion: body axes align with world axes
 *   (Forward=North, Right=East, Down=Down)
 *
 * ── Quaternion Convention ──
 *   Hamilton, scalar-first: q = [w, x, y, z]
 *   q_wb rotates a vector from body to world:
 *     v_world = q_wb * [0, v_body] * conj(q_wb)
 *
 * ── Rotor Layout (X-config quadrotor, viewed from above) ──
 *
 *       Front
 *    1 (CCW)  2 (CW)
 *        \  /
 *         \/
 *         /\
 *        /  \
 *    4 (CW)  3 (CCW)
 *       Rear
 *
 *   Rotor 1: front-left,  CCW (+1), position [ +l/sqrt2, -l/sqrt2, 0]
 *   Rotor 2: front-right, CW  (-1), position [ +l/sqrt2, +l/sqrt2, 0]
 *   Rotor 3: rear-right,  CCW (+1), position [ -l/sqrt2, +l/sqrt2, 0]
 *   Rotor 4: rear-left,   CW  (-1), position [ -l/sqrt2, -l/sqrt2, 0]
 *
 * ── Thrust Direction ──
 *   In body frame: F_thrust = [0, 0, -T_total] (upward = negative z in FRD)
 *
 * ── Angular Convention ──
 *   Positive roll:  right side down (rotation about +x body)
 *   Positive pitch: nose down (rotation about +y body)
 *   Positive yaw:   nose right (rotation about +z body)
 *
 * ── Euler Angles ──
 *   ZYX intrinsic (yaw-pitch-roll), matches aerospace convention.
 */

import { v3Create } from '@lib/math';
import type { Vec3 } from './types';

// ── Constants ──

export const GRAVITY_MPS2 = 9.80665;

/** Gravity vector in NED world frame [0, 0, +g] */
export const GRAVITY_NED: Vec3 = v3Create(0, 0, GRAVITY_MPS2);

/** Standard rotor directions for X-config: [CCW, CW, CCW, CW] */
export const X_QUAD_ROTOR_DIRS = [1, -1, 1, -1] as const;

/** Number of rotors in standard X-quad */
export const X_QUAD_NUM_ROTORS = 4;

/**
 * Compute X-quad rotor positions in body frame given arm length.
 * Returns array of 4 Vec3 in FRD body frame.
 */
export function xQuadRotorPositions(armLength: number): Vec3[] {
  const d = armLength / Math.SQRT2;
  return [
    v3Create(+d, -d, 0),   // rotor 1: front-left
    v3Create(+d, +d, 0),   // rotor 2: front-right
    v3Create(-d, +d, 0),   // rotor 3: rear-right
    v3Create(-d, -d, 0),   // rotor 4: rear-left
  ];
}
