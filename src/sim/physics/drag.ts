/**
 * Body drag model.
 * STATUS: simplified — linear mode only, no rotor drag.
 *
 * Computes aerodynamic drag force in body frame from air-relative velocity.
 * Supports linear, quadratic, and combined modes.
 */

import type { Vec3, DroneParams } from '@sim/core/types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('drag', 'experimental', 'Linear/quadratic body drag + optional rotor drag (Faessler et al.)');

/**
 * Compute body-frame drag force from body-frame air-relative velocity.
 *
 * Linear:    F_drag_b = -D .* v_rel_b
 * Quadratic: F_drag_b = -C .* |v_rel_b| .* v_rel_b
 * Combined:  sum of both
 *
 * @param out       Output drag force in body frame (N)
 * @param vRelBody  Air-relative velocity in body frame (m/s)
 * @param params    Drone parameters
 */
export function computeBodyDrag(
  out: Vec3,
  vRelBody: Vec3,
  params: DroneParams,
): Vec3 {
  const vx = vRelBody[0], vy = vRelBody[1], vz = vRelBody[2];
  const mode = params.dragMode;

  if (mode === 'linear' || mode === 'combined') {
    const dl = params.dragCoeffLinear;
    out[0] = -dl[0] * vx;
    out[1] = -dl[1] * vy;
    out[2] = -dl[2] * vz;
  } else {
    out[0] = 0; out[1] = 0; out[2] = 0;
  }

  if (mode === 'quadratic' || mode === 'combined') {
    const dq = params.dragCoeffQuadratic;
    const speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
    if (mode === 'quadratic') {
      out[0] = -dq[0] * speed * vx;
      out[1] = -dq[1] * speed * vy;
      out[2] = -dq[2] * speed * vz;
    } else {
      // combined: add quadratic on top of linear
      out[0] -= dq[0] * speed * vx;
      out[1] -= dq[1] * speed * vy;
      out[2] -= dq[2] * speed * vz;
    }
  }

  // Rotor drag (Faessler et al.): F_rotor_drag = -D_r * v_rel_body
  const dr = params.dragCoeffRotor;
  if (dr[0] !== 0 || dr[1] !== 0 || dr[2] !== 0) {
    out[0] -= dr[0] * vx;
    out[1] -= dr[1] * vy;
    out[2] -= dr[2] * vz;
  }

  return out;
}
