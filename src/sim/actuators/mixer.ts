/**
 * X-config quadrotor mixer.
 * STATUS: experimental
 *
 * Maps between [T_total, tau_x, tau_y, tau_z] and individual rotor thrusts.
 *
 * Rotor layout (FRD body frame, X-config):
 *   1: front-left  [ +d, -d, 0] CCW (+1)
 *   2: front-right [ +d, +d, 0] CW  (-1)
 *   3: rear-right  [ -d, +d, 0] CCW (+1)
 *   4: rear-left   [ -d, -d, 0] CW  (-1)
 *   where d = armLength / sqrt(2)
 *
 * Thrust direction: each rotor produces thrust along -z_body.
 *
 * T_total = T1 + T2 + T3 + T4
 * tau_x = d * (-T1 - T2 + T3 + T4)    (roll torque, from rotor y-positions)
 * tau_y = d * ( T1 + T2 - T3 - T4)     (pitch torque, from rotor x-positions WAIT - need to be careful)
 *
 * Let's derive carefully:
 *   Thrust force on rotor i in body frame: F_i = [0, 0, -Ti]
 *   Position of rotor i in body frame: r_i
 *   Torque from rotor i thrust: tau_i = r_i × F_i
 *
 *   r_1 = [+d, -d, 0], F_1 = [0, 0, -T1]
 *   tau_1 = [(-d)(-T1) - 0, 0 - (+d)(-T1), (+d)(0) - (-d)(0)]
 *         = [dT1, dT1, 0]
 *
 *   r_2 = [+d, +d, 0], F_2 = [0, 0, -T2]
 *   tau_2 = [(+d)(-T2) - 0, 0 - (+d)(-T2), 0]
 *         = [-dT2, dT2, 0]
 *
 *   r_3 = [-d, +d, 0], F_3 = [0, 0, -T3]
 *   tau_3 = [(+d)(-T3), 0 - (-d)(-T3), 0]
 *         = [-dT3, -dT3, 0]
 *
 *   r_4 = [-d, -d, 0], F_4 = [0, 0, -T4]
 *   tau_4 = [(-d)(-T4), 0 - (-d)(-T4), 0]
 *         = [dT4, -dT4, 0]
 *
 * Sum thrust torques:
 *   tau_x = d(T1 - T2 - T3 + T4)
 *   tau_y = d(T1 + T2 - T3 - T4)
 *
 * Yaw torque (reactive drag torques):
 *   tau_z = s1*Q1 + s2*Q2 + s3*Q3 + s4*Q4
 *   With kQ/kT ratio: tau_z = (kQ/kT)(s1*T1 + s2*T2 + s3*T3 + s4*T4)
 *   tau_z = (kQ/kT)(T1 - T2 + T3 - T4)
 *
 * Matrix form: [T_total, tau_x, tau_y, tau_z]^T = A * [T1, T2, T3, T4]^T
 *
 *     [ 1      1      1      1    ]
 * A = [ d     -d     -d      d    ]
 *     [ d      d     -d     -d    ]
 *     [ c     -c      c     -c    ]     where c = kQ/kT
 *
 * Allocation: [T1..T4] = A_inv * [T_total, tau_x, tau_y, tau_z]
 */

import { clamp } from '@lib/math';
import type { DroneParams } from '@sim/core/types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('mixer', 'experimental', 'X-config quadrotor mixer with saturation');

export interface MixerState {
  /** Allocation matrix 4x4 (row-major): [T_total, tau_x, tau_y, tau_z] -> [T1..T4] */
  allocMatrix: Float64Array;
  /** Inverse: [T1..T4] -> [T_total, tau_x, tau_y, tau_z] */
  forwardMatrix: Float64Array;
  kT: number;
  kQ: number;
  omegaMax: number;
}

/** Build mixer matrices from drone params. */
export function createMixer(params: DroneParams): MixerState {
  const d = params.armLength / Math.SQRT2;
  const c = params.kQ / params.kT;

  // Forward matrix A: [T1..T4] -> [T_total, tau_x, tau_y, tau_z] (row-major)
  const A = new Float64Array([
    1,  1,  1,  1,
    d, -d, -d,  d,
    d,  d, -d, -d,
    c, -c,  c, -c,
  ]);

  // Analytical inverse of A for this symmetric X-quad config
  const invD = 1 / (4 * d);
  const invC = 1 / (4 * c);
  const Ainv = new Float64Array([
    0.25,  invD,  invD,  invC,
    0.25, -invD,  invD, -invC,
    0.25, -invD, -invD,  invC,
    0.25,  invD, -invD, -invC,
  ]);

  return {
    allocMatrix: Ainv,
    forwardMatrix: A,
    kT: params.kT,
    kQ: params.kQ,
    omegaMax: params.motorOmegaMax,
  };
}

/**
 * Allocate motor commands from desired wrench [T_total, tau_x, tau_y, tau_z].
 * Returns commanded rotor speeds (rad/s) with per-rotor saturation.
 *
 * @param out         Output: commanded rotor speeds (rad/s), length 4
 * @param totalThrust Desired total thrust (N, positive = upward)
 * @param tauX        Desired roll torque (N*m)
 * @param tauY        Desired pitch torque (N*m)
 * @param tauZ        Desired yaw torque (N*m)
 * @param mixer       Mixer state
 * @param params      Drone params
 */
export function allocate(
  out: Float64Array,
  totalThrust: number,
  tauX: number,
  tauY: number,
  tauZ: number,
  mixer: MixerState,
  params: DroneParams,
): Float64Array {
  const M = mixer.allocMatrix;
  const kT = mixer.kT;
  const omegaMax = params.motorOmegaMax;
  const omegaMin = params.motorOmegaMin;
  const maxThrust = kT * omegaMax * omegaMax;

  // Compute desired per-rotor thrust: Ti = Ainv * [T, tau_x, tau_y, tau_z]
  for (let i = 0; i < 4; i++) {
    const row = i * 4;
    const Ti = M[row] * totalThrust + M[row + 1] * tauX + M[row + 2] * tauY + M[row + 3] * tauZ;
    // Clamp thrust, then convert to omega
    const TiClamped = clamp(Ti, 0, maxThrust);
    out[i] = Math.sqrt(TiClamped / kT);
    out[i] = clamp(out[i], omegaMin, omegaMax);
  }
  return out;
}

/**
 * Compute wrench [T_total, tau_x, tau_y, tau_z] from actual motor speeds.
 */
export function computeWrench(
  out: Float64Array,
  motorSpeeds: Float64Array,
  mixer: MixerState,
): Float64Array {
  const A = mixer.forwardMatrix;
  const kT = mixer.kT;

  // Convert speeds to thrusts
  const T0 = kT * motorSpeeds[0] * motorSpeeds[0];
  const T1 = kT * motorSpeeds[1] * motorSpeeds[1];
  const T2 = kT * motorSpeeds[2] * motorSpeeds[2];
  const T3 = kT * motorSpeeds[3] * motorSpeeds[3];

  // Apply forward matrix
  out[0] = A[0] * T0 + A[1] * T1 + A[2] * T2 + A[3] * T3;
  out[1] = A[4] * T0 + A[5] * T1 + A[6] * T2 + A[7] * T3;
  out[2] = A[8] * T0 + A[9] * T1 + A[10] * T2 + A[11] * T3;
  out[3] = A[12] * T0 + A[13] * T1 + A[14] * T2 + A[15] * T3;
  return out;
}
