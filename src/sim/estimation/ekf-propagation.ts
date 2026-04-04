/**
 * EKF IMU propagation: nominal state update + covariance propagation.
 *
 * State vector order: [δp(3), δv(3), δθ(3), δbg(3), δba(3), δbbaro(1)]
 * Indices:             0-2     3-5    6-8     9-11   12-14    15
 */

import { v3Create, v3Scale, v3Add, skew3, m3Create } from '@lib/math';
import { qRotateVector, qDerivative, qNormalize, qToRotationMatrix } from '@sim/physics/quaternion';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import type { Vec3, Quat, Mat3 } from '@sim/core/types';
import type { EstimatedState, EKFConfig } from './estimator-types';
import {
  N, m16Create, m16Identity, m16Mul, m16MulTransposeRight,
  m16Add, m16Symmetrize, m16Set,
} from './ekf-math';

// Pre-allocated scratch
const _accelCorrected = v3Create();
const _omegaCorrected = v3Create();
const _accelWorld = v3Create();
const _gravity = v3Create(0, 0, GRAVITY_MPS2);
const _qDot = new Float64Array(4);
const _R = m3Create();
const _skewAc = m3Create();
const _skewWc = m3Create();

// 16x16 scratch for propagation
const _Phi = m16Create();
const _PhiP = m16Create();
const _Q = m16Create();

/**
 * Propagate nominal state from IMU measurements.
 * Modifies state in-place.
 */
export function propagateNominal(
  state: EstimatedState,
  gyroMeas: Vec3,
  accelMeas: Vec3,
  dt: number,
): void {
  // Bias-corrected measurements
  _omegaCorrected[0] = gyroMeas[0] - state.gyroBias[0];
  _omegaCorrected[1] = gyroMeas[1] - state.gyroBias[1];
  _omegaCorrected[2] = gyroMeas[2] - state.gyroBias[2];

  _accelCorrected[0] = accelMeas[0] - state.accelBias[0];
  _accelCorrected[1] = accelMeas[1] - state.accelBias[1];
  _accelCorrected[2] = accelMeas[2] - state.accelBias[2];

  // Store corrected angular velocity for controller use
  state.angularVelocity[0] = _omegaCorrected[0];
  state.angularVelocity[1] = _omegaCorrected[1];
  state.angularVelocity[2] = _omegaCorrected[2];

  // Rotate corrected accel to world frame
  qRotateVector(_accelWorld, state.quaternion, _accelCorrected);

  // Position: p += v * dt
  state.position[0] += state.velocity[0] * dt;
  state.position[1] += state.velocity[1] * dt;
  state.position[2] += state.velocity[2] * dt;

  // Velocity: v += (R * a_c + g) * dt
  state.velocity[0] += (_accelWorld[0] + _gravity[0]) * dt;
  state.velocity[1] += (_accelWorld[1] + _gravity[1]) * dt;
  state.velocity[2] += (_accelWorld[2] + _gravity[2]) * dt;

  // Attitude: q += 0.5 * q * [0, ω_c] * dt; normalize
  qDerivative(_qDot, state.quaternion, _omegaCorrected);
  state.quaternion[0] += _qDot[0] * dt;
  state.quaternion[1] += _qDot[1] * dt;
  state.quaternion[2] += _qDot[2] * dt;
  state.quaternion[3] += _qDot[3] * dt;
  qNormalize(state.quaternion, state.quaternion);

  // Biases: no change during propagation (random walk in Q)
  state.timestamp += dt;
}

/**
 * Build Φ = I + F·dt (first-order state transition matrix).
 *
 * F matrix layout (see plan):
 *   δp row:  [0, I, 0, 0, 0, 0]
 *   δv row:  [0, 0, -R·[a_c]×, 0, -R, 0]
 *   δθ row:  [0, 0, -[ω_c]×, -I, 0, 0]
 *   δbg,δba,δbbaro: all zero (biases are random walk, handled in Q)
 */
export function buildPhi(
  out: Float64Array,
  state: EstimatedState,
  gyroMeas: Vec3,
  accelMeas: Vec3,
  dt: number,
): void {
  m16Identity(out);

  // Bias-corrected
  _omegaCorrected[0] = gyroMeas[0] - state.gyroBias[0];
  _omegaCorrected[1] = gyroMeas[1] - state.gyroBias[1];
  _omegaCorrected[2] = gyroMeas[2] - state.gyroBias[2];

  _accelCorrected[0] = accelMeas[0] - state.accelBias[0];
  _accelCorrected[1] = accelMeas[1] - state.accelBias[1];
  _accelCorrected[2] = accelMeas[2] - state.accelBias[2];

  // Rotation matrix R_wb (body to world)
  qToRotationMatrix(_R, state.quaternion);

  // skew(a_corrected) — body-frame
  skew3(_skewAc, _accelCorrected);
  // skew(ω_corrected) — body-frame
  skew3(_skewWc, _omegaCorrected);

  // δp row: Phi[0:3, 3:6] += I * dt  (δp depends on δv)
  for (let i = 0; i < 3; i++) {
    out[i * N + (3 + i)] += dt;
  }

  // δv row: Phi[3:6, 6:9] += -R * skew(a_c) * dt
  // -R * [a_c]× is a 3x3 block. R is column-major, skew is column-major.
  // We need to compute -R * skew(a_c) and place in rows 3-5, cols 6-8
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      let val = 0;
      for (let k = 0; k < 3; k++) {
        // R column-major: R(i,k) = R[k*3+i]
        // skew column-major: skew(k,j) = skew[j*3+k]
        val += _R[k * 3 + i] * _skewAc[j * 3 + k];
      }
      out[(3 + i) * N + (6 + j)] += -val * dt;
    }
  }

  // δv row: Phi[3:6, 12:15] += -R * dt  (δv depends on δba through -R)
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      // R column-major: R(i,j) = R[j*3+i]
      out[(3 + i) * N + (12 + j)] += -_R[j * 3 + i] * dt;
    }
  }

  // δθ row: Phi[6:9, 6:9] += -skew(ω_c) * dt
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      // skew column-major: skew(i,j) = skew[j*3+i]
      out[(6 + i) * N + (6 + j)] += -_skewWc[j * 3 + i] * dt;
    }
  }

  // δθ row: Phi[6:9, 9:12] += -I * dt  (δθ depends on δbg through -I)
  for (let i = 0; i < 3; i++) {
    out[(6 + i) * N + (9 + i)] += -dt;
  }
}

/**
 * Build process noise covariance Q (16×16, diagonal blocks).
 */
export function buildQ(out: Float64Array, config: EKFConfig, dt: number): void {
  out.fill(0);
  // Q_vel = accelNoisePSD * dt (velocity noise from accel white noise)
  const qVel = config.accelNoisePSD * dt;
  out[3 * N + 3] = qVel;
  out[4 * N + 4] = qVel;
  out[5 * N + 5] = qVel;

  // Q_att = gyroNoisePSD * dt (attitude noise from gyro white noise)
  const qAtt = config.gyroNoisePSD * dt;
  out[6 * N + 6] = qAtt;
  out[7 * N + 7] = qAtt;
  out[8 * N + 8] = qAtt;

  // Q_bg = gyroBiasRWPSD * dt (gyro bias random walk)
  const qBg = config.gyroBiasRWPSD * dt;
  out[9 * N + 9] = qBg;
  out[10 * N + 10] = qBg;
  out[11 * N + 11] = qBg;

  // Q_ba = accelBiasRWPSD * dt (accel bias random walk)
  const qBa = config.accelBiasRWPSD * dt;
  out[12 * N + 12] = qBa;
  out[13 * N + 13] = qBa;
  out[14 * N + 14] = qBa;

  // Q_bbaro = baroBiasRWPSD * dt
  out[15 * N + 15] = config.baroBiasRWPSD * dt;
}

/**
 * Propagate covariance: P = Φ * P * Φ^T + Q. Modifies P in-place.
 * scratch1, scratch2 must be Float64Array(256).
 */
export function propagateCovariance(
  P: Float64Array,
  Phi: Float64Array,
  Q: Float64Array,
  scratch1: Float64Array,
  scratch2: Float64Array,
): void {
  // scratch1 = Φ * P
  m16Mul(scratch1, Phi, P);
  // P = scratch1 * Φ^T
  m16MulTransposeRight(P, scratch1, Phi);
  // P += Q
  m16Add(P, P, Q);
  // Enforce symmetry
  m16Symmetrize(P);
}
