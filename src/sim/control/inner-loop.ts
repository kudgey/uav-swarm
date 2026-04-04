/**
 * Inner loop: geometric SO(3) attitude controller (Lee et al. 2010).
 * STATUS: experimental
 *
 * e_R = 0.5 * vee(R_des^T * R - R^T * R_des)
 * τ = -kr * e_R - kw * (ω - ω_des) + ω × (J * ω)
 */

import { v3Create, v3Cross, m3Create, m3Mul, m3Transpose, m3Sub, m3MulV3, vee3, m3FromDiag } from '@lib/math';
import { qToRotationMatrix } from '@sim/physics/quaternion';
import type { Vec3, Quat, DroneParams } from '@sim/core/types';
import type { ControllerConfig } from './controller-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('inner-loop', 'experimental', 'Geometric SO(3) attitude controller (Lee et al.)');

// Pre-allocated scratch
const _Rest = m3Create();
const _Rdes = m3Create();
const _RdtR = m3Create();  // R_des^T * R
const _RtRd = m3Create();  // R^T * R_des
const _errMat = m3Create(); // R_des^T*R - R^T*R_des
const _eR = v3Create();
const _Jomega = v3Create();
const _omegaCrossJomega = v3Create();
const _torque = v3Create();
const _Jmat = m3Create();
const _RdesT = m3Create();
const _RestT = m3Create();

/**
 * Compute body-frame torque command from attitude error.
 */
export function computeInnerLoop(
  qEst: Quat,
  omegaEst: Vec3,
  qDes: Quat,
  params: DroneParams,
  config: ControllerConfig,
): Vec3 {
  // Build rotation matrices
  qToRotationMatrix(_Rest, qEst);
  qToRotationMatrix(_Rdes, qDes);

  // e_R = 0.5 * vee(R_des^T * R - R^T * R_des)
  m3Transpose(_RdesT, _Rdes);
  m3Transpose(_RestT, _Rest);
  m3Mul(_RdtR, _RdesT, _Rest);   // R_des^T * R_est
  m3Mul(_RtRd, _RestT, _Rdes);   // R_est^T * R_des
  m3Sub(_errMat, _RdtR, _RtRd);  // R_des^T*R - R^T*R_des

  // Vee map: extract vector from skew-symmetric matrix, scale by 0.5
  vee3(_eR, _errMat);
  _eR[0] *= 0.5; _eR[1] *= 0.5; _eR[2] *= 0.5;

  // Gyroscopic feedforward: ω × (J * ω)
  m3FromDiag(_Jmat, params.inertia);
  m3MulV3(_Jomega, _Jmat, omegaEst);
  v3Cross(_omegaCrossJomega, omegaEst, _Jomega);

  // τ = -kr * e_R - kw * ω + ω × (J * ω)
  // (ω_des = 0 for hover/waypoint)
  _torque[0] = -config.krAtt[0] * _eR[0] - config.kwRate[0] * omegaEst[0] + _omegaCrossJomega[0];
  _torque[1] = -config.krAtt[1] * _eR[1] - config.kwRate[1] * omegaEst[1] + _omegaCrossJomega[1];
  _torque[2] = -config.krAtt[2] * _eR[2] - config.kwRate[2] * omegaEst[2] + _omegaCrossJomega[2];

  return _torque;
}

/**
 * Failsafe rate damping: τ = -kw_failsafe * ω_raw (using raw gyro, no attitude estimate).
 */
export function computeRateDamping(
  omegaRaw: Vec3,
  config: ControllerConfig,
): Vec3 {
  _torque[0] = -config.kwFailsafe[0] * omegaRaw[0];
  _torque[1] = -config.kwFailsafe[1] * omegaRaw[1];
  _torque[2] = -config.kwFailsafe[2] * omegaRaw[2];
  return _torque;
}
