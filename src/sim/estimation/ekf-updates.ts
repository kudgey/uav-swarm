/**
 * EKF measurement update functions.
 * Each follows: predict → innovate → gate → Kalman gain → inject → Joseph update.
 *
 * State order: [δp(3), δv(3), δθ(3), δbg(3), δba(3), δbbaro(1)]
 * Indices:      0-2     3-5    6-8     9-11   12-14    15
 */

import { v3Create, skew3, m3Create } from '@lib/math';
import { qRotateVector, qRotateVectorInverse, qConjugate, qToRotationMatrix, qMultiply, qNormalize, qCreate } from '@sim/physics/quaternion';
import { isaPressure, isaDensity } from '@sim/environment/atmosphere';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import type { Vec3, Mat3 } from '@sim/core/types';
import type { EstimatedState, EKFConfig, InnovationRecord } from './estimator-types';
import type { WorldGeometry } from '@sim/environment/world-geometry';
import {
  N, mulPHt, computeS, invertSmall, computeK, computeDx,
  josephUpdate, mahalanobis, m16Symmetrize,
} from './ekf-math';

// ── Pre-allocated scratch ──

// Mag update (m=3)
const _hMag = v3Create();
const _yMag = new Float64Array(3);
const _Hmag = new Float64Array(3 * N); // 3×16
const _PHtMag = new Float64Array(N * 3);
const _Smag = new Float64Array(9);
const _SinvMag = new Float64Array(9);
const _Kmag = new Float64Array(N * 3);
const _Rmag = new Float64Array(9);
const _dxMag = new Float64Array(N);
const _skewHmag = m3Create();

// Baro update (m=1)
const _yBaro = new Float64Array(1);
const _Hbaro = new Float64Array(N);
const _PHtBaro = new Float64Array(N);
const _Sbaro = new Float64Array(1);
const _SinvBaro = new Float64Array(1);
const _Kbaro = new Float64Array(N);
const _Rbaro = new Float64Array(1);
const _dxBaro = new Float64Array(N);

// Range update (m=1)
const _beamWorld = v3Create();
const _yRange = new Float64Array(1);
const _Hrange = new Float64Array(N);
const _PHtRange = new Float64Array(N);
const _Srange = new Float64Array(1);
const _SinvRange = new Float64Array(1);
const _Krange = new Float64Array(N);
const _Rrange = new Float64Array(1);
const _dxRange = new Float64Array(N);
const _beamBody = v3Create(0, 0, 1); // FRD z-axis

// Flow update (m=2)
const _yFlow = new Float64Array(2);
const _Hflow = new Float64Array(2 * N);
const _PHtFlow = new Float64Array(N * 2);
const _Sflow = new Float64Array(4);
const _SinvFlow = new Float64Array(4);
const _Kflow = new Float64Array(N * 2);
const _Rflow = new Float64Array(4);
const _dxFlow = new Float64Array(N);
const _vBody = v3Create();
const _R = m3Create();

// Shared
const _joseph1 = new Float64Array(N * N);
const _joseph2 = new Float64Array(N * N);
const _halfDtheta = qCreate();
const _qScratch = qCreate();

// ── Error injection ──

function injectError(state: EstimatedState, dx: Float64Array): void {
  state.position[0] += dx[0];
  state.position[1] += dx[1];
  state.position[2] += dx[2];
  state.velocity[0] += dx[3];
  state.velocity[1] += dx[4];
  state.velocity[2] += dx[5];

  // Attitude: q_new = q * [1, δθ/2] (small-angle quaternion)
  _halfDtheta[0] = 1;
  _halfDtheta[1] = dx[6] * 0.5;
  _halfDtheta[2] = dx[7] * 0.5;
  _halfDtheta[3] = dx[8] * 0.5;
  qMultiply(_qScratch, state.quaternion, _halfDtheta);
  qNormalize(state.quaternion, _qScratch);

  state.gyroBias[0] += dx[9];
  state.gyroBias[1] += dx[10];
  state.gyroBias[2] += dx[11];
  state.accelBias[0] += dx[12];
  state.accelBias[1] += dx[13];
  state.accelBias[2] += dx[14];
  state.baroBias += dx[15];
}

// ── Magnetometer update (m=3) ──

export function magUpdate(
  state: EstimatedState, P: Float64Array, config: EKFConfig,
  magMeas: Vec3, timestamp: number,
): InnovationRecord {
  // Predicted: h = R^T * B_earth (body-frame expected field)
  qRotateVectorInverse(_hMag, state.quaternion, config.earthMagField);

  // Innovation
  _yMag[0] = magMeas[0] - _hMag[0];
  _yMag[1] = magMeas[1] - _hMag[1];
  _yMag[2] = magMeas[2] - _hMag[2];
  const innovNorm = Math.sqrt(_yMag[0] ** 2 + _yMag[1] ** 2 + _yMag[2] ** 2);

  // Jacobian H[3×16]: H[:, 6:9] = skew(h_predicted)
  _Hmag.fill(0);
  skew3(_skewHmag, _hMag);
  // skew3 is column-major, H is row-major 3×16
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      // skew column-major: skew(i,j) = _skewHmag[j*3+i]
      _Hmag[i * N + (6 + j)] = _skewHmag[j * 3 + i];
    }
  }

  // R (measurement noise)
  _Rmag.fill(0);
  _Rmag[0] = config.magNoiseVar;
  _Rmag[4] = config.magNoiseVar;
  _Rmag[8] = config.magNoiseVar;

  // S = H * P * H^T + R
  mulPHt(_PHtMag, P, _Hmag, 3);
  computeS(_Smag, _Hmag, _PHtMag, _Rmag, 3);

  // Mahalanobis gating
  _SinvMag.set(_Smag);
  if (!invertSmall(_SinvMag, 3)) {
    return { timestamp, source: 'mag', innovationNorm: innovNorm, gated: true };
  }
  const maha = mahalanobis(_yMag, _SinvMag, 3);
  if (maha > config.magGateThreshold) {
    return { timestamp, source: 'mag', innovationNorm: innovNorm, gated: true };
  }

  // Kalman gain
  computeK(_Kmag, _PHtMag, _SinvMag, 3);
  computeDx(_dxMag, _Kmag, _yMag, 3);

  // Inject error + Joseph update
  injectError(state, _dxMag);
  josephUpdate(P, _Kmag, _Hmag, _Rmag, 3, _joseph1, _joseph2);

  return { timestamp, source: 'mag', innovationNorm: innovNorm, gated: false };
}

// ── Barometer update (m=1) ──

export function baroUpdate(
  state: EstimatedState, P: Float64Array, config: EKFConfig,
  pressureMeas: number, timestamp: number,
): InnovationRecord {
  // Predicted: h = isaPressure(-p_z_est) + bbaro_est
  const altEst = Math.max(0, -state.position[2]);
  const pPred = isaPressure(altEst) + state.baroBias;

  // Innovation
  _yBaro[0] = pressureMeas - pPred;
  const innovNorm = Math.abs(_yBaro[0]);

  // Jacobian H[1×16]: H[0,2] = ρ*g (dp/dp_z), H[0,15] = 1 (bbaro)
  _Hbaro.fill(0);
  const rho = isaDensity(altEst);
  _Hbaro[2] = rho * GRAVITY_MPS2; // dp/dp_z = dp/dh * dh/dp_z = (-ρg)*(-1) = ρg
  _Hbaro[15] = 1;

  _Rbaro[0] = config.baroNoiseVar;

  mulPHt(_PHtBaro, P, _Hbaro, 1);
  computeS(_Sbaro, _Hbaro, _PHtBaro, _Rbaro, 1);

  _SinvBaro[0] = _Sbaro[0];
  if (!invertSmall(_SinvBaro, 1)) {
    return { timestamp, source: 'baro', innovationNorm: innovNorm, gated: true };
  }

  computeK(_Kbaro, _PHtBaro, _SinvBaro, 1);
  computeDx(_dxBaro, _Kbaro, _yBaro, 1);
  injectError(state, _dxBaro);
  josephUpdate(P, _Kbaro, _Hbaro, _Rbaro, 1, _joseph1, _joseph2);

  return { timestamp, source: 'baro', innovationNorm: innovNorm, gated: false };
}

// ── Rangefinder update (m=1) ──

let _lastRangeSurfaceZ = 0; // track active surface for switch gating
let _rangeSwitchCooldown = 0;

export function rangeUpdate(
  state: EstimatedState, P: Float64Array, config: EKFConfig,
  rangeMeas: number, worldGeo: WorldGeometry, timestamp: number,
): InnovationRecord {
  // Beam direction in world frame from estimated attitude
  qRotateVector(_beamWorld, state.quaternion, _beamBody);

  // Predicted range via raycast from estimated position
  const rayResult = worldGeo.raycast(state.position, _beamWorld);

  if (!rayResult.hit || _beamWorld[2] <= 0) {
    return { timestamp, source: 'range', innovationNorm: 0, gated: true };
  }

  const rangePred = rayResult.distance;

  // Surface-switch gating: detect when active surface changes
  const currentSurfaceZ = state.position[2] + _beamWorld[2] * (rangePred / Math.sqrt(
    _beamWorld[0] ** 2 + _beamWorld[1] ** 2 + _beamWorld[2] ** 2));
  if (Math.abs(currentSurfaceZ - _lastRangeSurfaceZ) > 0.5 && _rangeSwitchCooldown <= 0) {
    _rangeSwitchCooldown = 3; // skip + tighten for 3 cycles
    _lastRangeSurfaceZ = currentSurfaceZ;
    return { timestamp, source: 'range', innovationNorm: 0, gated: true };
  }
  _lastRangeSurfaceZ = currentSurfaceZ;
  if (_rangeSwitchCooldown > 0) _rangeSwitchCooldown--;

  // Innovation
  _yRange[0] = rangeMeas - rangePred;
  const innovNorm = Math.abs(_yRange[0]);

  // Jacobian H[1×16]: simplified for locally flat surface
  _Hrange.fill(0);
  const dwz = _beamWorld[2];
  if (Math.abs(dwz) > 1e-6) {
    _Hrange[2] = -1 / dwz; // d(range)/d(p_z) for flat ground
  }

  const gateThresh = _rangeSwitchCooldown > 0 ? config.rangeGateThreshold * 0.5 : config.rangeGateThreshold;

  _Rrange[0] = config.rangeNoiseVar;
  mulPHt(_PHtRange, P, _Hrange, 1);
  computeS(_Srange, _Hrange, _PHtRange, _Rrange, 1);

  _SinvRange[0] = _Srange[0];
  if (!invertSmall(_SinvRange, 1)) {
    return { timestamp, source: 'range', innovationNorm: innovNorm, gated: true };
  }

  // Innovation gating
  const normInnov = _yRange[0] * _yRange[0] * _SinvRange[0];
  if (normInnov > gateThresh) {
    return { timestamp, source: 'range', innovationNorm: innovNorm, gated: true };
  }

  computeK(_Krange, _PHtRange, _SinvRange, 1);
  computeDx(_dxRange, _Krange, _yRange, 1);
  injectError(state, _dxRange);
  josephUpdate(P, _Krange, _Hrange, _Rrange, 1, _joseph1, _joseph2);

  return { timestamp, source: 'range', innovationNorm: innovNorm, gated: false };
}

// ── Optical flow update (m=2) ──

export function flowUpdate(
  state: EstimatedState, P: Float64Array, config: EKFConfig,
  flowXMeas: number, flowYMeas: number, heightEst: number, timestamp: number,
): InnovationRecord {
  // Guard: skip if height too low
  if (heightEst < 0.1) {
    return { timestamp, source: 'flow', innovationNorm: 0, gated: true };
  }

  // Body-frame velocity
  qRotateVectorInverse(_vBody, state.quaternion, state.velocity);
  const h = heightEst;
  const invH = 1 / h;

  // Corrected angular velocity
  const omegaX = state.angularVelocity[0];
  const omegaY = state.angularVelocity[1];

  // Predicted flow
  const flowXPred = _vBody[0] * invH + omegaY;
  const flowYPred = _vBody[1] * invH - omegaX;

  // Innovation
  _yFlow[0] = flowXMeas - flowXPred;
  _yFlow[1] = flowYMeas - flowYPred;
  const innovNorm = Math.sqrt(_yFlow[0] ** 2 + _yFlow[1] ** 2);

  // Jacobian H[2×16]: velocity and attitude terms
  // flow_x = v_body_x/h + omega_y
  // flow_y = v_body_y/h - omega_x
  // v_body = R^T * v_world, so d(v_body)/d(v_world) = R^T (3×3 in cols 3-5)
  // d(v_body)/d(δθ) involves cross product with v_body
  _Hflow.fill(0);

  // Get rotation matrix
  qToRotationMatrix(_R, state.quaternion);

  // d(flow)/d(velocity) = (1/h) * R^T applied to first two body axes
  // R^T row i col j = R col-major transposed: R^T(i,j) = R[i*3+j]
  for (let j = 0; j < 3; j++) {
    _Hflow[0 * N + (3 + j)] = _R[j * 3 + 0] * invH; // flow_x depends on v via R^T row 0
    _Hflow[1 * N + (3 + j)] = _R[j * 3 + 1] * invH; // flow_y depends on v via R^T row 1
  }

  // d(flow)/d(δθ): v_body changes with attitude → [v_body]× contribution
  // d(R^T v)/dδθ = -R^T [v]× ... complex, skip for simplified model
  // Also d(omega_corrected)/d(δbg) = -I, contributing to flow through omega terms
  // Gyro bias columns (9-11):
  // d(flow_x)/d(δbg) = d(omega_y)/d(δbg_y) = -1 → H[0, 10] = -1  wait, omega_y = omega_meas_y - bg_y
  // But bg is the estimated bias that was already subtracted in propagation.
  // The error-state δbg changes the corrected omega: delta_omega_y = -delta_bg_y
  // flow_x += omega_y → d(flow_x)/d(δbg_y) = d(omega_y)/d(δbg_y) = -1
  _Hflow[0 * N + 10] = -1; // flow_x depends on δbg_y (omega_y)
  _Hflow[1 * N + 9] = 1;   // flow_y depends on δbg_x (-omega_x, so d/d(δbg_x) = -(-1) = 1)

  _Rflow.fill(0);
  _Rflow[0] = config.flowNoiseVar;
  _Rflow[3] = config.flowNoiseVar;

  mulPHt(_PHtFlow, P, _Hflow, 2);
  computeS(_Sflow, _Hflow, _PHtFlow, _Rflow, 2);

  _SinvFlow.set(_Sflow);
  if (!invertSmall(_SinvFlow, 2)) {
    return { timestamp, source: 'flow', innovationNorm: innovNorm, gated: true };
  }

  const maha = mahalanobis(_yFlow, _SinvFlow, 2);
  if (maha > config.flowGateThreshold) {
    return { timestamp, source: 'flow', innovationNorm: innovNorm, gated: true };
  }

  computeK(_Kflow, _PHtFlow, _SinvFlow, 2);
  computeDx(_dxFlow, _Kflow, _yFlow, 2);
  injectError(state, _dxFlow);
  josephUpdate(P, _Kflow, _Hflow, _Rflow, 2, _joseph1, _joseph2);

  return { timestamp, source: 'flow', innovationNorm: innovNorm, gated: false };
}

/** Reset range surface tracking state. */
export function resetRangeGating(): void {
  _lastRangeSurfaceZ = 0;
  _rangeSwitchCooldown = 0;
}

// ── VIO Position Update (m=3) — THE XY ANCHOR ──

const _yVioPos = new Float64Array(3);
const _HvioPos = new Float64Array(3 * N);
const _PHtVioPos = new Float64Array(N * 3);
const _SvioPos = new Float64Array(9);
const _SinvVioPos = new Float64Array(9);
const _KvioPos = new Float64Array(N * 3);
const _RvioPos = new Float64Array(9);
const _dxVioPos = new Float64Array(N);

export function vioPositionUpdate(
  state: EstimatedState, P: Float64Array, config: EKFConfig,
  positionMeas: Vec3, noiseVar: number, timestamp: number,
): InnovationRecord {
  _yVioPos[0] = positionMeas[0] - state.position[0];
  _yVioPos[1] = positionMeas[1] - state.position[1];
  _yVioPos[2] = positionMeas[2] - state.position[2];
  const innovNorm = Math.sqrt(_yVioPos[0] ** 2 + _yVioPos[1] ** 2 + _yVioPos[2] ** 2);

  // H[3×16]: H[:, 0:3] = I₃ (direct position observation)
  _HvioPos.fill(0);
  _HvioPos[0 * N + 0] = 1;
  _HvioPos[1 * N + 1] = 1;
  _HvioPos[2 * N + 2] = 1;

  _RvioPos.fill(0);
  _RvioPos[0] = noiseVar; _RvioPos[4] = noiseVar; _RvioPos[8] = noiseVar;

  mulPHt(_PHtVioPos, P, _HvioPos, 3);
  computeS(_SvioPos, _HvioPos, _PHtVioPos, _RvioPos, 3);
  _SinvVioPos.set(_SvioPos);
  if (!invertSmall(_SinvVioPos, 3)) {
    return { timestamp, source: 'vio-pos', innovationNorm: innovNorm, gated: true };
  }
  const maha = mahalanobis(_yVioPos, _SinvVioPos, 3);
  if (maha > config.vioGateThreshold) {
    return { timestamp, source: 'vio-pos', innovationNorm: innovNorm, gated: true };
  }

  computeK(_KvioPos, _PHtVioPos, _SinvVioPos, 3);
  computeDx(_dxVioPos, _KvioPos, _yVioPos, 3);
  injectError(state, _dxVioPos);
  josephUpdate(P, _KvioPos, _HvioPos, _RvioPos, 3, _joseph1, _joseph2);

  return { timestamp, source: 'vio-pos', innovationNorm: innovNorm, gated: false };
}

// ── VIO Attitude Update (m=3) ──

const _yVioAtt = new Float64Array(3);
const _HvioAtt = new Float64Array(3 * N);
const _PHtVioAtt = new Float64Array(N * 3);
const _SvioAtt = new Float64Array(9);
const _SinvVioAtt = new Float64Array(9);
const _KvioAtt = new Float64Array(N * 3);
const _RvioAtt = new Float64Array(9);
const _dxVioAtt = new Float64Array(N);
const _qErrVio = qCreate();
const _qConjNomVio = qCreate();

export function vioAttitudeUpdate(
  state: EstimatedState, P: Float64Array, config: EKFConfig,
  quatMeas: Float64Array, noiseVar: number, timestamp: number,
): InnovationRecord {
  // Error quaternion: q_err = conj(q_nom) * q_meas
  qConjugate(_qConjNomVio, state.quaternion);
  qMultiply(_qErrVio, _qConjNomVio, quatMeas);
  // Ensure shortest path (w > 0)
  if (_qErrVio[0] < 0) {
    _qErrVio[0] = -_qErrVio[0]; _qErrVio[1] = -_qErrVio[1];
    _qErrVio[2] = -_qErrVio[2]; _qErrVio[3] = -_qErrVio[3];
  }
  // Innovation: y = 2 * q_err.xyz (small-angle)
  _yVioAtt[0] = 2 * _qErrVio[1];
  _yVioAtt[1] = 2 * _qErrVio[2];
  _yVioAtt[2] = 2 * _qErrVio[3];
  const innovNorm = Math.sqrt(_yVioAtt[0] ** 2 + _yVioAtt[1] ** 2 + _yVioAtt[2] ** 2);

  // H[3×16]: H[:, 6:9] = I₃
  _HvioAtt.fill(0);
  _HvioAtt[0 * N + 6] = 1;
  _HvioAtt[1 * N + 7] = 1;
  _HvioAtt[2 * N + 8] = 1;

  _RvioAtt.fill(0);
  _RvioAtt[0] = noiseVar; _RvioAtt[4] = noiseVar; _RvioAtt[8] = noiseVar;

  mulPHt(_PHtVioAtt, P, _HvioAtt, 3);
  computeS(_SvioAtt, _HvioAtt, _PHtVioAtt, _RvioAtt, 3);
  _SinvVioAtt.set(_SvioAtt);
  if (!invertSmall(_SinvVioAtt, 3)) {
    return { timestamp, source: 'vio-att', innovationNorm: innovNorm, gated: true };
  }
  const maha = mahalanobis(_yVioAtt, _SinvVioAtt, 3);
  if (maha > config.vioGateThreshold) {
    return { timestamp, source: 'vio-att', innovationNorm: innovNorm, gated: true };
  }

  computeK(_KvioAtt, _PHtVioAtt, _SinvVioAtt, 3);
  computeDx(_dxVioAtt, _KvioAtt, _yVioAtt, 3);
  injectError(state, _dxVioAtt);
  josephUpdate(P, _KvioAtt, _HvioAtt, _RvioAtt, 3, _joseph1, _joseph2);

  return { timestamp, source: 'vio-att', innovationNorm: innovNorm, gated: false };
}

// ── UWB Range Update (m=1, per anchor) ──

const _yUwb = new Float64Array(1);
const _Huwb = new Float64Array(N);
const _PHtUwb = new Float64Array(N);
const _Suwb = new Float64Array(1);
const _SinvUwb = new Float64Array(1);
const _Kuwb = new Float64Array(N);
const _Ruwb = new Float64Array(1);
const _dxUwb = new Float64Array(N);

export function uwbRangeUpdate(
  state: EstimatedState, P: Float64Array, config: EKFConfig,
  rangeMeas: number, anchorPos: Vec3, isNLOS: boolean, timestamp?: number,
  noiseVarOverride?: number,
): InnovationRecord {
  const ts = timestamp ?? state.timestamp;
  const dx = state.position[0] - anchorPos[0];
  const dy = state.position[1] - anchorPos[1];
  const dz = state.position[2] - anchorPos[2];
  const predRange = Math.sqrt(dx * dx + dy * dy + dz * dz);

  if (predRange < 0.01) {
    return { timestamp: ts, source: 'uwb', innovationNorm: 0, gated: true };
  }

  _yUwb[0] = rangeMeas - predRange;
  const innovNorm = Math.abs(_yUwb[0]);

  _Huwb.fill(0);
  const invRange = 1 / predRange;
  _Huwb[0] = dx * invRange;
  _Huwb[1] = dy * invRange;
  _Huwb[2] = dz * invRange;

  _Ruwb[0] = noiseVarOverride ?? (isNLOS ? config.uwbNlosNoiseVar : config.uwbLosNoiseVar);

  mulPHt(_PHtUwb, P, _Huwb, 1);
  computeS(_Suwb, _Huwb, _PHtUwb, _Ruwb, 1);
  _SinvUwb[0] = _Suwb[0];
  if (!invertSmall(_SinvUwb, 1)) {
    return { timestamp: ts, source: 'uwb', innovationNorm: innovNorm, gated: true };
  }

  const normInnov = _yUwb[0] * _yUwb[0] * _SinvUwb[0];
  if (normInnov > config.uwbGateThreshold) {
    return { timestamp: ts, source: 'uwb', innovationNorm: innovNorm, gated: true };
  }

  computeK(_Kuwb, _PHtUwb, _SinvUwb, 1);
  computeDx(_dxUwb, _Kuwb, _yUwb, 1);
  injectError(state, _dxUwb);
  josephUpdate(P, _Kuwb, _Huwb, _Ruwb, 1, _joseph1, _joseph2);

  return { timestamp: ts, source: 'uwb', innovationNorm: innovNorm, gated: false };
}
