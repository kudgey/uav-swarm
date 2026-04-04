/**
 * Outer loop: position PD → desired thrust + attitude.
 * STATUS: experimental
 *
 * F_des = mass * (a_des - kp*(p-p_des) - kd*(v-v_des) - g_NED)
 * thrust = |F_des|
 * z_body_des = -F_des / |F_des|
 * q_des = buildQuaternionFromZAxisAndYaw(z_body_des, yawDes)
 */

import { v3Create, v3Len, v3Normalize, v3Cross, m3Create } from '@lib/math';
import { qCreate, qFromRotationMatrix, qNormalize } from '@sim/physics/quaternion';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import type { Vec3, Quat, DroneParams, Mat3 } from '@sim/core/types';
import type { EstimatedState } from '@sim/estimation/estimator-types';
import type { GuidanceOutput, ControllerConfig } from './controller-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('outer-loop', 'experimental', 'Position PD with tilt limiting');

// Pre-allocated scratch
const _Fdes = v3Create();
const _zDes = v3Create();
const _xDes = v3Create();
const _yDes = v3Create();
const _yawDir = v3Create();
const _Rdes = m3Create();

export interface OuterLoopOutput {
  thrust: number;
  desiredQuat: Quat;
}

const _qDes = qCreate();
const _outRef: OuterLoopOutput = { thrust: 0, desiredQuat: _qDes };

/**
 * Compute desired thrust and attitude from position/velocity error.
 */
export function computeOuterLoop(
  estimate: EstimatedState,
  guidance: GuidanceOutput,
  params: DroneParams,
  config: ControllerConfig,
): OuterLoopOutput {
  const m = params.mass;

  // F_des = m * (a_des - kp*(p-p_des) - kd*(v-v_des) - g_NED)
  // g_NED = [0, 0, +9.81], so -g_NED = [0, 0, -9.81]
  for (let i = 0; i < 3; i++) {
    const posErr = estimate.position[i] - guidance.positionDes[i];
    const velErr = estimate.velocity[i] - guidance.velocityDes[i];
    _Fdes[i] = m * (guidance.accelerationDes[i]
      - config.kpPos[i] * posErr
      - config.kdPos[i] * velErr);
  }
  // Subtract gravity: F_thrust = m*a_des - m*g → need to add -m*g component
  _Fdes[2] -= m * GRAVITY_MPS2; // In NED: thrust must overcome gravity (pointing up = negative z)

  let thrust: number;

  // Desired body z-axis in world frame: z_des = -F_des / |F_des|
  // In NED, body z normally points down ([0,0,+1]). Thrust acts along -body_z (upward).
  // If Fdes[2] is strongly positive (need net downward force to descend), -Fdes/|Fdes| would
  // point body z upward, inverting thrust. Instead: go level and cut thrust — let gravity descend.
  if (_Fdes[2] > 0) {
    // Drone needs to descend: keep level, reduce thrust below hover to let gravity pull down.
    // Horizontal components still generate tilt for XY tracking.
    _Fdes[2] = 0; // remove upward-pointing component, keep only horizontal
    thrust = v3Len(_Fdes);
    // Set thrust below hover so drone descends under gravity
    const hoverThrust = m * GRAVITY_MPS2;
    // Scale: more Fdes_z was requested → less thrust (faster descent)
    thrust = Math.max(config.minThrust, Math.min(hoverThrust * 0.5, thrust));
  } else {
    thrust = v3Len(_Fdes);
    thrust = Math.max(config.minThrust, Math.min(config.maxThrust, thrust));
  }

  if (thrust > 1e-6) {
    _zDes[0] = -_Fdes[0] / v3Len(_Fdes) || 0;
    _zDes[1] = -_Fdes[1] / v3Len(_Fdes) || 0;
    _zDes[2] = -_Fdes[2] / v3Len(_Fdes) || 1;
    // Ensure z_des has positive z (body z points downward in NED)
    if (_zDes[2] < 0) {
      _zDes[0] = 0; _zDes[1] = 0; _zDes[2] = 1;
    }
  } else {
    _zDes[0] = 0; _zDes[1] = 0; _zDes[2] = 1; // default: body z = world down
  }

  // Tilt limiting: clamp angle between z_des and [0,0,1] (NED down)
  const cosAngle = _zDes[2]; // dot(z_des, [0,0,1])
  const cosLimit = Math.cos(config.maxTiltRad);
  if (cosAngle < cosLimit) {
    const t = cosLimit;
    const xyLen = Math.sqrt(_zDes[0] * _zDes[0] + _zDes[1] * _zDes[1]);
    if (xyLen > 1e-10) {
      const sinAngle = Math.sin(config.maxTiltRad);
      _zDes[0] = _zDes[0] / xyLen * sinAngle;
      _zDes[1] = _zDes[1] / xyLen * sinAngle;
      _zDes[2] = t;
    }
  }

  // Build rotation matrix from z_des + yaw
  // x_des direction from yaw projected perpendicular to z_des
  _yawDir[0] = Math.cos(guidance.yawDes); // North component
  _yawDir[1] = Math.sin(guidance.yawDes); // East component
  _yawDir[2] = 0;

  // y_des = cross(z_des, yaw_dir); normalize
  v3Cross(_yDes, _zDes, _yawDir);
  const yLen = v3Len(_yDes);
  if (yLen > 1e-10) {
    _yDes[0] /= yLen; _yDes[1] /= yLen; _yDes[2] /= yLen;
  } else {
    // Singularity: z_des aligned with yaw_dir, pick arbitrary perpendicular
    _yDes[0] = -Math.sin(guidance.yawDes);
    _yDes[1] = Math.cos(guidance.yawDes);
    _yDes[2] = 0;
  }

  // x_des = cross(y_des, z_des)
  v3Cross(_xDes, _yDes, _zDes);
  v3Normalize(_xDes, _xDes);

  // Build R_des column-major: [x_des | y_des | z_des]
  _Rdes[0] = _xDes[0]; _Rdes[1] = _xDes[1]; _Rdes[2] = _xDes[2]; // col 0
  _Rdes[3] = _yDes[0]; _Rdes[4] = _yDes[1]; _Rdes[5] = _yDes[2]; // col 1
  _Rdes[6] = _zDes[0]; _Rdes[7] = _zDes[1]; _Rdes[8] = _zDes[2]; // col 2

  qFromRotationMatrix(_qDes, _Rdes);

  _outRef.thrust = thrust;
  _outRef.desiredQuat = _qDes;
  return _outRef;
}
