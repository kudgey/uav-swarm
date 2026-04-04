/**
 * 6DoF rigid-body dynamics derivative function.
 * STATUS: experimental
 *
 * Computes state derivative for a quadrotor given current state and environment.
 *
 * Translational dynamics (world frame, NED):
 *   dot(p) = v
 *   m * dot(v) = F_gravity_w + F_thrust_w + F_drag_w + F_env_w
 *
 * Rotational dynamics (body frame, FRD):
 *   dot(q) = 0.5 * q * [0, omega_b]
 *   J * dot(omega) = tau_total - omega × (J * omega)
 *
 * Gyroscopic rotor torque: DEFERRED TO PHASE 2.
 *   Correct form: tau_gyro = -omega_b × [0, 0, J_rotor * sum(s_i * Omega_i)]
 *   Omitted because J_rotor is not yet in DroneParams.
 *   Effect is small at hover/moderate speeds.
 *
 * Motor dynamics:
 *   Handled by the hybrid actuator model (motor-model.ts).
 *   The derivative for motor speeds is computed as (omegaNew - omega) / dt,
 *   where omegaNew comes from the actuator model step.
 */

import { v3Create, v3Cross, v3Zero, m3MulV3, m3Create, m3DiagInv, m3FromDiag } from '@lib/math';
import { qRotateVector, qRotateVectorInverse, qDerivative } from './quaternion';
import { computeBodyDrag } from './drag';
import { stepMotor } from '@sim/actuators/motor-model';
import type { DroneState, DroneStateDerivative, DroneParams, EnvironmentOutput } from '@sim/core/types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('6dof-physics', 'experimental',
  'Rigid-body 6DoF dynamics without gyroscopic rotor torque (deferred to Phase 2)');

// Pre-allocated scratch buffers (module-level, never re-created)
const _gravity = v3Create();
const _thrustBody = v3Create();
const _thrustWorld = v3Create();
const _dragBody = v3Create();
const _dragWorld = v3Create();
const _vRelBody = v3Create();
const _tauThrust = v3Create();
const _Jomega = v3Create();
const _omegaCrossJomega = v3Create();
const _Jinv = m3Create();
const _Jmat = m3Create();
// _angAccel removed - intermediate written directly into out.dAngularVelocity
const _tmp3 = v3Create();
// Pre-allocated per-rotor thrust scratch (max 8 rotors; avoids per-call allocation)
const _rotorThrusts = new Float64Array(8);

/**
 * Compute the full state derivative for a single quadrotor.
 *
 * @param out    Output derivative (pre-allocated)
 * @param state  Current drone state
 * @param params Drone parameters
 * @param env    Environment output (wind, gravity, density)
 * @param dt     Current integration substep (for motor model)
 */
export function computeDerivative(
  out: DroneStateDerivative,
  state: DroneState,
  params: DroneParams,
  env: EnvironmentOutput,
  dt: number,
): void {
  const { position: _p, velocity, quaternion, angularVelocity, motorSpeeds, motorCommands } = state;
  const { mass, inertia, kT, kQ, rotorDirections, numRotors } = params;

  // ── Position derivative ──
  out.dPosition[0] = velocity[0];
  out.dPosition[1] = velocity[1];
  out.dPosition[2] = velocity[2];

  // ── Gravity from environment (world frame) ──
  _gravity[0] = env.gravity[0];
  _gravity[1] = env.gravity[1];
  _gravity[2] = env.gravity[2];

  // ── Total thrust in body frame: [0, 0, -sum(Ti)] with ground effect ──
  let totalThrust = 0;
  for (let i = 0; i < numRotors; i++) {
    totalThrust += kT * motorSpeeds[i] * motorSpeeds[i];
  }
  totalThrust *= env.groundEffectMultiplier;
  v3Zero(_thrustBody);
  _thrustBody[2] = -totalThrust;  // Upward = -z in FRD

  // Rotate thrust to world frame
  qRotateVector(_thrustWorld, quaternion, _thrustBody);

  // ── Body drag in body frame ──
  // Air-relative velocity in body frame: v_rel_body = R^T * (v_world - wind_world)
  _tmp3[0] = velocity[0] - env.wind[0];
  _tmp3[1] = velocity[1] - env.wind[1];
  _tmp3[2] = velocity[2] - env.wind[2];
  qRotateVectorInverse(_vRelBody, quaternion, _tmp3);
  computeBodyDrag(_dragBody, _vRelBody, params);

  // Rotate drag to world frame
  qRotateVector(_dragWorld, quaternion, _dragBody);

  // ── Translational acceleration: a = g + (F_thrust + F_drag) / m ──
  out.dVelocity[0] = _gravity[0] + (_thrustWorld[0] + _dragWorld[0]) / mass;
  out.dVelocity[1] = _gravity[1] + (_thrustWorld[1] + _dragWorld[1]) / mass;
  out.dVelocity[2] = _gravity[2] + (_thrustWorld[2] + _dragWorld[2]) / mass;

  // ── Quaternion derivative: q_dot = 0.5 * q * [0, omega] ──
  qDerivative(out.dQuaternion, quaternion, angularVelocity);

  // ── Rotational dynamics: J * alpha = tau_total - omega × (J * omega) ──

  // Thrust-generated torques (body frame)
  // tau_x = d * (T1 - T2 - T3 + T4)
  // tau_y = d * (T1 + T2 - T3 - T4)
  // tau_z = sum(s_i * kQ * Omega_i^2)
  const d = params.armLength / Math.SQRT2;
  for (let i = 0; i < numRotors; i++) {
    _rotorThrusts[i] = kT * motorSpeeds[i] * motorSpeeds[i];
  }

  _tauThrust[0] = d * (_rotorThrusts[0] - _rotorThrusts[1] - _rotorThrusts[2] + _rotorThrusts[3]);
  _tauThrust[1] = d * (_rotorThrusts[0] + _rotorThrusts[1] - _rotorThrusts[2] - _rotorThrusts[3]);
  _tauThrust[2] = 0;
  for (let i = 0; i < numRotors; i++) {
    _tauThrust[2] += rotorDirections[i] * kQ * motorSpeeds[i] * motorSpeeds[i];
  }

  // J * omega
  m3FromDiag(_Jmat, inertia);
  m3MulV3(_Jomega, _Jmat, angularVelocity);

  // omega × (J * omega)
  v3Cross(_omegaCrossJomega, angularVelocity, _Jomega);

  // Gyroscopic rotor torque: τ_gyro = -ω × [0, 0, J_rotor * Σ(s_i * Ω_i)]
  if (params.rotorInertia > 0) {
    let rotorAngMomentum = 0;
    for (let i = 0; i < numRotors; i++) {
      rotorAngMomentum += rotorDirections[i] * motorSpeeds[i];
    }
    rotorAngMomentum *= params.rotorInertia;
    // τ_gyro = -ω × [0, 0, h] = [ω_y * h, -ω_x * h, 0]
    _tauThrust[0] += angularVelocity[1] * rotorAngMomentum;
    _tauThrust[1] -= angularVelocity[0] * rotorAngMomentum;
  }

  // alpha = J^-1 * (tau - omega × J*omega)
  m3DiagInv(_Jinv, inertia);
  _tmp3[0] = _tauThrust[0] - _omegaCrossJomega[0];
  _tmp3[1] = _tauThrust[1] - _omegaCrossJomega[1];
  _tmp3[2] = _tauThrust[2] - _omegaCrossJomega[2];
  m3MulV3(out.dAngularVelocity, _Jinv, _tmp3);

  // ── Motor speed derivatives ──
  // Use hybrid actuator model to compute next motor speeds,
  // then express as rate for RK4 compatibility
  for (let i = 0; i < numRotors; i++) {
    const omegaNew = stepMotor(motorSpeeds[i], motorCommands[i], dt, params);
    out.dMotorSpeeds[i] = (omegaNew - motorSpeeds[i]) / dt;
  }
}
