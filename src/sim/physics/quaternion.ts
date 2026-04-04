/**
 * Hamilton quaternion math, scalar-first [w, x, y, z].
 * All operations are allocation-free: write into pre-allocated output.
 */

import type { Quat, Vec3, Mat3 } from '@sim/core/types';

export function qCreate(w = 1, x = 0, y = 0, z = 0): Quat {
  const q = new Float64Array(4);
  q[0] = w; q[1] = x; q[2] = y; q[3] = z;
  return q;
}

export function qIdentity(out: Quat): Quat {
  out[0] = 1; out[1] = 0; out[2] = 0; out[3] = 0;
  return out;
}

export function qCopy(out: Quat, a: Quat): Quat {
  out[0] = a[0]; out[1] = a[1]; out[2] = a[2]; out[3] = a[3];
  return out;
}

/**
 * Hamilton quaternion multiply: out = a * b.
 * out may alias neither a nor b.
 */
export function qMultiply(out: Quat, a: Quat, b: Quat): Quat {
  const aw = a[0], ax = a[1], ay = a[2], az = a[3];
  const bw = b[0], bx = b[1], by = b[2], bz = b[3];
  out[0] = aw * bw - ax * bx - ay * by - az * bz;
  out[1] = aw * bx + ax * bw + ay * bz - az * by;
  out[2] = aw * by - ax * bz + ay * bw + az * bx;
  out[3] = aw * bz + ax * by - ay * bx + az * bw;
  return out;
}

/** Conjugate: out = conj(a) = [w, -x, -y, -z]. */
export function qConjugate(out: Quat, a: Quat): Quat {
  out[0] = a[0]; out[1] = -a[1]; out[2] = -a[2]; out[3] = -a[3];
  return out;
}

/** Normalize to unit quaternion. */
export function qNormalize(out: Quat, a: Quat): Quat {
  const len = Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
  if (len > 1e-15) {
    const inv = 1 / len;
    out[0] = a[0] * inv; out[1] = a[1] * inv;
    out[2] = a[2] * inv; out[3] = a[3] * inv;
  } else {
    out[0] = 1; out[1] = 0; out[2] = 0; out[3] = 0;
  }
  return out;
}

export function qNorm(a: Quat): number {
  return Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
}

/**
 * Rotate vector by quaternion: v_world = q * [0,v] * conj(q).
 * Optimized formula avoiding double multiply.
 */
export function qRotateVector(out: Vec3, q: Quat, v: Vec3): Vec3 {
  const qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  const vx = v[0], vy = v[1], vz = v[2];

  // t = 2 * cross(q.xyz, v)
  const tx = 2 * (qy * vz - qz * vy);
  const ty = 2 * (qz * vx - qx * vz);
  const tz = 2 * (qx * vy - qy * vx);

  // result = v + qw * t + cross(q.xyz, t)
  out[0] = vx + qw * tx + (qy * tz - qz * ty);
  out[1] = vy + qw * ty + (qz * tx - qx * tz);
  out[2] = vz + qw * tz + (qx * ty - qy * tx);
  return out;
}

/**
 * Rotate vector by conjugate (inverse rotation): v_body = conj(q) * [0,v] * q.
 */
export function qRotateVectorInverse(out: Vec3, q: Quat, v: Vec3): Vec3 {
  const qw = q[0], qx = -q[1], qy = -q[2], qz = -q[3];
  const vx = v[0], vy = v[1], vz = v[2];
  const tx = 2 * (qy * vz - qz * vy);
  const ty = 2 * (qz * vx - qx * vz);
  const tz = 2 * (qx * vy - qy * vx);
  out[0] = vx + qw * tx + (qy * tz - qz * ty);
  out[1] = vy + qw * ty + (qz * tx - qx * tz);
  out[2] = vz + qw * tz + (qx * ty - qy * tx);
  return out;
}

/** From axis-angle. axis must be unit length. */
export function qFromAxisAngle(out: Quat, axis: Vec3, angle: number): Quat {
  const half = angle * 0.5;
  const s = Math.sin(half);
  out[0] = Math.cos(half);
  out[1] = axis[0] * s;
  out[2] = axis[1] * s;
  out[3] = axis[2] * s;
  return out;
}

/**
 * Convert quaternion to rotation matrix (column-major Mat3).
 * Rotates body->world: v_w = R * v_b.
 */
export function qToRotationMatrix(out: Mat3, q: Quat): Mat3 {
  const w = q[0], x = q[1], y = q[2], z = q[3];
  const xx = x * x, yy = y * y, zz = z * z;
  const xy = x * y, xz = x * z, yz = y * z;
  const wx = w * x, wy = w * y, wz = w * z;

  // Column 0
  out[0] = 1 - 2 * (yy + zz);
  out[1] = 2 * (xy + wz);
  out[2] = 2 * (xz - wy);
  // Column 1
  out[3] = 2 * (xy - wz);
  out[4] = 1 - 2 * (xx + zz);
  out[5] = 2 * (yz + wx);
  // Column 2
  out[6] = 2 * (xz + wy);
  out[7] = 2 * (yz - wx);
  out[8] = 1 - 2 * (xx + yy);
  return out;
}

/**
 * ZYX intrinsic Euler angles (yaw, pitch, roll) to quaternion.
 * Aerospace convention: rotate yaw(z) then pitch(y) then roll(x).
 */
export function qFromEulerZYX(out: Quat, roll: number, pitch: number, yaw: number): Quat {
  const cr = Math.cos(roll * 0.5), sr = Math.sin(roll * 0.5);
  const cp = Math.cos(pitch * 0.5), sp = Math.sin(pitch * 0.5);
  const cy = Math.cos(yaw * 0.5), sy = Math.sin(yaw * 0.5);

  out[0] = cr * cp * cy + sr * sp * sy;
  out[1] = sr * cp * cy - cr * sp * sy;
  out[2] = cr * sp * cy + sr * cp * sy;
  out[3] = cr * cp * sy - sr * sp * cy;
  return out;
}

/**
 * Quaternion to ZYX Euler angles: [roll, pitch, yaw].
 * Returns angles in radians. Pitch in [-pi/2, pi/2].
 */
export function qToEulerZYX(q: Quat): [number, number, number] {
  const w = q[0], x = q[1], y = q[2], z = q[3];

  // Roll (x-axis rotation)
  const sinr_cosp = 2 * (w * x + y * z);
  const cosr_cosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  const sinp = 2 * (w * y - z * x);
  const pitch = Math.abs(sinp) >= 1
    ? Math.sign(sinp) * (Math.PI / 2)
    : Math.asin(sinp);

  // Yaw (z-axis rotation)
  const siny_cosp = 2 * (w * z + x * y);
  const cosy_cosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);

  return [roll, pitch, yaw];
}

/**
 * Quaternion kinematic derivative: q_dot = 0.5 * q * [0, omega_body].
 * Used for attitude propagation in the integrator.
 */
export function qDerivative(out: Quat, q: Quat, omega: Vec3): Quat {
  const qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  const wx = omega[0], wy = omega[1], wz = omega[2];

  // out = 0.5 * q * [0, omega]
  out[0] = 0.5 * (-qx * wx - qy * wy - qz * wz);
  out[1] = 0.5 * ( qw * wx + qy * wz - qz * wy);
  out[2] = 0.5 * ( qw * wy - qx * wz + qz * wx);
  out[3] = 0.5 * ( qw * wz + qx * wy - qy * wx);
  return out;
}

/** Scale quaternion: out = a * s. */
export function qScale(out: Quat, a: Quat, s: number): Quat {
  out[0] = a[0] * s; out[1] = a[1] * s;
  out[2] = a[2] * s; out[3] = a[3] * s;
  return out;
}

/** Add quaternions: out = a + b. */
export function qAdd(out: Quat, a: Quat, b: Quat): Quat {
  out[0] = a[0] + b[0]; out[1] = a[1] + b[1];
  out[2] = a[2] + b[2]; out[3] = a[3] + b[3];
  return out;
}

/** out = a + b * s (scale-add). */
export function qScaleAdd(out: Quat, a: Quat, b: Quat, s: number): Quat {
  out[0] = a[0] + b[0] * s; out[1] = a[1] + b[1] * s;
  out[2] = a[2] + b[2] * s; out[3] = a[3] + b[3] * s;
  return out;
}

/**
 * Convert rotation matrix (column-major Mat3) to quaternion.
 * Shepperd method — numerically robust for all orientations.
 */
export function qFromRotationMatrix(out: Quat, R: Mat3): Quat {
  // Column-major: R(row, col) = R[col*3 + row]
  const r00 = R[0], r10 = R[1], r20 = R[2];
  const r01 = R[3], r11 = R[4], r21 = R[5];
  const r02 = R[6], r12 = R[7], r22 = R[8];

  const trace = r00 + r11 + r22;

  if (trace > 0) {
    const s = 2 * Math.sqrt(trace + 1);
    out[0] = 0.25 * s;
    out[1] = (r21 - r12) / s;
    out[2] = (r02 - r20) / s;
    out[3] = (r10 - r01) / s;
  } else if (r00 > r11 && r00 > r22) {
    const s = 2 * Math.sqrt(1 + r00 - r11 - r22);
    out[0] = (r21 - r12) / s;
    out[1] = 0.25 * s;
    out[2] = (r01 + r10) / s;
    out[3] = (r02 + r20) / s;
  } else if (r11 > r22) {
    const s = 2 * Math.sqrt(1 + r11 - r00 - r22);
    out[0] = (r02 - r20) / s;
    out[1] = (r01 + r10) / s;
    out[2] = 0.25 * s;
    out[3] = (r12 + r21) / s;
  } else {
    const s = 2 * Math.sqrt(1 + r22 - r00 - r11);
    out[0] = (r10 - r01) / s;
    out[1] = (r02 + r20) / s;
    out[2] = (r12 + r21) / s;
    out[3] = 0.25 * s;
  }

  // Ensure consistent sign (w positive)
  if (out[0] < 0) {
    out[0] = -out[0]; out[1] = -out[1]; out[2] = -out[2]; out[3] = -out[3];
  }

  return qNormalize(out, out);
}
