/**
 * Allocation-free vector and matrix math on Float64Array.
 * All functions write into pre-allocated output parameters.
 */

// ── Vec3 helpers ──

export function v3Create(x = 0, y = 0, z = 0): Float64Array {
  const v = new Float64Array(3);
  v[0] = x; v[1] = y; v[2] = z;
  return v;
}

export function v3Set(out: Float64Array, x: number, y: number, z: number): Float64Array {
  out[0] = x; out[1] = y; out[2] = z;
  return out;
}

export function v3Copy(out: Float64Array, a: Float64Array): Float64Array {
  out[0] = a[0]; out[1] = a[1]; out[2] = a[2];
  return out;
}

export function v3Add(out: Float64Array, a: Float64Array, b: Float64Array): Float64Array {
  out[0] = a[0] + b[0]; out[1] = a[1] + b[1]; out[2] = a[2] + b[2];
  return out;
}

export function v3Sub(out: Float64Array, a: Float64Array, b: Float64Array): Float64Array {
  out[0] = a[0] - b[0]; out[1] = a[1] - b[1]; out[2] = a[2] - b[2];
  return out;
}

export function v3Scale(out: Float64Array, a: Float64Array, s: number): Float64Array {
  out[0] = a[0] * s; out[1] = a[1] * s; out[2] = a[2] * s;
  return out;
}

export function v3ScaleAdd(out: Float64Array, a: Float64Array, b: Float64Array, s: number): Float64Array {
  out[0] = a[0] + b[0] * s; out[1] = a[1] + b[1] * s; out[2] = a[2] + b[2] * s;
  return out;
}

export function v3Dot(a: Float64Array, b: Float64Array): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function v3Cross(out: Float64Array, a: Float64Array, b: Float64Array): Float64Array {
  const ax = a[0], ay = a[1], az = a[2];
  const bx = b[0], by = b[1], bz = b[2];
  out[0] = ay * bz - az * by;
  out[1] = az * bx - ax * bz;
  out[2] = ax * by - ay * bx;
  return out;
}

export function v3Len(a: Float64Array): number {
  return Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

export function v3LenSq(a: Float64Array): number {
  return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
}

export function v3Normalize(out: Float64Array, a: Float64Array): Float64Array {
  const len = v3Len(a);
  if (len > 1e-15) {
    const inv = 1 / len;
    out[0] = a[0] * inv; out[1] = a[1] * inv; out[2] = a[2] * inv;
  } else {
    out[0] = 0; out[1] = 0; out[2] = 0;
  }
  return out;
}

export function v3Negate(out: Float64Array, a: Float64Array): Float64Array {
  out[0] = -a[0]; out[1] = -a[1]; out[2] = -a[2];
  return out;
}

export function v3Zero(out: Float64Array): Float64Array {
  out[0] = 0; out[1] = 0; out[2] = 0;
  return out;
}

export function v3MulElementwise(out: Float64Array, a: Float64Array, b: Float64Array): Float64Array {
  out[0] = a[0] * b[0]; out[1] = a[1] * b[1]; out[2] = a[2] * b[2];
  return out;
}

// ── Mat3 helpers (column-major Float64Array(9)) ──

export function m3Create(): Float64Array {
  return new Float64Array(9);
}

export function m3Identity(out: Float64Array): Float64Array {
  out[0] = 1; out[1] = 0; out[2] = 0;
  out[3] = 0; out[4] = 1; out[5] = 0;
  out[6] = 0; out[7] = 0; out[8] = 1;
  return out;
}

/** Matrix-vector multiply: out = M * v (column-major M) */
export function m3MulV3(out: Float64Array, m: Float64Array, v: Float64Array): Float64Array {
  const x = v[0], y = v[1], z = v[2];
  out[0] = m[0] * x + m[3] * y + m[6] * z;
  out[1] = m[1] * x + m[4] * y + m[7] * z;
  out[2] = m[2] * x + m[5] * y + m[8] * z;
  return out;
}

/** Diagonal 3x3 inverse: out = diag(1/a, 1/b, 1/c) for diagonal matrix */
export function m3DiagInv(out: Float64Array, diag: Float64Array): Float64Array {
  out[0] = 1 / diag[0]; out[1] = 0; out[2] = 0;
  out[3] = 0; out[4] = 1 / diag[1]; out[5] = 0;
  out[6] = 0; out[7] = 0; out[8] = 1 / diag[2];
  return out;
}

/** Set diagonal matrix from Vec3 */
export function m3FromDiag(out: Float64Array, diag: Float64Array): Float64Array {
  out[0] = diag[0]; out[1] = 0; out[2] = 0;
  out[3] = 0; out[4] = diag[1]; out[5] = 0;
  out[6] = 0; out[7] = 0; out[8] = diag[2];
  return out;
}

/** 3×3 matrix multiply: out = A * B (column-major). out must NOT alias A or B. */
export function m3Mul(out: Float64Array, A: Float64Array, B: Float64Array): Float64Array {
  for (let j = 0; j < 3; j++) {
    for (let i = 0; i < 3; i++) {
      out[j * 3 + i] = A[i] * B[j * 3] + A[3 + i] * B[j * 3 + 1] + A[6 + i] * B[j * 3 + 2];
    }
  }
  return out;
}

/** 3×3 transpose (column-major). */
export function m3Transpose(out: Float64Array, A: Float64Array): Float64Array {
  out[0] = A[0]; out[1] = A[3]; out[2] = A[6];
  out[3] = A[1]; out[4] = A[4]; out[5] = A[7];
  out[6] = A[2]; out[7] = A[5]; out[8] = A[8];
  return out;
}

/** 3×3 subtract: out = A - B (column-major). */
export function m3Sub(out: Float64Array, A: Float64Array, B: Float64Array): Float64Array {
  for (let i = 0; i < 9; i++) out[i] = A[i] - B[i];
  return out;
}

/** Build skew-symmetric matrix from vector v (column-major):
 *  [  0  -vz   vy ]
 *  [ vz   0   -vx ]
 *  [-vy  vx    0  ]
 */
export function skew3(out: Float64Array, v: Float64Array): Float64Array {
  out[0] = 0;     out[1] = v[2];  out[2] = -v[1];
  out[3] = -v[2]; out[4] = 0;     out[5] = v[0];
  out[6] = v[1];  out[7] = -v[0]; out[8] = 0;
  return out;
}

/** Extract vector from skew-symmetric matrix (vee map, column-major). */
export function vee3(out: Float64Array, S: Float64Array): Float64Array {
  out[0] = S[5]; // S[2][1] = S[1*3+2] col-major = S[5]... wait
  // Column-major: S(row,col) = S[col*3 + row]
  // S(2,1) = S[1*3 + 2] = S[5]
  // S(0,2) = S[2*3 + 0] = S[6]
  // S(1,0) = S[0*3 + 1] = S[1]
  out[0] = S[5];  // S(2,1)
  out[1] = S[6];  // S(0,2)
  out[2] = S[1];  // S(1,0)
  return out;
}

// ── Scalar helpers ──

export function clamp(x: number, lo: number, hi: number): number {
  return x < lo ? lo : x > hi ? hi : x;
}

export function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}
