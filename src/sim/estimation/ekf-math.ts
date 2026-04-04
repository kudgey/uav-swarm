/**
 * 16×16 matrix operations for the error-state EKF.
 * Row-major Float64Array(256). Element (i,j) at index i*16 + j.
 *
 * Also includes small-matrix helpers (m ≤ 3) for Kalman gain computation.
 * All functions are allocation-free: write into pre-allocated output params.
 */

export const N = 16;
const N2 = N * N; // 256

export function m16Create(): Float64Array {
  return new Float64Array(N2);
}

export function m16Zero(out: Float64Array): Float64Array {
  out.fill(0);
  return out;
}

export function m16Identity(out: Float64Array): Float64Array {
  out.fill(0);
  for (let i = 0; i < N; i++) out[i * N + i] = 1;
  return out;
}

export function m16Copy(out: Float64Array, a: Float64Array): Float64Array {
  out.set(a);
  return out;
}

/** C = A * B. out must NOT alias A or B. */
export function m16Mul(out: Float64Array, A: Float64Array, B: Float64Array): Float64Array {
  for (let i = 0; i < N; i++) {
    for (let j = 0; j < N; j++) {
      let sum = 0;
      for (let k = 0; k < N; k++) {
        sum += A[i * N + k] * B[k * N + j];
      }
      out[i * N + j] = sum;
    }
  }
  return out;
}

/** C = A * B^T. out must NOT alias A or B. */
export function m16MulTransposeRight(out: Float64Array, A: Float64Array, B: Float64Array): Float64Array {
  for (let i = 0; i < N; i++) {
    for (let j = 0; j < N; j++) {
      let sum = 0;
      for (let k = 0; k < N; k++) {
        sum += A[i * N + k] * B[j * N + k]; // B^T[k][j] = B[j][k]
      }
      out[i * N + j] = sum;
    }
  }
  return out;
}

/** P = (P + P^T) / 2, in-place. */
export function m16Symmetrize(P: Float64Array): void {
  for (let i = 0; i < N; i++) {
    for (let j = i + 1; j < N; j++) {
      const avg = 0.5 * (P[i * N + j] + P[j * N + i]);
      P[i * N + j] = avg;
      P[j * N + i] = avg;
    }
  }
}

/** C = A + B. */
export function m16Add(out: Float64Array, A: Float64Array, B: Float64Array): Float64Array {
  for (let i = 0; i < N2; i++) out[i] = A[i] + B[i];
  return out;
}

/** C = A * s. */
export function m16Scale(out: Float64Array, A: Float64Array, s: number): Float64Array {
  for (let i = 0; i < N2; i++) out[i] = A[i] * s;
  return out;
}

/** Set a 3×3 block at (row, col) from a 3×3 row-major source. */
export function m16SetBlock3(P: Float64Array, row: number, col: number, block: Float64Array): void {
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      P[(row + i) * N + (col + j)] = block[i * 3 + j];
    }
  }
}

/** Set a scalar at (row, col). */
export function m16Set(P: Float64Array, row: number, col: number, val: number): void {
  P[row * N + col] = val;
}

/** Get element at (row, col). */
export function m16Get(P: Float64Array, row: number, col: number): number {
  return P[row * N + col];
}

/** Check basic SPD properties: all diagonal elements positive. */
export function m16CheckDiagPositive(P: Float64Array): boolean {
  for (let i = 0; i < N; i++) {
    if (P[i * N + i] <= 0) return false;
  }
  return true;
}

// ── Small-matrix Kalman helpers (m ≤ 3) ──

/**
 * Compute PHt = P[16×16] * H^T[16×m], result is 16×m stored row-major in Float64Array(16*m).
 * H is m×16 row-major.
 */
export function mulPHt(
  out: Float64Array, P: Float64Array, H: Float64Array, m: number,
): Float64Array {
  for (let i = 0; i < N; i++) {
    for (let j = 0; j < m; j++) {
      let sum = 0;
      for (let k = 0; k < N; k++) {
        sum += P[i * N + k] * H[j * N + k]; // H^T[k][j] = H[j][k]
      }
      out[i * m + j] = sum;
    }
  }
  return out;
}

/**
 * Compute S = H * PHt + R, where S is m×m, H is m×16, PHt is 16×m, R is m×m.
 * All row-major.
 */
export function computeS(
  out: Float64Array, H: Float64Array, PHt: Float64Array, R: Float64Array, m: number,
): Float64Array {
  for (let i = 0; i < m; i++) {
    for (let j = 0; j < m; j++) {
      let sum = 0;
      for (let k = 0; k < N; k++) {
        sum += H[i * N + k] * PHt[k * m + j];
      }
      out[i * m + j] = sum + R[i * m + j];
    }
  }
  return out;
}

/** Invert a small symmetric matrix (m=1,2,3) in-place. Returns false if singular. */
export function invertSmall(S: Float64Array, m: number): boolean {
  if (m === 1) {
    if (Math.abs(S[0]) < 1e-30) return false;
    S[0] = 1 / S[0];
    return true;
  }
  if (m === 2) {
    const a = S[0], b = S[1], c = S[2], d = S[3];
    const det = a * d - b * c;
    if (Math.abs(det) < 1e-30) return false;
    const inv = 1 / det;
    S[0] = d * inv; S[1] = -b * inv;
    S[2] = -c * inv; S[3] = a * inv;
    return true;
  }
  if (m === 3) {
    // Adjugate method for 3×3
    const a = S[0], b = S[1], c = S[2];
    const d = S[3], e = S[4], f = S[5];
    const g = S[6], h = S[7], k = S[8];
    const det = a * (e * k - f * h) - b * (d * k - f * g) + c * (d * h - e * g);
    if (Math.abs(det) < 1e-30) return false;
    const inv = 1 / det;
    S[0] = (e * k - f * h) * inv;
    S[1] = (c * h - b * k) * inv;
    S[2] = (b * f - c * e) * inv;
    S[3] = (f * g - d * k) * inv;
    S[4] = (a * k - c * g) * inv;
    S[5] = (c * d - a * f) * inv;
    S[6] = (d * h - e * g) * inv;
    S[7] = (b * g - a * h) * inv;
    S[8] = (a * e - b * d) * inv;
    return true;
  }
  return false;
}

/**
 * Compute Kalman gain K[16×m] = PHt[16×m] * Sinv[m×m].
 */
export function computeK(
  out: Float64Array, PHt: Float64Array, Sinv: Float64Array, m: number,
): Float64Array {
  for (let i = 0; i < N; i++) {
    for (let j = 0; j < m; j++) {
      let sum = 0;
      for (let k = 0; k < m; k++) {
        sum += PHt[i * m + k] * Sinv[k * m + j];
      }
      out[i * m + j] = sum;
    }
  }
  return out;
}

/**
 * Compute dx[16] = K[16×m] * y[m].
 */
export function computeDx(
  out: Float64Array, K: Float64Array, y: Float64Array, m: number,
): Float64Array {
  for (let i = 0; i < N; i++) {
    let sum = 0;
    for (let j = 0; j < m; j++) {
      sum += K[i * m + j] * y[j];
    }
    out[i] = sum;
  }
  return out;
}

/**
 * Joseph-form covariance update: P = (I - K*H) * P * (I - K*H)^T + K * R * K^T.
 * Numerically stable, preserves symmetry.
 * scratch must be at least Float64Array(256).
 */
export function josephUpdate(
  P: Float64Array, K: Float64Array, H: Float64Array, R: Float64Array,
  m: number, scratch1: Float64Array, scratch2: Float64Array,
): void {
  // IKH[16×16] = I - K[16×m] * H[m×16]
  m16Identity(scratch1);
  for (let i = 0; i < N; i++) {
    for (let j = 0; j < N; j++) {
      let sum = 0;
      for (let k = 0; k < m; k++) {
        sum += K[i * m + k] * H[k * N + j];
      }
      scratch1[i * N + j] -= sum;
    }
  }

  // temp = IKH * P
  m16Mul(scratch2, scratch1, P);
  // P_new = temp * IKH^T
  m16MulTransposeRight(P, scratch2, scratch1);

  // Add K * R * K^T (small m, inline)
  for (let i = 0; i < N; i++) {
    for (let j = 0; j < N; j++) {
      let sum = 0;
      for (let a = 0; a < m; a++) {
        for (let b = 0; b < m; b++) {
          sum += K[i * m + a] * R[a * m + b] * K[j * m + b];
        }
      }
      P[i * N + j] += sum;
    }
  }

  m16Symmetrize(P);
}

/**
 * Mahalanobis distance: y^T * Sinv * y (scalar).
 * S must already be inverted.
 */
export function mahalanobis(y: Float64Array, Sinv: Float64Array, m: number): number {
  let result = 0;
  for (let i = 0; i < m; i++) {
    let row = 0;
    for (let j = 0; j < m; j++) {
      row += Sinv[i * m + j] * y[j];
    }
    result += y[i] * row;
  }
  return result;
}
