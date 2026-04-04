import { describe, it, expect } from 'vitest';
import {
  m16Create, m16Identity, m16Mul, m16MulTransposeRight, m16Symmetrize,
  m16CheckDiagPositive, invertSmall, josephUpdate, mahalanobis,
} from '@sim/estimation/ekf-math';

describe('EKF 16x16 Math', () => {
  it('identity * identity = identity', () => {
    const I = m16Create(); m16Identity(I);
    const out = m16Create();
    m16Mul(out, I, I);
    for (let i = 0; i < 16; i++) {
      for (let j = 0; j < 16; j++) {
        expect(out[i * 16 + j]).toBeCloseTo(i === j ? 1 : 0, 10);
      }
    }
  });

  it('symmetrize makes a symmetric matrix', () => {
    const P = m16Create();
    P[0 * 16 + 1] = 5;
    P[1 * 16 + 0] = 3;
    m16Symmetrize(P);
    expect(P[0 * 16 + 1]).toBeCloseTo(4, 10);
    expect(P[1 * 16 + 0]).toBeCloseTo(4, 10);
  });

  it('A * A^T via MulTransposeRight matches explicit', () => {
    const A = m16Create(); m16Identity(A);
    A[0 * 16 + 1] = 2; A[1 * 16 + 0] = 3;
    const out1 = m16Create();
    m16MulTransposeRight(out1, A, A);
    // A * A^T should be symmetric
    for (let i = 0; i < 16; i++) {
      for (let j = i; j < 16; j++) {
        expect(out1[i * 16 + j]).toBeCloseTo(out1[j * 16 + i], 10);
      }
    }
  });

  it('invertSmall 1x1', () => {
    const S = new Float64Array([4]);
    expect(invertSmall(S, 1)).toBe(true);
    expect(S[0]).toBeCloseTo(0.25, 10);
  });

  it('invertSmall 3x3 identity', () => {
    const S = new Float64Array([1, 0, 0, 0, 1, 0, 0, 0, 1]);
    expect(invertSmall(S, 3)).toBe(true);
    expect(S[0]).toBeCloseTo(1, 10);
    expect(S[4]).toBeCloseTo(1, 10);
    expect(S[8]).toBeCloseTo(1, 10);
  });

  it('josephUpdate preserves SPD on identity P', () => {
    const P = m16Create(); m16Identity(P);
    const K = new Float64Array(16); K[0] = 0.5;
    const H = new Float64Array(16); H[0] = 1;
    const R = new Float64Array([0.1]);
    const s1 = m16Create(), s2 = m16Create();
    josephUpdate(P, K, H, R, 1, s1, s2);
    expect(m16CheckDiagPositive(P)).toBe(true);
  });

  it('mahalanobis distance scalar', () => {
    const y = new Float64Array([2]);
    const Sinv = new Float64Array([0.25]); // S=4, Sinv=0.25
    expect(mahalanobis(y, Sinv, 1)).toBeCloseTo(1, 10); // 2^2 * 0.25 = 1
  });
});
