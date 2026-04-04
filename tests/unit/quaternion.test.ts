import { describe, it, expect } from 'vitest';
import {
  qCreate, qMultiply, qNormalize, qRotateVector, qConjugate,
  qFromAxisAngle, qFromEulerZYX, qToEulerZYX, qDerivative, qNorm,
} from '@sim/physics/quaternion';
import { v3Create } from '@lib/math';

const EPSILON = 1e-10;

function expectClose(a: Float64Array | number[], b: number[], tol = EPSILON) {
  for (let i = 0; i < b.length; i++) {
    expect(a[i]).toBeCloseTo(b[i], -Math.log10(tol));
  }
}

describe('Quaternion', () => {
  it('identity rotation leaves vector unchanged', () => {
    const q = qCreate(1, 0, 0, 0);
    const v = v3Create(1, 2, 3);
    const out = v3Create();
    qRotateVector(out, q, v);
    expectClose(out, [1, 2, 3]);
  });

  it('90-degree rotation about z-axis', () => {
    const q = qCreate();
    const axis = v3Create(0, 0, 1);
    qFromAxisAngle(q, axis, Math.PI / 2);
    const v = v3Create(1, 0, 0);
    const out = v3Create();
    qRotateVector(out, q, v);
    expectClose(out, [0, 1, 0], 1e-9);
  });

  it('90-degree rotation about x-axis', () => {
    const q = qCreate();
    const axis = v3Create(1, 0, 0);
    qFromAxisAngle(q, axis, Math.PI / 2);
    const v = v3Create(0, 1, 0);
    const out = v3Create();
    qRotateVector(out, q, v);
    expectClose(out, [0, 0, 1], 1e-9);
  });

  it('normalize preserves direction', () => {
    const q = qCreate(2, 0, 0, 0);
    const out = qCreate();
    qNormalize(out, q);
    expect(out[0]).toBeCloseTo(1, 10);
    expect(qNorm(out)).toBeCloseTo(1, 10);
  });

  it('multiply is associative', () => {
    const a = qCreate();
    const b = qCreate();
    const c = qCreate();
    qFromAxisAngle(a, v3Create(1, 0, 0), 0.3);
    qFromAxisAngle(b, v3Create(0, 1, 0), 0.5);
    qFromAxisAngle(c, v3Create(0, 0, 1), 0.7);

    const ab = qCreate(), abc1 = qCreate();
    qMultiply(ab, a, b);
    qMultiply(abc1, ab, c);

    const bc = qCreate(), abc2 = qCreate();
    qMultiply(bc, b, c);
    qMultiply(abc2, a, bc);

    expectClose(abc1, [...abc2], 1e-12);
  });

  it('conjugate reverses rotation', () => {
    const q = qCreate();
    qFromAxisAngle(q, v3Create(0, 1, 0), 1.0);
    const qc = qCreate();
    qConjugate(qc, q);
    const prod = qCreate();
    qMultiply(prod, q, qc);
    expectClose(prod, [1, 0, 0, 0], 1e-12);
  });

  it('euler ZYX roundtrip', () => {
    const roll = 0.3, pitch = -0.2, yaw = 1.5;
    const q = qCreate();
    qFromEulerZYX(q, roll, pitch, yaw);
    const [r2, p2, y2] = qToEulerZYX(q);
    expect(r2).toBeCloseTo(roll, 10);
    expect(p2).toBeCloseTo(pitch, 10);
    expect(y2).toBeCloseTo(yaw, 10);
  });

  it('derivative is zero for zero angular velocity', () => {
    const q = qCreate(1, 0, 0, 0);
    const omega = v3Create(0, 0, 0);
    const out = qCreate();
    qDerivative(out, q, omega);
    expectClose(out, [0, 0, 0, 0]);
  });
});
