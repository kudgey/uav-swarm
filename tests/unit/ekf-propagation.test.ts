import { describe, it, expect } from 'vitest';
import { propagateNominal, buildPhi } from '@sim/estimation/ekf-propagation';
import { v3Create } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import { m16Create, N } from '@sim/estimation/ekf-math';
import type { EstimatedState } from '@sim/estimation/estimator-types';

function makeState(): EstimatedState {
  return {
    position: v3Create(0, 0, -2), velocity: v3Create(0, 0, 0),
    quaternion: qCreate(1, 0, 0, 0), gyroBias: v3Create(0, 0, 0),
    accelBias: v3Create(0, 0, 0), baroBias: 0,
    angularVelocity: v3Create(0, 0, 0), timestamp: 0,
  };
}

describe('EKF Propagation', () => {
  it('zero gyro + zero accel: position stays, velocity changes by gravity only', () => {
    const state = makeState();
    const gyro = v3Create(0, 0, 0);
    const accel = v3Create(0, 0, 0);
    propagateNominal(state, gyro, accel, 0.004);
    // v should change by g*dt = 9.81 * 0.004 ≈ 0.039 in NED z
    expect(state.velocity[2]).toBeCloseTo(GRAVITY_MPS2 * 0.004, 4);
    expect(state.velocity[0]).toBeCloseTo(0, 10);
  });

  it('hover accel cancels gravity: velocity stays zero', () => {
    const state = makeState();
    const gyro = v3Create(0, 0, 0);
    // At identity quat, body [0,0,-g] rotated to world = [0,0,-g], plus g = 0
    const accel = v3Create(0, 0, -GRAVITY_MPS2);
    propagateNominal(state, gyro, accel, 0.004);
    expect(Math.abs(state.velocity[0])).toBeLessThan(1e-10);
    expect(Math.abs(state.velocity[1])).toBeLessThan(1e-10);
    expect(Math.abs(state.velocity[2])).toBeLessThan(1e-6);
  });

  it('F matrix: δba→δv coupling is -R (cols 12-14 in rows 3-5)', () => {
    const state = makeState();
    const gyro = v3Create(0, 0, 0);
    const accel = v3Create(0, 0, -GRAVITY_MPS2);
    const Phi = m16Create();
    buildPhi(Phi, state, gyro, accel, 0.004);
    // At identity R, -R*dt should give -dt on diagonal of block (3:6, 12:15)
    // Phi[3,12] = -1*0.004, Phi[4,13] = -1*0.004, Phi[5,14] = -1*0.004
    expect(Phi[3 * N + 12]).toBeCloseTo(-0.004, 6);
    expect(Phi[4 * N + 13]).toBeCloseTo(-0.004, 6);
    expect(Phi[5 * N + 14]).toBeCloseTo(-0.004, 6);
    // δbg→δv should be ZERO (cols 9-11 in rows 3-5)
    expect(Phi[3 * N + 9]).toBeCloseTo(0, 10);
    expect(Phi[4 * N + 10]).toBeCloseTo(0, 10);
    expect(Phi[5 * N + 11]).toBeCloseTo(0, 10);
  });

  it('F matrix: δbg→δθ coupling is -I (cols 9-11 in rows 6-8)', () => {
    const state = makeState();
    const Phi = m16Create();
    buildPhi(Phi, state, v3Create(0, 0, 0), v3Create(0, 0, -GRAVITY_MPS2), 0.004);
    // Phi[6,9] = -dt, Phi[7,10] = -dt, Phi[8,11] = -dt
    expect(Phi[6 * N + 9]).toBeCloseTo(-0.004, 6);
    expect(Phi[7 * N + 10]).toBeCloseTo(-0.004, 6);
    expect(Phi[8 * N + 11]).toBeCloseTo(-0.004, 6);
  });
});
