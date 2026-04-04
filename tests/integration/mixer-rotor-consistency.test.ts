/**
 * Integration test: Mixer forward/inverse roundtrip and rotor consistency.
 */

import { describe, it, expect } from 'vitest';
import { createMixer, allocate, computeWrench } from '@sim/actuators/mixer';
import { defaultDroneParams } from '@sim/core/config-defaults';
import { GRAVITY_MPS2 } from '@sim/core/frames';

describe('Mixer-rotor consistency', () => {
  const params = defaultDroneParams();
  const mixer = createMixer(params);

  it('forward/inverse roundtrip within tolerance', () => {
    const T = params.mass * GRAVITY_MPS2;
    const tauX = 0.02, tauY = -0.01, tauZ = 0.005;

    const speeds = new Float64Array(4);
    allocate(speeds, T, tauX, tauY, tauZ, mixer, params);

    const wrench = new Float64Array(4);
    computeWrench(wrench, speeds, mixer);

    expect(wrench[0]).toBeCloseTo(T, 1);
    expect(wrench[1]).toBeCloseTo(tauX, 2);
    expect(wrench[2]).toBeCloseTo(tauY, 2);
    expect(wrench[3]).toBeCloseTo(tauZ, 2);
  });

  it('roll command produces correct rotor differential per frames.ts', () => {
    const T = params.mass * GRAVITY_MPS2;
    const rollTorque = 0.05;
    const speeds = new Float64Array(4);
    allocate(speeds, T, rollTorque, 0, 0, mixer, params);

    // Per frames.ts: tau_x = d*(T1 - T2 - T3 + T4)
    // Positive roll: T1,T4 > T2,T3
    const thrusts = speeds.map(w => params.kT * w * w);
    expect(thrusts[0] + thrusts[3]).toBeGreaterThan(thrusts[1] + thrusts[2]);
  });

  it('pitch command produces correct differential', () => {
    const T = params.mass * GRAVITY_MPS2;
    const pitchTorque = 0.05;
    const speeds = new Float64Array(4);
    allocate(speeds, T, 0, pitchTorque, 0, mixer, params);

    // tau_y = d*(T1 + T2 - T3 - T4)
    // Positive pitch: T1,T2 > T3,T4
    const thrusts = speeds.map(w => params.kT * w * w);
    expect(thrusts[0] + thrusts[1]).toBeGreaterThan(thrusts[2] + thrusts[3]);
  });

  it('yaw command produces correct differential', () => {
    const T = params.mass * GRAVITY_MPS2;
    const yawTorque = 0.005;
    const speeds = new Float64Array(4);
    allocate(speeds, T, 0, 0, yawTorque, mixer, params);

    // tau_z ~ (kQ/kT)*(T1 - T2 + T3 - T4), positive yaw: CCW rotors (1,3) > CW (2,4)
    const thrusts = speeds.map(w => params.kT * w * w);
    expect(thrusts[0] + thrusts[2]).toBeGreaterThan(thrusts[1] + thrusts[3]);
  });
});
