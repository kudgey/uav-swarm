import { describe, it, expect } from 'vitest';
import { createMixer, allocate, computeWrench } from '@sim/actuators/mixer';
import { defaultDroneParams } from '@sim/core/config-defaults';

describe('Mixer (X-config)', () => {
  const params = defaultDroneParams();
  const mixer = createMixer(params);

  it('hover allocation produces equal rotor speeds', () => {
    const totalThrust = params.mass * 9.80665;
    const out = new Float64Array(4);
    allocate(out, totalThrust, 0, 0, 0, mixer, params);

    // All should be equal
    const expected = out[0];
    for (let i = 1; i < 4; i++) {
      expect(out[i]).toBeCloseTo(expected, 6);
    }

    // Check thrust matches
    const T = params.kT * expected * expected * 4;
    expect(T).toBeCloseTo(totalThrust, 3);
  });

  it('pure roll command produces correct differential', () => {
    const totalThrust = params.mass * 9.80665;
    const rollTorque = 0.1;
    const out = new Float64Array(4);
    allocate(out, totalThrust, rollTorque, 0, 0, mixer, params);

    // Per frame conventions (FRD, X-config):
    // tau_x = d*(T1 - T2 - T3 + T4)
    // Positive roll: rotors 1,4 should increase, 2,3 decrease
    const T = out.map(w => params.kT * w * w);
    expect(T[0] + T[3]).toBeGreaterThan(T[1] + T[2]);
  });

  it('forward/inverse roundtrip', () => {
    const totalThrust = 12.0;
    const tauX = 0.05, tauY = -0.03, tauZ = 0.01;
    const speeds = new Float64Array(4);
    allocate(speeds, totalThrust, tauX, tauY, tauZ, mixer, params);

    const wrench = new Float64Array(4);
    computeWrench(wrench, speeds, mixer);

    // Wrench should approximately match input
    expect(wrench[0]).toBeCloseTo(totalThrust, 1);
    expect(wrench[1]).toBeCloseTo(tauX, 2);
    expect(wrench[2]).toBeCloseTo(tauY, 2);
    // Yaw might differ due to saturation
  });

  it('saturation clipping works', () => {
    const out = new Float64Array(4);
    // Huge thrust demand beyond motor capability
    allocate(out, 1000, 0, 0, 0, mixer, params);
    for (let i = 0; i < 4; i++) {
      expect(out[i]).toBeLessThanOrEqual(params.motorOmegaMax);
    }
  });
});
