import { describe, it, expect } from 'vitest';
import { readRangefinder } from '@sim/sensors/rangefinder';
import { createDroneState } from '@sim/physics/state';
import { defaultSensorSuiteConfig } from '@sim/core/config-defaults';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { DeterministicRNG } from '@sim/core/rng';
import { qFromAxisAngle, qCreate } from '@sim/physics/quaternion';
import { v3Create } from '@lib/math';

describe('Rangefinder Sensor', () => {
  const worldGeo = new WorldGeometry(openField());

  it('identity quaternion at 2m height: range = 2m', () => {
    const state = createDroneState(4);
    state.position[2] = -2; // 2m above ground in NED
    const config = { ...defaultSensorSuiteConfig().rangefinder, noise: 0, dropoutProbability: 0 };
    const rng = new DeterministicRNG(42);

    const r = readRangefinder(state, config, rng, worldGeo, 0);
    expect(r.valid).toBe(true);
    expect(r.measurement.range).toBeCloseTo(2, 2);
  });

  it('tilted 20 degrees: range > height', () => {
    const state = createDroneState(4);
    state.position[2] = -2;
    const q = qCreate();
    qFromAxisAngle(q, v3Create(0, 1, 0), 20 * Math.PI / 180); // 20 deg pitch
    state.quaternion.set(q);
    const config = { ...defaultSensorSuiteConfig().rangefinder, noise: 0, dropoutProbability: 0 };
    const rng = new DeterministicRNG(42);

    const r = readRangefinder(state, config, rng, worldGeo, 0);
    expect(r.valid).toBe(true);
    expect(r.measurement.range).toBeGreaterThan(2);
  });

  it('steep incidence angle: invalid', () => {
    const state = createDroneState(4);
    state.position[2] = -2;
    const q = qCreate();
    qFromAxisAngle(q, v3Create(0, 1, 0), 60 * Math.PI / 180); // 60 deg pitch
    state.quaternion.set(q);
    const config = { ...defaultSensorSuiteConfig().rangefinder,
      noise: 0, dropoutProbability: 0, maxIncidenceAngle: Math.PI / 4 };
    const rng = new DeterministicRNG(42);

    const r = readRangefinder(state, config, rng, worldGeo, 0);
    expect(r.valid).toBe(false);
  });

  it('range > maxRange: invalid', () => {
    const state = createDroneState(4);
    state.position[2] = -20; // 20m altitude
    const config = { ...defaultSensorSuiteConfig().rangefinder,
      noise: 0, dropoutProbability: 0, maxRange: 8 };
    const rng = new DeterministicRNG(42);

    const r = readRangefinder(state, config, rng, worldGeo, 0);
    expect(r.valid).toBe(false);
  });

  it('dropout produces invalid at configured rate', () => {
    const state = createDroneState(4);
    state.position[2] = -2;
    const config = { ...defaultSensorSuiteConfig().rangefinder, noise: 0, dropoutProbability: 0.5 };
    let invalid = 0;
    for (let i = 0; i < 1000; i++) {
      const rng = new DeterministicRNG(i);
      const r = readRangefinder(state, config, rng, worldGeo, 0);
      if (!r.valid) invalid++;
    }
    // ~50% dropout
    expect(invalid).toBeGreaterThan(350);
    expect(invalid).toBeLessThan(650);
  });
});
