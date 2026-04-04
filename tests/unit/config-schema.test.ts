import { describe, it, expect } from 'vitest';
import { validateSimConfig, droneParamsSchema } from '@sim/core/config-schema';
import { defaultSimConfig } from '@sim/core/config-defaults';

/**
 * Convert a config object to plain JSON-safe form for schema validation.
 * Float64Array -> number[], strips derived fields (rotorPositions).
 */
function toPlain(obj: unknown): unknown {
  if (obj instanceof Float64Array) return [...obj];
  if (Array.isArray(obj)) return obj.map(toPlain);
  if (obj !== null && typeof obj === 'object') {
    const result: Record<string, unknown> = {};
    for (const [k, v] of Object.entries(obj as Record<string, unknown>)) {
      if (k === 'rotorPositions') continue; // derived, not in schema
      result[k] = toPlain(v);
    }
    return result;
  }
  return obj;
}

describe('Config Schema', () => {
  it('valid default config passes validation (including environment + sensors)', () => {
    const config = defaultSimConfig();
    const plain = toPlain(config);
    expect(() => validateSimConfig(plain)).not.toThrow();
  });

  it('missing required field is rejected', () => {
    expect(() => validateSimConfig({ seed: 42 })).toThrow();
  });

  it('negative mass is rejected', () => {
    expect(() => droneParamsSchema.parse({
      mass: -1, inertia: [1, 1, 1], armLength: 0.1, numRotors: 4,
      rotorDirections: [1, -1, 1, -1], kT: 1e-6, kQ: 1e-7,
      motorTau: 0.02, motorOmegaMin: 0, motorOmegaMax: 1000,
      motorRateLimit: 50000, motorDeadZone: 0,
      dragCoeffLinear: [0, 0, 0], dragCoeffQuadratic: [0, 0, 0], dragMode: 'linear',
    })).toThrow();
  });

  it('invalid dragMode is rejected', () => {
    expect(() => droneParamsSchema.parse({
      mass: 1, inertia: [1, 1, 1], armLength: 0.1, numRotors: 4,
      rotorDirections: [1, -1, 1, -1], kT: 1e-6, kQ: 1e-7,
      motorTau: 0.02, motorOmegaMin: 0, motorOmegaMax: 1000,
      motorRateLimit: 50000, motorDeadZone: 0,
      dragCoeffLinear: [0, 0, 0], dragCoeffQuadratic: [0, 0, 0], dragMode: 'invalid',
    })).toThrow();
  });

  it('missing environment config is rejected', () => {
    const config = toPlain(defaultSimConfig()) as Record<string, unknown>;
    delete config.environment;
    expect(() => validateSimConfig(config)).toThrow();
  });

  it('missing sensors config is rejected', () => {
    const config = toPlain(defaultSimConfig()) as Record<string, unknown>;
    delete config.sensors;
    expect(() => validateSimConfig(config)).toThrow();
  });

  it('unknown top-level key is rejected (strict mode)', () => {
    const config = toPlain(defaultSimConfig()) as Record<string, unknown>;
    config.bogusTopLevel = 42;
    expect(() => validateSimConfig(config)).toThrow(/unrecognized/i);
  });

  it('unknown nested key in environment is rejected (strict mode)', () => {
    const config = toPlain(defaultSimConfig()) as Record<string, unknown>;
    (config.environment as Record<string, unknown>).bogusEnvKey = true;
    expect(() => validateSimConfig(config)).toThrow(/unrecognized/i);
  });

  it('unknown nested key in estimation is rejected (strict mode)', () => {
    const config = toPlain(defaultSimConfig()) as Record<string, unknown>;
    (config.estimation as Record<string, unknown>).bogusEstKey = 99;
    expect(() => validateSimConfig(config)).toThrow(/unrecognized/i);
  });

  it('unknown nested key in sensors is rejected (strict mode)', () => {
    const config = toPlain(defaultSimConfig()) as Record<string, unknown>;
    const sensors = config.sensors as Record<string, unknown>;
    (sensors.imu as Record<string, unknown>).bogusImuKey = 0;
    expect(() => validateSimConfig(config)).toThrow(/unrecognized/i);
  });
});
