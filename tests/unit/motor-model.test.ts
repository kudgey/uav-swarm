import { describe, it, expect } from 'vitest';
import { stepMotor } from '@sim/actuators/motor-model';
import { defaultDroneParams } from '@sim/core/config-defaults';

describe('Motor Model (hybrid actuator)', () => {
  const params = defaultDroneParams();
  const dt = 0.001;

  it('lag approaches command over time', () => {
    let omega = 0;
    const cmd = 500;
    for (let i = 0; i < 200; i++) {
      omega = stepMotor(omega, cmd, dt, params);
    }
    // After 200ms (10x tau=0.02), should be very close to command
    expect(omega).toBeCloseTo(cmd, 0);
  });

  it('saturation clips at omegaMax', () => {
    let omega = 800;
    const cmd = 2000; // above max
    for (let i = 0; i < 500; i++) {
      omega = stepMotor(omega, cmd, dt, params);
    }
    expect(omega).toBeLessThanOrEqual(params.motorOmegaMax);
  });

  it('dead zone zeros below threshold', () => {
    const omega = 100;
    const cmd = 10; // below deadZone=30
    const result = stepMotor(omega, cmd, dt, params);
    // Should be heading toward 0 (dead zone converts cmd to 0)
    expect(result).toBeLessThan(omega);
  });

  it('rate limit caps acceleration', () => {
    const omega = 0;
    const cmd = params.motorOmegaMax; // big step
    const result = stepMotor(omega, cmd, dt, params);
    const maxDelta = params.motorRateLimit * dt;
    expect(result - omega).toBeLessThanOrEqual(maxDelta + 1e-10);
  });

  it('output never goes negative', () => {
    let omega = 50;
    for (let i = 0; i < 1000; i++) {
      omega = stepMotor(omega, 0, dt, params);
      expect(omega).toBeGreaterThanOrEqual(0);
    }
  });
});
