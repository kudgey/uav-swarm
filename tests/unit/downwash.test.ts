import { describe, it, expect } from 'vitest';
import { computeDownwashMultiplier, defaultDownwashConfig } from '@sim/environment/downwash';

function drone(id: number, x: number, y: number, z: number) {
  return { id, position: new Float64Array([x, y, z]) };
}

describe('Downwash model', () => {
  it('returns 1.0 when disabled', () => {
    const cfg = { ...defaultDownwashConfig(), enabled: false };
    const target = drone(0, 0, 0, -2);
    const others = [drone(1, 0, 0, -3)]; // above target (NED: z=-3 is higher)
    expect(computeDownwashMultiplier(target, [target, ...others], 0.15, cfg)).toBe(1.0);
  });

  it('returns 1.0 when no drones above', () => {
    const cfg = { ...defaultDownwashConfig(), enabled: true };
    const target = drone(0, 0, 0, -2);
    const others = [drone(1, 0, 0, -1)]; // below target (z=-1 is lower)
    expect(computeDownwashMultiplier(target, [target, ...others], 0.15, cfg)).toBe(1.0);
  });

  it('reduces thrust when drone is directly above', () => {
    const cfg = { ...defaultDownwashConfig(), enabled: true, strength: 0.3 };
    const target = drone(0, 0, 0, -2);
    const others = [drone(1, 0, 0, -2.3)]; // 0.3m above (NED: more negative = higher)
    const mult = computeDownwashMultiplier(target, [target, ...others], 0.15, cfg);
    expect(mult).toBeLessThan(1.0);
    expect(mult).toBeGreaterThan(0.5);
  });

  it('effect decreases with horizontal distance', () => {
    const cfg = { ...defaultDownwashConfig(), enabled: true, strength: 0.3 };
    const target = drone(0, 0, 0, -2);
    const near = [drone(1, 0, 0, -2.5)];   // directly above at 0.5m
    const far = [drone(1, 2, 0, -2.5)];    // 2m away, 0.5m above
    const multNear = computeDownwashMultiplier(target, [target, ...near], 0.15, cfg);
    const multFar = computeDownwashMultiplier(target, [target, ...far], 0.15, cfg);
    expect(multNear).toBeLessThan(multFar); // near has more effect
    expect(multFar).toBeCloseTo(1.0, 2); // far is nearly unaffected
  });

  it('effect decreases with vertical distance', () => {
    const cfg = { ...defaultDownwashConfig(), enabled: true, strength: 0.3 };
    const target = drone(0, 0, 0, -2);
    const close = [drone(1, 0, 0, -2.3)]; // 0.3m above
    const far = [drone(1, 0, 0, -4)];      // 2m above
    const multClose = computeDownwashMultiplier(target, [target, ...close], 0.15, cfg);
    const multFar = computeDownwashMultiplier(target, [target, ...far], 0.15, cfg);
    expect(multClose).toBeLessThan(multFar);
  });

  it('ignores destroyed drones', () => {
    const cfg = { ...defaultDownwashConfig(), enabled: true, strength: 0.3 };
    const target = { id: 0, position: new Float64Array([0, 0, -2]) };
    const dead = { id: 1, position: new Float64Array([0, 0, -2.3]), destroyed: true };
    const alive = { id: 2, position: new Float64Array([0, 0, -2.3]) };
    const multDead = computeDownwashMultiplier(target, [target, dead], 0.15, cfg);
    const multAlive = computeDownwashMultiplier(target, [target, alive], 0.15, cfg);
    expect(multDead).toBe(1.0);
    expect(multAlive).toBeLessThan(1.0);
  });

  it('caps reduction at 0.5 (avoids negative thrust)', () => {
    const cfg = { ...defaultDownwashConfig(), enabled: true, strength: 5.0 }; // unrealistic
    const target = drone(0, 0, 0, -2);
    const others = [
      drone(1, 0, 0, -2.1),
      drone(2, 0.1, 0, -2.1),
      drone(3, 0, 0.1, -2.1),
    ];
    const mult = computeDownwashMultiplier(target, [target, ...others], 0.15, cfg);
    expect(mult).toBeGreaterThanOrEqual(0.5);
  });
});
