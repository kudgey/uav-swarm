import { describe, it, expect } from 'vitest';
import { createBatteryState, defaultBatteryConfig, stepBattery } from '@sim/actuators/battery';

describe('Battery model', () => {
  it('disabled returns 1.0 and does not deplete', () => {
    const cfg = defaultBatteryConfig();
    const s = createBatteryState();
    const m = stepBattery(s, cfg, 10, 0.01);
    expect(m).toBe(1.0);
    expect(s.soc).toBe(1.0);
  });

  it('enabled: SOC decreases with current draw', () => {
    const cfg = { ...defaultBatteryConfig(), enabled: true, capacityWh: 10 };
    const s = createBatteryState();
    for (let i = 0; i < 1000; i++) stepBattery(s, cfg, 10, 0.01); // 10s
    expect(s.soc).toBeLessThan(1.0);
  });

  it('voltage sag reduces thrust under heavy load', () => {
    const cfg = {
      ...defaultBatteryConfig(), enabled: true,
      internalResistance: 0.1, // high, exaggerate sag
    };
    const s = createBatteryState();
    const mLight = stepBattery(s, cfg, 1, 0.001);
    const mHeavy = stepBattery(s, cfg, 50, 0.001);
    expect(mHeavy).toBeLessThan(mLight);
  });

  it('cutoff: thrust goes to 0 below cutoff voltage', () => {
    const cfg = { ...defaultBatteryConfig(), enabled: true, capacityWh: 0.05 };
    const s = createBatteryState(0.1); // start low
    // Drain hard
    for (let i = 0; i < 5000; i++) stepBattery(s, cfg, 20, 0.01);
    const m = stepBattery(s, cfg, 20, 0.01);
    expect(m).toBeLessThan(0.1);
    expect(s.soc).toBeCloseTo(0, 1);
  });
});
