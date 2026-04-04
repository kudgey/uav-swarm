/**
 * Integration test: VIO and UWB fire at correct rates.
 */
import { describe, it, expect } from 'vitest';
import { DeterministicScheduler } from '@sim/core/scheduler';

describe('Sensor timing', () => {
  it('VIO at 30Hz and UWB at 10Hz fire at correct rates', () => {
    const scheduler = new DeterministicScheduler(0.001);
    let vioCount = 0;
    let uwbCount = 0;

    scheduler.addTask('sensor-camera', 1 / 30, 55, () => { vioCount++; });
    scheduler.addTask('sensor-uwb', 1 / 10, 58, () => { uwbCount++; });

    // Run 1 second (1000 steps)
    for (let i = 0; i < 1000; i++) scheduler.step();

    expect(vioCount).toBe(30);
    expect(uwbCount).toBe(10);
  });
});
