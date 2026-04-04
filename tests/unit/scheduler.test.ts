import { describe, it, expect } from 'vitest';
import { DeterministicScheduler } from '@sim/core/scheduler';

describe('DeterministicScheduler', () => {
  it('advances simTime by simDt per step', () => {
    const s = new DeterministicScheduler(0.001);
    s.addTask('physics', 0.001, 1, () => {});
    for (let i = 0; i < 1000; i++) s.step();
    expect(s.simTime).toBeCloseTo(1.0, 10);
    expect(s.stepCount).toBe(1000);
  });

  it('fires tasks at correct multi-rate frequencies', () => {
    const s = new DeterministicScheduler(0.001);
    let fastCount = 0;
    let slowCount = 0;
    s.addTask('fast', 0.001, 0, () => { fastCount++; });
    s.addTask('slow', 0.004, 1, () => { slowCount++; });

    for (let i = 0; i < 100; i++) s.step();

    expect(fastCount).toBe(100);
    expect(slowCount).toBe(25);
  });

  it('respects priority ordering at coinciding ticks', () => {
    const s = new DeterministicScheduler(0.004);
    const order: string[] = [];
    s.addTask('command', 0.004, 0, () => { order.push('cmd'); });
    s.addTask('physics', 0.004, 1, () => { order.push('phys'); });
    s.addTask('telemetry', 0.004, 2, () => { order.push('telem'); });

    s.step();
    expect(order).toEqual(['cmd', 'phys', 'telem']);
  });

  it('two identical schedulers produce identical results', () => {
    const a = new DeterministicScheduler(0.001);
    const b = new DeterministicScheduler(0.001);
    const aLog: number[] = [];
    const bLog: number[] = [];
    a.addTask('t', 0.001, 0, (_dt, t) => { aLog.push(t); });
    b.addTask('t', 0.001, 0, (_dt, t) => { bLog.push(t); });

    for (let i = 0; i < 100; i++) { a.step(); b.step(); }
    expect(aLog).toEqual(bLog);
  });

  it('reset clears state and accumulators', () => {
    const s = new DeterministicScheduler(0.001);
    let count = 0;
    s.addTask('t', 0.001, 0, () => { count++; });
    for (let i = 0; i < 50; i++) s.step();
    expect(s.simTime).toBeCloseTo(0.05, 10);
    s.reset();
    expect(s.simTime).toBe(0);
    expect(s.stepCount).toBe(0);
  });
});
