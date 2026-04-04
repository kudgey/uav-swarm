/**
 * Deterministic fixed-step multi-rate scheduler.
 * STATUS: experimental
 *
 * Core semantics:
 *   - Each step advances simTime by exactly simDt
 *   - Wall clock only determines how many steps to run per frame
 *   - Tasks execute at their own rates via accumulator-based firing
 *
 * Execution order at coinciding boundaries (within a single step):
 *   1. Command latch (ZOH) — priority 0
 *   2. Physics integration — priority 1
 *   3. Telemetry snapshot — priority 2
 *
 * Lower priority number = executes first.
 */

export interface ScheduledTask {
  id: string;
  period: number;          // seconds between fires
  priority: number;        // lower = runs first at coinciding ticks
  callback: (dt: number, simTime: number) => void;
  accumulator: number;     // time accumulated since last fire
}

export class DeterministicScheduler {
  private tasks: ScheduledTask[] = [];
  private _simTime = 0;
  private _stepCount = 0;
  private _simDt: number;

  constructor(simDt: number) {
    this._simDt = simDt;
  }

  get simTime(): number { return this._simTime; }
  get stepCount(): number { return this._stepCount; }
  get simDt(): number { return this._simDt; }

  /** Optional timing callback: called with (taskId, durationMs) after each task fires. */
  onTaskTiming: ((taskId: string, durationMs: number) => void) | null = null;

  addTask(
    id: string,
    period: number,
    priority: number,
    callback: (dt: number, simTime: number) => void,
  ): void {
    this.tasks.push({ id, period, priority, callback, accumulator: 0 });
    // Keep sorted by priority (stable sort)
    this.tasks.sort((a, b) => a.priority - b.priority);
  }

  removeTask(id: string): void {
    const idx = this.tasks.findIndex(t => t.id === id);
    if (idx >= 0) this.tasks.splice(idx, 1);
  }

  /**
   * Execute one fixed-step tick. Advances simTime by simDt.
   * All tasks whose accumulator reaches their period fire in priority order.
   */
  step(): void {
    const dt = this._simDt;

    // Accumulate time for all tasks
    for (const task of this.tasks) {
      task.accumulator += dt;
    }

    // Fire tasks that are due (in priority order, already sorted)
    const timing = this.onTaskTiming;
    for (const task of this.tasks) {
      if (task.accumulator >= task.period - 1e-12) {
        if (timing) {
          const t0 = performance.now();
          task.callback(task.period, this._simTime);
          timing(task.id, performance.now() - t0);
        } else {
          task.callback(task.period, this._simTime);
        }
        task.accumulator -= task.period;
        // Clamp small negative from floating point
        if (task.accumulator < 0) task.accumulator = 0;
      }
    }

    this._simTime += dt;
    this._stepCount++;
  }

  /** Reset scheduler state. Does not remove tasks. */
  reset(): void {
    this._simTime = 0;
    this._stepCount = 0;
    for (const task of this.tasks) {
      task.accumulator = 0;
    }
  }
}
