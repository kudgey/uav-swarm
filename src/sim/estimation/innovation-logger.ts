/**
 * Ring buffer of innovation records for telemetry/validation.
 */

import type { InnovationRecord } from './estimator-types';

const DEFAULT_CAPACITY = 500;

export class InnovationLogger {
  private records: InnovationRecord[];
  private writeIdx = 0;
  private count = 0;
  private capacity: number;

  constructor(capacity = DEFAULT_CAPACITY) {
    this.capacity = capacity;
    this.records = new Array(capacity);
    for (let i = 0; i < capacity; i++) {
      this.records[i] = { timestamp: 0, source: 'mag', innovationNorm: 0, gated: false };
    }
  }

  record(entry: InnovationRecord): void {
    const r = this.records[this.writeIdx];
    r.timestamp = entry.timestamp;
    r.source = entry.source;
    r.innovationNorm = entry.innovationNorm;
    r.gated = entry.gated;
    this.writeIdx = (this.writeIdx + 1) % this.capacity;
    if (this.count < this.capacity) this.count++;
  }

  getRecent(n: number): InnovationRecord[] {
    const actual = Math.min(n, this.count);
    const result: InnovationRecord[] = [];
    for (let i = 0; i < actual; i++) {
      const idx = (this.writeIdx - actual + i + this.capacity) % this.capacity;
      result.push({ ...this.records[idx] });
    }
    return result;
  }

  reset(): void {
    this.writeIdx = 0;
    this.count = 0;
  }
}
