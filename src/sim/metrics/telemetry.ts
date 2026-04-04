/**
 * Ring-buffer telemetry collector.
 * Records TelemetryFrames at the telemetry rate.
 * Emits batches for transfer to UI thread.
 */

import type { DroneState } from '@sim/core/types';
import type { TelemetryBatch, TelemetryFrame } from './telemetry-types';

const DEFAULT_BUFFER_SIZE = 600; // ~10 seconds at 60 Hz

export class TelemetryCollector {
  private frames: TelemetryFrame[];
  private writeIndex = 0;
  private count = 0;
  private bufferSize: number;
  private batchWriteIndex = 0;

  constructor(bufferSize = DEFAULT_BUFFER_SIZE) {
    this.bufferSize = bufferSize;
    this.frames = new Array(bufferSize);
    for (let i = 0; i < bufferSize; i++) {
      this.frames[i] = this.emptyFrame();
    }
  }

  private emptyFrame(): TelemetryFrame {
    return {
      time: 0, posX: 0, posY: 0, posZ: 0,
      velX: 0, velY: 0, velZ: 0,
      qW: 1, qX: 0, qY: 0, qZ: 0,
      omegaX: 0, omegaY: 0, omegaZ: 0,
      motor0: 0, motor1: 0, motor2: 0, motor3: 0,
      cmd0: 0, cmd1: 0, cmd2: 0, cmd3: 0,
      totalThrust: 0,
    };
  }

  /** Record a snapshot from current drone state. */
  record(state: DroneState, simTime: number, kT: number): void {
    const f = this.frames[this.writeIndex];
    f.time = simTime;
    f.posX = state.position[0]; f.posY = state.position[1]; f.posZ = state.position[2];
    f.velX = state.velocity[0]; f.velY = state.velocity[1]; f.velZ = state.velocity[2];
    f.qW = state.quaternion[0]; f.qX = state.quaternion[1];
    f.qY = state.quaternion[2]; f.qZ = state.quaternion[3];
    f.omegaX = state.angularVelocity[0]; f.omegaY = state.angularVelocity[1];
    f.omegaZ = state.angularVelocity[2];
    f.motor0 = state.motorSpeeds[0]; f.motor1 = state.motorSpeeds[1];
    f.motor2 = state.motorSpeeds[2]; f.motor3 = state.motorSpeeds[3];
    f.cmd0 = state.motorCommands[0]; f.cmd1 = state.motorCommands[1];
    f.cmd2 = state.motorCommands[2]; f.cmd3 = state.motorCommands[3];
    // Total thrust
    let T = 0;
    for (let i = 0; i < state.motorSpeeds.length; i++) {
      T += kT * state.motorSpeeds[i] * state.motorSpeeds[i];
    }
    f.totalThrust = T;

    this.writeIndex = (this.writeIndex + 1) % this.bufferSize;
    if (this.count < this.bufferSize) this.count++;
  }

  /** Build a transferable batch of new frames since last batch. */
  buildBatch(): TelemetryBatch | null {
    if (this.count === 0) return null;

    // How many new frames since last batch
    let newCount: number;
    if (this.writeIndex >= this.batchWriteIndex) {
      newCount = this.writeIndex - this.batchWriteIndex;
    } else {
      newCount = this.bufferSize - this.batchWriteIndex + this.writeIndex;
    }
    if (newCount <= 0) return null;

    const startIdx = this.batchWriteIndex;

    const timestamps = new Float64Array(newCount);
    const positions = new Float64Array(newCount * 3);
    const velocities = new Float64Array(newCount * 3);
    const motorSpeeds = new Float64Array(newCount * 4);
    const motorCommands = new Float64Array(newCount * 4);

    for (let i = 0; i < newCount; i++) {
      const idx = (startIdx + i) % this.bufferSize;
      const f = this.frames[idx];
      timestamps[i] = f.time;
      const p = i * 3;
      positions[p] = f.posX; positions[p + 1] = f.posY; positions[p + 2] = f.posZ;
      velocities[p] = f.velX; velocities[p + 1] = f.velY; velocities[p + 2] = f.velZ;
      const m = i * 4;
      motorSpeeds[m] = f.motor0; motorSpeeds[m + 1] = f.motor1;
      motorSpeeds[m + 2] = f.motor2; motorSpeeds[m + 3] = f.motor3;
      motorCommands[m] = f.cmd0; motorCommands[m + 1] = f.cmd1;
      motorCommands[m + 2] = f.cmd2; motorCommands[m + 3] = f.cmd3;
    }

    this.batchWriteIndex = this.writeIndex;
    return { startIndex: startIdx, count: newCount, timestamps, positions, velocities, motorSpeeds, motorCommands };
  }

  /** Get the last N frames as an array (for charts). */
  getRecentFrames(n: number): TelemetryFrame[] {
    const actual = Math.min(n, this.count);
    const result: TelemetryFrame[] = [];
    for (let i = 0; i < actual; i++) {
      const idx = (this.writeIndex - actual + i + this.bufferSize) % this.bufferSize;
      result.push({ ...this.frames[idx] });
    }
    return result;
  }

  reset(): void {
    this.writeIndex = 0;
    this.count = 0;
    this.batchWriteIndex = 0;
  }
}
