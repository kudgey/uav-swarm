/**
 * Communication system: neighbor graph, message queuing, delivery.
 * STATUS: experimental
 *
 * Neighbor graph rebuilt per step from distance + LOS.
 * Messages delivered after latency + jitter, with probabilistic drop.
 * NOT ideal all-to-all.
 */

import { v3Create } from '@lib/math';
import type { Vec3 } from '@sim/core/types';
import type { DeterministicRNG } from '@sim/core/rng';
import type { WorldGeometry } from '@sim/environment/world-geometry';
import type { CommConfig, CommMessage } from './comm-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('comm-system', 'experimental',
  'Distance+LOS neighbor graph, latency/jitter/drop, no MAC contention');

export class CommunicationSystem {
  private config: CommConfig;
  private rng: DeterministicRNG;
  private worldGeo: WorldGeometry;

  /** Neighbor sets: neighborGraph.get(droneId) → Set of neighbor IDs */
  private neighborGraph: Map<number, Set<number>> = new Map();

  /** Pending messages waiting for delivery (sorted by deliveryTime would be ideal but linear scan is fine for small N) */
  private pending: CommMessage[] = [];

  /** Delivered messages per receiver, cleared after consumption */
  private delivered: Map<number, CommMessage[]> = new Map();

  /** Instrumentation counters for metrics */
  sentCount = 0;
  deliveredCount = 0;
  droppedCount = 0;
  totalLatency = 0;

  constructor(config: CommConfig, rng: DeterministicRNG, worldGeo: WorldGeometry) {
    this.config = config;
    this.rng = rng;
    this.worldGeo = worldGeo;
  }

  /**
   * Rebuild neighbor graph from drone positions.
   * Two drones are neighbors if distance ≤ range AND line-of-sight exists.
   */
  updateNeighborGraph(dronePositions: Map<number, Vec3>): void {
    this.neighborGraph.clear();
    const ids = Array.from(dronePositions.keys());

    for (const id of ids) {
      this.neighborGraph.set(id, new Set());
    }

    for (let i = 0; i < ids.length; i++) {
      for (let j = i + 1; j < ids.length; j++) {
        const a = dronePositions.get(ids[i])!;
        const b = dronePositions.get(ids[j])!;
        const dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
        const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

        if (dist <= this.config.range && this.worldGeo.hasLineOfSight(a, b)) {
          this.neighborGraph.get(ids[i])!.add(ids[j]);
          this.neighborGraph.get(ids[j])!.add(ids[i]);
        }
      }
    }
  }

  /**
   * Send a message. Computes delivery time and applies drop probability.
   * If receiverId === -1, broadcasts to all current neighbors of sender.
   */
  send(msg: Omit<CommMessage, 'deliveryTime'>): void {
    const targets = msg.receiverId === -1
      ? Array.from(this.neighborGraph.get(msg.senderId) ?? [])
      : [msg.receiverId];

    for (const targetId of targets) {
      // Check neighbor graph
      if (!this.neighborGraph.get(msg.senderId)?.has(targetId)) continue;

      this.sentCount++;

      // Probabilistic drop
      if (this.rng.next() < this.config.dropRate) { this.droppedCount++; continue; }

      // Compute delivery time with latency + jitter
      const jitter = this.config.latencyJitter * this.rng.gaussian();
      const deliveryTime = msg.sendTime + this.config.latencyMean + jitter;

      this.pending.push({
        ...msg,
        receiverId: targetId,
        deliveryTime: Math.max(msg.sendTime, deliveryTime), // can't arrive before sent
      } as CommMessage);
    }
  }

  /**
   * Deliver messages whose deliveryTime ≤ simTime.
   */
  deliver(simTime: number): void {
    const stillPending: CommMessage[] = [];

    for (const msg of this.pending) {
      if (msg.deliveryTime <= simTime) {
        if (!this.delivered.has(msg.receiverId)) {
          this.delivered.set(msg.receiverId, []);
        }
        this.delivered.get(msg.receiverId)!.push(msg);
        this.deliveredCount++;
        this.totalLatency += msg.deliveryTime - msg.sendTime;
      } else {
        stillPending.push(msg);
      }
    }

    this.pending = stillPending;
  }

  /**
   * Get and clear delivered messages for a drone.
   */
  getMessages(droneId: number): CommMessage[] {
    const msgs = this.delivered.get(droneId) ?? [];
    this.delivered.set(droneId, []);
    return msgs;
  }

  /**
   * Get current neighbor set for a drone.
   */
  getNeighbors(droneId: number): Set<number> {
    return this.neighborGraph.get(droneId) ?? new Set();
  }

  /**
   * Get all active links for snapshot/visualization.
   */
  getLinks(): { from: number; to: number }[] {
    const links: { from: number; to: number }[] = [];
    const seen = new Set<string>();
    for (const [id, neighbors] of this.neighborGraph) {
      for (const nid of neighbors) {
        const key = id < nid ? `${id}-${nid}` : `${nid}-${id}`;
        if (!seen.has(key)) {
          seen.add(key);
          links.push({ from: id, to: nid });
        }
      }
    }
    return links;
  }

  reset(): void {
    this.neighborGraph.clear();
    this.pending = [];
    this.sentCount = 0; this.deliveredCount = 0; this.droppedCount = 0; this.totalLatency = 0;
    this.delivered.clear();
  }
}
