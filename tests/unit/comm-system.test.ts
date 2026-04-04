import { describe, it, expect } from 'vitest';
import { CommunicationSystem } from '@sim/communications/comm-system';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { DeterministicRNG } from '@sim/core/rng';
import { v3Create } from '@lib/math';
import type { CommConfig } from '@sim/core/types';

const defaultComm: CommConfig = {
  range: 15, latencyMean: 0.05, latencyJitter: 0.01, dropRate: 0,
  bandwidth: 100, estimatePublishRate: 10, maneuverAccelStdDev: 2, estimateTimeout: 2,
};

describe('CommunicationSystem', () => {
  it('neighbor graph: drones within range are neighbors', () => {
    const comm = new CommunicationSystem(defaultComm, new DeterministicRNG(42), new WorldGeometry(openField()));
    const positions = new Map<number, Float64Array>();
    positions.set(0, v3Create(0, 0, -2));
    positions.set(1, v3Create(5, 0, -2));   // 5m away < 15m range
    positions.set(2, v3Create(20, 0, -2));  // 20m away > 15m range

    comm.updateNeighborGraph(positions);
    expect(comm.getNeighbors(0).has(1)).toBe(true);
    expect(comm.getNeighbors(0).has(2)).toBe(false);
    expect(comm.getNeighbors(1).has(0)).toBe(true);
  });

  it('message delivered after latency', () => {
    const comm = new CommunicationSystem(
      { ...defaultComm, latencyMean: 0.1, latencyJitter: 0, dropRate: 0 },
      new DeterministicRNG(42), new WorldGeometry(openField()));

    const positions = new Map<number, Float64Array>();
    positions.set(0, v3Create(0, 0, -2));
    positions.set(1, v3Create(5, 0, -2));
    comm.updateNeighborGraph(positions);

    comm.send({
      senderId: 0, receiverId: 1, sendTime: 1.0,
      type: 'state-estimate', payload: {} as never,
    });

    // Before latency
    comm.deliver(1.05);
    expect(comm.getMessages(1).length).toBe(0);

    // After latency
    comm.deliver(1.11);
    expect(comm.getMessages(1).length).toBe(1);
  });

  it('drop rate causes message loss', () => {
    const comm = new CommunicationSystem(
      { ...defaultComm, dropRate: 0.5, latencyMean: 0, latencyJitter: 0 },
      new DeterministicRNG(42), new WorldGeometry(openField()));

    const positions = new Map<number, Float64Array>();
    positions.set(0, v3Create(0, 0, -2));
    positions.set(1, v3Create(5, 0, -2));
    comm.updateNeighborGraph(positions);

    let delivered = 0;
    for (let i = 0; i < 200; i++) {
      comm.send({
        senderId: 0, receiverId: 1, sendTime: i * 0.01,
        type: 'state-estimate', payload: {} as never,
      });
      comm.deliver(i * 0.01 + 0.001);
      delivered += comm.getMessages(1).length;
    }

    expect(delivered).toBeGreaterThan(60);
    expect(delivered).toBeLessThan(140);
  });

  it('getLinks returns unique undirected links', () => {
    const comm = new CommunicationSystem(defaultComm, new DeterministicRNG(42), new WorldGeometry(openField()));
    const positions = new Map<number, Float64Array>();
    positions.set(0, v3Create(0, 0, -2));
    positions.set(1, v3Create(5, 0, -2));
    positions.set(2, v3Create(10, 0, -2));
    comm.updateNeighborGraph(positions);

    const links = comm.getLinks();
    expect(links.length).toBe(3); // 0-1, 1-2, 0-2 (all within 15m)
  });
});
