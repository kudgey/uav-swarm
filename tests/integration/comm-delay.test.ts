import { describe, it, expect } from 'vitest';
import { CommunicationSystem } from '@sim/communications/comm-system';
import { WorldGeometry } from '@sim/environment/world-geometry';
import { openField } from '@sim/environment/scene-presets';
import { DeterministicRNG } from '@sim/core/rng';
import { v3Create } from '@lib/math';
import type { CommConfig } from '@sim/core/types';

describe('Communication delay', () => {
  it('message arrives after latency, not before', () => {
    const cfg: CommConfig = {
      range: 30, latencyMean: 0.1, latencyJitter: 0, dropRate: 0,
      bandwidth: 100, estimatePublishRate: 10, maneuverAccelStdDev: 2, estimateTimeout: 2,
    };
    const comm = new CommunicationSystem(cfg, new DeterministicRNG(42), new WorldGeometry(openField()));
    const positions = new Map<number, Float64Array>();
    positions.set(0, v3Create(0, 0, -2));
    positions.set(1, v3Create(5, 0, -2));
    comm.updateNeighborGraph(positions);

    const sendTime = 1.0;
    comm.send({
      senderId: 0, receiverId: 1, sendTime,
      type: 'state-estimate', payload: {} as never,
    });

    // At sendTime + 0.05: not yet delivered (latency = 0.1)
    comm.deliver(sendTime + 0.05);
    expect(comm.getMessages(1).length).toBe(0);

    // At sendTime + 0.10: delivered
    comm.deliver(sendTime + 0.10);
    const msgs = comm.getMessages(1);
    expect(msgs.length).toBe(1);
    expect(msgs[0].senderId).toBe(0);
  });
});
