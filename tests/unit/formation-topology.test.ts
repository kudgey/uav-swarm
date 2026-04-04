import { describe, it, expect } from 'vitest';
import { computeTopologyOffsets } from '@sim/swarm/formation';

describe('Formation topology offsets', () => {
  it('line: 4 drones with 2m spacing', () => {
    const t = computeTopologyOffsets(4, 2, 'line');
    expect(t.offsets.length).toBe(4);
    // Centered: -3, -1, +1, +3
    expect(t.offsets[0][0]).toBeCloseTo(-3, 5);
    expect(t.offsets[1][0]).toBeCloseTo(-1, 5);
    expect(t.offsets[2][0]).toBeCloseTo(1, 5);
    expect(t.offsets[3][0]).toBeCloseTo(3, 5);
    // Y and Z should be 0
    for (const o of t.offsets) {
      expect(o[1]).toBeCloseTo(0, 10);
      expect(o[2]).toBeCloseTo(0, 10);
    }
  });

  it('circle: 4 drones with 2m radius', () => {
    const t = computeTopologyOffsets(4, 2, 'circle');
    expect(t.offsets.length).toBe(4);
    // Should be at cardinal directions, radius=2
    expect(t.offsets[0][0]).toBeCloseTo(2, 5); // 0°
    expect(t.offsets[1][1]).toBeCloseTo(2, 5); // 90°
  });

  it('grid: 4 drones with 2m spacing', () => {
    const t = computeTopologyOffsets(4, 2, 'grid');
    expect(t.offsets.length).toBe(4);
    // 2x2 grid
    expect(Math.abs(t.offsets[0][0] - t.offsets[1][0])).toBeCloseTo(2, 5);
  });

  it('single drone: offset at origin', () => {
    const t = computeTopologyOffsets(1, 2, 'line');
    expect(t.offsets[0][0]).toBeCloseTo(0, 10);
  });
});
