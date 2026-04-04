/**
 * Integration test: formation control files don't import DroneState.
 * formation.ts and formation-types.ts are estimate-only.
 * formation-metrics.ts IS allowed (truth-based validation metrics).
 */
import { describe, it, expect } from 'vitest';
import * as fs from 'fs';
import * as path from 'path';

describe('Formation no truth leak', () => {
  it('formation.ts and formation-types.ts never import DroneState', () => {
    const root = path.resolve(__dirname, '../../src/sim/swarm');
    const files = ['formation.ts', 'formation-types.ts'];
    const violations: string[] = [];

    for (const file of files) {
      const fullPath = path.join(root, file);
      if (!fs.existsSync(fullPath)) continue;
      const content = fs.readFileSync(fullPath, 'utf-8');
      if (/^\s*import\s[^;]*\bDroneState\b/m.test(content)) {
        violations.push(file);
      }
    }

    expect(violations).toEqual([]);
  });
});
