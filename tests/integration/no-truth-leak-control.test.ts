/**
 * Verify controller module also has no DroneState imports.
 * (Covered by no-truth-leak.test.ts which checks both estimation/ and control/)
 */
import { describe, it, expect } from 'vitest';

describe('Controller no-truth-leak', () => {
  it('controller.ts does not import DroneState', async () => {
    // Read the controller source
    const fs = await import('fs');
    const path = await import('path');
    const controllerPath = path.resolve(__dirname, '../../src/sim/control/controller.ts');
    const content = fs.readFileSync(controllerPath, 'utf-8');
    expect(content).not.toMatch(/\bDroneState\b/);
  });
});
