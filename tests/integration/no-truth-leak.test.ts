/**
 * Integration test: no truth state leak.
 * Grep estimation/ and control/ for DroneState imports → zero matches.
 */
import { describe, it, expect } from 'vitest';
import * as fs from 'fs';
import * as path from 'path';

function findTSFiles(dir: string): string[] {
  const files: string[] = [];
  if (!fs.existsSync(dir)) return files;
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) files.push(...findTSFiles(full));
    else if (entry.name.endsWith('.ts')) files.push(full);
  }
  return files;
}

describe('No truth leak', () => {
  it('estimation/ and control/ files never import DroneState', () => {
    const root = path.resolve(__dirname, '../../src/sim');
    const dirs = ['estimation', 'control'];
    const violations: string[] = [];

    for (const dir of dirs) {
      const files = findTSFiles(path.join(root, dir));
      for (const file of files) {
        const content = fs.readFileSync(file, 'utf-8');
        // Check for actual TypeScript import statement containing DroneState
        // Must match: "import ... DroneState ... from" or "import type { ... DroneState"
        // Anchored to line start to exclude comments
        if (/^\s*import\s[^;]*\bDroneState\b/m.test(content)) {
          violations.push(path.relative(root, file));
        }
      }
    }

    expect(violations).toEqual([]);
  });
});
