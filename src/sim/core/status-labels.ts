/**
 * Subsystem status registry.
 *
 * Canonical taxonomy (from AGENTS.md / ROADMAP.md):
 *   stub         — interface exists, no real implementation
 *   simplified   — implemented with documented simplifications
 *   experimental — fully implemented + tests, NOT yet audited
 *   validated    — tests pass AND audit approved
 *
 * Code may only set: stub, simplified, experimental.
 * Promotion to 'validated' requires explicit external audit.
 */

export type SubsystemStatus = 'stub' | 'simplified' | 'experimental' | 'validated';

export interface SubsystemEntry {
  status: SubsystemStatus;
  description: string;
  demoOnly?: boolean;
  simplifications?: string[];
}

const registry = new Map<string, SubsystemEntry>();

/** Register a subsystem with its status. Code MUST NOT pass 'validated'. */
export function registerSubsystem(
  id: string,
  status: 'stub' | 'simplified' | 'experimental',
  description: string,
  opts?: { demoOnly?: boolean; simplifications?: string[] },
): void {
  registry.set(id, {
    status,
    description,
    demoOnly: opts?.demoOnly,
    simplifications: opts?.simplifications,
  });
}

/** Get a subsystem entry. */
export function getSubsystem(id: string): SubsystemEntry | undefined {
  return registry.get(id);
}

/** Get all registered subsystems. */
export function getAllSubsystems(): Map<string, SubsystemEntry> {
  return new Map(registry);
}

/**
 * Promote a subsystem to 'validated'. This should ONLY be called
 * through an external audit process, never by simulation code.
 */
export function promoteToValidated(id: string): void {
  const entry = registry.get(id);
  if (entry) {
    entry.status = 'validated';
  }
}

/** Reset all registrations (for testing). */
export function clearRegistry(): void {
  registry.clear();
}
