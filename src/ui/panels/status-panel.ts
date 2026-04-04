/**
 * Subsystem status badges panel.
 * Displays canonical taxonomy: stub / simplified / experimental / validated.
 *
 * Merges two sources:
 *   1. Main-thread registry (stubs + demo modules registered via side-effect imports)
 *   2. Worker registry (physics/actuator modules that register in the worker context)
 * The worker sends its registry via 'subsystem-status' message on init/reset.
 */

import { getAllSubsystems } from '@sim/core/status-labels';
import type { SubsystemStatusEntry } from '@worker/worker-protocol';

interface DisplayEntry {
  id: string;
  status: string;
  description: string;
  demoOnly?: boolean;
}

export class StatusPanel {
  private container: HTMLDivElement;
  private badgeContainer: HTMLDivElement;
  private workerEntries: DisplayEntry[] = [];

  constructor(parent: HTMLElement) {
    this.container = document.createElement('div');
    this.container.className = 'panel';
    parent.appendChild(this.container);

    const title = document.createElement('div');
    title.className = 'panel-title';
    title.textContent = 'Subsystem Status';
    this.container.appendChild(title);

    this.badgeContainer = document.createElement('div');
    this.container.appendChild(this.badgeContainer);

    this.render();
  }

  /** Called when the worker sends its subsystem registry. */
  updateWorkerStatus(entries: SubsystemStatusEntry[]): void {
    this.workerEntries = entries;
    this.render();
  }

  private render(): void {
    while (this.badgeContainer.firstChild) {
      this.badgeContainer.removeChild(this.badgeContainer.firstChild);
    }

    // Collect all entries: worker entries first (physics), then main-thread (stubs)
    const seen = new Set<string>();
    const all: DisplayEntry[] = [];

    // Worker-side entries (physics, actuators, integrator, etc.)
    for (const entry of this.workerEntries) {
      all.push(entry);
      seen.add(entry.id);
    }

    // Main-thread entries (stubs registered via side-effect imports)
    const mainSubs = getAllSubsystems();
    for (const [id, entry] of mainSubs) {
      if (!seen.has(id)) {
        all.push({ id, status: entry.status, description: entry.description, demoOnly: entry.demoOnly });
      }
    }

    for (const entry of all) {
      const row = document.createElement('div');
      row.className = 'param-row';

      const label = document.createElement('span');
      label.className = 'param-label';
      label.textContent = entry.id;
      if (entry.demoOnly) {
        label.textContent += ' (demo)';
      }
      row.appendChild(label);

      const badge = document.createElement('span');
      badge.className = `status-badge ${entry.status}`;
      badge.textContent = entry.status;
      badge.title = entry.description;
      row.appendChild(badge);

      this.badgeContainer.appendChild(row);
    }
  }
}
