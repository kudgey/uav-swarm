/**
 * Controller panel: arm state, control mode, estimate display.
 */

import type { WorkerCommand, DroneSnapshot } from '@worker/worker-protocol';

export class ControllerPanel {
  private container: HTMLDivElement;
  private sendCommand: (cmd: WorkerCommand) => void;
  private armBadge: HTMLSpanElement;
  private modeBadge: HTMLSpanElement;
  private estReadout: HTMLDivElement;

  constructor(parent: HTMLElement, sendCommand: (cmd: WorkerCommand) => void) {
    this.sendCommand = sendCommand;
    this.container = document.createElement('div');
    this.container.className = 'panel';
    parent.appendChild(this.container);

    const title = document.createElement('div');
    title.className = 'panel-title';
    title.textContent = 'Controller';
    this.container.appendChild(title);

    // Arm state + mode badges
    const badgeRow = document.createElement('div');
    badgeRow.className = 'param-row';
    badgeRow.style.marginBottom = '6px';

    this.armBadge = document.createElement('span');
    this.armBadge.className = 'status-badge stub';
    this.armBadge.textContent = 'DISARMED';
    badgeRow.appendChild(this.armBadge);

    this.modeBadge = document.createElement('span');
    this.modeBadge.className = 'status-badge stub';
    this.modeBadge.textContent = 'MANUAL';
    this.modeBadge.style.marginLeft = '4px';
    badgeRow.appendChild(this.modeBadge);

    this.container.appendChild(badgeRow);

    // Mode buttons
    const btnRow = document.createElement('div');
    btnRow.style.cssText = 'display:flex;gap:4px;margin-bottom:6px;flex-wrap:wrap;';

    this.makeBtn(btnRow, 'Manual', () => {
      this.sendCommand({ type: 'set-controller-mode', droneId: this.lastDroneId, mode: 'manual' });
    });

    this.makeBtn(btnRow, 'Hover Here', () => {
      const p = this.lastEstPos;
      this.sendCommand({
        type: 'set-controller-mode', droneId: this.lastDroneId, mode: 'hover',
        params: { position: p, yaw: 0 },
      });
    });

    this.makeBtn(btnRow, 'WP +5m N', () => {
      const p = this.lastEstPos;
      this.sendCommand({
        type: 'set-controller-mode', droneId: this.lastDroneId, mode: 'waypoint',
        params: { position: [p[0] + 5, p[1], p[2]], yaw: 0, speed: 1.0 },
      });
    });

    this.container.appendChild(btnRow);

    // Swarm-wide buttons
    const swarmRow = document.createElement('div');
    swarmRow.style.cssText = 'display:flex;gap:4px;margin-bottom:6px;flex-wrap:wrap;';

    this.makeBtn(swarmRow, 'Hover All', () => {
      for (let i = 0; i < this.droneCount; i++) {
        const pos = this.dronePositions.get(i);
        if (pos) {
          this.sendCommand({
            type: 'set-controller-mode', droneId: i, mode: 'hover',
            params: { position: [pos[0], pos[1], pos[2]], yaw: 0 },
          });
        }
      }
    });

    this.makeBtn(swarmRow, 'Land All', () => {
      for (let i = 0; i < this.droneCount; i++) {
        this.sendCommand({ type: 'set-controller-mode', droneId: i, mode: 'manual' });
      }
    });

    this.container.appendChild(swarmRow);

    // Estimate readout
    this.estReadout = document.createElement('div');
    this.estReadout.style.cssText = 'font-size:var(--font-size-sm);';
    this.container.appendChild(this.estReadout);
  }

  private lastEstPos: [number, number, number] = [0, 0, -2];
  private lastDroneId = 0;
  private droneCount = 1;
  private dronePositions: Map<number, [number, number, number]> = new Map();

  private makeBtn(parent: HTMLElement, label: string, onclick: () => void): void {
    const btn = document.createElement('button');
    btn.className = 'btn';
    btn.textContent = label;
    btn.style.fontSize = '10px';
    btn.addEventListener('click', onclick);
    parent.appendChild(btn);
  }

  /** Update all drone positions for swarm-wide commands. */
  updateSwarm(drones: DroneSnapshot[]): void {
    this.droneCount = drones.length;
    for (const d of drones) {
      if (d.estimate) {
        this.dronePositions.set(d.id, d.estimate.position);
      }
    }
  }

  update(snap: DroneSnapshot & { droneCount?: number; commLinkCount?: number }): void {
    // Track selected drone id and estimated position
    this.lastDroneId = snap.id;
    if (snap.estimate) {
      this.lastEstPos = snap.estimate.position;
    }

    // Arm badge
    const arm = snap.armState;
    this.armBadge.textContent = arm.toUpperCase();
    this.armBadge.className = 'status-badge ' + (
      arm === 'armed' ? 'validated' : arm === 'aligning' ? 'experimental' : 'stub');

    // Mode badge
    const mode = snap.controlMode;
    this.modeBadge.textContent = mode.toUpperCase();
    this.modeBadge.className = 'status-badge ' + (
      mode === 'manual' ? 'stub' : 'experimental');

    // Estimate readout
    const e = snap.estimate;
    if (e) {
      const h = e.health;
      const lines = [
        `Est pos: [${e.position[0].toFixed(2)}, ${e.position[1].toFixed(2)}, ${e.position[2].toFixed(2)}]`,
        `Est vel: [${e.velocity[0].toFixed(2)}, ${e.velocity[1].toFixed(2)}, ${e.velocity[2].toFixed(2)}]`,
        `Att cov: ${h.attCovMax.toFixed(4)} | Vert: ${h.vertCov.toFixed(3)} | Horiz: ${h.horizCovMax.toFixed(1)}`,
        `${h.aligned ? 'ALIGNED' : 'NOT ALIGNED'} | Att: ${h.attitudeHealthy ? 'OK' : 'LOST'} | Vert: ${h.verticalHealthy ? 'OK' : 'LOST'}`,
      ];
      this.estReadout.textContent = lines.join('\n');
      this.estReadout.style.whiteSpace = 'pre';
    } else {
      this.estReadout.textContent = 'EKF not initialized';
    }
  }
}
