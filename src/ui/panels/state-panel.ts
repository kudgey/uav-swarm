/**
 * Truth state readout panel.
 * Shows position, velocity, Euler angles, angular velocity, motor speeds, thrust.
 */

import type { DroneSnapshot } from '@worker/worker-protocol';
import { RAD_TO_DEG } from '@sim/core/units';

export class StatePanel {
  private container: HTMLDivElement;
  private rows: Map<string, HTMLSpanElement> = new Map();

  constructor(parent: HTMLElement) {
    this.container = document.createElement('div');
    this.container.className = 'panel';

    const title = document.createElement('div');
    title.className = 'panel-title';
    title.textContent = 'Truth State';
    this.container.appendChild(title);

    parent.appendChild(this.container);

    const fields = [
      'pos X', 'pos Y', 'pos Z',
      'vel X', 'vel Y', 'vel Z',
      'roll', 'pitch', 'yaw',
      'ω X', 'ω Y', 'ω Z',
      'M1', 'M2', 'M3', 'M4',
      'Thrust',
    ];

    for (const label of fields) {
      const row = document.createElement('div');
      row.className = 'param-row';
      const lbl = document.createElement('span');
      lbl.className = 'param-label';
      lbl.textContent = label;
      const val = document.createElement('span');
      val.className = 'param-value';
      val.textContent = '0.000';
      row.appendChild(lbl);
      row.appendChild(val);
      this.container.appendChild(row);
      this.rows.set(label, val);
    }
  }

  update(snap: DroneSnapshot): void {
    const fmt = (v: number) => v.toFixed(3);
    const fmtInt = (v: number) => v.toFixed(0);

    this.rows.get('pos X')!.textContent = fmt(snap.position[0]);
    this.rows.get('pos Y')!.textContent = fmt(snap.position[1]);
    this.rows.get('pos Z')!.textContent = fmt(snap.position[2]);

    this.rows.get('vel X')!.textContent = fmt(snap.velocity[0]);
    this.rows.get('vel Y')!.textContent = fmt(snap.velocity[1]);
    this.rows.get('vel Z')!.textContent = fmt(snap.velocity[2]);

    // Euler from quaternion
    const [qw, qx, qy, qz] = snap.quaternion;
    const sinr = 2 * (qw * qx + qy * qz);
    const cosr = 1 - 2 * (qx * qx + qy * qy);
    const roll = Math.atan2(sinr, cosr);
    const sinp = 2 * (qw * qy - qz * qx);
    const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);
    const siny = 2 * (qw * qz + qx * qy);
    const cosy = 1 - 2 * (qy * qy + qz * qz);
    const yaw = Math.atan2(siny, cosy);

    this.rows.get('roll')!.textContent = fmt(roll * RAD_TO_DEG) + '\u00B0';
    this.rows.get('pitch')!.textContent = fmt(pitch * RAD_TO_DEG) + '\u00B0';
    this.rows.get('yaw')!.textContent = fmt(yaw * RAD_TO_DEG) + '\u00B0';

    this.rows.get('ω X')!.textContent = fmt(snap.angularVelocity[0]);
    this.rows.get('ω Y')!.textContent = fmt(snap.angularVelocity[1]);
    this.rows.get('ω Z')!.textContent = fmt(snap.angularVelocity[2]);

    this.rows.get('M1')!.textContent = fmtInt(snap.motorSpeeds[0]);
    this.rows.get('M2')!.textContent = fmtInt(snap.motorSpeeds[1]);
    this.rows.get('M3')!.textContent = fmtInt(snap.motorSpeeds[2]);
    this.rows.get('M4')!.textContent = fmtInt(snap.motorSpeeds[3]);

    this.rows.get('Thrust')!.textContent = fmt(snap.totalThrust) + ' N';
  }
}
