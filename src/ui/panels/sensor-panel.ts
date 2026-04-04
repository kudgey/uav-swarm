/**
 * Sensor inspector panel.
 * Per-sensor enable toggles, live readouts with valid/invalid indicators.
 */

import type { WorkerCommand, DroneSnapshot, SensorSnapshot } from '@worker/worker-protocol';

export class SensorPanel {
  private container: HTMLDivElement;
  private sendCommand: (cmd: WorkerCommand) => void;
  private rows: Map<string, HTMLSpanElement> = new Map();
  private badges: Map<string, HTMLSpanElement> = new Map();

  constructor(parent: HTMLElement, sendCommand: (cmd: WorkerCommand) => void) {
    this.sendCommand = sendCommand;
    this.container = document.createElement('div');
    this.container.className = 'panel';
    parent.appendChild(this.container);

    const title = document.createElement('div');
    title.className = 'panel-title';
    title.textContent = 'Sensors';
    this.container.appendChild(title);

    this.addSensorGroup('IMU', 'imu', true,
      ['gyro X', 'gyro Y', 'gyro Z', 'accel X', 'accel Y', 'accel Z']);
    this.addSensorGroup('MAG', 'magnetometer', true,
      ['mag X', 'mag Y', 'mag Z']);
    this.addSensorGroup('BARO', 'barometer', true,
      ['pressure', 'baro alt']);
    this.addSensorGroup('RANGE', 'rangefinder', true,
      ['range']);
    this.addSensorGroup('FLOW', 'opticalFlow', true,
      ['flow X', 'flow Y']);
    this.addSensorGroup('VIO', 'cameraVIO', false,
      ['vio pX', 'vio pY', 'vio pZ', 'fQuality']);

    // Phase 11: Camera intrinsics on/off toggle
    let vioIntrOn = false;
    this.addPhase11Toggle('VIO intrinsics', (btn) => {
      vioIntrOn = !vioIntrOn;
      btn.textContent = vioIntrOn ? 'VIO intrinsics ON' : 'VIO intrinsics';
      btn.classList.toggle('active', vioIntrOn);
      this.sendCommand({ type: 'set-sensor-config', config: { cameraVIO: { cameraIntrinsicsEnabled: vioIntrOn } } } as WorkerCommand);
    });

    this.addSensorGroup('UWB', 'uwb', false,
      ['uwb info']);
  }

  private addSensorGroup(displayName: string, configKey: string, initialEnabled: boolean, fields: string[]): void {
    const header = document.createElement('div');
    header.style.cssText = 'display:flex;align-items:center;gap:6px;margin:6px 0 2px;';

    const label = document.createElement('span');
    label.textContent = displayName;
    label.style.cssText = 'color:var(--text-muted);font-size:10px;text-transform:uppercase;flex:1;';
    header.appendChild(label);

    // Enable/disable toggle
    const toggleBtn = document.createElement('button');
    toggleBtn.className = 'btn';
    toggleBtn.style.cssText = 'font-size:9px;padding:1px 6px;';
    toggleBtn.textContent = initialEnabled ? 'ON' : 'OFF';
    toggleBtn.classList.toggle('active', initialEnabled);
    let enabled = initialEnabled;
    toggleBtn.addEventListener('click', () => {
      enabled = !enabled;
      toggleBtn.textContent = enabled ? 'ON' : 'OFF';
      toggleBtn.classList.toggle('active', enabled);
      this.sendCommand({
        type: 'set-sensor-config',
        config: { [configKey]: { enabled } } as Record<string, unknown>,
      } as WorkerCommand);
    });
    header.appendChild(toggleBtn);

    const badge = document.createElement('span');
    badge.style.cssText = 'font-size:9px;padding:1px 4px;border-radius:3px;font-weight:600;';
    badge.textContent = '--';
    header.appendChild(badge);
    this.badges.set(displayName, badge);

    this.container.appendChild(header);

    for (const field of fields) {
      const row = document.createElement('div');
      row.className = 'param-row';
      const lbl = document.createElement('span');
      lbl.className = 'param-label';
      lbl.textContent = field;
      const val = document.createElement('span');
      val.className = 'param-value';
      val.textContent = '--';
      row.appendChild(lbl);
      row.appendChild(val);
      this.container.appendChild(row);
      this.rows.set(field, val);
    }
  }

  private addPhase11Toggle(label: string, onclick: (btn: HTMLButtonElement) => void): void {
    const btn = document.createElement('button');
    btn.className = 'btn'; btn.textContent = label; btn.style.cssText = 'font-size:9px;margin:2px 0;';
    btn.addEventListener('click', () => onclick(btn));
    this.container.appendChild(btn);
  }

  update(snap: DroneSnapshot): void {
    const s = snap.sensors;
    this.updateIMU(s);
    this.updateMag(s);
    this.updateBaro(s);
    this.updateRange(s);
    this.updateFlow(s);
    this.updateVIO(s);
    this.updateUWB(s);
  }

  private setBadge(name: string, valid: boolean | null): void {
    const b = this.badges.get(name);
    if (!b) return;
    if (valid === null) {
      b.textContent = '--'; b.style.background = 'var(--stub-color)'; b.style.color = '#fff';
    } else if (valid) {
      b.textContent = 'OK'; b.style.background = 'var(--success)'; b.style.color = '#000';
    } else {
      b.textContent = 'INV'; b.style.background = 'var(--error)'; b.style.color = '#fff';
    }
  }

  private updateIMU(s: SensorSnapshot): void {
    if (!s.imu) { this.setBadge('IMU', null); return; }
    this.setBadge('IMU', s.imu.valid);
    if (s.imu.valid) {
      this.rows.get('gyro X')!.textContent = s.imu.gyro[0].toFixed(3);
      this.rows.get('gyro Y')!.textContent = s.imu.gyro[1].toFixed(3);
      this.rows.get('gyro Z')!.textContent = s.imu.gyro[2].toFixed(3);
      this.rows.get('accel X')!.textContent = s.imu.accel[0].toFixed(2);
      this.rows.get('accel Y')!.textContent = s.imu.accel[1].toFixed(2);
      this.rows.get('accel Z')!.textContent = s.imu.accel[2].toFixed(2);
    }
  }

  private updateMag(s: SensorSnapshot): void {
    if (!s.mag) { this.setBadge('MAG', null); return; }
    this.setBadge('MAG', s.mag.valid);
    if (s.mag.valid) {
      this.rows.get('mag X')!.textContent = (s.mag.field[0] * 1e6).toFixed(1);
      this.rows.get('mag Y')!.textContent = (s.mag.field[1] * 1e6).toFixed(1);
      this.rows.get('mag Z')!.textContent = (s.mag.field[2] * 1e6).toFixed(1);
    }
  }

  private updateBaro(s: SensorSnapshot): void {
    if (!s.baro) { this.setBadge('BARO', null); return; }
    this.setBadge('BARO', s.baro.valid);
    if (s.baro.valid) {
      this.rows.get('pressure')!.textContent = s.baro.pressure.toFixed(0) + ' Pa';
      this.rows.get('baro alt')!.textContent = s.baro.altitude.toFixed(2) + ' m';
    }
  }

  private updateRange(s: SensorSnapshot): void {
    if (!s.range) { this.setBadge('RANGE', null); return; }
    this.setBadge('RANGE', s.range.valid);
    if (s.range.valid) {
      this.rows.get('range')!.textContent = s.range.range.toFixed(3) + ' m';
    }
  }

  private updateFlow(s: SensorSnapshot): void {
    if (!s.flow) { this.setBadge('FLOW', null); return; }
    this.setBadge('FLOW', s.flow.valid);
    if (s.flow.valid) {
      this.rows.get('flow X')!.textContent = s.flow.flowX.toFixed(3);
      this.rows.get('flow Y')!.textContent = s.flow.flowY.toFixed(3);
    }
  }

  private updateVIO(s: SensorSnapshot): void {
    if (!s.vio) { this.setBadge('VIO', null); return; }
    this.setBadge('VIO', s.vio.valid);
    if (s.vio.valid) {
      this.rows.get('vio pX')!.textContent = s.vio.position[0].toFixed(2);
      this.rows.get('vio pY')!.textContent = s.vio.position[1].toFixed(2);
      this.rows.get('vio pZ')!.textContent = s.vio.position[2].toFixed(2);
      this.rows.get('fQuality')!.textContent = s.vio.featureQuality.toFixed(2);
    }
  }

  private updateUWB(s: SensorSnapshot): void {
    if (!s.uwb || s.uwb.length === 0) { this.setBadge('UWB', null); return; }
    const validCount = s.uwb.filter(u => u.valid).length;
    this.setBadge('UWB', validCount > 0);
    const info = s.uwb.map(u =>
      `${u.anchorId}:${u.valid ? u.range.toFixed(1) + 'm' : 'X'}${u.isNLOS ? '*' : ''}`,
    ).join(' ');
    this.rows.get('uwb info')!.textContent = info;
  }
}
