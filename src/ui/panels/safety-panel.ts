/**
 * Safety configuration and metrics panel.
 * Full controls: enable/disable, minSeparation, orcaRadius, timeHorizon, maxSpeed, minAltitude.
 */

import type { WorkerCommand, SwarmSnapshot } from '@worker/worker-protocol';

export class SafetyPanel {
  private container: HTMLDivElement;
  private sendCommand: (cmd: WorkerCommand) => void;
  private metricsDiv: HTMLDivElement;
  private eventsDiv: HTMLDivElement;
  private minSepInput!: HTMLInputElement;
  private orcaRadInput!: HTMLInputElement;
  private timeHorizInput!: HTMLInputElement;
  private maxSpeedInput!: HTMLInputElement;
  private minAltInput!: HTMLInputElement;

  constructor(parent: HTMLElement, sendCommand: (cmd: WorkerCommand) => void) {
    this.sendCommand = sendCommand;
    this.container = document.createElement('div');
    this.container.className = 'panel';
    parent.appendChild(this.container);

    const title = document.createElement('div');
    title.className = 'panel-title';
    title.textContent = 'Safety';
    this.container.appendChild(title);

    // Enable/disable
    const btnRow = document.createElement('div');
    btnRow.style.cssText = 'display:flex;gap:4px;margin-bottom:6px;';
    this.makeBtn(btnRow, 'Enable', () => this.send({ enabled: true }));
    this.makeBtn(btnRow, 'Disable', () => this.send({ enabled: false }));
    this.container.appendChild(btnRow);

    // Config controls
    this.minSepInput = this.addNumRow('Min separation (m)', 0.5, 0.1, 5, 0.1);
    this.orcaRadInput = this.addNumRow('ORCA radius (m)', 0.3, 0.1, 2, 0.1);
    this.timeHorizInput = this.addNumRow('Time horizon (s)', 3, 0.5, 10, 0.5);
    this.maxSpeedInput = this.addNumRow('Max speed (m/s)', 5, 1, 20, 1);
    this.minAltInput = this.addNumRow('Min altitude (m)', 0.3, 0.1, 5, 0.1);

    const applyBtn = document.createElement('button');
    applyBtn.className = 'btn';
    applyBtn.textContent = 'Apply';
    applyBtn.style.marginTop = '4px';
    applyBtn.addEventListener('click', () => this.sendAll());
    this.container.appendChild(applyBtn);

    // Phase 11 advanced toggles
    const advRow = document.createElement('div');
    advRow.style.cssText = 'display:flex;gap:4px;margin:4px 0;flex-wrap:wrap;';
    const nlosBtn = document.createElement('button');
    nlosBtn.className = 'btn'; nlosBtn.textContent = 'NLOS detect'; nlosBtn.style.fontSize = '9px';
    let nlosOn = false;
    nlosBtn.addEventListener('click', () => {
      nlosOn = !nlosOn;
      nlosBtn.textContent = nlosOn ? 'NLOS detect ON' : 'NLOS detect';
      nlosBtn.classList.toggle('active', nlosOn);
      this.sendCommand({ type: 'set-estimation-config', config: { innovationNLOSDetection: nlosOn } } as WorkerCommand);
    });
    advRow.appendChild(nlosBtn);
    const losVarBtn = document.createElement('button');
    losVarBtn.className = 'btn'; losVarBtn.textContent = 'LOS variance'; losVarBtn.style.fontSize = '9px';
    let losEnabled = false;
    losVarBtn.addEventListener('click', () => {
      losEnabled = !losEnabled;
      losVarBtn.classList.toggle('active', losEnabled);
      this.sendCommand({ type: 'set-comm-config', config: { losProjectedVariance: losEnabled } } as WorkerCommand);
    });
    advRow.appendChild(losVarBtn);
    this.container.appendChild(advRow);

    // Metrics readout
    this.metricsDiv = document.createElement('div');
    this.metricsDiv.style.cssText = 'font-size:var(--font-size-sm);white-space:pre;margin-top:6px;';
    this.container.appendChild(this.metricsDiv);

    // Events readout
    this.eventsDiv = document.createElement('div');
    this.eventsDiv.style.cssText = 'font-size:9px;white-space:pre;margin-top:4px;color:var(--text-muted);max-height:60px;overflow-y:auto;';
    this.container.appendChild(this.eventsDiv);
  }

  private makeBtn(parent: HTMLElement, label: string, onclick: () => void): void {
    const btn = document.createElement('button');
    btn.className = 'btn'; btn.textContent = label;
    btn.addEventListener('click', onclick);
    parent.appendChild(btn);
  }

  private addNumRow(label: string, value: number, min: number, max: number, step: number): HTMLInputElement {
    const row = document.createElement('div');
    row.className = 'param-row';
    const lbl = document.createElement('span');
    lbl.className = 'param-label'; lbl.textContent = label;
    const inp = document.createElement('input');
    inp.type = 'number'; inp.value = String(value);
    inp.min = String(min); inp.max = String(max); inp.step = String(step);
    inp.style.width = '55px';
    row.appendChild(lbl); row.appendChild(inp);
    this.container.appendChild(row);
    return inp;
  }

  private send(partial: Record<string, unknown>): void {
    this.sendCommand({ type: 'set-safety-config', config: partial as never });
  }

  private sendAll(): void {
    this.send({
      minSeparation: parseFloat(this.minSepInput.value),
      orcaRadius: parseFloat(this.orcaRadInput.value),
      orcaTimeHorizon: parseFloat(this.timeHorizInput.value),
      maxSpeed: parseFloat(this.maxSpeedInput.value),
      minAltitude: parseFloat(this.minAltInput.value),
    });
  }

  update(swarm: SwarmSnapshot): void {
    // Sync controls from worker state
    const s = swarm.safety;
    if (s) {
      if (this.minSepInput.value !== String(s.minSeparation)) this.minSepInput.value = String(s.minSeparation);
      if (this.orcaRadInput.value !== String(s.orcaRadius)) this.orcaRadInput.value = String(s.orcaRadius);
      if (this.timeHorizInput.value !== String(s.orcaTimeHorizon)) this.timeHorizInput.value = String(s.orcaTimeHorizon);
      if (this.maxSpeedInput.value !== String(s.maxSpeed)) this.maxSpeedInput.value = String(s.maxSpeed);
      if (this.minAltInput.value !== String(s.minAltitude)) this.minAltInput.value = String(s.minAltitude);
    }

    // Metrics
    const m = swarm.safetyMetrics;
    const lines = [
      `${s?.enabled ? 'ENABLED' : 'DISABLED'}`,
      `Min sep: ${m.minSeparation === Infinity ? '--' : m.minSeparation.toFixed(2)}m`,
      `Collisions: ${m.collisionCount} | Overrides: ${m.safetyOverrideCount} | Emergency: ${m.emergencyStopCount}`,
    ];
    const selected = swarm.drones.find(d => d.id === swarm.selectedDroneId);
    if (selected?.safetyState?.active) {
      lines.push(`D${selected.id}: ${selected.safetyState.reason} (${selected.safetyState.constraintCount})`);
    }
    this.metricsDiv.textContent = lines.join('\n');

    // Events
    if (swarm.safetyEvents.length > 0) {
      this.eventsDiv.textContent = swarm.safetyEvents
        .map(e => `t=${e.timestamp.toFixed(2)} D${e.droneId} ${e.type}`)
        .join('\n');
    }
  }
}
