/**
 * Formation control panel.
 * Mode, topology, spacing, enable/disable, per-drone comm toggle.
 */

import type { WorkerCommand, SwarmSnapshot } from '@worker/worker-protocol';

export class FormationPanel {
  private container: HTMLDivElement;
  private sendCommand: (cmd: WorkerCommand) => void;
  private statusDiv: HTMLDivElement;
  private modeSelect!: HTMLSelectElement;
  private topoSelect!: HTMLSelectElement;
  private spacingInput!: HTMLInputElement;
  private gainInput!: HTMLInputElement;
  private gainRow!: HTMLDivElement;
  private leaderInput!: HTMLInputElement;
  private leaderRow!: HTMLDivElement;
  private fallbackSelect!: HTMLSelectElement;
  private offsetFrameSelect!: HTMLSelectElement;
  private commToggles: HTMLDivElement;

  constructor(parent: HTMLElement, sendCommand: (cmd: WorkerCommand) => void) {
    this.sendCommand = sendCommand;
    this.container = document.createElement('div');
    this.container.className = 'panel';
    parent.appendChild(this.container);

    const title = document.createElement('div');
    title.className = 'panel-title';
    title.textContent = 'Formation';
    this.container.appendChild(title);

    // Mode
    this.modeSelect = this.addSelect('Mode', ['leader-follower', 'consensus']);
    this.modeSelect.addEventListener('change', () => this.updateVisibility());

    // Topology
    this.topoSelect = this.addSelect('Topology', ['line', 'grid', 'circle']);

    // Spacing
    this.spacingInput = this.addNumberInput('Spacing (m)', 2, 0.5, 10, 0.5);

    // Consensus gain (visible only in consensus mode)
    this.gainRow = document.createElement('div');
    this.gainRow.className = 'param-row';
    const gainLabel = document.createElement('span');
    gainLabel.className = 'param-label';
    gainLabel.textContent = 'Consensus gain';
    this.gainInput = document.createElement('input');
    this.gainInput.type = 'number';
    this.gainInput.value = '0.5';
    this.gainInput.min = '0'; this.gainInput.max = '1'; this.gainInput.step = '0.1';
    this.gainInput.style.width = '50px';
    this.gainRow.appendChild(gainLabel);
    this.gainRow.appendChild(this.gainInput);
    this.container.appendChild(this.gainRow);

    // Leader ID (visible only in leader-follower mode)
    this.leaderRow = document.createElement('div');
    this.leaderRow.className = 'param-row';
    const leaderLabel = document.createElement('span');
    leaderLabel.className = 'param-label';
    leaderLabel.textContent = 'Leader ID';
    this.leaderInput = document.createElement('input');
    this.leaderInput.type = 'number';
    this.leaderInput.value = '0';
    this.leaderInput.min = '0'; this.leaderInput.step = '1';
    this.leaderInput.style.width = '50px';
    this.leaderRow.appendChild(leaderLabel);
    this.leaderRow.appendChild(this.leaderInput);
    this.container.appendChild(this.leaderRow);

    // Fallback
    this.fallbackSelect = this.addSelect('Leader loss', ['hover', 'reelect']);

    // Offset frame (Phase 11)
    this.offsetFrameSelect = this.addSelect('Offset frame', ['world', 'heading']);

    // Enable/Disable buttons
    const btnRow = document.createElement('div');
    btnRow.style.cssText = 'display:flex;gap:4px;margin:6px 0;';
    const enableBtn = document.createElement('button');
    enableBtn.className = 'btn';
    enableBtn.textContent = 'Enable';
    enableBtn.addEventListener('click', () => this.sendFormationConfig(true));
    btnRow.appendChild(enableBtn);
    const disableBtn = document.createElement('button');
    disableBtn.className = 'btn';
    disableBtn.textContent = 'Disable';
    disableBtn.addEventListener('click', () => this.sendFormationConfig(false));
    btnRow.appendChild(disableBtn);
    this.container.appendChild(btnRow);

    // Per-drone comm toggles
    this.commToggles = document.createElement('div');
    this.commToggles.style.cssText = 'font-size:var(--font-size-sm);margin-top:4px;';
    this.container.appendChild(this.commToggles);

    // Status readout
    this.statusDiv = document.createElement('div');
    this.statusDiv.style.cssText = 'font-size:var(--font-size-sm);margin-top:4px;white-space:pre;';
    this.container.appendChild(this.statusDiv);

    this.updateVisibility();
  }

  private addSelect(label: string, options: string[]): HTMLSelectElement {
    const row = document.createElement('div');
    row.className = 'param-row';
    const lbl = document.createElement('span');
    lbl.className = 'param-label';
    lbl.textContent = label;
    const sel = document.createElement('select');
    sel.style.cssText = 'font-size:10px;';
    for (const opt of options) {
      const o = document.createElement('option');
      o.value = opt; o.textContent = opt;
      sel.appendChild(o);
    }
    row.appendChild(lbl);
    row.appendChild(sel);
    this.container.appendChild(row);
    return sel;
  }

  private addNumberInput(label: string, value: number, min: number, max: number, step: number): HTMLInputElement {
    const row = document.createElement('div');
    row.className = 'param-row';
    const lbl = document.createElement('span');
    lbl.className = 'param-label';
    lbl.textContent = label;
    const inp = document.createElement('input');
    inp.type = 'number';
    inp.value = String(value);
    inp.min = String(min); inp.max = String(max); inp.step = String(step);
    inp.style.width = '50px';
    row.appendChild(lbl);
    row.appendChild(inp);
    this.container.appendChild(row);
    return inp;
  }

  private updateVisibility(): void {
    const isConsensus = this.modeSelect.value === 'consensus';
    this.gainRow.style.display = isConsensus ? 'flex' : 'none';
    this.leaderRow.style.display = isConsensus ? 'none' : 'flex';
  }

  private sendFormationConfig(enabled: boolean): void {
    this.sendCommand({
      type: 'set-formation-config',
      config: {
        enabled,
        mode: this.modeSelect.value as 'leader-follower' | 'consensus',
        topology: this.topoSelect.value as 'line' | 'grid' | 'circle',
        spacing: parseFloat(this.spacingInput.value) || 2,
        leaderDroneId: parseInt(this.leaderInput.value) || 0,
        consensusGain: parseFloat(this.gainInput.value) || 0.5,
        leaderLossFallback: this.fallbackSelect.value as 'hover' | 'reelect',
        offsetFrame: (this.offsetFrameSelect?.value ?? 'world') as 'world' | 'heading',
      },
    });
  }

  private lastDroneCount = 0;

  update(swarm: SwarmSnapshot): void {
    const fm = swarm.formation;

    // Sync controls from worker state (prevents drift after init/reset/rejected config)
    if (this.modeSelect.value !== fm.mode) this.modeSelect.value = fm.mode;
    if (this.topoSelect.value !== fm.topology) this.topoSelect.value = fm.topology;
    if (this.spacingInput.value !== String(fm.spacing)) this.spacingInput.value = String(fm.spacing);
    if (this.gainInput.value !== String(fm.consensusGain)) this.gainInput.value = String(fm.consensusGain);
    if (this.leaderInput.value !== String(fm.leaderDroneId)) this.leaderInput.value = String(fm.leaderDroneId);
    if (this.fallbackSelect.value !== fm.leaderLossFallback) this.fallbackSelect.value = fm.leaderLossFallback;
    if (this.offsetFrameSelect.value !== fm.offsetFrame) this.offsetFrameSelect.value = fm.offsetFrame;
    this.updateVisibility();

    // Rebuild per-drone comm toggles if drone count changed
    if (swarm.drones.length !== this.lastDroneCount) {
      this.lastDroneCount = swarm.drones.length;
      while (this.commToggles.firstChild) this.commToggles.removeChild(this.commToggles.firstChild);
      for (const d of swarm.drones) {
        const row = document.createElement('div');
        row.style.cssText = 'display:inline-flex;align-items:center;gap:2px;margin-right:6px;';
        const cb = document.createElement('input');
        cb.type = 'checkbox';
        cb.checked = d.commEnabled;
        const droneId = d.id;
        cb.addEventListener('change', () => {
          this.sendCommand({ type: 'set-drone-comm', droneId, commEnabled: cb.checked });
        });
        const lbl = document.createElement('span');
        lbl.textContent = `D${d.id}`;
        lbl.style.fontSize = '9px';
        row.appendChild(cb);
        row.appendChild(lbl);
        this.commToggles.appendChild(row);
      }
    }

    // Status
    const lines: string[] = [];
    lines.push(`${fm.enabled ? 'ENABLED' : 'DISABLED'} | ${fm.mode} | ${fm.topology} | ${fm.spacing}m`);
    if (fm.enabled) {
      const selected = swarm.drones.find(d => d.id === swarm.selectedDroneId);
      if (selected?.formationState) {
        const fs = selected.formationState;
        lines.push(`Role: ${fs.role} | Ref: ${fs.referenceHealthy ? 'OK' : 'LOST'} | Nbrs: ${fs.neighborCount}`);
      }
    }
    this.statusDiv.textContent = lines.join('\n');
  }
}
