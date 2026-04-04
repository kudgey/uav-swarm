/**
 * Simulation control panel: Start/Pause/Reset/Step + speed slider + swarm config.
 */

import type { WorkerCommand } from '@worker/worker-protocol';

export class ControlPanel {
  private container: HTMLDivElement;
  private sendCommand: (cmd: WorkerCommand) => void;
  private running = false;
  private startBtn!: HTMLButtonElement;
  private speedDisplay!: HTMLSpanElement;
  private droneCountInput!: HTMLInputElement;
  private patternSelect!: HTMLSelectElement;
  private spacingInput!: HTMLInputElement;

  constructor(parent: HTMLElement, sendCommand: (cmd: WorkerCommand) => void) {
    this.sendCommand = sendCommand;
    this.container = document.createElement('div');
    this.container.style.cssText = `
      display: flex; align-items: center; gap: 6px; padding: 6px 10px;
      background: var(--bg-secondary); border-bottom: 1px solid var(--border);
      flex-wrap: wrap;
    `;
    parent.appendChild(this.container);
    this.build();
  }

  private build(): void {
    // Start/Pause
    this.startBtn = this.makeBtn('Start', () => {
      this.running = !this.running;
      this.startBtn.textContent = this.running ? 'Pause' : 'Start';
      this.startBtn.classList.toggle('active', this.running);
      this.sendCommand({ type: this.running ? 'start' : 'pause' });
    });

    // Reset
    this.makeBtn('Reset', () => {
      this.running = false;
      this.startBtn.textContent = 'Start';
      this.startBtn.classList.remove('active');
      this.sendCommand({ type: 'reset' });
    });

    // Step
    this.makeBtn('Step', () => {
      this.sendCommand({ type: 'step', count: 1 });
    });

    // Separator
    const sep = document.createElement('div');
    sep.style.cssText = 'width:1px;height:20px;background:var(--border);margin:0 4px;';
    this.container.appendChild(sep);

    // Speed slider
    const speedLabel = document.createElement('span');
    speedLabel.textContent = 'Speed:';
    speedLabel.style.cssText = 'color:var(--text-secondary);font-size:var(--font-size-sm);';
    this.container.appendChild(speedLabel);

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = '0.1';
    slider.max = '5';
    slider.step = '0.1';
    slider.value = '1';
    slider.style.width = '100px';
    this.container.appendChild(slider);

    this.speedDisplay = document.createElement('span');
    this.speedDisplay.textContent = '1.0x';
    this.speedDisplay.style.cssText = 'color:var(--text-primary);font-size:var(--font-size-sm);min-width:30px;';
    this.container.appendChild(this.speedDisplay);

    slider.addEventListener('input', () => {
      const val = parseFloat(slider.value);
      this.speedDisplay.textContent = val.toFixed(1) + 'x';
      this.sendCommand({ type: 'set-timescale', value: val });
    });

    // Sim time display
    const sep2 = document.createElement('div');
    sep2.style.cssText = 'width:1px;height:20px;background:var(--border);margin:0 4px;';
    this.container.appendChild(sep2);

    const timeLabel = document.createElement('span');
    timeLabel.id = 'sim-time-display';
    timeLabel.textContent = 't = 0.000s';
    timeLabel.style.cssText = 'color:var(--text-secondary);font-size:var(--font-size-sm);font-variant-numeric:tabular-nums;';
    this.container.appendChild(timeLabel);

    // Separator
    const sep3 = document.createElement('div');
    sep3.style.cssText = 'width:1px;height:20px;background:var(--border);margin:0 4px;';
    this.container.appendChild(sep3);

    // Swarm config: drone count
    const countLabel = document.createElement('span');
    countLabel.textContent = 'Drones:';
    countLabel.style.cssText = 'color:var(--text-secondary);font-size:var(--font-size-sm);';
    this.container.appendChild(countLabel);

    this.droneCountInput = document.createElement('input');
    this.droneCountInput.type = 'number';
    this.droneCountInput.min = '1';
    this.droneCountInput.max = '20';
    this.droneCountInput.value = '3';
    this.droneCountInput.style.cssText = 'width:42px;background:var(--bg-tertiary);border:1px solid var(--border);color:var(--text-primary);font-family:var(--font-mono);font-size:var(--font-size-sm);padding:2px 4px;border-radius:var(--radius);';
    this.container.appendChild(this.droneCountInput);

    // Pattern selector
    this.patternSelect = document.createElement('select');
    this.patternSelect.style.cssText = 'background:var(--bg-tertiary);border:1px solid var(--border);color:var(--text-primary);font-family:var(--font-ui);font-size:var(--font-size-sm);padding:2px 4px;border-radius:var(--radius);';
    for (const p of ['line', 'grid', 'circle']) {
      const o = document.createElement('option');
      o.value = p; o.textContent = p;
      this.patternSelect.appendChild(o);
    }
    this.container.appendChild(this.patternSelect);

    // Spacing
    const spacingLabel = document.createElement('span');
    spacingLabel.textContent = 'Sp:';
    spacingLabel.style.cssText = 'color:var(--text-secondary);font-size:var(--font-size-sm);';
    this.container.appendChild(spacingLabel);

    this.spacingInput = document.createElement('input');
    this.spacingInput.type = 'number';
    this.spacingInput.min = '0.5';
    this.spacingInput.max = '10';
    this.spacingInput.step = '0.5';
    this.spacingInput.value = '2';
    this.spacingInput.style.cssText = 'width:42px;background:var(--bg-tertiary);border:1px solid var(--border);color:var(--text-primary);font-family:var(--font-mono);font-size:var(--font-size-sm);padding:2px 4px;border-radius:var(--radius);';
    this.container.appendChild(this.spacingInput);
  }

  private makeBtn(label: string, onclick: () => void): HTMLButtonElement {
    const btn = document.createElement('button');
    btn.className = 'btn';
    btn.textContent = label;
    btn.addEventListener('click', onclick);
    this.container.appendChild(btn);
    return btn;
  }

  updateTime(simTime: number): void {
    const el = document.getElementById('sim-time-display');
    if (el) el.textContent = `t = ${simTime.toFixed(3)}s`;
  }

  /** Get current swarm settings from toolbar controls. */
  getSwarmSettings(): { droneCount: number; initialPattern: 'line' | 'grid' | 'circle'; patternSpacing: number } {
    return {
      droneCount: Math.max(1, Math.min(20, parseInt(this.droneCountInput.value) || 3)),
      initialPattern: this.patternSelect.value as 'line' | 'grid' | 'circle',
      patternSpacing: Math.max(0.5, parseFloat(this.spacingInput.value) || 2),
    };
  }
}
