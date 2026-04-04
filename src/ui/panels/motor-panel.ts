/**
 * Manual motor speed sliders + hover trim button.
 * STATUS: experimental, demoOnly
 */

import type { WorkerCommand } from '@worker/worker-protocol';
import { computeHoverTrim, type TrimResult } from '@sim/demo/open-loop-trim';
import { defaultDroneParams } from '@sim/core/config-defaults';
import type { DroneParams } from '@sim/core/types';

export class MotorPanel {
  private container: HTMLDivElement;
  private sendCommand: (cmd: WorkerCommand) => void;
  private sliders: HTMLInputElement[] = [];
  private valueDisplays: HTMLSpanElement[] = [];
  private trimInfo: HTMLDivElement;
  private params: DroneParams;
  private selectedDroneId = 0;

  constructor(parent: HTMLElement, sendCommand: (cmd: WorkerCommand) => void) {
    this.sendCommand = sendCommand;
    this.params = defaultDroneParams();
    this.container = document.createElement('div');
    this.container.className = 'panel';
    parent.appendChild(this.container);

    const title = document.createElement('div');
    title.className = 'panel-title';
    title.textContent = 'Motor Commands (Demo)';
    this.container.appendChild(title);

    // 4 motor sliders
    for (let i = 0; i < 4; i++) {
      const row = document.createElement('div');
      row.className = 'slider-container';
      row.style.marginBottom = '4px';

      const label = document.createElement('span');
      label.className = 'param-label';
      label.textContent = `M${i + 1}`;
      row.appendChild(label);

      const slider = document.createElement('input');
      slider.type = 'range';
      slider.min = '0';
      slider.max = String(this.params.motorOmegaMax);
      slider.step = '1';
      slider.value = '0';
      row.appendChild(slider);

      const val = document.createElement('span');
      val.className = 'param-value';
      val.style.minWidth = '45px';
      val.textContent = '0';
      row.appendChild(val);

      slider.addEventListener('input', () => {
        val.textContent = slider.value;
        this.sendMotorCommands();
      });

      this.sliders.push(slider);
      this.valueDisplays.push(val);
      this.container.appendChild(row);
    }

    // Hover trim button
    const btnRow = document.createElement('div');
    btnRow.style.cssText = 'margin-top:8px;display:flex;gap:6px;align-items:center;';

    const trimBtn = document.createElement('button');
    trimBtn.className = 'btn';
    trimBtn.textContent = 'Hover Trim';
    trimBtn.addEventListener('click', () => this.applyTrim());
    btnRow.appendChild(trimBtn);

    const zeroBtn = document.createElement('button');
    zeroBtn.className = 'btn';
    zeroBtn.textContent = 'Zero';
    zeroBtn.addEventListener('click', () => {
      for (let i = 0; i < 4; i++) {
        this.sliders[i].value = '0';
        this.valueDisplays[i].textContent = '0';
      }
      this.sendMotorCommands();
    });
    btnRow.appendChild(zeroBtn);

    this.container.appendChild(btnRow);

    // Trim info display
    this.trimInfo = document.createElement('div');
    this.trimInfo.style.cssText = 'margin-top:6px;font-size:var(--font-size-sm);';
    this.container.appendChild(this.trimInfo);
  }

  private sendMotorCommands(): void {
    const commands = this.sliders.map(s => parseFloat(s.value));
    this.sendCommand({ type: 'set-motor-commands', droneId: this.selectedDroneId, commands });
  }

  private applyTrim(): void {
    const result: TrimResult = computeHoverTrim(this.params);

    if (result.feasible) {
      const omega = Math.round(result.omegaHover);
      for (let i = 0; i < 4; i++) {
        this.sliders[i].value = String(omega);
        this.valueDisplays[i].textContent = String(omega);
      }
      this.sendMotorCommands();
      this.trimInfo.textContent = `Trim: ${omega} rad/s (${result.utilizationPercent.toFixed(1)}% of max)`;
      this.trimInfo.style.color = 'var(--text-secondary)';
    } else {
      this.trimInfo.textContent = `TRIM INFEASIBLE: need ${Math.round(result.omegaHover)} rad/s, max ${result.omegaMax} (deficit ${result.deficitPercent!.toFixed(1)}%)`;
      this.trimInfo.style.color = 'var(--error)';
      // Set to max as fallback but warn
      for (let i = 0; i < 4; i++) {
        this.sliders[i].value = String(result.omegaMax);
        this.valueDisplays[i].textContent = String(result.omegaMax);
      }
      this.sendMotorCommands();
    }
  }

  /** Update selected drone for motor commands. */
  setSelectedDroneId(id: number): void {
    this.selectedDroneId = id;
  }

  /** Get current params snapshot (for syncing). */
  getParams(): DroneParams {
    return { ...this.params };
  }

  updateParams(params: DroneParams): void {
    this.params = params;
    for (const slider of this.sliders) {
      slider.max = String(params.motorOmegaMax);
    }
  }
}
