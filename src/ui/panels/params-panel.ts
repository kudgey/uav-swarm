/**
 * Drone parameters editor panel.
 * Groups into hot-updatable and reset-required sections.
 */

import type { WorkerCommand } from '@worker/worker-protocol';
import { defaultDroneParams } from '@sim/core/config-defaults';

interface ParamDef {
  key: string;
  label: string;
  min: number;
  max: number;
  step: number;
  unit: string;
  hot: boolean;
}

const PARAM_DEFS: ParamDef[] = [
  { key: 'mass', label: 'Mass', min: 0.1, max: 10, step: 0.1, unit: 'kg', hot: true },
  { key: 'kT', label: 'kT', min: 1e-7, max: 1e-4, step: 1e-7, unit: 'N/(rad/s)\u00B2', hot: true },
  { key: 'kQ', label: 'kQ', min: 1e-8, max: 1e-5, step: 1e-8, unit: 'Nm/(rad/s)\u00B2', hot: true },
  { key: 'motorTau', label: 'Motor \u03C4', min: 0.001, max: 0.1, step: 0.001, unit: 's', hot: true },
  { key: 'motorOmegaMax', label: '\u03A9 max', min: 100, max: 2000, step: 10, unit: 'rad/s', hot: true },
  { key: 'armLength', label: 'Arm length', min: 0.05, max: 0.5, step: 0.01, unit: 'm', hot: false },
  { key: 'rotorInertia', label: 'Rotor J', min: 0, max: 0.01, step: 0.0001, unit: 'kg·m²', hot: true },
  { key: 'dragCoeffRotorX', label: 'Rotor drag X', min: 0, max: 1, step: 0.01, unit: 'N/(m/s)', hot: true },
  { key: 'dragCoeffRotorY', label: 'Rotor drag Y', min: 0, max: 1, step: 0.01, unit: 'N/(m/s)', hot: true },
  { key: 'dragCoeffRotorZ', label: 'Rotor drag Z', min: 0, max: 1, step: 0.01, unit: 'N/(m/s)', hot: true },
];

export class ParamsPanel {
  private container: HTMLDivElement;
  private sendCommand: (cmd: WorkerCommand) => void;
  private inputs: Map<string, HTMLInputElement> = new Map();
  private pendingResetParams: Record<string, number> = {};

  constructor(parent: HTMLElement, sendCommand: (cmd: WorkerCommand) => void) {
    this.sendCommand = sendCommand;
    this.container = document.createElement('div');
    this.container.className = 'panel';
    parent.appendChild(this.container);

    const title = document.createElement('div');
    title.className = 'panel-title';
    title.textContent = 'Parameters';
    this.container.appendChild(title);

    const defaults = defaultDroneParams();

    // Hot params section
    const hotLabel = document.createElement('div');
    hotLabel.textContent = 'Hot-updatable';
    hotLabel.style.cssText = 'color:var(--text-muted);font-size:10px;margin:6px 0 2px;text-transform:uppercase;';
    this.container.appendChild(hotLabel);

    for (const def of PARAM_DEFS.filter(d => d.hot)) {
      this.addParam(def, (defaults as unknown as Record<string, number>)[def.key]);
    }

    // Reset-required section
    const resetLabel = document.createElement('div');
    resetLabel.textContent = 'Reset required';
    resetLabel.style.cssText = 'color:var(--warning);font-size:10px;margin:8px 0 2px;text-transform:uppercase;';
    this.container.appendChild(resetLabel);

    for (const def of PARAM_DEFS.filter(d => !d.hot)) {
      this.addParam(def, (defaults as unknown as Record<string, number>)[def.key]);
    }
  }

  private addParam(def: ParamDef, defaultVal: number): void {
    const row = document.createElement('div');
    row.className = 'param-row';

    const label = document.createElement('span');
    label.className = 'param-label';
    label.textContent = def.label;
    row.appendChild(label);

    const input = document.createElement('input');
    input.className = 'param-input';
    input.type = 'number';
    input.min = String(def.min);
    input.max = String(def.max);
    input.step = String(def.step);
    input.value = String(defaultVal);
    row.appendChild(input);

    const unit = document.createElement('span');
    unit.style.cssText = 'color:var(--text-muted);font-size:10px;min-width:50px;';
    unit.textContent = def.unit;
    row.appendChild(unit);

    input.addEventListener('change', () => {
      const val = parseFloat(input.value);
      if (!isNaN(val) && val >= def.min && val <= def.max) {
        if (def.hot) {
          this.sendCommand({ type: 'update-params', params: { [def.key]: val } });
        } else {
          // Store for application on next reset
          this.pendingResetParams[def.key] = val;
          input.style.borderColor = 'var(--warning)';
        }
      }
    });

    this.inputs.set(def.key, input);
    this.container.appendChild(row);
  }

  /** Get pending reset-required param changes and clear them. */
  consumePendingResetParams(): Record<string, number> {
    const pending = { ...this.pendingResetParams };
    this.pendingResetParams = {};
    // Clear warning borders on reset-required inputs
    for (const def of PARAM_DEFS.filter(d => !d.hot)) {
      const input = this.inputs.get(def.key);
      if (input) input.style.borderColor = '';
    }
    return pending;
  }

  /** Check if there are pending reset-required changes. */
  hasPendingResetParams(): boolean {
    return Object.keys(this.pendingResetParams).length > 0;
  }
}
