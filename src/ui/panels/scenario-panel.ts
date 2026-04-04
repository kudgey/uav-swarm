/**
 * Scenario runner panel: select preset, run, view results.
 */

import type { WorkerCommand } from '@worker/worker-protocol';
import { ALL_PRESETS } from '@sim/scenarios/scenario-presets';

export class ScenarioPanel {
  private container: HTMLDivElement;
  private sendCommand: (cmd: WorkerCommand) => void;
  private resultDiv: HTMLDivElement;
  private presetSelect: HTMLSelectElement;

  constructor(parent: HTMLElement, sendCommand: (cmd: WorkerCommand) => void) {
    this.sendCommand = sendCommand;
    this.container = document.createElement('div');
    this.container.className = 'panel';
    parent.appendChild(this.container);

    const title = document.createElement('div');
    title.className = 'panel-title';
    title.textContent = 'Scenarios';
    this.container.appendChild(title);

    // Preset selector
    const row = document.createElement('div');
    row.className = 'param-row';
    const lbl = document.createElement('span');
    lbl.className = 'param-label'; lbl.textContent = 'Preset';
    this.presetSelect = document.createElement('select');
    this.presetSelect.style.cssText = 'font-size:10px;max-width:160px;';
    for (const p of ALL_PRESETS) {
      const opt = document.createElement('option');
      opt.value = p.name; opt.textContent = p.name;
      this.presetSelect.appendChild(opt);
    }
    row.appendChild(lbl); row.appendChild(this.presetSelect);
    this.container.appendChild(row);

    // Buttons
    const btnRow = document.createElement('div');
    btnRow.style.cssText = 'display:flex;gap:4px;margin:6px 0;';
    const runBtn = document.createElement('button');
    runBtn.className = 'btn'; runBtn.textContent = 'Run';
    runBtn.addEventListener('click', () => {
      const preset = ALL_PRESETS.find(p => p.name === this.presetSelect.value);
      if (preset) this.sendCommand({ type: 'run-scenario', scenario: preset } as WorkerCommand);
    });
    btnRow.appendChild(runBtn);
    const batchBtn = document.createElement('button');
    batchBtn.className = 'btn'; batchBtn.textContent = 'Batch';
    batchBtn.addEventListener('click', () => {
      const preset = ALL_PRESETS.find(p => p.name === this.presetSelect.value);
      if (preset) this.sendCommand({ type: 'run-batch', scenario: preset, seeds: preset.monteCarloRanges?.seeds ?? [42, 43, 44] } as WorkerCommand);
    });
    btnRow.appendChild(batchBtn);
    this.container.appendChild(btnRow);

    this.resultDiv = document.createElement('div');
    this.resultDiv.style.cssText = 'font-size:var(--font-size-sm);white-space:pre;max-height:120px;overflow-y:auto;';
    this.container.appendChild(this.resultDiv);
  }

  showResult(result: unknown): void {
    const r = result as { seed: number; scenarioName: string; passed: boolean; metrics: Record<string, number>; failedCriteria: string[] };
    const lines = [
      `${r.scenarioName} seed=${r.seed}: ${r.passed ? 'PASS' : 'FAIL'}`,
    ];
    if (r.failedCriteria.length > 0) lines.push(`Failed: ${r.failedCriteria.join(', ')}`);
    const m = r.metrics;
    lines.push(`minSep=${m.minSeparation === Infinity ? '--' : m.minSeparation?.toFixed(2)} collision=${m.collisionCount}`);
    lines.push(`trackErr=${m.rmsExecutedTrackingError?.toFixed(3)} estErr=${m.rmsEstimationError?.toFixed(3)}`);
    lines.push(`altErr=${m.rmsAltitudeError?.toFixed(3)} drift=${m.maxHorizontalDrift?.toFixed(2)}`);
    lines.push(`rt=${m.realTimeFactor?.toFixed(1)}x`);
    this.resultDiv.textContent = lines.join('\n');
  }

  showBatchResults(results: unknown[]): void {
    const rs = results as { seed: number; passed: boolean; metrics: Record<string, number> }[];
    const passCount = rs.filter(r => r.passed).length;
    const lines = [`Batch: ${passCount}/${rs.length} passed`];
    for (const r of rs.slice(0, 5)) {
      lines.push(`  seed=${r.seed}: ${r.passed ? 'PASS' : 'FAIL'} rt=${r.metrics.realTimeFactor?.toFixed(1)}x`);
    }
    if (rs.length > 5) lines.push(`  ... and ${rs.length - 5} more`);
    this.resultDiv.textContent = lines.join('\n');
  }
}
