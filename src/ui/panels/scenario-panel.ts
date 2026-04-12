/**
 * Scenario runner panel: select preset, run single/batch, view results as table with stats.
 */

import type { WorkerCommand } from '@worker/worker-protocol';
import { ALL_PRESETS } from '@sim/scenarios/scenario-presets';

interface SingleResult {
  seed: number;
  scenarioName: string;
  passed: boolean;
  metrics: Record<string, number>;
  failedCriteria: string[];
}

const METRIC_COLS: { key: string; label: string; fmt: (v: number) => string }[] = [
  { key: 'minSeparation', label: 'MinSep', fmt: v => (v === Infinity ? '--' : v.toFixed(2)) },
  { key: 'collisionCount', label: 'Coll', fmt: v => String(v) },
  { key: 'rmsExecutedTrackingError', label: 'TrkRMS', fmt: v => v.toFixed(2) },
  { key: 'rmsEstimationError', label: 'EstRMS', fmt: v => v.toFixed(2) },
  { key: 'maxHorizontalDrift', label: 'Drift', fmt: v => v.toFixed(2) },
  { key: 'formationRMS', label: 'FmRMS', fmt: v => (isNaN(v) ? '--' : v.toFixed(2)) },
  { key: 'realTimeFactor', label: 'RT', fmt: v => v.toFixed(1) + 'x' },
];

export class ScenarioPanel {
  private container: HTMLDivElement;
  private sendCommand: (cmd: WorkerCommand) => void;
  private resultContainer: HTMLDivElement;
  private presetSelect: HTMLSelectElement;
  private lastResults: SingleResult[] = [];

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
    this.presetSelect.style.cssText = 'font-family:var(--font-ui);font-size:var(--font-size-sm);max-width:180px;';
    for (const p of ALL_PRESETS) {
      const opt = document.createElement('option');
      opt.value = p.name; opt.textContent = p.name;
      this.presetSelect.appendChild(opt);
    }
    row.appendChild(lbl); row.appendChild(this.presetSelect);
    this.container.appendChild(row);

    // Seeds input
    const seedsRow = document.createElement('div');
    seedsRow.className = 'param-row';
    const seedsLbl = document.createElement('span');
    seedsLbl.className = 'param-label'; seedsLbl.textContent = 'Seeds';
    const seedsInput = document.createElement('input');
    seedsInput.className = 'param-input';
    seedsInput.value = '42,43,44,45,46';
    seedsInput.style.width = '120px';
    seedsInput.title = 'Comma-separated seeds for batch run';
    seedsRow.appendChild(seedsLbl); seedsRow.appendChild(seedsInput);
    this.container.appendChild(seedsRow);

    // Buttons
    const btnRow = document.createElement('div');
    btnRow.style.cssText = 'display:flex;gap:4px;margin:6px 0;';
    const runBtn = document.createElement('button');
    runBtn.className = 'btn'; runBtn.textContent = 'Run';
    runBtn.addEventListener('click', () => {
      const preset = ALL_PRESETS.find(p => p.name === this.presetSelect.value);
      if (preset) {
        this.showStatus('Running…');
        this.sendCommand({ type: 'run-scenario', scenario: preset } as WorkerCommand);
      }
    });
    btnRow.appendChild(runBtn);

    const batchBtn = document.createElement('button');
    batchBtn.className = 'btn'; batchBtn.textContent = 'Batch';
    batchBtn.addEventListener('click', () => {
      const preset = ALL_PRESETS.find(p => p.name === this.presetSelect.value);
      if (!preset) return;
      const seeds = seedsInput.value.split(',').map(s => parseInt(s.trim())).filter(n => Number.isFinite(n));
      this.showStatus(`Running batch (${seeds.length} seeds)…`);
      this.sendCommand({ type: 'run-batch', scenario: preset, seeds } as WorkerCommand);
    });
    btnRow.appendChild(batchBtn);

    const exportBtn = document.createElement('button');
    exportBtn.className = 'btn'; exportBtn.textContent = 'CSV';
    exportBtn.title = 'Export last results as CSV';
    exportBtn.addEventListener('click', () => this.exportCSV());
    btnRow.appendChild(exportBtn);

    this.container.appendChild(btnRow);

    this.resultContainer = document.createElement('div');
    this.resultContainer.style.cssText = 'font-family:var(--font-mono);font-size:11px;max-height:300px;overflow:auto;background:var(--bg-tertiary);border-radius:var(--radius);padding:6px;';
    this.container.appendChild(this.resultContainer);
  }

  private showStatus(msg: string): void {
    this.resultContainer.textContent = '';
    const status = document.createElement('div');
    status.textContent = msg;
    status.style.color = 'var(--text-muted)';
    this.resultContainer.appendChild(status);
  }

  showResult(result: unknown): void {
    const r = result as SingleResult;
    this.lastResults = [r];
    this.renderResults([r], null);
  }

  showBatchResults(results: unknown[]): void {
    const rs = results as SingleResult[];
    this.lastResults = rs;
    const stats = this.computeStats(rs);
    this.renderResults(rs, stats);
  }

  private computeStats(rs: SingleResult[]): Record<string, { mean: number; min: number; max: number; p95: number }> {
    const stats: Record<string, { mean: number; min: number; max: number; p95: number }> = {};
    for (const col of METRIC_COLS) {
      const vals = rs.map(r => r.metrics[col.key]).filter(v => Number.isFinite(v));
      if (vals.length === 0) continue;
      const sorted = [...vals].sort((a, b) => a - b);
      const sum = vals.reduce((s, v) => s + v, 0);
      const p95Idx = Math.min(sorted.length - 1, Math.floor(sorted.length * 0.95));
      stats[col.key] = {
        mean: sum / vals.length,
        min: sorted[0],
        max: sorted[sorted.length - 1],
        p95: sorted[p95Idx],
      };
    }
    return stats;
  }

  private renderResults(rs: SingleResult[], stats: ReturnType<ScenarioPanel['computeStats']> | null): void {
    this.resultContainer.textContent = '';

    // Summary header
    const passCount = rs.filter(r => r.passed).length;
    const summary = document.createElement('div');
    summary.style.cssText = 'font-family:var(--font-ui);font-weight:600;margin-bottom:6px;';
    const passRate = (passCount / rs.length * 100).toFixed(0);
    const passColor = passCount === rs.length ? 'var(--success)' : passCount > 0 ? 'var(--warning)' : 'var(--error)';
    summary.textContent = `${rs[0]?.scenarioName ?? '—'}: `;
    const rate = document.createElement('span');
    rate.textContent = `${passCount}/${rs.length} passed (${passRate}%)`;
    rate.style.color = passColor;
    summary.appendChild(rate);
    this.resultContainer.appendChild(summary);

    // Table
    const table = document.createElement('table');
    table.style.cssText = 'width:100%;border-collapse:collapse;font-family:var(--font-mono);font-size:10px;';

    // Header
    const thead = document.createElement('thead');
    const trh = document.createElement('tr');
    trh.style.cssText = 'background:var(--bg-secondary);border-bottom:1px solid var(--border);';
    const headers = ['Seed', 'Pass', ...METRIC_COLS.map(c => c.label)];
    for (const h of headers) {
      const th = document.createElement('th');
      th.textContent = h;
      th.style.cssText = 'padding:3px 6px;text-align:right;';
      if (h === 'Seed' || h === 'Pass') th.style.textAlign = 'left';
      trh.appendChild(th);
    }
    thead.appendChild(trh);
    table.appendChild(thead);

    // Rows
    const tbody = document.createElement('tbody');
    for (const r of rs) {
      const tr = document.createElement('tr');
      tr.style.borderBottom = '1px solid var(--border)';
      const seedCell = document.createElement('td');
      seedCell.textContent = String(r.seed);
      seedCell.style.cssText = 'padding:3px 6px;';
      tr.appendChild(seedCell);

      const passCell = document.createElement('td');
      passCell.textContent = r.passed ? 'PASS' : 'FAIL';
      passCell.style.cssText = `padding:3px 6px;color:${r.passed ? 'var(--success)' : 'var(--error)'};font-weight:600;`;
      if (!r.passed && r.failedCriteria.length > 0) {
        passCell.title = `Failed: ${r.failedCriteria.join(', ')}`;
      }
      tr.appendChild(passCell);

      for (const col of METRIC_COLS) {
        const td = document.createElement('td');
        const val = r.metrics[col.key];
        td.textContent = val === undefined ? '—' : col.fmt(val);
        td.style.cssText = 'padding:3px 6px;text-align:right;';
        tr.appendChild(td);
      }
      tbody.appendChild(tr);
    }
    table.appendChild(tbody);

    // Stats footer (only for batch)
    if (stats) {
      const tfoot = document.createElement('tfoot');
      const statRows: { label: string; key: 'mean' | 'p95' | 'min' | 'max' }[] = [
        { label: 'mean', key: 'mean' },
        { label: 'p95', key: 'p95' },
        { label: 'min', key: 'min' },
        { label: 'max', key: 'max' },
      ];
      for (const sr of statRows) {
        const tr = document.createElement('tr');
        tr.style.cssText = 'background:var(--bg-secondary);border-top:1px solid var(--border);';
        const lblCell = document.createElement('td');
        lblCell.textContent = sr.label;
        lblCell.style.cssText = 'padding:3px 6px;font-weight:600;color:var(--text-muted);';
        tr.appendChild(lblCell);
        const emptyCell = document.createElement('td');
        emptyCell.style.padding = '3px 6px';
        tr.appendChild(emptyCell);
        for (const col of METRIC_COLS) {
          const td = document.createElement('td');
          const s = stats[col.key];
          td.textContent = s ? col.fmt(s[sr.key]) : '—';
          td.style.cssText = 'padding:3px 6px;text-align:right;color:var(--text-muted);';
          tr.appendChild(td);
        }
        tfoot.appendChild(tr);
      }
      table.appendChild(tfoot);
    }

    this.resultContainer.appendChild(table);
  }

  private exportCSV(): void {
    if (this.lastResults.length === 0) return;
    const headers = ['seed', 'scenarioName', 'passed', 'failedCriteria', ...METRIC_COLS.map(c => c.key)];
    const rows = [headers.join(',')];
    for (const r of this.lastResults) {
      const vals = [
        String(r.seed),
        r.scenarioName,
        String(r.passed),
        `"${r.failedCriteria.join(';')}"`,
        ...METRIC_COLS.map(c => {
          const v = r.metrics[c.key];
          return v === undefined || !Number.isFinite(v) ? '' : String(v);
        }),
      ];
      rows.push(vals.join(','));
    }
    const csv = rows.join('\n');
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `scenario-${this.lastResults[0].scenarioName}-${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.csv`;
    a.click();
    URL.revokeObjectURL(url);
  }
}
