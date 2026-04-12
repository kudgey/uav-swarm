/**
 * Main application shell.
 * Mission-first layout: MissionPanel (sidebar) + 3D Viewport + Charts.
 * Advanced panels available under collapsible "Advanced" section.
 */

import { injectTheme, makeCollapsible } from './theme';
import { ViewportPanel } from './panels/viewport-panel';
import { MissionPanel } from './panels/mission-panel';
import { StatePanel } from './panels/state-panel';
import { ParamsPanel } from './panels/params-panel';
import { MotorPanel } from './panels/motor-panel';
import { StatusPanel } from './panels/status-panel';
import { EnvironmentPanel } from './panels/environment-panel';
import { SensorPanel } from './panels/sensor-panel';
import { ControllerPanel } from './panels/controller-panel';
import { FormationPanel } from './panels/formation-panel';
import { SafetyPanel } from './panels/safety-panel';
import { ScenarioPanel } from './panels/scenario-panel';
import { TelemetryChart } from './charts/telemetry-chart';
import type { WorkerCommand, WorkerEvent, SwarmSnapshot } from '@worker/worker-protocol';

// Import stubs to trigger their registration
import '@sim/environment/environment';
import '@sim/sensors/sensors';
import '@sim/estimation/estimation';
import '@sim/swarm/swarm';
import '@sim/safety/safety';
import '@sim/scenarios/scenarios';

export class App {
  private worker!: Worker;
  private viewport!: ViewportPanel;
  private missionPanel!: MissionPanel;
  private statePanel!: StatePanel;
  private paramsPanel!: ParamsPanel;
  private motorPanel!: MotorPanel;
  private statusPanel!: StatusPanel;
  private envPanel!: EnvironmentPanel;
  private sensorPanel!: SensorPanel;
  private controllerPanel!: ControllerPanel;
  private formationPanel!: FormationPanel;
  private safetyPanel!: SafetyPanel;
  private scenarioPanel!: ScenarioPanel;
  private posChart!: TelemetryChart;
  private velChart!: TelemetryChart;
  private motorChart!: TelemetryChart;
  private lastFlightLog = '';
  private statusBar!: HTMLDivElement;
  private chartToolbar!: HTMLDivElement;
  private droneSelect!: HTMLSelectElement;
  private chartPaused = false;
  private chartTimeWindow = 30; // seconds
  private currentChartDroneId = 0;
  private lastKnownDroneIds: number[] = [];

  constructor(private root: HTMLElement) {
    injectTheme();
    this.buildLayout();
    this.startWorker();
    this.startChartLoop();
  }

  private buildLayout(): void {
    const style = document.createElement('style');
    style.textContent = `
      .app-layout {
        display: grid;
        grid-template-rows: 32px 1fr 28px 200px;
        grid-template-columns: 300px 1fr;
        width: 100%; height: 100%;
      }
      .app-chart-toolbar {
        grid-column: 1 / -1;
        grid-row: 3;
        display: flex;
        align-items: center;
        gap: 8px;
        padding: 0 10px;
        background: var(--bg-secondary);
        border-top: 1px solid var(--border);
        font-family: var(--font-ui);
        font-size: var(--font-size-sm);
        color: var(--text-secondary);
      }
      .app-chart-toolbar select, .app-chart-toolbar button {
        font-family: var(--font-ui);
        font-size: var(--font-size-sm);
        padding: 2px 8px;
        border: 1px solid var(--border);
        background: var(--bg-tertiary);
        color: var(--text-primary);
        border-radius: var(--radius);
        cursor: pointer;
      }
      .app-chart-toolbar button:hover { background: var(--bg-hover); }
      .app-chart-toolbar button.active { background: var(--accent); color: #fff; border-color: var(--accent); }
      .app-status-bar {
        grid-column: 1 / -1;
        grid-row: 1;
        display: flex;
        align-items: center;
        gap: 16px;
        padding: 0 12px;
        background: var(--bg-secondary);
        border-bottom: 1px solid var(--border);
        font-family: var(--font-ui);
        font-size: var(--font-size-sm);
        color: var(--text-secondary);
        overflow: hidden;
        white-space: nowrap;
      }
      .app-status-bar .status-value { color: var(--text-primary); font-weight: 600; font-family: var(--font-mono); }
      .app-status-bar .status-sep { width: 1px; height: 16px; background: var(--border); flex-shrink: 0; }
      .app-status-bar .status-progress { height: 4px; background: var(--accent); border-radius: 2px; transition: width 0.3s; }
      .app-sidebar {
        grid-row: 2;
        overflow-y: auto;
        border-right: 1px solid var(--border);
        background: var(--bg-primary);
      }
      .app-viewport { grid-row: 2; min-height: 0; }
      .app-charts {
        grid-column: 1 / -1;
        grid-row: 4;
        padding: 6px 8px;
        display: flex;
        gap: 8px;
        overflow: hidden;
        background: var(--bg-secondary);
        align-items: stretch;
        min-height: 0;
      }
      .advanced-toggle {
        padding: 8px 12px; font-size: 11px; color: var(--text-muted);
        cursor: pointer; border-top: 1px solid var(--border);
        text-transform: uppercase; letter-spacing: 0.5px;
        user-select: none;
      }
      .advanced-toggle:hover { color: var(--text-primary); }
      .advanced-body { display: none; padding: 6px; }
      .advanced-body.open { display: flex; flex-direction: column; gap: 6px; }
      @media (max-width: 700px) {
        .app-layout {
          grid-template-columns: 1fr;
          grid-template-rows: 32px auto 1fr 28px auto;
        }
        .app-sidebar {
          grid-row: 2;
          max-height: 40vh;
          border-right: none;
          border-bottom: 1px solid var(--border);
        }
        .app-viewport { grid-row: 3; }
        .app-chart-toolbar { grid-row: 4; }
        .app-charts { grid-row: 5; min-height: 100px; flex-wrap: wrap; max-height: none; }
      }
    `;
    document.head.appendChild(style);

    const layout = document.createElement('div');
    layout.className = 'app-layout';
    this.root.appendChild(layout);

    // Status bar
    this.statusBar = document.createElement('div');
    this.statusBar.className = 'app-status-bar';
    const sbItems = [
      ['Drones', 'sb-drones', '0'],
      ['Selected', 'sb-selected', 'D0'],
      ['t', 'sb-time', '0.0s'],
      ['Min sep', 'sb-minsep', '--'],
      ['Collisions', 'sb-collisions', '0'],
    ];
    for (let i = 0; i < sbItems.length; i++) {
      if (i > 0) { const sep = document.createElement('span'); sep.className = 'status-sep'; this.statusBar.appendChild(sep); }
      const [label, id, def] = sbItems[i];
      const span = document.createElement('span');
      span.textContent = label + ': ';
      const val = document.createElement('span');
      val.className = 'status-value';
      val.id = id;
      val.textContent = def;
      span.appendChild(val);
      this.statusBar.appendChild(span);
    }
    // Mission progress bar
    const sep = document.createElement('span'); sep.className = 'status-sep'; this.statusBar.appendChild(sep);
    const progWrap = document.createElement('span');
    progWrap.style.cssText = 'display:flex;align-items:center;gap:6px;';
    progWrap.textContent = 'Mission: ';
    const progTrack = document.createElement('span');
    progTrack.style.cssText = 'width:80px;height:4px;background:var(--bg-tertiary);border-radius:2px;overflow:hidden;';
    const progFill = document.createElement('span');
    progFill.className = 'status-progress';
    progFill.id = 'sb-progress';
    progFill.style.cssText = 'display:block;height:100%;width:0%;background:var(--accent);border-radius:2px;transition:width 0.3s;';
    progTrack.appendChild(progFill);
    progWrap.appendChild(progTrack);
    this.statusBar.appendChild(progWrap);
    layout.appendChild(this.statusBar);

    // Sidebar: Mission Panel + Advanced
    const sidebar = document.createElement('div');
    sidebar.className = 'app-sidebar';
    layout.appendChild(sidebar);

    // Mission panel — primary UI
    this.missionPanel = new MissionPanel(
      sidebar,
      cmd => this.sendCommand(cmd),
      cfg => {
        // onInit callback: send init to worker, then step once so viewport shows drones
        this.viewport.clearTrails();
        this.worker.postMessage({ type: 'init', config: cfg } satisfies WorkerCommand);
        this.worker.postMessage({ type: 'step', count: 1 } satisfies WorkerCommand);
      },
    );

    // Advanced section — collapsible
    const advToggle = document.createElement('div');
    advToggle.className = 'advanced-toggle';
    advToggle.textContent = 'Advanced >';
    sidebar.appendChild(advToggle);

    const advBody = document.createElement('div');
    advBody.className = 'advanced-body';
    sidebar.appendChild(advBody);

    advToggle.addEventListener('click', () => {
      const open = advBody.classList.toggle('open');
      advToggle.textContent = open ? 'Advanced v' : 'Advanced >';
    });

    this.statePanel = new StatePanel(advBody);
    this.controllerPanel = new ControllerPanel(advBody, cmd => this.sendCommand(cmd));
    this.formationPanel = new FormationPanel(advBody, cmd => this.sendCommand(cmd));
    this.safetyPanel = new SafetyPanel(advBody, cmd => this.sendCommand(cmd));
    this.scenarioPanel = new ScenarioPanel(advBody, cmd => this.sendCommand(cmd));
    this.sensorPanel = new SensorPanel(advBody, cmd => this.sendCommand(cmd));
    this.envPanel = new EnvironmentPanel(advBody, cmd => this.sendCommand(cmd));
    this.motorPanel = new MotorPanel(advBody, cmd => this.sendCommand(cmd));
    this.paramsPanel = new ParamsPanel(advBody, cmd => this.sendCommand(cmd));
    this.statusPanel = new StatusPanel(advBody);

    // Make advanced panels collapsible
    const panels = advBody.querySelectorAll<HTMLDivElement>('.panel');
    panels.forEach(p => makeCollapsible(p, false));

    // Viewport
    const viewportContainer = document.createElement('div');
    viewportContainer.className = 'app-viewport';
    layout.appendChild(viewportContainer);
    this.viewport = new ViewportPanel(viewportContainer);
    this.viewport.onDroneSelected = (id) => {
      this.sendCommand({ type: 'select-drone', droneId: id });
      this.currentChartDroneId = id;
      this.droneSelect.value = String(id);
      this.clearCharts();
    };

    // Chart toolbar
    this.chartToolbar = document.createElement('div');
    this.chartToolbar.className = 'app-chart-toolbar';
    layout.appendChild(this.chartToolbar);

    // Drone selector
    const droneLabel = document.createElement('span');
    droneLabel.textContent = 'Drone:';
    this.chartToolbar.appendChild(droneLabel);
    this.droneSelect = document.createElement('select');
    const defaultOpt = document.createElement('option');
    defaultOpt.value = '0'; defaultOpt.textContent = 'D0';
    this.droneSelect.appendChild(defaultOpt);
    this.droneSelect.addEventListener('change', () => {
      const id = parseInt(this.droneSelect.value);
      this.currentChartDroneId = id;
      this.sendCommand({ type: 'select-drone', droneId: id });
      this.clearCharts();
    });
    this.chartToolbar.appendChild(this.droneSelect);

    // Time window
    const windowLabel = document.createElement('span');
    windowLabel.textContent = 'Window:';
    windowLabel.style.marginLeft = '8px';
    this.chartToolbar.appendChild(windowLabel);
    for (const sec of [10, 30, 60]) {
      const btn = document.createElement('button');
      btn.textContent = `${sec}s`;
      if (sec === this.chartTimeWindow) btn.classList.add('active');
      btn.addEventListener('click', () => {
        this.chartTimeWindow = sec;
        this.chartToolbar.querySelectorAll('button.window-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        this.posChart.setTimeWindow(sec);
        this.velChart.setTimeWindow(sec);
        this.motorChart.setTimeWindow(sec);
      });
      btn.classList.add('window-btn');
      this.chartToolbar.appendChild(btn);
    }

    // Pause / Clear
    const spacer = document.createElement('span'); spacer.style.flex = '1'; this.chartToolbar.appendChild(spacer);
    const pauseBtn = document.createElement('button');
    pauseBtn.textContent = 'Pause';
    pauseBtn.addEventListener('click', () => {
      this.chartPaused = !this.chartPaused;
      pauseBtn.textContent = this.chartPaused ? 'Resume' : 'Pause';
      pauseBtn.classList.toggle('active', this.chartPaused);
    });
    this.chartToolbar.appendChild(pauseBtn);

    const clearBtn = document.createElement('button');
    clearBtn.textContent = 'Clear';
    clearBtn.addEventListener('click', () => this.clearCharts());
    this.chartToolbar.appendChild(clearBtn);

    // Charts
    const chartsRow = document.createElement('div');
    chartsRow.className = 'app-charts';
    layout.appendChild(chartsRow);

    this.posChart = new TelemetryChart(chartsRow, 'Position (m)');
    this.posChart.addSeries('X', '#ff6666');
    this.posChart.addSeries('Y', '#66ff66');
    this.posChart.addSeries('Z', '#6666ff');

    this.velChart = new TelemetryChart(chartsRow, 'Velocity (m/s)');
    this.velChart.addSeries('X', '#ff6666');
    this.velChart.addSeries('Y', '#66ff66');
    this.velChart.addSeries('Z', '#6666ff');

    this.motorChart = new TelemetryChart(chartsRow, 'Motors (rad/s)');
    this.motorChart.addSeries('M1', '#44aa66');
    this.motorChart.addSeries('M2', '#aa6644');
    this.motorChart.addSeries('M3', '#44aa66');
    this.motorChart.addSeries('M4', '#aa6644');
  }

  private startWorker(): void {
    this.worker = new Worker(
      new URL('../worker/sim-worker.ts', import.meta.url),
      { type: 'module' },
    );

    this.worker.onmessage = (e: MessageEvent<WorkerEvent>) => {
      const msg = e.data;
      switch (msg.type) {
        case 'state':
          this.onState(msg.snapshot);
          break;
        case 'subsystem-status':
          this.statusPanel.updateWorkerStatus(msg.entries);
          break;
        case 'ready':
          break;
        case 'error':
          console.error('[Worker]', msg.message);
          break;
        case 'scenario-result':
          this.scenarioPanel.showResult(msg.result);
          break;
        case 'batch-complete':
          this.scenarioPanel.showBatchResults(msg.results);
          break;
        case 'mission-complete':
          this.missionPanel.showResults(msg.metrics);
          break;
        case 'mission-log':
          this.lastFlightLog = msg.csv;
          this.missionPanel.enableLogDownload(() => this.downloadLog());
          break;
      }
    };

    // Init with default config from mission panel
    const cfg = this.missionPanel.buildConfig();
    this.worker.postMessage({ type: 'init', config: cfg } satisfies WorkerCommand);
  }

  private onState(swarm: SwarmSnapshot): void {
    const drone = swarm.drones.find(d => d.id === swarm.selectedDroneId) ?? swarm.drones[0];
    if (!drone) return;

    this.motorPanel.setSelectedDroneId(drone.id);

    const view = { ...drone, simTime: swarm.simTime, stepCount: swarm.stepCount,
      armState: drone.armState, controlMode: drone.controlMode };

    this.viewport.updateSwarm(swarm);
    this.missionPanel.update(swarm);
    this.statePanel.update(view);
    this.sensorPanel.update(view);
    this.envPanel.update(view);
    this.controllerPanel.updateSwarm(swarm.drones);
    this.controllerPanel.update({ ...view, droneCount: swarm.drones.length,
      commLinkCount: swarm.commLinks.length });
    this.formationPanel.update(swarm);
    this.safetyPanel.update(swarm);

    // Status bar
    const sbDrones = document.getElementById('sb-drones');
    const sbSelected = document.getElementById('sb-selected');
    const sbTime = document.getElementById('sb-time');
    const sbMinSep = document.getElementById('sb-minsep');
    const sbCollisions = document.getElementById('sb-collisions');
    if (sbDrones) sbDrones.textContent = String(swarm.drones.length);
    if (sbSelected) sbSelected.textContent = `D${drone.id}`;
    if (sbTime) sbTime.textContent = `${swarm.simTime.toFixed(1)}s`;
    if (sbMinSep) sbMinSep.textContent = swarm.safetyMetrics.minSeparation === Infinity ? '--' : `${swarm.safetyMetrics.minSeparation.toFixed(2)}m`;
    if (sbCollisions) sbCollisions.textContent = String(swarm.safetyMetrics.collisionCount);

    // Mission progress bar
    const sbProgress = document.getElementById('sb-progress');
    if (sbProgress) sbProgress.style.width = `${this.missionPanel.getProgressPercent()}%`;

    // Sync drone dropdown options if drone count changed
    this.syncDroneDropdown(swarm.drones.map(d => d.id));

    // Which drone to chart: toolbar selection overrides click
    const chartDroneId = this.currentChartDroneId;
    const chartDrone = swarm.drones.find(d => d.id === chartDroneId) ?? drone;

    // Charts (skip if paused)
    if (!this.chartPaused) {
      this.posChart.pushData(swarm.simTime, chartDrone.position);
      this.velChart.pushData(swarm.simTime, [...chartDrone.velocity]);
      this.motorChart.pushData(swarm.simTime, [...chartDrone.motorSpeeds]);
    }
  }

  private syncDroneDropdown(ids: number[]): void {
    // Only rebuild if drone set changed
    if (ids.length === this.lastKnownDroneIds.length &&
        ids.every((id, i) => id === this.lastKnownDroneIds[i])) return;
    this.lastKnownDroneIds = [...ids];
    const currentVal = this.droneSelect.value;
    this.droneSelect.textContent = '';
    for (const id of ids) {
      const opt = document.createElement('option');
      opt.value = String(id);
      opt.textContent = `D${id}`;
      this.droneSelect.appendChild(opt);
    }
    // Keep selection if still valid
    if (ids.includes(parseInt(currentVal))) {
      this.droneSelect.value = currentVal;
    } else {
      this.droneSelect.value = String(ids[0] ?? 0);
      this.currentChartDroneId = ids[0] ?? 0;
    }
  }

  private clearCharts(): void {
    this.posChart.clear();
    this.velChart.clear();
    this.motorChart.clear();
  }

  private startChartLoop(): void {
    let lastRender = 0;
    const loop = (now: number) => {
      requestAnimationFrame(loop);
      if (now - lastRender < 33) return;
      lastRender = now;
      this.posChart.render();
      this.velChart.render();
      this.motorChart.render();
    };
    requestAnimationFrame(loop);
  }

  private sendCommand(cmd: WorkerCommand): void {
    this.worker.postMessage(cmd);
  }

  private downloadLog(): void {
    if (!this.lastFlightLog) return;
    const blob = new Blob([this.lastFlightLog], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `flight-log-${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.csv`;
    a.click();
    URL.revokeObjectURL(url);
  }
}
