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
        grid-template-rows: 1fr auto;
        grid-template-columns: 300px 1fr;
        width: 100%; height: 100%;
      }
      .app-sidebar {
        grid-row: 1;
        overflow-y: auto;
        border-right: 1px solid var(--border);
        background: var(--bg-primary);
      }
      .app-viewport { grid-row: 1; min-height: 0; }
      .app-charts {
        grid-column: 1 / -1;
        border-top: 1px solid var(--border);
        padding: 8px;
        display: flex;
        gap: 8px;
        overflow-x: auto;
        background: var(--bg-secondary);
        min-height: 150px;
        max-height: clamp(150px, 22vh, 220px);
        align-items: stretch;
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
          grid-template-rows: auto 1fr auto;
        }
        .app-sidebar {
          grid-row: 1;
          max-height: 40vh;
          border-right: none;
          border-bottom: 1px solid var(--border);
        }
        .app-viewport { grid-row: 2; }
        .app-charts { grid-row: 3; min-height: 100px; flex-wrap: wrap; max-height: none; }
      }
    `;
    document.head.appendChild(style);

    const layout = document.createElement('div');
    layout.className = 'app-layout';
    this.root.appendChild(layout);

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

    // Charts
    this.posChart.pushData(swarm.simTime, drone.position);
    this.velChart.pushData(swarm.simTime, [...drone.velocity]);
    this.motorChart.pushData(swarm.simTime, [...drone.motorSpeeds]);
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
