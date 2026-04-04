/**
 * Mission Panel: unified wizard for swarm simulation.
 * Step 1: Configure swarm (count, pattern, spacing)
 * Step 2: Choose mission (flight scenario, parameters)
 * Step 3: Watch + results
 */

import type { WorkerCommand, SwarmSnapshot } from '@worker/worker-protocol';
import type { SimConfig } from '@sim/core/types';
import { defaultSimConfig } from '@sim/core/config-defaults';
import { buildPatrolMissionSteps, buildPatrolConfig } from '@sim/missions/mission-builder';

export type MissionType =
  | 'formation-transit'
  | 'formation-hold-wind'
  | 'formation-reconfig'
  | 'leader-loss'
  | 'multi-loss'
  | 'long-range-patrol'
  | 'consensus-transit'
  | 'consensus-loss'
  | 'consensus-patrol';

interface MissionDef {
  type: MissionType;
  label: string;
  description: string;
  minDrones: number;
}

type SwarmPattern = 'line' | 'grid' | 'circle';
type MissionStep = { time: number; droneId: number | 'all'; action: string; params?: Record<string, unknown> };

const DEFAULT_MISSION_CENTER: [number, number, number] = [5, 5, -2];

const MISSIONS: MissionDef[] = [
  { type: 'formation-transit', label: 'Formation Transit', description: 'Swarm flies in formation to waypoint and back, maintaining structure', minDrones: 3 },
  { type: 'formation-hold-wind', label: 'Formation + Wind', description: 'Hold tight formation while wind gusts hit — tests swarm stability', minDrones: 3 },
  { type: 'formation-reconfig', label: 'Formation Reconfig', description: 'Formation transit, one drone killed mid-flight — swarm reconfigures and continues', minDrones: 4 },
  { type: 'leader-loss', label: 'Leader Loss + Handoff', description: 'Leader killed during patrol — new leader elected, mission plan handed off', minDrones: 4 },
  { type: 'multi-loss', label: 'Multi Drone Loss', description: 'Two drones killed at different times — swarm adapts twice and finishes mission', minDrones: 6 },
  { type: 'long-range-patrol', label: 'Long Range Patrol', description: 'GPS-denied multi-waypoint patrol with VIO drift, no anchors, return to base', minDrones: 3 },
  { type: 'consensus-transit', label: 'Consensus Transit (no leader)', description: 'Every drone has the same route + own offset. No leader — consensus syncs progress', minDrones: 3 },
  { type: 'consensus-loss', label: 'Consensus + Drone Loss', description: 'Leaderless transit, one drone killed — no handoff needed, swarm continues', minDrones: 4 },
  { type: 'consensus-patrol', label: 'Consensus Patrol (no leader)', description: 'GPS-denied leaderless patrol. Every drone navigates independently, consensus keeps formation', minDrones: 3 },
];

function computePathBounds(
  positions: [number, number, number][],
  missionTargets: [number, number, number][],
): { min: [number, number, number]; max: [number, number, number] } {
  const pts = [...positions, ...missionTargets];
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (const p of pts) {
    if (p[0] < minX) minX = p[0];
    if (p[1] < minY) minY = p[1];
    if (p[2] < minZ) minZ = p[2];
    if (p[0] > maxX) maxX = p[0];
    if (p[1] > maxY) maxY = p[1];
    if (p[2] > maxZ) maxZ = p[2];
  }
  return {
    min: [minX - 20, minY - 20, minZ - 15],
    max: [maxX + 20, maxY + 20, Math.min(0, maxZ + 5)],
  };
}

function pushWaypointLeg(
  steps: MissionStep[],
  droneId: number,
  time: number,
  position: [number, number, number],
  yaw: number,
  speed: number,
): void {
  steps.push({ time, droneId, action: 'waypoint', params: { position, yaw, speed } });
}

export function computePatternPositions(
  droneCount: number,
  pattern: SwarmPattern,
  spacing: number,
  center: [number, number, number] = DEFAULT_MISSION_CENTER,
): [number, number, number][] {
  const positions: [number, number, number][] = [];
  const [cx, cy, cz] = center;

  if (pattern === 'line') {
    for (let i = 0; i < droneCount; i++) {
      positions.push([cx + (i - (droneCount - 1) / 2) * spacing, cy, cz]);
    }
    return positions;
  }

  if (pattern === 'grid') {
    const cols = Math.ceil(Math.sqrt(droneCount));
    for (let i = 0; i < droneCount; i++) {
      const row = Math.floor(i / cols);
      const col = i % cols;
      positions.push([
        cx + (col - (cols - 1) / 2) * spacing,
        cy + (row - (cols - 1) / 2) * spacing,
        cz,
      ]);
    }
    return positions;
  }

  for (let i = 0; i < droneCount; i++) {
    const angle = (2 * Math.PI * i) / droneCount;
    const r = droneCount === 1 ? 0 : spacing;
    positions.push([cx + r * Math.cos(angle), cy + r * Math.sin(angle), cz]);
  }
  return positions;
}

export function buildMissionConfig(
  droneCount: number,
  pattern: SwarmPattern,
  spacing: number,
  opts?: { type?: MissionType; distance?: number; wind?: number },
): SimConfig {
  const cfg = defaultSimConfig();
  const distance = opts?.distance ?? 10;
  cfg.swarm.droneCount = Math.max(1, Math.min(20, droneCount || 5));
  cfg.swarm.initialPattern = pattern;
  cfg.swarm.patternSpacing = Math.max(0.5, spacing || 2);
  cfg.swarm.patternCenter = new Float64Array(DEFAULT_MISSION_CENTER);
  cfg.swarm.initialPositions = undefined;
  cfg.swarm.formation.topology = pattern;
  cfg.swarm.formation.spacing = cfg.swarm.patternSpacing;
  cfg.swarm.formation.offsetFrame = 'heading';
  cfg.swarm.communication.range = Math.max(cfg.swarm.communication.range, 60);
  cfg.sensors.cameraVIO = { ...cfg.sensors.cameraVIO!, enabled: true };
  cfg.sensors.uwb = { ...cfg.sensors.uwb!, enabled: droneCount > 1 };
  if ((opts?.wind ?? 0) > 0) cfg.environment.wind.meanSpeed = opts?.wind ?? 0;

  // Compute scene bounds from all possible mission targets
  const hoverTargets = computePatternPositions(cfg.swarm.droneCount, pattern, cfg.swarm.patternSpacing, DEFAULT_MISSION_CENTER);
  const extraTargets: [number, number, number][] = [
    [DEFAULT_MISSION_CENTER[0] + distance, DEFAULT_MISSION_CENTER[1], DEFAULT_MISSION_CENTER[2]],
    [DEFAULT_MISSION_CENTER[0] - distance, DEFAULT_MISSION_CENTER[1], DEFAULT_MISSION_CENTER[2]],
    [DEFAULT_MISSION_CENTER[0], DEFAULT_MISSION_CENTER[1] + distance, DEFAULT_MISSION_CENTER[2]],
    [DEFAULT_MISSION_CENTER[0], DEFAULT_MISSION_CENTER[1] - distance, DEFAULT_MISSION_CENTER[2]],
  ];
  cfg.environment.scene.sceneBounds = computePathBounds(hoverTargets, extraTargets);

  // Consensus missions: switch formation mode
  if (opts?.type?.startsWith('consensus')) {
    cfg.swarm.formation.mode = 'consensus';
    cfg.swarm.formation.consensusGain = 0.5;
  }

  // Long-range patrol / consensus patrol: use shared config builder
  if (opts?.type === 'long-range-patrol' || opts?.type === 'consensus-patrol') {
    return buildPatrolConfig({
      droneCount: cfg.swarm.droneCount,
      pattern: cfg.swarm.initialPattern as 'line' | 'grid' | 'circle',
      spacing: cfg.swarm.patternSpacing,
      distance,
      wind: opts?.wind,
      consensusMode: opts?.type === 'consensus-patrol',
    });
  }

  return cfg;
}

export function buildVisualMissionSteps(opts: {
  type: MissionType;
  droneCount: number;
  pattern: SwarmPattern;
  spacing: number;
  distance: number;
  speed: number;
  wind: number;
  center?: [number, number, number];
}): MissionStep[] {
  const {
    type,
    droneCount,
    pattern,
    spacing,
    distance,
    speed,
    wind,
    center = DEFAULT_MISSION_CENTER,
  } = opts;
  const steps: MissionStep[] = [];
  const hoverTargets = computePatternPositions(droneCount, pattern, spacing, center);

  steps.push({ time: 0, droneId: 'all', action: 'arm' });
  for (let i = 0; i < hoverTargets.length; i++) {
    steps.push({ time: 0.5, droneId: i, action: 'hover', params: { position: hoverTargets[i], yaw: 0 } });
  }

  const legTime = distance / Math.max(0.5, speed);
  // All swarm missions enable formation (consensus or leader-follower — set in config)
  steps.push({ time: 2, droneId: 'all', action: 'formation-enable' });

  switch (type) {
    // ── Formation transit: fly out and back as a swarm ──
    case 'formation-transit': {
      // Leader mission plan: out → back
      steps.push({ time: 3, droneId: 0, action: 'set-mission-plan', params: { waypoints: [
        { position: [center[0] + distance, center[1], center[2]], yaw: 0, speed },
        { position: [center[0], center[1], center[2]], yaw: 0, speed },
      ]}});
      break;
    }

    // ── Formation hold under wind ──
    case 'formation-hold-wind': {
      // Just hold formation, wind hits at t=4 and t=8 (two gusts)
      steps.push({
        time: 4, droneId: 'all', action: 'inject-fault',
        params: { type: 'wind-gust', speed: Math.max(wind, 5) },
      });
      break;
    }

    // ── Formation reconfig: kill one drone mid-transit ──
    case 'formation-reconfig': {
      // Leader patrols
      steps.push({ time: 3, droneId: 0, action: 'set-mission-plan', params: { waypoints: [
        { position: [center[0] + distance, center[1], center[2]], yaw: 0, speed },
        { position: [center[0], center[1], center[2]], yaw: 0, speed },
      ]}});
      // Kill drone 2 mid-flight
      const killTime = Math.max(5, legTime * 0.4);
      steps.push({ time: killTime, droneId: 2, action: 'kill-drone' });
      break;
    }

    // ── Leader loss: leader killed, mission handed off to new leader ──
    case 'leader-loss': {
      // Leader gets a long mission plan
      steps.push({ time: 3, droneId: 0, action: 'set-mission-plan', params: { waypoints: [
        { position: [center[0] + distance * 0.5, center[1], center[2]], yaw: 0, speed },
        { position: [center[0] + distance, center[1], center[2]], yaw: 0, speed },
        { position: [center[0], center[1], center[2]], yaw: 0, speed },
      ]}});
      // Kill leader at ~40% of outbound leg
      const killTime = Math.max(6, legTime * 0.3);
      steps.push({ time: killTime, droneId: 0, action: 'kill-drone' });
      break;
    }

    // ── Multi-loss: two drones killed at different times ──
    case 'multi-loss': {
      steps.push({ time: 3, droneId: 0, action: 'set-mission-plan', params: { waypoints: [
        { position: [center[0] + distance * 0.5, center[1], center[2]], yaw: 0, speed },
        { position: [center[0] + distance, center[1], center[2]], yaw: 0, speed },
        { position: [center[0], center[1], center[2]], yaw: 0, speed },
      ]}});
      // First loss at 30% of outbound
      const kill1 = Math.max(5, legTime * 0.3);
      steps.push({ time: kill1, droneId: 3, action: 'kill-drone' });
      // Second loss at 60%
      const kill2 = Math.max(kill1 + 4, legTime * 0.6);
      steps.push({ time: kill2, droneId: 4, action: 'kill-drone' });
      break;
    }

    // ── Long range patrol: uses shared mission builder ──
    case 'long-range-patrol':
      return buildPatrolMissionSteps({
        droneCount, distance, waypointCount: Math.max(3, Math.ceil(distance / 20)),
        speed, returnToBase: true, center, spacing, pattern,
      });

    // ══ LEADERLESS (consensus) missions ══
    // Each drone gets OFFSET-ADJUSTED waypoints: centroid route + own pattern offset.
    // No formation-enable, no leader. Consensus progress sync keeps them aligned.

    case 'consensus-transit': {
      const centroidRoute: [number, number, number][] = [
        [center[0] + distance, center[1], center[2]],
        [center[0], center[1], center[2]],
      ];
      for (let i = 0; i < droneCount; i++) {
        const off = hoverTargets[i];
        const droneWps = centroidRoute.map(wp => ({
          position: [wp[0] + (off[0] - center[0]), wp[1] + (off[1] - center[1]), wp[2]] as [number, number, number],
          yaw: 0, speed,
        }));
        steps.push({ time: 3, droneId: i, action: 'set-mission-plan', params: { waypoints: droneWps } });
      }
      break;
    }

    case 'consensus-loss': {
      const centroidRoute: [number, number, number][] = [
        [center[0] + distance * 0.5, center[1], center[2]],
        [center[0] + distance, center[1], center[2]],
        [center[0], center[1], center[2]],
      ];
      for (let i = 0; i < droneCount; i++) {
        const off = hoverTargets[i];
        const droneWps = centroidRoute.map(wp => ({
          position: [wp[0] + (off[0] - center[0]), wp[1] + (off[1] - center[1]), wp[2]] as [number, number, number],
          yaw: 0, speed,
        }));
        steps.push({ time: 3, droneId: i, action: 'set-mission-plan', params: { waypoints: droneWps } });
      }
      const killTime = Math.max(6, legTime * 0.4);
      steps.push({ time: killTime, droneId: 2, action: 'kill-drone' });
      break;
    }

    case 'consensus-patrol': {
      const wpCount = Math.max(3, Math.ceil(distance / 20));
      const centroidRoute: [number, number, number][] = [];
      for (let j = 1; j <= wpCount; j++) centroidRoute.push([center[0] + distance * j / wpCount, center[1], center[2]]);
      for (let j = wpCount - 1; j >= 0; j--) centroidRoute.push([center[0] + distance * j / wpCount, center[1], center[2]]);

      for (let i = 0; i < droneCount; i++) {
        const off = hoverTargets[i];
        const droneWps = centroidRoute.map(wp => ({
          position: [wp[0] + (off[0] - center[0]), wp[1] + (off[1] - center[1]), wp[2]] as [number, number, number],
          yaw: 0, speed,
        }));
        steps.push({ time: 3, droneId: i, action: 'set-mission-plan', params: { waypoints: droneWps } });
      }
      break;
    }
  }

  // Apply wind for all formation missions if set
  if (wind > 0 && type !== 'formation-hold-wind') {
    steps.push({
      time: 0, droneId: 'all', action: 'set-environment',
      params: {
        wind: { meanSpeed: wind, meanDirection: [1, 0, 0], shearEnabled: false, shearExponent: 0.14, gustEvents: [] },
      },
    });
  }

  return steps;
}

export class MissionPanel {
  private container: HTMLDivElement;
  private sendCommand: (cmd: WorkerCommand) => void;
  private onInit: (config: SimConfig) => void;

  // Step 1 controls
  private droneCountInput!: HTMLInputElement;
  private patternSelect!: HTMLSelectElement;
  private spacingInput!: HTMLInputElement;

  // Step 2 controls
  private missionSelect!: HTMLSelectElement;
  private durationInput!: HTMLInputElement;
  private speedInput!: HTMLInputElement;
  private distanceInput!: HTMLInputElement;
  private windSpeedInput!: HTMLInputElement;

  // Step 3
  private progressBar!: HTMLDivElement;
  private progressFill!: HTMLDivElement;
  private progressLabel!: HTMLSpanElement;
  private resultsDiv!: HTMLDivElement;
  private downloadBtn!: HTMLButtonElement;

  // Buttons
  private formBtn!: HTMLButtonElement;
  private launchBtn!: HTMLButtonElement;
  private stopBtn!: HTMLButtonElement;

  // State
  private phase: 'setup' | 'formed' | 'running' | 'done' = 'setup';
  private missionStartTime = Number.NaN;
  private missionDuration = 15;

  constructor(
    parent: HTMLElement,
    sendCommand: (cmd: WorkerCommand) => void,
    onInit: (config: SimConfig) => void,
  ) {
    this.sendCommand = sendCommand;
    this.onInit = onInit;

    this.container = document.createElement('div');
    this.container.style.cssText = 'display:flex;flex-direction:column;gap:12px;padding:12px;height:100%;overflow-y:auto;';
    parent.appendChild(this.container);

    this.buildStep1();
    this.buildStep2();
    this.buildStep3();
    this.updatePhase('setup');
  }

  // ── Step 1: Swarm Configuration ──

  private buildStep1(): void {
    const section = this.makeSection('1. Swarm');

    this.droneCountInput = this.addNumberRow(section, 'Drones', 5, 1, 20, 1);
    this.patternSelect = this.addSelectRow(section, 'Pattern', ['line', 'grid', 'circle'], 'circle');
    this.spacingInput = this.addNumberRow(section, 'Spacing (m)', 2, 0.5, 10, 0.5);

    this.formBtn = document.createElement('button');
    this.formBtn.textContent = 'Form Swarm';
    this.formBtn.style.cssText = 'padding:8px 16px;font-size:13px;font-weight:600;cursor:pointer;border:none;border-radius:var(--radius);background:var(--accent);color:#fff;width:100%;margin-top:4px;';
    this.formBtn.addEventListener('click', () => this.formSwarm());
    section.appendChild(this.formBtn);
  }

  // ── Step 2: Mission Selection ──

  private buildStep2(): void {
    const section = this.makeSection('2. Mission');
    section.id = 'mission-step2';

    this.missionSelect = this.addSelectRow(section, 'Type', MISSIONS.map(m => m.type), 'formation-transit');
    const options = this.missionSelect.querySelectorAll('option');
    options.forEach((opt, i) => { opt.textContent = MISSIONS[i].label; });

    const desc = document.createElement('div');
    desc.id = 'mission-desc';
    desc.style.cssText = 'font-size:11px;color:var(--text-muted);margin:-4px 0 4px;';
    desc.textContent = MISSIONS.find(m => m.type === 'formation-transit')?.description ?? '';
    section.appendChild(desc);
    this.missionSelect.addEventListener('change', () => {
      const m = MISSIONS.find(x => x.type === this.missionSelect.value);
      desc.textContent = m?.description ?? '';
    });

    this.distanceInput = this.addNumberRow(section, 'Distance (m)', 10, 2, 50, 1);
    this.speedInput = this.addNumberRow(section, 'Speed (m/s)', 1.5, 0.5, 5, 0.5);
    this.durationInput = this.addNumberRow(section, 'Duration (s)', 15, 5, 120, 5);
    this.windSpeedInput = this.addNumberRow(section, 'Wind (m/s)', 0, 0, 10, 0.5);

    this.launchBtn = document.createElement('button');
    this.launchBtn.textContent = 'Launch Mission';
    this.launchBtn.style.cssText = 'padding:8px 16px;font-size:13px;font-weight:600;cursor:pointer;border:none;border-radius:var(--radius);background:#22aa44;color:#fff;width:100%;margin-top:4px;';
    this.launchBtn.addEventListener('click', () => this.launchMission());
    section.appendChild(this.launchBtn);
  }

  // ── Step 3: Progress + Results ──

  private buildStep3(): void {
    const section = this.makeSection('3. Flight');
    section.id = 'mission-step3';

    // Progress bar
    this.progressBar = document.createElement('div');
    this.progressBar.style.cssText = 'width:100%;height:20px;background:var(--bg-tertiary);border-radius:var(--radius);overflow:hidden;position:relative;';
    this.progressFill = document.createElement('div');
    this.progressFill.style.cssText = 'height:100%;background:var(--accent);width:0%;transition:width 0.3s;';
    this.progressBar.appendChild(this.progressFill);
    this.progressLabel = document.createElement('span');
    this.progressLabel.style.cssText = 'position:absolute;top:0;left:0;right:0;text-align:center;font-size:11px;line-height:20px;color:var(--text-primary);';
    this.progressLabel.textContent = 'Waiting...';
    this.progressBar.appendChild(this.progressLabel);
    section.appendChild(this.progressBar);

    // Stop button
    this.stopBtn = document.createElement('button');
    this.stopBtn.textContent = 'Stop';
    this.stopBtn.style.cssText = 'padding:6px 12px;font-size:12px;cursor:pointer;border:none;border-radius:var(--radius);background:#cc4444;color:#fff;width:100%;margin-top:4px;';
    this.stopBtn.addEventListener('click', () => this.stopMission());
    section.appendChild(this.stopBtn);

    // Results
    this.resultsDiv = document.createElement('div');
    this.resultsDiv.style.cssText = 'font-size:11px;font-family:var(--font-mono);white-space:pre;background:var(--bg-tertiary);border-radius:var(--radius);padding:8px;max-height:200px;overflow-y:auto;display:none;';
    section.appendChild(this.resultsDiv);

    this.downloadBtn = document.createElement('button');
    this.downloadBtn.textContent = 'Download Flight Log (CSV)';
    this.downloadBtn.style.cssText = 'padding:6px 12px;font-size:11px;cursor:pointer;border:none;border-radius:var(--radius);background:var(--accent);color:#fff;width:100%;margin-top:4px;display:none;';
    section.appendChild(this.downloadBtn);
  }

  // ── Actions ──

  private formSwarm(): void {
    const cfg = this.buildConfig();
    this.onInit(cfg);
    this.updatePhase('formed');
  }

  private launchMission(): void {
    this.downloadBtn.style.display = 'none';
    const type = this.missionSelect.value as MissionType;
    const droneCount = parseInt(this.droneCountInput.value) || 5;
    const pattern = this.patternSelect.value as SwarmPattern;
    const spacing = parseFloat(this.spacingInput.value) || 2;
    const distance = parseFloat(this.distanceInput.value) || 10;
    const speed = parseFloat(this.speedInput.value) || 1.5;
    const wind = parseFloat(this.windSpeedInput.value) || 0;
    this.missionDuration = parseFloat(this.durationInput.value) || 15;
    const cfg = buildMissionConfig(droneCount, pattern, spacing, { type, distance, wind });

    const steps = buildVisualMissionSteps({ type, droneCount, pattern, spacing, distance, speed, wind });
    this.onInit(cfg);

    this.sendCommand({
      type: 'run-visual-mission',
      steps,
      duration: this.missionDuration,
      wind,
    });

    this.missionStartTime = Number.NaN;
    this.updatePhase('running');
  }

  private stopMission(): void {
    this.sendCommand({ type: 'stop-visual-mission' });
    this.updatePhase('formed');
  }

  buildConfig(): SimConfig {
    return buildMissionConfig(
      parseInt(this.droneCountInput.value) || 5,
      this.patternSelect.value as SwarmPattern,
      parseFloat(this.spacingInput.value) || 2,
      {
        type: (this.missionSelect?.value as MissionType | undefined) ?? 'formation-transit',
        distance: parseFloat(this.distanceInput?.value ?? '10') || 10,
        wind: parseFloat(this.windSpeedInput?.value ?? '0') || 0,
      },
    );
  }

  // ── State update from worker ──

  update(swarm: SwarmSnapshot): void {
    if (this.phase === 'running') {
      if (!Number.isFinite(this.missionStartTime)) this.missionStartTime = swarm.simTime;
      const simTime = swarm.simTime - this.missionStartTime;
      const pct = Math.min(100, (simTime / this.missionDuration) * 100);
      this.progressFill.style.width = pct + '%';
      this.progressLabel.textContent = `${simTime.toFixed(1)}s / ${this.missionDuration}s`;

      // Collect live metrics
      const nDrones = swarm.drones.length;
      const minSep = swarm.safetyMetrics.minSeparation;
      const collisions = swarm.safetyMetrics.collisionCount;
      this.resultsDiv.style.display = 'block';
      this.resultsDiv.textContent = [
        `Drones: ${nDrones} | t = ${simTime.toFixed(1)}s`,
        `Min separation: ${minSep === Infinity ? '--' : minSep.toFixed(2)}m`,
        `Collisions: ${collisions}`,
        `Safety overrides: ${swarm.safetyMetrics.safetyOverrideCount}`,
      ].join('\n');
    }
  }

  /** Show download button for flight log CSV. */
  enableLogDownload(onDownload: () => void): void {
    this.downloadBtn.style.display = 'block';
    this.downloadBtn.onclick = onDownload;
  }

  /** Called when worker reports mission complete with metrics. */
  showResults(metrics: Record<string, number>): void {
    this.updatePhase('done');
    this.progressFill.style.width = '100%';
    this.progressLabel.textContent = 'Complete';
    this.resultsDiv.style.display = 'block';
    const lines = [
      '=== Mission Complete ===',
      '',
      `Collisions:          ${metrics.collisionCount ?? 0}`,
      `Min separation:      ${metrics.minSeparation === Infinity ? '--' : (metrics.minSeparation ?? 0).toFixed(2)}m`,
      `Tracking error RMS:  ${(metrics.rmsExecutedTrackingError ?? 0).toFixed(3)}m`,
      `Estimation error:    ${(metrics.rmsEstimationError ?? 0).toFixed(3)}m`,
      `Altitude error RMS:  ${(metrics.rmsAltitudeError ?? 0).toFixed(3)}m`,
      `Max horiz drift:     ${(metrics.maxHorizontalDrift ?? 0).toFixed(2)}m`,
      `Formation RMS:       ${isNaN(metrics.formationRMS) ? 'N/A' : (metrics.formationRMS ?? 0).toFixed(3)}m`,
      `Packet delivery:     ${((metrics.packetDeliveryRate ?? 1) * 100).toFixed(0)}%`,
      `Safety overrides:    ${metrics.safetyOverrideCount ?? 0}`,
      `Emergency stops:     ${metrics.emergencyStopCount ?? 0}`,
      `Real-time factor:    ${(metrics.realTimeFactor ?? 0).toFixed(1)}x`,
    ];
    this.resultsDiv.textContent = lines.join('\n');
  }

  // ── Phase management ──

  private updatePhase(phase: 'setup' | 'formed' | 'running' | 'done'): void {
    this.phase = phase;

    // Step 1 always visible
    this.formBtn.textContent = phase === 'setup' ? 'Form Swarm' : 'Reform Swarm';

    // Step 2 enabled after forming
    const step2 = this.container.querySelector('#mission-step2') as HTMLElement;
    if (step2) step2.style.opacity = phase === 'setup' ? '0.4' : '1';
    this.launchBtn.disabled = phase === 'setup';
    this.launchBtn.style.opacity = phase === 'setup' ? '0.4' : '1';

    // Step 3
    const step3 = this.container.querySelector('#mission-step3') as HTMLElement;
    if (step3) step3.style.display = (phase === 'running' || phase === 'done') ? 'block' : 'none';

    this.stopBtn.style.display = phase === 'running' ? 'block' : 'none';
  }

  // ── Helpers ──

  private makeSection(title: string): HTMLDivElement {
    const section = document.createElement('div');
    section.style.cssText = 'background:var(--bg-secondary);border:1px solid var(--border);border-radius:var(--radius);padding:10px;';

    const h = document.createElement('div');
    h.textContent = title;
    h.style.cssText = 'font-size:12px;font-weight:600;color:var(--text-primary);margin-bottom:8px;text-transform:uppercase;letter-spacing:0.5px;';
    section.appendChild(h);

    this.container.appendChild(section);
    return section;
  }

  private addNumberRow(parent: HTMLElement, label: string, value: number, min: number, max: number, step: number): HTMLInputElement {
    const row = document.createElement('div');
    row.style.cssText = 'display:flex;align-items:center;justify-content:space-between;margin-bottom:4px;';

    const lbl = document.createElement('span');
    lbl.textContent = label;
    lbl.style.cssText = 'font-size:12px;color:var(--text-secondary);';
    row.appendChild(lbl);

    const input = document.createElement('input');
    input.type = 'number';
    input.value = String(value);
    input.min = String(min); input.max = String(max); input.step = String(step);
    input.style.cssText = 'width:60px;background:var(--bg-tertiary);border:1px solid var(--border);color:var(--text-primary);font-family:var(--font-mono);font-size:12px;padding:3px 6px;border-radius:var(--radius);text-align:right;';
    row.appendChild(input);

    parent.appendChild(row);
    return input;
  }

  private addSelectRow(parent: HTMLElement, label: string, options: string[], initial: string): HTMLSelectElement {
    const row = document.createElement('div');
    row.style.cssText = 'display:flex;align-items:center;justify-content:space-between;margin-bottom:4px;';

    const lbl = document.createElement('span');
    lbl.textContent = label;
    lbl.style.cssText = 'font-size:12px;color:var(--text-secondary);';
    row.appendChild(lbl);

    const select = document.createElement('select');
    select.style.cssText = 'width:120px;background:var(--bg-tertiary);border:1px solid var(--border);color:var(--text-primary);font-family:var(--font-ui);font-size:12px;padding:3px 6px;border-radius:var(--radius);';
    for (const opt of options) {
      const o = document.createElement('option');
      o.value = opt; o.textContent = opt;
      if (opt === initial) o.selected = true;
      select.appendChild(o);
    }
    row.appendChild(select);

    parent.appendChild(row);
    return select;
  }
}
