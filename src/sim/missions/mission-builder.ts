/**
 * Shared mission builder: generates waypoint routes and mission steps.
 * Used by BOTH mission-panel.ts (UI) and scenario presets (scenario runner).
 * Single source of truth for route generation.
 */

import type { SimConfig } from '@sim/core/types';
import type { MissionStep } from '@sim/scenarios/scenario-types';
import type { MissionWaypoint } from '@sim/control/guidance';
import { defaultSimConfig } from '@sim/core/config-defaults';

export interface PatrolRouteOpts {
  distance: number;        // total one-way distance (m)
  waypointCount: number;   // number of waypoints on outbound leg
  speed: number;           // m/s
  returnToBase: boolean;   // add reverse waypoints
  center: [number, number, number]; // start position (NED)
  direction?: [number, number]; // unit direction XY (default [1,0] = North)
}

/**
 * Generate a patrol route: evenly spaced waypoints along a line, optionally with return.
 */
export function buildPatrolRoute(opts: PatrolRouteOpts): MissionWaypoint[] {
  const { distance, waypointCount, speed, returnToBase, center } = opts;
  const dir = opts.direction ?? [1, 0];
  const waypoints: MissionWaypoint[] = [];
  const n = Math.max(1, waypointCount);

  // Outbound waypoints
  for (let i = 1; i <= n; i++) {
    const frac = i / n;
    waypoints.push({
      position: [
        center[0] + dir[0] * distance * frac,
        center[1] + dir[1] * distance * frac,
        center[2],
      ],
      yaw: 0,
      speed,
    });
  }

  // Return waypoints (reverse order, skip last since it's the turnaround point)
  if (returnToBase) {
    for (let i = n - 1; i >= 0; i--) {
      const frac = i / n;
      waypoints.push({
        position: [
          center[0] + dir[0] * distance * frac,
          center[1] + dir[1] * distance * frac,
          center[2],
        ],
        yaw: 0,
        speed,
      });
    }
  }

  return waypoints;
}

export interface PatrolMissionOpts {
  droneCount: number;
  distance: number;
  waypointCount: number;
  speed: number;
  returnToBase: boolean;
  center: [number, number, number];
  spacing: number;
  pattern: 'line' | 'grid' | 'circle';
  killDroneId?: number;    // optional: drone to kill
  killTime?: number;       // optional: when to kill it (s from mission start)
}

/**
 * Build complete mission step sequence for a patrol mission.
 * Includes: arm, hover at pattern positions, formation enable, leader mission plan.
 */
export function buildPatrolMissionSteps(opts: PatrolMissionOpts): MissionStep[] {
  const steps: MissionStep[] = [];

  // Arm all
  steps.push({ time: 0, droneId: 'all', action: 'arm' });

  // Hover at pattern positions (compute inline to avoid circular dep with mission-panel)
  const positions = computeSpawnPositions(opts.droneCount, opts.pattern, opts.spacing, opts.center);
  for (let i = 0; i < positions.length; i++) {
    steps.push({ time: 0.5, droneId: i, action: 'hover', params: { position: positions[i], yaw: 0 } });
  }

  // Formation enable
  steps.push({ time: 2, droneId: 'all', action: 'formation-enable' });

  // Leader gets mission plan
  const route = buildPatrolRoute({
    distance: opts.distance, waypointCount: opts.waypointCount,
    speed: opts.speed, returnToBase: opts.returnToBase, center: opts.center,
  });
  steps.push({ time: 3, droneId: 0, action: 'set-mission-plan', params: { waypoints: route } });

  // Optional drone kill
  if (opts.killDroneId !== undefined && opts.killTime !== undefined) {
    steps.push({ time: opts.killTime, droneId: opts.killDroneId, action: 'kill-drone' });
  }

  return steps;
}

function computeSpawnPositions(
  n: number, pattern: 'line' | 'grid' | 'circle', spacing: number,
  center: [number, number, number],
): [number, number, number][] {
  const positions: [number, number, number][] = [];
  if (pattern === 'line') {
    for (let i = 0; i < n; i++) positions.push([center[0] + (i - (n - 1) / 2) * spacing, center[1], center[2]]);
  } else if (pattern === 'grid') {
    const cols = Math.ceil(Math.sqrt(n));
    for (let i = 0; i < n; i++) {
      const row = Math.floor(i / cols), col = i % cols;
      positions.push([center[0] + (col - (cols - 1) / 2) * spacing, center[1] + (row - (cols - 1) / 2) * spacing, center[2]]);
    }
  } else {
    for (let i = 0; i < n; i++) {
      const angle = (2 * Math.PI * i) / n;
      const r = n === 1 ? 0 : spacing;
      positions.push([center[0] + r * Math.cos(angle), center[1] + r * Math.sin(angle), center[2]]);
    }
  }
  return positions;
}

/**
 * Build a SimConfig for long-range patrol missions.
 * Single source of truth — used by both UI (mission-panel) and scenario presets.
 */
export function buildPatrolConfig(opts: {
  droneCount: number;
  pattern: 'line' | 'grid' | 'circle';
  spacing: number;
  distance: number;
  center?: [number, number, number];
  wind?: number;
  anchorFree?: boolean;
  consensusMode?: boolean;
}): SimConfig {
  const center = opts.center ?? [5, 5, -2];
  const cfg = defaultSimConfig();
  cfg.swarm.droneCount = Math.max(1, Math.min(20, opts.droneCount));
  cfg.swarm.initialPattern = opts.pattern;
  cfg.swarm.patternSpacing = Math.max(0.5, opts.spacing);
  cfg.swarm.patternCenter = new Float64Array(center);
  cfg.swarm.initialPositions = undefined;
  cfg.swarm.formation.topology = opts.pattern;
  cfg.swarm.formation.spacing = cfg.swarm.patternSpacing;
  cfg.swarm.formation.offsetFrame = 'heading';
  cfg.swarm.communication.range = Math.max(cfg.swarm.communication.range, 60);
  // Enable GPS-denied aiding sensors
  cfg.sensors.cameraVIO = { ...cfg.sensors.cameraVIO!, enabled: true };
  cfg.sensors.uwb = { ...cfg.sensors.uwb!, enabled: opts.droneCount > 1 };
  if ((opts.wind ?? 0) > 0) cfg.environment.wind.meanSpeed = opts.wind!;
  // Scene bounds: cover full mission area
  const d = opts.distance;
  cfg.environment.scene.sceneBounds = {
    min: [center[0] - d - 20, center[1] - d - 20, center[2] - 15] as [number, number, number],
    max: [center[0] + d + 20, center[1] + d + 20, Math.min(0, center[2] + 5)] as [number, number, number],
  };
  // Anchor-free mode: no ground beacons, enable VIO drift
  if (opts.anchorFree !== false) {
    cfg.environment.scene.uwbAnchors = [];
    if (cfg.sensors.cameraVIO) cfg.sensors.cameraVIO.vioDriftEnabled = true;
  }
  // Consensus mode: leaderless formation
  if (opts.consensusMode) {
    cfg.swarm.formation.mode = 'consensus';
    cfg.swarm.formation.consensusGain = 0.5;
    cfg.swarm.formation.enabled = true;
  }
  return cfg;
}
