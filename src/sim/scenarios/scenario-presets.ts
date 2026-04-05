/**
 * Built-in scenario presets covering the full validation matrix.
 * CLAUDE.md §17.1 (single-drone), §17.2 (multi-drone), §17.3 (Monte Carlo).
 */

import { defaultSimConfig } from '@sim/core/config-defaults';
import type { ScenarioDefinition, MissionStep } from './scenario-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('scenario-presets', 'experimental', '22 validation presets');

function baseConfig() { return JSON.parse(JSON.stringify(defaultSimConfig())); }

function singleDroneConfig() {
  const c = baseConfig();
  c.swarm.droneCount = 1;
  c.swarm.initialPattern = 'explicit';
  c.swarm.initialPositions = [[5, 5, -2]];
  return c;
}

function multiDroneConfig(n: number) {
  const c = baseConfig();
  c.swarm.droneCount = n;
  c.swarm.initialPattern = 'line';
  c.swarm.patternSpacing = 3;
  c.swarm.patternCenter = [5, 5, -2];
  return c;
}

// ── Single-drone (§17.1) ──

export const hoverCalm: ScenarioDefinition = {
  name: 'hover-calm', description: 'Single drone hover in calm air',
  config: singleDroneConfig(), duration: 10,
  mission: { steps: [
    { time: 0, droneId: 0, action: 'arm' },
    { time: 0.5, droneId: 0, action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
  ]},
  acceptanceCriteria: { maxRmsAltitudeError: 0.3, maxMaxAltitudeError: 0.5 },
};

export const hoverImuBias: ScenarioDefinition = {
  name: 'hover-imu-bias', description: 'Hover with elevated IMU bias random walk',
  config: (() => { const c = singleDroneConfig(); c.sensors.imu.gyroBiasRW *= 3; c.sensors.imu.accelBiasRW *= 3; return c; })(),
  duration: 10,
  mission: { steps: [
    { time: 0, droneId: 0, action: 'arm' },
    { time: 0.5, droneId: 0, action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
  ]},
  acceptanceCriteria: { maxRmsAltitudeError: 0.5 },
};

export const forwardFlightBodyDrag: ScenarioDefinition = {
  name: 'forward-flight-body-drag',
  description: 'Forward flight with body drag (rotor drag deferred to Phase 11)',
  config: singleDroneConfig(), duration: 15,
  mission: { steps: [
    { time: 0, droneId: 0, action: 'arm' },
    { time: 0.5, droneId: 0, action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
    { time: 3, droneId: 0, action: 'waypoint', params: { position: [15, 5, -2], yaw: 0, speed: 2 } },
  ]},
  acceptanceCriteria: { maxRmsExecutedTrackingError: 1.0 },
};

export const windGust: ScenarioDefinition = {
  name: 'wind-gust', description: 'Wind gust rejection',
  config: singleDroneConfig(), duration: 10,
  mission: { steps: [
    { time: 0, droneId: 0, action: 'arm' },
    { time: 0.5, droneId: 0, action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
    { time: 2, droneId: 'all', action: 'inject-fault', params: { type: 'wind-gust', speed: 3 } },
  ]},
  acceptanceCriteria: { maxRecoveryTime: 5 },
  recoverySpec: { triggerTime: 2, metric: 'positionErrorNorm', threshold: 0.5, holdTime: 1 },
};

export const windGustCorridor: ScenarioDefinition = {
  name: 'wind-gust-corridor', description: 'Sustained crosswind',
  config: (() => { const c = singleDroneConfig(); c.environment.wind.meanSpeed = 2; return c; })(),
  duration: 15,
  mission: { steps: [
    { time: 0, droneId: 0, action: 'arm' },
    { time: 0.5, droneId: 0, action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
  ]},
  acceptanceCriteria: { maxRmsExecutedTrackingError: 1.0 },
};

export const magAnomaly: ScenarioDefinition = {
  name: 'mag-anomaly', description: 'Magnetometer anomaly rejection',
  config: singleDroneConfig(), duration: 8,
  mission: { steps: [
    { time: 0, droneId: 0, action: 'arm' },
    { time: 0.5, droneId: 0, action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
    { time: 3, droneId: 'all', action: 'inject-fault', params: { type: 'mag-anomaly' } },
  ]},
  acceptanceCriteria: { minMaxInnovationNorm: 0.001, minInnovationGatedFraction: 0.01 },
};

export const baroDrift: ScenarioDefinition = {
  name: 'baro-drift', description: 'Barometer bias drift',
  config: (() => { const c = singleDroneConfig(); c.sensors.barometer.biasRW *= 10; return c; })(),
  duration: 20,
  mission: { steps: [
    { time: 0, droneId: 0, action: 'arm' },
    { time: 0.5, droneId: 0, action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
  ]},
  acceptanceCriteria: {},
};

export const flowFeatureless: ScenarioDefinition = {
  name: 'flow-featureless', description: 'Optical flow on featureless surface',
  config: singleDroneConfig(), duration: 8,
  mission: { steps: [
    { time: 0, droneId: 0, action: 'arm' },
    { time: 0.5, droneId: 0, action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
    { time: 3, droneId: 'all', action: 'set-environment', params: { scene: { surfaceTextureQuality: 0.05 } } },
  ]},
  acceptanceCriteria: { maxFlowValidFraction: 0.3 },
};

export const vioFeaturePoor: ScenarioDefinition = {
  name: 'vio-feature-poor', description: 'VIO in low-texture environment',
  config: (() => { const c = singleDroneConfig(); c.sensors.cameraVIO = { ...c.sensors.cameraVIO!, enabled: true }; c.environment.scene.surfaceTextureQuality = 0.05; return c; })(),
  duration: 10,
  mission: { steps: [
    { time: 0, droneId: 0, action: 'arm' },
    { time: 0.5, droneId: 0, action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
  ]},
  acceptanceCriteria: { maxVioValidFraction: 0.3 },
};

export const uwbNlos: ScenarioDefinition = {
  name: 'uwb-nlos', description: 'UWB under NLOS conditions',
  config: (() => { const c = singleDroneConfig(); c.sensors.uwb = { ...c.sensors.uwb!, enabled: true }; return c; })(),
  duration: 10,
  mission: { steps: [
    { time: 0, droneId: 0, action: 'arm' },
    { time: 0.5, droneId: 0, action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
    { time: 3, droneId: 'all', action: 'inject-fault', params: { type: 'uwb-nlos' } },
  ]},
  acceptanceCriteria: { minUwbRangeRmse: 0.1 },
};

export const takeoffLanding: ScenarioDefinition = {
  name: 'takeoff-landing', description: 'Takeoff from ground to 1m hover',
  config: (() => { const c = singleDroneConfig(); c.swarm.initialPositions = [[5, 5, -0.5]]; return c; })(),
  duration: 8,
  mission: { steps: [
    { time: 0, droneId: 0, action: 'arm' },
    { time: 0.5, droneId: 0, action: 'hover', params: { position: [5, 5, -1], yaw: 0 } },
  ]},
  acceptanceCriteria: { maxMaxAltitudeError: 0.3 },
};

export const longHorizonDrift: ScenarioDefinition = {
  name: 'long-horizon-drift', description: '60s GPS-denied hover, flow only (no VIO/UWB)',
  config: singleDroneConfig(), duration: 60,
  mission: { steps: [
    { time: 0, droneId: 0, action: 'arm' },
    { time: 0.5, droneId: 0, action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
  ]},
  acceptanceCriteria: {}, // drift is expected, just measured
};

// ── Multi-drone (§17.2) ──

export const crossing2: ScenarioDefinition = {
  name: 'crossing-2', description: '2 drones crossing paths with ORCA',
  config: (() => { const c = multiDroneConfig(2); c.swarm.initialPattern = 'explicit'; c.swarm.initialPositions = [[1, 5, -2], [9, 5, -2]]; return c; })(),
  duration: 10,
  mission: { steps: [
    { time: 0, droneId: 'all', action: 'arm' },
    { time: 0.5, droneId: 0, action: 'waypoint', params: { position: [9, 5, -2], yaw: 0, speed: 2 } },
    { time: 0.5, droneId: 1, action: 'waypoint', params: { position: [1, 5, -2], yaw: 0, speed: 2 } },
  ]},
  acceptanceCriteria: { minMinSeparation: 0.5, maxCollisionCount: 0 },
};

export const formationLine4: ScenarioDefinition = {
  name: 'formation-line-4', description: '4-drone line formation',
  config: multiDroneConfig(4), duration: 15,
  mission: { steps: [
    { time: 0, droneId: 'all', action: 'arm' },
    { time: 0.5, droneId: 0, action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
    { time: 0.5, droneId: 1, action: 'hover', params: { position: [8, 5, -2], yaw: 0 } },
    { time: 0.5, droneId: 2, action: 'hover', params: { position: [2, 5, -2], yaw: 0 } },
    { time: 0.5, droneId: 3, action: 'hover', params: { position: [11, 5, -2], yaw: 0 } },
    { time: 3, droneId: 'all', action: 'formation-enable' },
  ]},
  acceptanceCriteria: { maxFormationRMS: 1.0, maxCollisionCount: 0 },
};

export const leaderLoss: ScenarioDefinition = {
  name: 'leader-loss', description: 'Leader comm disabled, followers detect and hover',
  config: multiDroneConfig(3), duration: 15,
  mission: { steps: [
    { time: 0, droneId: 'all', action: 'arm' },
    { time: 0.5, droneId: 0, action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
    { time: 0.5, droneId: 1, action: 'hover', params: { position: [7, 5, -2], yaw: 0 } },
    { time: 0.5, droneId: 2, action: 'hover', params: { position: [9, 5, -2], yaw: 0 } },
    { time: 3, droneId: 'all', action: 'formation-enable' },
    { time: 5, droneId: 0, action: 'set-drone-comm', params: { commEnabled: false } },
  ]},
  acceptanceCriteria: { maxCollisionCount: 0 },
};

export const packetLossBurst: ScenarioDefinition = {
  name: 'packet-loss-burst', description: '80% packet loss for 5 seconds',
  config: multiDroneConfig(4), duration: 15,
  mission: { steps: [
    { time: 0, droneId: 'all', action: 'arm' },
    { time: 0.5, droneId: 'all', action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
    { time: 3, droneId: 'all', action: 'inject-fault', params: { type: 'comm-delay-spike', dropRate: 0.8 } },
    { time: 8, droneId: 'all', action: 'inject-fault', params: { type: 'comm-delay-spike', dropRate: 0.02 } },
  ]},
  acceptanceCriteria: { maxCollisionCount: 0 },
};

export const delaySpike: ScenarioDefinition = {
  name: 'delay-spike', description: 'Communication latency spike during formation',
  config: multiDroneConfig(4), duration: 15,
  mission: { steps: [
    { time: 0, droneId: 'all', action: 'arm' },
    { time: 0.5, droneId: 'all', action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
    { time: 2, droneId: 'all', action: 'formation-enable' },
    { time: 3, droneId: 'all', action: 'inject-fault', params: { type: 'comm-delay-spike', latency: 0.2 } },
    { time: 8, droneId: 'all', action: 'inject-fault', params: { type: 'comm-delay-spike', latency: 0.02 } },
  ]},
  acceptanceCriteria: { maxCollisionCount: 0 },
};

export const corridorPassing: ScenarioDefinition = {
  name: 'corridor-passing', description: '2 drones passing in corridor',
  config: (() => { const c = multiDroneConfig(2); c.swarm.initialPattern = 'explicit'; c.swarm.initialPositions = [[2, 5, -2], [8, 5.3, -2]]; return c; })(),
  duration: 10,
  mission: { steps: [
    { time: 0, droneId: 'all', action: 'arm' },
    { time: 0.5, droneId: 0, action: 'waypoint', params: { position: [8, 5, -2], yaw: 0, speed: 1.5 } },
    { time: 0.5, droneId: 1, action: 'waypoint', params: { position: [2, 5, -2], yaw: 0, speed: 1.5 } },
  ]},
  acceptanceCriteria: { minMinSeparation: 0.5 },
};

export const uwbNlosSwarm: ScenarioDefinition = {
  name: 'uwb-nlos-swarm', description: 'Inter-drone UWB NLOS',
  config: (() => { const c = multiDroneConfig(3); c.sensors.uwb = { ...c.sensors.uwb!, enabled: true }; return c; })(),
  duration: 10,
  mission: { steps: [
    { time: 0, droneId: 'all', action: 'arm' },
    { time: 0.5, droneId: 'all', action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
  ]},
  acceptanceCriteria: {},
};

export const delayedNeighborAvoidance: ScenarioDefinition = {
  name: 'delayed-neighbor-avoidance', description: 'Head-on with 200ms comm latency',
  config: (() => { const c = multiDroneConfig(2); c.swarm.initialPattern = 'explicit'; c.swarm.initialPositions = [[1, 5, -2], [9, 5, -2]]; c.swarm.communication.latencyMean = 0.2; return c; })(),
  duration: 10,
  mission: { steps: [
    { time: 0, droneId: 'all', action: 'arm' },
    { time: 0.5, droneId: 0, action: 'waypoint', params: { position: [9, 5, -2], yaw: 0, speed: 2 } },
    { time: 0.5, droneId: 1, action: 'waypoint', params: { position: [1, 5, -2], yaw: 0, speed: 2 } },
  ]},
  acceptanceCriteria: { minMinSeparation: 0.5 },
};

// ── Monte Carlo (§17.3) ──

export const mcHover: ScenarioDefinition = {
  name: 'mc-hover', description: 'Monte Carlo hover with varied IMU bias and wind',
  config: singleDroneConfig(), duration: 30,
  mission: { steps: [
    { time: 0, droneId: 0, action: 'arm' },
    { time: 0.5, droneId: 0, action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
  ]},
  monteCarloRanges: {
    seeds: Array.from({ length: 20 }, (_, i) => 100 + i),
    imuBiasScale: [0.5, 3.0],
    windSpeed: [0, 3],
  },
  batchAcceptanceCriteria: { meanMaxHorizontalDrift: 5, p95MaxHorizontalDrift: 10, passRate: 0.9 },
};

export const mcFormation: ScenarioDefinition = {
  name: 'mc-formation', description: 'Monte Carlo formation with varied comm',
  config: multiDroneConfig(4), duration: 15,
  mission: { steps: [
    { time: 0, droneId: 'all', action: 'arm' },
    { time: 0.5, droneId: 'all', action: 'hover', params: { position: [5, 5, -2], yaw: 0 } },
    { time: 3, droneId: 'all', action: 'formation-enable' },
  ]},
  monteCarloRanges: {
    seeds: Array.from({ length: 10 }, (_, i) => 200 + i),
    commLatency: [0.01, 0.1],
    sensorDropoutRate: [0, 0.1],
  },
  batchAcceptanceCriteria: { meanFormationRMS: 2, passRate: 0.8 },
};

// ── Long-range patrol (uses shared mission builder) ──

import { buildPatrolMissionSteps, buildPatrolConfig, buildConsensusMissionSteps } from '@sim/missions/mission-builder';

export const longRangePatrol: ScenarioDefinition = (() => {
  const c = buildPatrolConfig({ droneCount: 5, pattern: 'line', spacing: 3, distance: 100 });
  const steps = buildPatrolMissionSteps({
    droneCount: 5, distance: 100, waypointCount: 5, speed: 1.5,
    returnToBase: true, center: [5, 5, -2], spacing: 3, pattern: 'line',
  });
  return {
    name: 'long-range-patrol',
    description: 'GPS-denied 100m patrol with VIO drift, no anchors, return to base',
    config: c,
    duration: 2 * 100 / 1.5 + 10,
    mission: { steps },
    acceptanceCriteria: { maxCollisionCount: 0 },
  };
})();

export const longRangePatrolWithLoss: ScenarioDefinition = (() => {
  const c = buildPatrolConfig({ droneCount: 5, pattern: 'line', spacing: 3, distance: 100 });
  c.swarm.formation.leaderLossFallback = 'reelect';
  const steps = buildPatrolMissionSteps({
    droneCount: 5, distance: 100, waypointCount: 5, speed: 1.5,
    returnToBase: true, center: [5, 5, -2], spacing: 3, pattern: 'line',
    killDroneId: 2, killTime: 30,
  });
  return {
    name: 'long-range-patrol-loss',
    description: 'GPS-denied patrol with drone loss at t=30s, formation reconfiguration',
    config: c,
    duration: 2 * 100 / 1.5 + 15,
    mission: { steps },
    acceptanceCriteria: { maxCollisionCount: 0 },
  };
})();

// ── Consensus (leaderless) presets — uses shared builder ──

const CONSENSUS_CENTER: [number, number, number] = [5, 5, -2];

export const consensusTransit: ScenarioDefinition = {
  name: 'consensus-transit', description: 'Leaderless consensus transit 50m out and back',
  config: buildPatrolConfig({ droneCount: 4, pattern: 'line', spacing: 3, distance: 50, consensusMode: true }),
  duration: 2 * 50 / 1.5 + 10,
  mission: { steps: buildConsensusMissionSteps({
    droneCount: 4, speed: 1.5, center: CONSENSUS_CENTER, spacing: 3, pattern: 'line',
    centroidRoute: [[CONSENSUS_CENTER[0] + 50, CONSENSUS_CENTER[1], CONSENSUS_CENTER[2]], CONSENSUS_CENTER],
  }) },
  acceptanceCriteria: { maxCollisionCount: 0 },
};

export const consensusLoss: ScenarioDefinition = {
  name: 'consensus-loss', description: 'Leaderless transit with drone loss at t=15s',
  config: buildPatrolConfig({ droneCount: 5, pattern: 'line', spacing: 3, distance: 50, consensusMode: true }),
  duration: 2 * 50 / 1.5 + 15,
  mission: { steps: buildConsensusMissionSteps({
    droneCount: 5, speed: 1.5, center: CONSENSUS_CENTER, spacing: 3, pattern: 'line',
    centroidRoute: [[CONSENSUS_CENTER[0] + 50, CONSENSUS_CENTER[1], CONSENSUS_CENTER[2]], CONSENSUS_CENTER],
    killDroneId: 2, killTime: 15,
  }) },
  acceptanceCriteria: { maxCollisionCount: 0 },
};

export const consensusPatrol: ScenarioDefinition = (() => {
  const wpCount = 5;
  const distance = 100;
  const centroid: [number, number, number][] = [];
  for (let j = 1; j <= wpCount; j++) centroid.push([CONSENSUS_CENTER[0] + distance * j / wpCount, CONSENSUS_CENTER[1], CONSENSUS_CENTER[2]]);
  for (let j = wpCount - 1; j >= 0; j--) centroid.push([CONSENSUS_CENTER[0] + distance * j / wpCount, CONSENSUS_CENTER[1], CONSENSUS_CENTER[2]]);
  return {
    name: 'consensus-patrol', description: 'GPS-denied leaderless 100m patrol',
    config: buildPatrolConfig({ droneCount: 5, pattern: 'line', spacing: 3, distance, consensusMode: true }),
    duration: 2 * distance / 1.5 + 10,
    mission: { steps: buildConsensusMissionSteps({
      droneCount: 5, speed: 1.5, center: CONSENSUS_CENTER, spacing: 3, pattern: 'line',
      centroidRoute: centroid,
    }) },
    acceptanceCriteria: { maxCollisionCount: 0 },
  };
})();

// ── Registry ──

export const ALL_PRESETS: ScenarioDefinition[] = [
  hoverCalm, hoverImuBias, forwardFlightBodyDrag, windGust, windGustCorridor,
  magAnomaly, baroDrift, flowFeatureless, vioFeaturePoor, uwbNlos,
  takeoffLanding, longHorizonDrift,
  crossing2, corridorPassing, formationLine4, leaderLoss, packetLossBurst,
  delaySpike, uwbNlosSwarm, delayedNeighborAvoidance,
  mcHover, mcFormation,
  longRangePatrol, longRangePatrolWithLoss,
  consensusTransit, consensusLoss, consensusPatrol,
];

export function getPresetByName(name: string): ScenarioDefinition | undefined {
  return ALL_PRESETS.find(p => p.name === name);
}
