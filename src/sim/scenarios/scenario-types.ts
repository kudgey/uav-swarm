/**
 * Scenario Runner type definitions.
 * STATUS: experimental
 */

import type { SimConfig, Vec3 } from '@sim/core/types';

// ── Mission DSL ──

export type MissionAction =
  | 'arm' | 'hover' | 'waypoint'
  | 'formation-enable' | 'formation-disable'
  | 'set-drone-comm'
  | 'set-sensor-config'
  | 'set-environment'
  | 'set-safety-config'
  | 'inject-fault'
  | 'set-mission-plan'
  | 'kill-drone';

export interface MissionStep {
  time: number;                   // s
  droneId: number | 'all';
  action: MissionAction;
  params?: Record<string, unknown>;
}

export interface MissionSpec {
  steps: MissionStep[];
}

// ── Monte Carlo ──

export interface MCParameterRanges {
  seeds: number[];
  imuBiasScale?: [number, number];
  windSpeed?: [number, number];
  sensorDropoutRate?: [number, number];
  uwbNLOSProbability?: [number, number];
  commLatency?: [number, number];
}

// ── Acceptance ──

export interface RecoverySpec {
  triggerTime: number;
  metric: 'positionErrorNorm' | 'altitudeError' | 'formationRMS';
  threshold: number;
  holdTime: number;
}

export interface AcceptanceCriteria {
  maxCollisionCount?: number;
  minMinSeparation?: number;
  maxRmsExecutedTrackingError?: number;
  maxRmsEstimationError?: number;
  maxHorizontalDrift?: number;
  maxXYDriftRate?: number;
  maxRmsAltitudeError?: number;
  maxMaxAltitudeError?: number;
  maxSaturationTimeFraction?: number;
  maxFormationRMS?: number;
  maxInnovationGatedFraction?: number;
  minFlowValidFraction?: number;
  maxFlowValidFraction?: number;
  minVioValidFraction?: number;
  maxVioValidFraction?: number;
  maxRecoveryTime?: number;
  minPacketDeliveryRate?: number;
  maxUwbRangeRmse?: number;
  minUwbRangeRmse?: number;
  maxNeighborEstimateError?: number;
  minNeighborEstimateError?: number;
  minMaxInnovationNorm?: number;
  minInnovationGatedFraction?: number;
}

export interface BatchAcceptanceCriteria {
  meanMaxHorizontalDrift?: number;
  p95MaxHorizontalDrift?: number;
  meanRmsExecutedTrackingError?: number;
  meanFormationRMS?: number;
  meanRmsAltitudeError?: number;
  passRate?: number;               // default 1.0 = all must pass
}

// ── Scenario definition ──

export interface ScenarioDefinition {
  name: string;
  description: string;
  config: SimConfig;
  duration: number;                 // s
  mission: MissionSpec;
  monteCarloRanges?: MCParameterRanges;
  acceptanceCriteria?: AcceptanceCriteria;
  batchAcceptanceCriteria?: BatchAcceptanceCriteria;
  recoverySpec?: RecoverySpec;
}

// ── Metrics ──

export interface CpuTimeBySubsystem {
  physics: number;
  sensors: number;
  estimation: number;
  communication: number;
  safety: number;
  control: number;
  total: number;
}

export interface ScenarioMetrics {
  collisionCount: number;
  minSeparation: number;
  rmsMissionTrackingError: number;
  rmsExecutedTrackingError: number;
  rmsEstimationError: number;
  maxHorizontalDrift: number;
  xyDriftRate: number;
  rmsAltitudeError: number;
  maxAltitudeError: number;
  saturationTimeFraction: number;
  packetDeliveryRate: number;
  avgCommLatency: number;
  formationRMS: number;
  innovationGatedFraction: number;
  flowValidFraction: number;
  vioValidFraction: number;
  uwbRangeRmse: number;
  neighborEstimateError: number;
  maxInnovationNorm: number;
  rmsAttitudeError: number;
  rmsVelocityError: number;
  rmsBiasEstimationError: number;
  realTimeFactor: number;
  cpuTimeBySubsystem: CpuTimeBySubsystem;
  recoveryTime: number;
}

export interface ScenarioRunResult {
  seed: number;
  scenarioName: string;
  metrics: ScenarioMetrics;
  passed: boolean;                // per-run acceptance verdict
  failedCriteria: string[];       // which criteria failed
  duration: number;               // actual sim time run
}
