/**
 * Message types for Worker <-> UI communication.
 * Phase 7: multi-drone with SwarmSnapshot.
 */

import type { SimConfig, EnvironmentConfig, SensorSuiteConfig, FormationConfigType, SafetyConfigType } from '@sim/core/types';
import type { ScenarioDefinition, ScenarioRunResult } from '@sim/scenarios/scenario-types';

// ── UI -> Worker commands ──

export type WorkerCommand =
  | { type: 'init'; config: SimConfig }
  | { type: 'start' }
  | { type: 'pause' }
  | { type: 'reset'; seed?: number }
  | { type: 'step'; count?: number }
  | { type: 'set-timescale'; value: number }
  | { type: 'update-params'; params: Record<string, unknown> }
  | { type: 'set-motor-commands'; droneId: number; commands: number[] }
  | { type: 'set-environment-config'; config: Partial<EnvironmentConfig> }
  | { type: 'set-sensor-config'; config: Partial<SensorSuiteConfig> }
  | { type: 'set-controller-mode'; droneId: number; mode: 'manual' | 'hover' | 'waypoint';
      params?: { position?: [number, number, number]; yaw?: number; speed?: number } }
  | { type: 'select-drone'; droneId: number }
  | { type: 'set-formation-config'; config: FormationConfigType }
  | { type: 'set-drone-comm'; droneId: number; commEnabled: boolean }
  | { type: 'set-safety-config'; config: Partial<SafetyConfigType> }
  | { type: 'set-estimation-config'; config: Record<string, unknown> }
  | { type: 'set-drone-params'; params: Record<string, unknown> }
  | { type: 'set-comm-config'; config: Record<string, unknown> }
  | { type: 'run-scenario'; scenario: ScenarioDefinition }
  | { type: 'run-batch'; scenario: ScenarioDefinition; seeds: number[] }
  | { type: 'run-visual-mission'; steps: VisualMissionStep[]; duration: number; wind: number }
  | { type: 'stop-visual-mission' };

export interface VisualMissionStep {
  time: number;
  droneId: number | 'all';
  action: string;
  params?: Record<string, unknown>;
}

// ── Worker -> UI events ──

export interface SensorSnapshot {
  imu: { valid: boolean; gyro: [number, number, number]; accel: [number, number, number] } | null;
  mag: { valid: boolean; field: [number, number, number] } | null;
  baro: { valid: boolean; pressure: number; altitude: number } | null;
  range: { valid: boolean; range: number } | null;
  flow: { valid: boolean; flowX: number; flowY: number } | null;
  vio: { valid: boolean; position: [number, number, number]; featureQuality: number } | null;
  uwb: { anchorId: string; range: number; valid: boolean; isNLOS: boolean }[] | null;
}

export interface EnvironmentSnapshot {
  windSpeed: number;
  heightAboveGround: number;
  groundEffectMultiplier: number;
  surfaceTextureQuality: number;
}

export interface EstimateSnapshot {
  position: [number, number, number];
  velocity: [number, number, number];
  quaternion: [number, number, number, number];
  gyroBias: [number, number, number];
  accelBias: [number, number, number];
  baroBias: number;
  health: {
    attitudeHealthy: boolean;
    verticalHealthy: boolean;
    aligned: boolean;
    attCovMax: number;
    vertCov: number;
    horizCovMax: number;
  };
}

/** Per-drone snapshot with stable id field. */
export interface DroneSnapshot {
  id: number;
  position: [number, number, number];
  velocity: [number, number, number];
  quaternion: [number, number, number, number];
  angularVelocity: [number, number, number];
  motorSpeeds: [number, number, number, number];
  motorCommands: [number, number, number, number];
  totalThrust: number;
  sensors: SensorSnapshot;
  environment: EnvironmentSnapshot;
  armState: string;
  controlMode: string;
  estimate?: EstimateSnapshot;
  commEnabled: boolean;
  formationState?: {
    active: boolean;
    role: 'leader' | 'follower' | 'peer';
    referenceHealthy: boolean;
    neighborCount: number;
  };
  safetyState?: {
    active: boolean;
    reason: string;
    constraintCount: number;
    minSeparationCurrent: number;
  };
}

/** Multi-drone swarm snapshot. */
export interface SwarmSnapshot {
  simTime: number;
  stepCount: number;
  selectedDroneId: number;
  drones: DroneSnapshot[];
  commLinks: { from: number; to: number; active: boolean }[];
  formation: {
    enabled: boolean;
    mode: string;
    topology: string;
    spacing: number;
    leaderDroneId: number;
    consensusGain: number;
    leaderLossFallback: string;
    offsetFrame: string;
  };
  safetyMetrics: {
    minSeparation: number;
    collisionCount: number;
    avgSeparation: number;
    safetyOverrideCount: number;
    emergencyStopCount: number;
  };
  safety: {
    enabled: boolean;
    minSeparation: number;
    orcaRadius: number;
    orcaTimeHorizon: number;
    maxSpeed: number;
    minAltitude: number;
    emergencyStopDistance: number;
  };
  safetyEvents: { timestamp: number; droneId: number; type: string; detail: string }[];
}

/** @deprecated Use DroneSnapshot from SwarmSnapshot.drones instead. */
export type StateSnapshot = SwarmSnapshot;

export interface SubsystemStatusEntry {
  id: string;
  status: string;
  description: string;
  demoOnly?: boolean;
}

export type WorkerEvent =
  | { type: 'state'; snapshot: SwarmSnapshot }
  | { type: 'telemetry'; batch: unknown }
  | { type: 'status'; running: boolean; simTime: number; stepCount: number; rtFactor: number }
  | { type: 'ready' }
  | { type: 'subsystem-status'; entries: SubsystemStatusEntry[] }
  | { type: 'error'; message: string }
  | { type: 'scenario-result'; result: ScenarioRunResult }
  | { type: 'batch-complete'; results: ScenarioRunResult[] }
  | { type: 'mission-complete'; metrics: Record<string, number> }
  | { type: 'mission-log'; csv: string };
