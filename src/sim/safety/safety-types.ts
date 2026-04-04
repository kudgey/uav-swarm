/**
 * Safety layer types: config, state, events.
 */

import type { Vec3 } from '@sim/core/types';

export interface SafetyConfig {
  enabled: boolean;
  minSeparation: number;          // m, ORCA policy threshold
  orcaRadius: number;             // m, agent body radius + margin
  orcaTimeHorizon: number;        // s, collision look-ahead
  maxSpeed: number;               // m/s, velocity cap
  minAltitude: number;            // m above ground
  noFlyZones: NoFlyZone[];
  emergencyStopDistance: number;   // m, last-resort level-hover trigger
}

export interface NoFlyZone {
  min: Vec3;
  max: Vec3;
}

export interface SafetyState {
  active: boolean;
  reason: string;                 // 'none' | 'orca' | 'emergency' | 'speed' | 'altitude' | 'nofly'
  constraintCount: number;
  minSeparationCurrent: number;
  collisionPredicted: boolean;
}

export interface SafetyEvent {
  timestamp: number;
  droneId: number;
  type: 'orca-override' | 'emergency-stop' | 'nofly-clamp' | 'speed-clamp' | 'altitude-clamp';
  detail: string;
}

export interface SwarmSafetyMetrics {
  minSeparation: number;          // truth-based closest pair
  collisionCount: number;         // pairs < minSeparation config
  avgSeparation: number;
  safetyOverrideCount: number;
  emergencyStopCount: number;
}
