/**
 * DroneInstance: bundles all per-drone state into one struct.
 * Each drone owns independent truth state, sensors, EKF, controller, RNG.
 * STATUS: experimental
 */

import { v3Create } from '@lib/math';
import { createDroneState } from '@sim/physics/state';
import { createRK4Scratch, type RK4Scratch } from '@sim/physics/integrator';
import { createDefaultEnv, defaultEKFConfig, defaultControllerConfig } from '@sim/core/config-defaults';
import { SensorManager } from '@sim/sensors/sensor-manager';
import { ErrorStateEKF } from '@sim/estimation/ekf-state';
import { CascadedController } from '@sim/control/controller';
import type { DroneState, DroneParams, EnvironmentOutput, SensorSuiteConfig } from '@sim/core/types';
import type { DeterministicRNG } from '@sim/core/rng';
import type { WorldGeometry } from '@sim/environment/world-geometry';
import type { Vec3 } from '@sim/core/types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('drone-instance', 'experimental', 'Per-drone state bundle');

export interface DroneInstance {
  id: number;
  state: DroneState;
  rk4Scratch: RK4Scratch;
  envOutput: EnvironmentOutput;
  sensorManager: SensorManager;
  ekf: ErrorStateEKF;
  controller: CascadedController;
  rng: DeterministicRNG;
  /** Last received neighbor estimates (ZOH). Key = neighbor drone id. */
  neighborEstimates: Map<number, NeighborEstimate>;
  /** Per-drone comm enable/disable for fault injection. */
  commEnabled: boolean;
  /** Persistent kill state: motors dead, no comm, no control. */
  destroyed: boolean;
  /** Cached formation state from last stepFormation() — side-effect-free for snapshots. */
  lastFormationState: import('./formation-types').FormationState | null;
  /** Safety-adjusted guidance (pre-controller). Cleared every tick. */
  safeGuidance: import('@sim/control/controller-types').GuidanceOutput | undefined;
  /** Cached safety state from last stepSafetyGuidance(). */
  lastSafetyState: import('@sim/safety/safety-types').SafetyState | null;
}

export interface NeighborEstimate {
  senderId: number;
  timestamp: number;
  position: Vec3;
  positionVariance: number;
  positionCov3x3?: Float64Array;  // Phase 11: 6 elements
  velocity: Vec3;
  quaternion?: Float64Array;       // Phase 11: heading-aligned formation
  aligned: boolean;
  attitudeHealthy: boolean;
  inFormation: boolean;
  missionProgress?: number;
}

/**
 * Create a single DroneInstance with all subsystems initialized.
 */
export function createDroneInstance(
  id: number,
  initialPosition: Vec3,
  droneParams: DroneParams,
  sensorConfig: SensorSuiteConfig,
  estimationConfig: import('@sim/estimation/estimator-types').EKFConfig,
  worldGeo: WorldGeometry,
  rng: DeterministicRNG,
): DroneInstance {
  const state = createDroneState(droneParams.numRotors);
  state.position[0] = initialPosition[0];
  state.position[1] = initialPosition[1];
  state.position[2] = initialPosition[2];

  // Pre-fill motors at hover trim so drone doesn't fall on Start
  const omegaTrim = Math.sqrt(droneParams.mass * 9.80665 / (droneParams.numRotors * droneParams.kT));
  for (let i = 0; i < droneParams.numRotors; i++) {
    state.motorSpeeds[i] = omegaTrim;
    state.motorCommands[i] = omegaTrim;
  }

  const rk4Scratch = createRK4Scratch(droneParams.numRotors);
  const envOutput = createDefaultEnv();

  // Deep-clone sensor config so per-drone faults don't affect other drones
  const ownSensorConfig = JSON.parse(JSON.stringify(sensorConfig));
  const sensorManager = new SensorManager(ownSensorConfig, rng, worldGeo, droneParams, initialPosition);

  const ekfConfig = estimationConfig;
  const ekf = new ErrorStateEKF(ekfConfig, worldGeo);
  ekf.reset(initialPosition);

  const ctrlConfig = defaultControllerConfig();
  const controller = new CascadedController(ctrlConfig, droneParams);

  return {
    id,
    state,
    rk4Scratch,
    envOutput,
    sensorManager,
    ekf,
    controller,
    rng,
    neighborEstimates: new Map(),
    commEnabled: true,
    destroyed: false,
    lastFormationState: null,
    safeGuidance: undefined,
    lastSafetyState: null,
  };
}
