/**
 * Default simulation configuration.
 *
 * Default drone: ~1kg X-quad, hover at ~64% of omegaMax (~36% headroom).
 * kT=8.5e-6, omegaMax=838 -> Omega_hover ≈ 537 rad/s ≈ 64% utilization.
 */

import { v3Create, m3Identity, m3Create } from '@lib/math';
import { xQuadRotorPositions, X_QUAD_ROTOR_DIRS, GRAVITY_MPS2 } from './frames';
import { ISA_SEA_LEVEL_PRESSURE, ISA_SEA_LEVEL_TEMPERATURE, ISA_SEA_LEVEL_DENSITY } from './units';
import type {
  DroneParams, SimConfig, EnvironmentOutput,
  EnvironmentConfig, SensorSuiteConfig,
  SwarmConfig, CommConfig,
} from './types';

export function defaultDroneParams(): DroneParams {
  const armLength = 0.17;
  return {
    mass: 1.0,
    inertia: v3Create(0.0135, 0.0135, 0.024),
    armLength,
    numRotors: 4,
    rotorPositions: xQuadRotorPositions(armLength),
    rotorDirections: [...X_QUAD_ROTOR_DIRS],
    kT: 8.5e-6,
    kQ: 1.2e-6,
    motorTau: 0.02,
    motorOmegaMin: 0,
    motorOmegaMax: 838,
    motorRateLimit: 50000,
    motorDeadZone: 30,
    dragCoeffLinear: v3Create(0.1, 0.1, 0.15),
    dragCoeffQuadratic: v3Create(0, 0, 0),
    dragMode: 'linear',
    dragCoeffRotor: v3Create(0, 0, 0),  // Phase 11: rotor drag off by default
    rotorInertia: 0,                     // Phase 11: gyro torque off by default
  };
}

export function defaultEnvironmentConfig(): EnvironmentConfig {
  return {
    wind: {
      meanSpeed: 0,
      meanDirection: v3Create(1, 0, 0),  // North
      shearEnabled: false,
      shearExponent: 0.14,
      gustEvents: [],
    },
    turbulence: {
      enabled: false,
      intensity: 1.0,
      correlationTime: 2.0,
      spatialCorrelation: false,   // Phase 11: off by default
      integralLengthScale: 5.0,
    },
    groundEffect: {
      enabled: true,
      propRadiusFactor: 0.7,
      wallCeilingEffectEnabled: false,  // Phase 11: off by default
    },
    magneticField: {
      fieldStrength: 50e-6,   // ~50 µT
      inclination: 1.15,      // ~66° (mid-latitude)
      declination: 0.05,      // ~3° east
      anomalies: [],
      wmmEnabled: false,      // Phase 11: off by default
      latitude: 55.75,        // Moscow
      longitude: 37.62,
      wmmDate: 2025.0,
    },
    scene: {
      preset: 'open_field',
      surfaceTextureQuality: 0.8,
      obstacles: [],
      uwbAnchors: [
        { id: 'A0', position: v3Create(0, 0, -0.5) },
        { id: 'A1', position: v3Create(10, 0, -0.5) },
        { id: 'A2', position: v3Create(10, 10, -0.5) },
        { id: 'A3', position: v3Create(0, 10, -0.5) },
      ],
    },
  };
}

export function defaultSensorSuiteConfig(): SensorSuiteConfig {
  const softIron = m3Create();
  m3Identity(softIron);
  return {
    imu: {
      enabled: true,
      rate: 250,
      gyroNoiseDensity: 0.004,
      gyroBiasRW: 0.001,
      gyroSaturation: 35,       // ~2000 deg/s
      accelNoiseDensity: 0.04,
      accelBiasRW: 0.01,
      accelSaturation: 160,     // ~16g
    },
    magnetometer: {
      enabled: true,
      rate: 50,
      noiseDensity: 1e-7,
      hardIron: v3Create(0, 0, 0),
      softIron,
      emiCoefficient: 1e-10,
      dropoutProbability: 0,
    },
    barometer: {
      enabled: true,
      rate: 25,
      noiseDensity: 0.5,
      biasRW: 0.01,
      biasInitial: 0,
    },
    rangefinder: {
      enabled: true,
      rate: 30,
      noise: 0.02,
      minRange: 0.05,
      maxRange: 8.0,
      maxIncidenceAngle: Math.PI / 4,  // 45°
      dropoutProbability: 0.01,
    },
    opticalFlow: {
      enabled: true,
      rate: 30,
      noiseBase: 0.05,
      heightScaleCoeff: 0.5,
      minHeight: 0.1,
      maxHeight: 10.0,
      textureThreshold: 0.2,
      dropoutProbability: 0.01,
    },
    cameraVIO: {
      enabled: false,  // off by default, enable via UI
      rate: 30,
      positionNoiseBase: 0.05,
      attitudeNoiseBase: 0.01,
      featureThreshold: 0.15,
      maxAngularRateForQuality: 3.0,
      maxHeightForQuality: 15.0,
      latencyFrames: 1,
      dropoutProbability: 0.01,
      cameraIntrinsicsEnabled: false,
      focalLength: 400,
      principalPoint: [320, 240],
      imageSize: [640, 480],
      distortionK1: 0,
      extrinsicsRotation: [0, 0, 0],
      extrinsicsTranslation: [0, 0, 0],
      vioDriftEnabled: false,
      vioScaleBiasRW: 0.001,
      vioYawDriftRW: 0.0005,
      vioPositionRW: 0.02,
    },
    uwb: {
      enabled: false,  // off by default, enable via UI
      rate: 10,
      losNoiseStdDev: 0.05,
      losBias: 0.05,
      nlosBiasBase: 0.5,
      nlosBiasStdDev: 0.3,
      nlosNoiseStdDev: 0.15,
      packetLossProbability: 0.02,
      minRange: 0.2,
      maxRange: 40.0,
    },
  };
}

/**
 * Factory for a neutral EnvironmentOutput.
 * Used by tests and as initial state before environment manager runs.
 */
export function createDefaultEnv(): EnvironmentOutput {
  return {
    wind: v3Create(0, 0, 0),
    gravity: v3Create(0, 0, GRAVITY_MPS2),
    airDensity: ISA_SEA_LEVEL_DENSITY,
    pressure: ISA_SEA_LEVEL_PRESSURE,
    temperature: ISA_SEA_LEVEL_TEMPERATURE,
    earthMagneticField: v3Create(0, 0, 0),
    heightAboveGround: 0,
    groundEffectMultiplier: 1.0,
    surfaceTextureQuality: 0.8,
  };
}

export function defaultEKFConfig(): import('@sim/estimation/estimator-types').EKFConfig {
  const envCfg = defaultEnvironmentConfig();
  const B = envCfg.magneticField.fieldStrength;
  const inc = envCfg.magneticField.inclination;
  const dec = envCfg.magneticField.declination;
  return {
    // Process noise (~3× actual sensor noise for conservative tuning)
    gyroNoisePSD: (0.004 * 3) ** 2,        // ~1.44e-4
    accelNoisePSD: (0.04 * 3) ** 2,         // ~0.0144
    gyroBiasRWPSD: (0.001 * 3) ** 2,        // ~9e-6
    accelBiasRWPSD: (0.01 * 3) ** 2,        // ~9e-4
    baroBiasRWPSD: (0.01 * 3) ** 2,         // ~9e-4
    // Measurement noise (slightly inflated for robustness)
    magNoiseVar: (5e-7) ** 2 * 50,          // inflated for EMI margin
    baroNoiseVar: (2.0) ** 2,               // Pa^2
    rangeNoiseVar: (0.05) ** 2,             // m^2
    flowNoiseVar: (0.2) ** 2,              // (rad/s)^2
    // Initial covariance
    initPosVar: 1.0,                        // m^2
    initVelVar: 0.1,                        // (m/s)^2
    initAttVar: 0.1,                        // rad^2 (~5.7 deg)
    initGyroBiasVar: 0.01,                  // (rad/s)^2
    initAccelBiasVar: 0.1,                  // (m/s^2)^2
    initBaroBiasVar: 100,                   // Pa^2
    // Expected earth field (from env config)
    earthMagField: v3Create(
      B * Math.cos(inc) * Math.cos(dec),
      B * Math.cos(inc) * Math.sin(dec),
      B * Math.sin(inc),
    ),
    // Innovation gating
    magGateThreshold: 11.34,                // chi2(3, 0.99)
    rangeGateThreshold: 9.0,                // 3-sigma
    flowGateThreshold: 9.21,                // chi2(2, 0.99)
    // VIO
    vioPositionNoiseVar: 0.01,              // m^2 (0.1m base, inflated)
    vioAttitudeNoiseVar: 0.0004,            // rad^2 (0.02 rad)
    vioGateThreshold: 11.34,               // chi2(3, 0.99)
    // UWB
    uwbLosNoiseVar: 0.01,                  // m^2 (0.1m)
    uwbNlosNoiseVar: 0.09,                 // m^2 (0.3m, larger for NLOS)
    uwbGateThreshold: 9.0,                 // 3-sigma
    // Phase 11: Innovation NLOS
    innovationNLOSDetection: false,
    nlosInnovThreshold: 6.0,
    nlosConsecutiveCount: 3,
    // Alignment
    alignAttVarThreshold: 0.01,             // rad^2 (~0.6 deg)
    alignAltVarThreshold: 0.5,              // m^2
    alignMinUpdates: 50,                    // ~1s of mag+baro
    // Health
    attHealthThreshold: 0.5,                // rad^2
    vertHealthThreshold: 5.0,               // m^2
    horizHealthThreshold: 2.0,              // m^2, achievable with VIO/UWB
  };
}

export function defaultControllerConfig(): import('@sim/control/controller-types').ControllerConfig {
  return {
    kpPos: v3Create(4, 4, 6),
    kdPos: v3Create(3, 3, 4),
    krAtt: v3Create(8, 8, 4),
    kwRate: v3Create(2, 2, 1),
    maxTiltRad: Math.PI / 4,               // 45 deg
    maxYawRate: Math.PI,                    // 180 deg/s
    maxThrust: 20,                          // N (~2g for 1kg drone)
    minThrust: 0.5,                         // N
    kwFailsafe: v3Create(1, 1, 0.5),
    hoverThrustFraction: 0.8,
    stationaryGyroThreshold: 0.1,           // rad/s
    stationaryAccelThreshold: 0.5,          // relative to g
  };
}

export function defaultCommConfig(): CommConfig {
  return {
    range: 30,
    latencyMean: 0.02,
    latencyJitter: 0.005,
    dropRate: 0.02,
    bandwidth: 100,
    estimatePublishRate: 10,
    maneuverAccelStdDev: 2.0,
    estimateTimeout: 2.0,
    losProjectedVariance: false,  // Phase 11: off by default
  };
}

export function defaultSwarmConfig(): SwarmConfig {
  return {
    droneCount: 1,
    initialPattern: 'explicit',
    initialPositions: [v3Create(5, 5, -2)],  // 2m altitude in NED (z=-2)
    patternSpacing: 2,
    patternCenter: v3Create(5, 5, -2),
    communication: defaultCommConfig(),
    formation: defaultFormationConfig(),
    safety: defaultSafetyConfig(),
  };
}

export function defaultFormationConfig(): import('@sim/core/types').FormationConfigType {
  return {
    enabled: false,
    mode: 'leader-follower',
    topology: 'line',
    spacing: 2,
    leaderDroneId: 0,
    consensusGain: 0.5,
    leaderLossFallback: 'hover',
    offsetFrame: 'world',
  };
}

export function defaultSafetyConfig(): import('@sim/core/types').SafetyConfigType {
  return {
    enabled: true,
    minSeparation: 0.5,
    orcaRadius: 0.3,
    orcaTimeHorizon: 3.0,
    maxSpeed: 5.0,
    minAltitude: 0.3,
    noFlyZones: [],
    emergencyStopDistance: 0.3,
  };
}

export function defaultSimConfig(): SimConfig {
  return {
    seed: 42,
    physicsDt: 0.001,
    commandRate: 250,
    telemetryRate: 60,
    maxSubSteps: 20,
    timeScale: 1.0,
    drone: defaultDroneParams(),
    environment: defaultEnvironmentConfig(),
    sensors: defaultSensorSuiteConfig(),
    estimation: defaultEKFConfig(),
    swarm: defaultSwarmConfig(),
  };
}
