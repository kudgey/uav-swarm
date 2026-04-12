/**
 * Zod schemas for simulation configuration.
 * Runtime validation of config files and UI parameter updates.
 */

import { z } from 'zod';

const vec3Schema = z.tuple([z.number(), z.number(), z.number()]);
const mat3Schema = z.array(z.number()).length(9);

// ── Drone params ──

export const droneParamsSchema = z.object({
  mass: z.number().positive(),
  inertia: vec3Schema.refine(v => v.every(x => x > 0), 'All inertia components must be positive'),
  armLength: z.number().positive(),
  numRotors: z.number().int().min(2),
  rotorDirections: z.array(z.union([z.literal(1), z.literal(-1)])),
  kT: z.number().positive(),
  kQ: z.number().positive(),
  motorTau: z.number().positive(),
  motorOmegaMin: z.number().nonnegative(),
  motorOmegaMax: z.number().positive(),
  motorRateLimit: z.number().positive(),
  motorDeadZone: z.number().nonnegative(),
  dragCoeffLinear: vec3Schema,
  dragCoeffQuadratic: vec3Schema,
  dragMode: z.enum(['linear', 'quadratic', 'combined']),
  dragCoeffRotor: vec3Schema,
  rotorInertia: z.number().nonnegative(),
}).strict();

// ── Environment config ──

const gustEventSchema = z.object({
  onsetTime: z.number().nonnegative(),
  duration: z.number().positive(),
  peakAmplitude: vec3Schema,
}).strict();

const windConfigSchema = z.object({
  meanSpeed: z.number().nonnegative(),
  meanDirection: vec3Schema,
  shearEnabled: z.boolean(),
  shearExponent: z.number().nonnegative(),
  gustEvents: z.array(gustEventSchema),
}).strict();

const turbulenceConfigSchema = z.object({
  enabled: z.boolean(),
  intensity: z.number().nonnegative(),
  correlationTime: z.number().positive(),
  spatialCorrelation: z.boolean(),
  integralLengthScale: z.number().positive(),
}).strict();

const magAnomalySchema = z.object({
  position: vec3Schema,
  radius: z.number().positive(),
  strength: vec3Schema,
}).strict();

const magneticFieldConfigSchema = z.object({
  fieldStrength: z.number().nonnegative(),
  inclination: z.number(),
  declination: z.number(),
  anomalies: z.array(magAnomalySchema),
  wmmEnabled: z.boolean(),
  latitude: z.number(),
  longitude: z.number(),
  wmmDate: z.number(),
}).strict();

const obstacleDefSchema = z.object({
  min: vec3Schema,
  max: vec3Schema,
  textureQuality: z.number().min(0).max(1).optional(),
}).strict();

const uwbAnchorDefSchema = z.object({
  id: z.string(),
  position: vec3Schema,
}).strict();

const sceneConfigSchema = z.object({
  preset: z.enum(['open_field', 'indoor_basic', 'warehouse', 'custom']),
  surfaceTextureQuality: z.number().min(0).max(1),
  obstacles: z.array(obstacleDefSchema),
  uwbAnchors: z.array(uwbAnchorDefSchema).optional(),
  sceneBounds: z.object({ min: z.tuple([z.number(), z.number(), z.number()]), max: z.tuple([z.number(), z.number(), z.number()]) }).strict().optional(),
}).strict();

const environmentConfigSchema = z.object({
  wind: windConfigSchema,
  turbulence: turbulenceConfigSchema,
  groundEffect: z.object({ enabled: z.boolean(), propRadiusFactor: z.number().positive(), wallCeilingEffectEnabled: z.boolean() }).strict(),
  downwash: z.object({ enabled: z.boolean(), strength: z.number().nonnegative(), verticalRange: z.number().positive() }).strict(),
  magneticField: magneticFieldConfigSchema,
  scene: sceneConfigSchema,
}).strict();

// ── Sensor suite config ──

const imuConfigSchema = z.object({
  enabled: z.boolean(),
  rate: z.number().positive(),
  gyroNoiseDensity: z.number().nonnegative(),
  gyroBiasRW: z.number().nonnegative(),
  gyroSaturation: z.number().positive(),
  accelNoiseDensity: z.number().nonnegative(),
  accelBiasRW: z.number().nonnegative(),
  accelSaturation: z.number().positive(),
}).strict();

const magSensorConfigSchema = z.object({
  enabled: z.boolean(),
  rate: z.number().positive(),
  noiseDensity: z.number().nonnegative(),
  hardIron: vec3Schema,
  softIron: mat3Schema,
  emiCoefficient: z.number().nonnegative(),
  dropoutProbability: z.number().min(0).max(1),
}).strict();

const baroConfigSchema = z.object({
  enabled: z.boolean(),
  rate: z.number().positive(),
  noiseDensity: z.number().nonnegative(),
  biasRW: z.number().nonnegative(),
  biasInitial: z.number(),
}).strict();

const rangeConfigSchema = z.object({
  enabled: z.boolean(),
  rate: z.number().positive(),
  noise: z.number().nonnegative(),
  minRange: z.number().nonnegative(),
  maxRange: z.number().positive(),
  maxIncidenceAngle: z.number().positive(),
  dropoutProbability: z.number().min(0).max(1),
}).strict();

const flowConfigSchema = z.object({
  enabled: z.boolean(),
  rate: z.number().positive(),
  noiseBase: z.number().nonnegative(),
  heightScaleCoeff: z.number().nonnegative(),
  minHeight: z.number().nonnegative(),
  maxHeight: z.number().positive(),
  textureThreshold: z.number().min(0).max(1),
  dropoutProbability: z.number().min(0).max(1),
}).strict();

const cameraVIOConfigSchema = z.object({
  enabled: z.boolean(),
  rate: z.number().positive(),
  positionNoiseBase: z.number().nonnegative(),
  attitudeNoiseBase: z.number().nonnegative(),
  featureThreshold: z.number().min(0).max(1),
  maxAngularRateForQuality: z.number().positive(),
  maxHeightForQuality: z.number().positive(),
  latencyFrames: z.number().int().nonnegative(),
  dropoutProbability: z.number().min(0).max(1),
  cameraIntrinsicsEnabled: z.boolean(),
  focalLength: z.number().positive(),
  principalPoint: z.tuple([z.number(), z.number()]),
  imageSize: z.tuple([z.number().positive(), z.number().positive()]),
  distortionK1: z.number(),
  extrinsicsRotation: z.tuple([z.number(), z.number(), z.number()]),
  extrinsicsTranslation: z.tuple([z.number(), z.number(), z.number()]),
  vioDriftEnabled: z.boolean(),
  vioScaleBiasRW: z.number().nonnegative(),
  vioYawDriftRW: z.number().nonnegative(),
  vioPositionRW: z.number().nonnegative(),
}).strict();

const uwbSensorConfigSchema = z.object({
  enabled: z.boolean(),
  rate: z.number().positive(),
  losNoiseStdDev: z.number().nonnegative(),
  losBias: z.number().nonnegative(),
  nlosBiasBase: z.number().nonnegative(),
  nlosBiasStdDev: z.number().nonnegative(),
  nlosNoiseStdDev: z.number().nonnegative(),
  packetLossProbability: z.number().min(0).max(1),
  minRange: z.number().nonnegative(),
  maxRange: z.number().positive(),
}).strict();

const sensorSuiteConfigSchema = z.object({
  imu: imuConfigSchema,
  magnetometer: magSensorConfigSchema,
  barometer: baroConfigSchema,
  rangefinder: rangeConfigSchema,
  opticalFlow: flowConfigSchema,
  cameraVIO: cameraVIOConfigSchema.optional(),
  uwb: uwbSensorConfigSchema.optional(),
}).strict();

// ── Communication config ──

const commConfigSchema = z.object({
  range: z.number().positive(),
  latencyMean: z.number().nonnegative(),
  latencyJitter: z.number().nonnegative(),
  dropRate: z.number().min(0).max(1),
  bandwidth: z.number().positive(),
  estimatePublishRate: z.number().positive(),
  maneuverAccelStdDev: z.number().nonnegative(),
  estimateTimeout: z.number().positive(),
  losProjectedVariance: z.boolean(),
}).strict();

const formationConfigSchema = z.object({
  enabled: z.boolean(),
  mode: z.enum(['leader-follower', 'consensus']),
  topology: z.enum(['line', 'grid', 'circle']),
  spacing: z.number().positive(),
  leaderDroneId: z.number().int().nonnegative(),
  consensusGain: z.number().min(0).max(1),
  leaderLossFallback: z.enum(['hover', 'reelect']),
  offsetFrame: z.enum(['world', 'heading']),
}).strict();

const noFlyZoneSchema = z.object({
  min: z.tuple([z.number(), z.number(), z.number()]),
  max: z.tuple([z.number(), z.number(), z.number()]),
}).strict();

const safetyConfigSchema = z.object({
  enabled: z.boolean(),
  minSeparation: z.number().positive(),
  orcaRadius: z.number().nonnegative(),
  orcaTimeHorizon: z.number().positive(),
  maxSpeed: z.number().positive(),
  minAltitude: z.number().nonnegative(),
  emergencyStopDistance: z.number().nonnegative(),
  noFlyZones: z.array(noFlyZoneSchema),
}).strict();

const swarmConfigSchema = z.object({
  droneCount: z.number().int().positive(),
  initialPattern: z.enum(['explicit', 'line', 'grid', 'circle']),
  initialPositions: z.array(vec3Schema).optional(),
  patternSpacing: z.number().positive(),
  patternCenter: vec3Schema,
  communication: commConfigSchema,
  formation: formationConfigSchema,
  safety: safetyConfigSchema,
}).strict();

// ── Top-level sim config ──

export const simConfigSchema = z.object({
  seed: z.number().int().nonnegative(),
  physicsDt: z.number().positive().max(0.01),
  commandRate: z.number().positive().max(10000),
  telemetryRate: z.number().positive().max(1000),
  maxSubSteps: z.number().int().positive(),
  timeScale: z.number().positive().max(100),
  drone: droneParamsSchema,
  environment: environmentConfigSchema,
  sensors: sensorSuiteConfigSchema,
  estimation: z.object({
    gyroNoisePSD: z.number().nonnegative(),
    accelNoisePSD: z.number().nonnegative(),
    gyroBiasRWPSD: z.number().nonnegative(),
    accelBiasRWPSD: z.number().nonnegative(),
    baroBiasRWPSD: z.number().nonnegative(),
    magNoiseVar: z.number().nonnegative(),
    baroNoiseVar: z.number().nonnegative(),
    rangeNoiseVar: z.number().nonnegative(),
    flowNoiseVar: z.number().nonnegative(),
    initPosVar: z.number().positive(),
    initVelVar: z.number().positive(),
    initAttVar: z.number().positive(),
    initGyroBiasVar: z.number().positive(),
    initAccelBiasVar: z.number().positive(),
    initBaroBiasVar: z.number().positive(),
    earthMagField: vec3Schema,
    magGateThreshold: z.number().positive(),
    rangeGateThreshold: z.number().positive(),
    flowGateThreshold: z.number().positive(),
    vioPositionNoiseVar: z.number().nonnegative(),
    vioAttitudeNoiseVar: z.number().nonnegative(),
    vioGateThreshold: z.number().positive(),
    uwbLosNoiseVar: z.number().nonnegative(),
    uwbNlosNoiseVar: z.number().nonnegative(),
    uwbGateThreshold: z.number().positive(),
    innovationNLOSDetection: z.boolean(),
    nlosInnovThreshold: z.number().positive(),
    nlosConsecutiveCount: z.number().int().positive(),
    alignAttVarThreshold: z.number().positive(),
    alignAltVarThreshold: z.number().positive(),
    alignMinUpdates: z.number().int().nonnegative(),
    attHealthThreshold: z.number().positive(),
    vertHealthThreshold: z.number().positive(),
    horizHealthThreshold: z.number().positive(),
  }).strict(),
  swarm: swarmConfigSchema,
}).strict();

export type DroneParamsInput = z.input<typeof droneParamsSchema>;
export type SimConfigInput = z.input<typeof simConfigSchema>;

/** Validate and return parsed config, or throw with details. */
export function validateSimConfig(input: unknown): z.infer<typeof simConfigSchema> {
  return simConfigSchema.parse(input);
}

/** Validate partial drone params for hot updates. */
export function validatePartialDroneParams(input: unknown): Partial<z.infer<typeof droneParamsSchema>> {
  return droneParamsSchema.partial().parse(input);
}
