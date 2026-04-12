/**
 * Core type definitions for the UAV swarm simulator.
 *
 * Vec3 = Float64Array(3), Quat = Float64Array(4) — for zero-copy and cache efficiency.
 * Quaternion convention: Hamilton, scalar-first [w, x, y, z].
 */

/** 3-component vector stored as Float64Array(3). Indices: 0=x, 1=y, 2=z. */
export type Vec3 = Float64Array;

/** Hamilton quaternion [w, x, y, z] stored as Float64Array(4). Scalar-first. */
export type Quat = Float64Array;

/** 3x3 matrix stored as Float64Array(9), column-major. */
export type Mat3 = Float64Array;

// ── Drone parameters ──

export interface DroneParams {
  mass: number;                  // kg
  inertia: Vec3;                 // [Jxx, Jyy, Jzz] kg*m^2 (diagonal)
  armLength: number;             // m, center-to-rotor
  numRotors: number;
  rotorPositions: Vec3[];        // body-frame positions of each rotor
  rotorDirections: number[];     // +1=CCW, -1=CW (viewed from above in NED)
  kT: number;                    // thrust coeff, N/(rad/s)^2
  kQ: number;                    // drag torque coeff, N*m/(rad/s)^2
  motorTau: number;              // first-order time constant, s
  motorOmegaMin: number;         // rad/s
  motorOmegaMax: number;         // rad/s
  motorRateLimit: number;        // rad/s^2
  motorDeadZone: number;         // rad/s
  dragCoeffLinear: Vec3;         // body-frame linear drag [Dx, Dy, Dz] N/(m/s)
  dragCoeffQuadratic: Vec3;      // body-frame quadratic drag [Cx, Cy, Cz]
  dragMode: 'linear' | 'quadratic' | 'combined';
  dragCoeffRotor: Vec3;          // rotor drag [Dr_x, Dr_y, Dr_z] N/(m/s) (Faessler et al.). [0,0,0] = off.
  rotorInertia: number;          // kg*m^2, single rotor. 0 = no gyroscopic torque.
}

// ── Drone state ──

export interface DroneState {
  /** World-frame position (NED) */
  position: Vec3;
  /** World-frame velocity (NED) */
  velocity: Vec3;
  /** Body-to-world quaternion, Hamilton scalar-first [w,x,y,z] */
  quaternion: Quat;
  /** Body-frame angular velocity [p, q, r] rad/s */
  angularVelocity: Vec3;
  /** Current rotor angular speeds rad/s */
  motorSpeeds: Float64Array;
  /** Commanded rotor speeds rad/s (motor model input) */
  motorCommands: Float64Array;
}

/** State derivative for integrator */
export interface DroneStateDerivative {
  dPosition: Vec3;
  dVelocity: Vec3;
  dQuaternion: Quat;
  dAngularVelocity: Vec3;
  dMotorSpeeds: Float64Array;
}

// ── Simulation config ──

export interface SimConfig {
  seed: number;
  physicsDt: number;        // s, e.g. 0.001 (1000 Hz)
  commandRate: number;       // Hz, e.g. 250
  telemetryRate: number;     // Hz, e.g. 60
  maxSubSteps: number;       // cap physics steps per frame
  timeScale: number;         // 1.0 = real-time
  drone: DroneParams;
  environment: EnvironmentConfig;
  sensors: SensorSuiteConfig;
  estimation: import('@sim/estimation/estimator-types').EKFConfig;
  swarm: SwarmConfig;
}

export interface SwarmConfig {
  droneCount: number;
  initialPattern: 'explicit' | 'line' | 'grid' | 'circle';
  initialPositions?: Vec3[];    // required when pattern='explicit'
  patternSpacing: number;       // m
  patternCenter: Vec3;          // NED center for pattern generation
  communication: CommConfig;
  formation: FormationConfigType;
  safety: SafetyConfigType;
}

export interface SafetyConfigType {
  enabled: boolean;
  minSeparation: number;
  orcaRadius: number;
  orcaTimeHorizon: number;
  maxSpeed: number;
  minAltitude: number;
  emergencyStopDistance: number;
  noFlyZones: { min: [number, number, number]; max: [number, number, number] }[];
}

export interface FormationConfigType {
  enabled: boolean;
  mode: 'leader-follower' | 'consensus';
  topology: 'line' | 'grid' | 'circle';
  spacing: number;
  leaderDroneId: number;
  consensusGain: number;
  leaderLossFallback: 'hover' | 'reelect';
  offsetFrame: 'world' | 'heading'; // Phase 11: heading-aligned offsets
}

export interface CommConfig {
  range: number;                // m, max communication distance
  latencyMean: number;          // s
  latencyJitter: number;        // s (std dev)
  dropRate: number;             // 0-1
  bandwidth: number;            // messages per second per link
  estimatePublishRate: number;  // Hz
  maneuverAccelStdDev: number;  // m/s², for inter-drone UWB variance
  estimateTimeout: number;      // s, expire stale neighbor estimates
  losProjectedVariance: boolean; // Phase 11: send P_3x3, compute LOS-projected R_eff
}

// ── Environment config (see environment-types.ts for details) ──

export interface EnvironmentConfig {
  wind: WindConfig;
  turbulence: TurbulenceConfig;
  groundEffect: { enabled: boolean; propRadiusFactor: number; wallCeilingEffectEnabled: boolean };
  downwash: { enabled: boolean; strength: number; verticalRange: number };
  magneticField: MagneticFieldConfig;
  scene: SceneConfig;
}

export interface WindConfig {
  meanSpeed: number;        // m/s
  meanDirection: Vec3;      // unit vector, world frame (NED)
  shearEnabled: boolean;
  shearExponent: number;    // power-law exponent (~0.14 for open terrain)
  gustEvents: GustEvent[];
}

export interface GustEvent {
  onsetTime: number;        // s
  duration: number;         // s
  peakAmplitude: Vec3;      // world-frame m/s
}

export interface TurbulenceConfig {
  enabled: boolean;
  intensity: number;        // m/s RMS
  correlationTime: number;  // s
  spatialCorrelation: boolean;     // Phase 11: lattice OU field
  integralLengthScale: number;     // m, spatial cell size (default 5)
}

export interface MagneticFieldConfig {
  fieldStrength: number;    // T (~50e-6 for Earth)
  inclination: number;      // rad (positive = field points into ground in NED)
  declination: number;      // rad (positive = east of north)
  anomalies: MagAnomaly[];
  wmmEnabled: boolean;      // Phase 11: compute field from lat/lon/date
  latitude: number;         // degrees N
  longitude: number;        // degrees E
  wmmDate: number;          // decimal year (e.g., 2025.0)
}

export interface MagAnomaly {
  position: Vec3;           // world-frame NED
  radius: number;           // m
  strength: Vec3;           // perturbation (T)
}

export interface SceneConfig {
  preset: 'open_field' | 'indoor_basic' | 'warehouse' | 'custom';
  surfaceTextureQuality: number;  // 0-1 default for floor
  obstacles: ObstacleDef[];
  uwbAnchors?: UWBAnchorDef[];
  sceneBounds?: { min: [number, number, number]; max: [number, number, number] }; // NED operational volume
}

export interface UWBAnchorDef {
  id: string;
  position: Vec3;  // NED world frame
}

export interface ObstacleDef {
  min: Vec3;  // NED AABB min corner
  max: Vec3;  // NED AABB max corner
  textureQuality?: number;  // per-surface override
}

// ── Sensor suite config ──

export interface SensorSuiteConfig {
  imu: ImuSensorConfig;
  magnetometer: MagSensorConfig;
  barometer: BaroSensorConfig;
  rangefinder: RangeSensorConfig;
  opticalFlow: FlowSensorConfig;
  cameraVIO?: CameraVIOSensorConfig;
  uwb?: UWBSensorConfig;
}

export interface CameraVIOSensorConfig {
  enabled: boolean;
  rate: number;                      // Hz
  positionNoiseBase: number;         // m (base sigma at quality=1)
  attitudeNoiseBase: number;         // rad (base sigma at quality=1)
  featureThreshold: number;          // 0-1, below this → invalid
  maxAngularRateForQuality: number;  // rad/s, blur degrades above
  maxHeightForQuality: number;       // m, fewer features at altitude
  latencyFrames: number;             // pipeline delay in frames
  dropoutProbability: number;
  // Phase 11: camera intrinsics
  cameraIntrinsicsEnabled: boolean;
  focalLength: number;               // pixels
  principalPoint: [number, number];  // [cx, cy] pixels
  imageSize: [number, number];       // [width, height] pixels
  distortionK1: number;              // radial distortion
  extrinsicsRotation: [number, number, number];    // Euler ZYX rad (camera-to-body)
  extrinsicsTranslation: [number, number, number]; // body-frame offset m
  // VIO odometric drift model
  vioDriftEnabled: boolean;       // false = absolute position (legacy), true = integrated odometry
  vioScaleBiasRW: number;         // scale bias random walk σ (default 0.001)
  vioYawDriftRW: number;          // heading drift random walk σ rad/√s (default 0.0005)
  vioPositionRW: number;          // position random walk σ m/√s (default 0.02)
}

export interface UWBSensorConfig {
  enabled: boolean;
  rate: number;                  // Hz per anchor
  losNoiseStdDev: number;        // m
  losBias: number;               // m (small positive)
  nlosBiasBase: number;          // m (mean of folded normal)
  nlosBiasStdDev: number;        // m (std of folded normal)
  nlosNoiseStdDev: number;       // m (larger than LOS)
  packetLossProbability: number;
  minRange: number;              // m
  maxRange: number;              // m
}

export interface ImuSensorConfig {
  enabled: boolean;
  rate: number;  // Hz
  gyroNoiseDensity: number;     // rad/s/sqrt(Hz)
  gyroBiasRW: number;           // rad/s^2/sqrt(Hz)
  gyroSaturation: number;       // rad/s
  accelNoiseDensity: number;    // m/s^2/sqrt(Hz)
  accelBiasRW: number;          // m/s^3/sqrt(Hz)
  accelSaturation: number;      // m/s^2
}

export interface MagSensorConfig {
  enabled: boolean;
  rate: number;
  noiseDensity: number;         // T/sqrt(Hz)
  hardIron: Vec3;               // body-frame bias (T)
  softIron: Mat3;               // 3x3 distortion matrix
  emiCoefficient: number;       // T/(rad/s)^2 from motors
  dropoutProbability: number;
}

export interface BaroSensorConfig {
  enabled: boolean;
  rate: number;
  noiseDensity: number;         // Pa/sqrt(Hz)
  biasRW: number;               // Pa/sqrt(s)
  biasInitial: number;          // Pa
}

export interface RangeSensorConfig {
  enabled: boolean;
  rate: number;
  noise: number;                // m
  minRange: number;             // m
  maxRange: number;             // m
  maxIncidenceAngle: number;    // rad
  dropoutProbability: number;
}

export interface FlowSensorConfig {
  enabled: boolean;
  rate: number;
  noiseBase: number;            // rad/s
  heightScaleCoeff: number;     // noise scaling with height
  minHeight: number;            // m
  maxHeight: number;            // m
  textureThreshold: number;     // minimum surfaceTextureQuality for valid output
  dropoutProbability: number;
}

// ── Environment output ──

export interface EnvironmentOutput {
  wind: Vec3;                    // world-frame wind velocity m/s
  gravity: Vec3;                 // world-frame gravity vector m/s^2
  airDensity: number;            // kg/m^3
  pressure: number;              // Pa
  temperature: number;           // K
  earthMagneticField: Vec3;      // world-frame earth+anomaly field (T), excludes motor EMI
  heightAboveGround: number;     // m, >= 0 (clamped, 0 = on/below ground)
  groundEffectMultiplier: number; // >= 1.0
  downwashMultiplier: number;    // <= 1.0 (0.5..1.0), thrust reduction from drones above
  surfaceTextureQuality: number; // 0-1, from scene config / surface below drone
}

// ── Parameter mutability ──

/** Parameters that can be changed without reset */
export type HotUpdatableParams = Pick<DroneParams,
  'mass' | 'dragCoeffLinear' | 'dragCoeffQuadratic' | 'dragMode' |
  'motorTau' | 'motorOmegaMax' | 'motorDeadZone' | 'motorRateLimit' |
  'kT' | 'kQ' | 'motorOmegaMin'
>;

/** Parameters that require simulation reset */
export type ResetRequiredParams = Pick<DroneParams,
  'inertia' | 'armLength' | 'numRotors' | 'rotorPositions' | 'rotorDirections'
>;
