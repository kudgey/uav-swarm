/**
 * Camera/VIO sensor: abstracted pose measurement.
 * STATUS: simplified — no camera model, lens distortion, or feature tracking.
 *
 * Quality depends on: scene texture, angular rate (motion blur), height.
 * When quality < threshold → invalid → EKF gets no update → drift.
 */

import { v3Create, v3Len, clamp } from '@lib/math';
import { qCreate, qMultiply, qNormalize, qRotateVectorInverse } from '@sim/physics/quaternion';
import type { Vec3, Quat, DroneState, EnvironmentOutput, CameraVIOSensorConfig } from '@sim/core/types';
import type { DeterministicRNG } from '@sim/core/rng';
import type { WorldGeometry } from '@sim/environment/world-geometry';
import type { SensorOutput, VIOMeasurement } from './sensor-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('sensor-camera-vio', 'simplified',
  'Abstracted pose measurement, no camera model/lens/features', {
    simplifications: ['No camera intrinsics or distortion',
      'No feature tracking or map maintenance',
      'Noise model is quality-dependent Gaussian, not feature-based'],
  });

// Pre-allocated scratch
const _posOut = v3Create();
const _quatOut = qCreate();
const _pertQuat = qCreate();

/** Persistent VIO odometric state — maintained across ticks by SensorManager. */
export interface VIODriftState {
  position: Vec3;      // integrated VIO position (world NED)
  biasScale: number;   // velocity scale bias (dimensionless, ~0)
  yawDrift: number;    // accumulated heading error (rad)
  lastTime: number;    // last update simTime
  initialized: boolean;
}

export function createVIODriftState(initialPos: Vec3): VIODriftState {
  return {
    position: v3Create(initialPos[0], initialPos[1], initialPos[2]),
    biasScale: 0,
    yawDrift: 0,
    lastTime: 0,
    initialized: false,
  };
}

export function readCameraVIO(
  state: DroneState,
  env: EnvironmentOutput,
  config: CameraVIOSensorConfig,
  rng: DeterministicRNG,
  simTime: number,
  worldGeo?: WorldGeometry,
  vioDrift?: VIODriftState,
): SensorOutput<VIOMeasurement> {
  const invalid: SensorOutput<VIOMeasurement> = {
    timestamp: simTime, valid: false,
    measurement: { position: _posOut, quaternion: _quatOut, featureQuality: 0 },
  };

  if (!config.enabled) return invalid;

  // Dropout
  if (rng.next() < config.dropoutProbability) return invalid;

  // Compute feature quality
  let featureQuality: number;

  if (config.cameraIntrinsicsEnabled && worldGeo) {
    featureQuality = computeProjectedFeatureQuality(state, config, worldGeo);
  } else {
    const angRate = v3Len(state.angularVelocity);
    const h = env.heightAboveGround;
    const motionFactor = clamp(1 - angRate / config.maxAngularRateForQuality, 0, 1);
    const heightFactor = clamp(1 - h / config.maxHeightForQuality, 0.1, 1);
    featureQuality = env.surfaceTextureQuality * motionFactor * heightFactor;
  }

  if (featureQuality < config.featureThreshold) return invalid;

  // Quality-dependent noise scaling
  const invQ = 1 / Math.max(0.01, featureQuality);
  const sigmaA = config.attitudeNoiseBase * invQ;

  // ── Position: odometric drift model or legacy absolute ──
  if (config.vioDriftEnabled && vioDrift) {
    // Initialize on first valid reading
    if (!vioDrift.initialized) {
      vioDrift.position[0] = state.position[0];
      vioDrift.position[1] = state.position[1];
      vioDrift.position[2] = state.position[2];
      vioDrift.lastTime = simTime;
      vioDrift.initialized = true;
    }

    const dt = simTime - vioDrift.lastTime;
    if (dt > 0) {
      // True displacement in world frame (NED)
      const dpX = state.velocity[0] * dt;
      const dpY = state.velocity[1] * dt;
      const dpZ = state.velocity[2] * dt;

      // Scale error: measured = true * (1 + biasScale)
      const s = 1 + vioDrift.biasScale;
      const sdpX = dpX * s;
      const sdpY = dpY * s;
      const sdpZ = dpZ * s;

      // Yaw drift rotation around NED z-axis
      const cy = Math.cos(vioDrift.yawDrift);
      const sy = Math.sin(vioDrift.yawDrift);
      const rdpX = cy * sdpX - sy * sdpY;
      const rdpY = sy * sdpX + cy * sdpY;
      const rdpZ = sdpZ; // z unaffected by yaw

      // All drift components scale inversely with feature quality.
      // Quality=1 → baseline rate, quality=0.1 → ~3x, quality=0.05 → ~4.5x.
      // This captures real VIO behavior: feature-poor scenes drift faster.
      const qClamped = Math.max(0.05, featureQuality);
      const qualityScale = 1.0 / Math.sqrt(qClamped);

      const sqrtDt = Math.sqrt(dt);
      const sigmaRW = config.vioPositionRW * qualityScale;

      // Integrate
      vioDrift.position[0] += rdpX + sigmaRW * sqrtDt * rng.gaussian();
      vioDrift.position[1] += rdpY + sigmaRW * sqrtDt * rng.gaussian();
      vioDrift.position[2] += rdpZ + sigmaRW * sqrtDt * rng.gaussian();

      // Evolve biases (scale with quality too — poor features → faster scale/yaw drift)
      vioDrift.biasScale += config.vioScaleBiasRW * qualityScale * sqrtDt * rng.gaussian();
      vioDrift.yawDrift += config.vioYawDriftRW * qualityScale * sqrtDt * rng.gaussian();

      vioDrift.lastTime = simTime;
    }

    _posOut[0] = vioDrift.position[0];
    _posOut[1] = vioDrift.position[1];
    _posOut[2] = vioDrift.position[2];
  } else {
    // Legacy: absolute position + gaussian noise
    const sigmaP = config.positionNoiseBase * invQ;
    _posOut[0] = state.position[0] + sigmaP * rng.gaussian();
    _posOut[1] = state.position[1] + sigmaP * rng.gaussian();
    _posOut[2] = state.position[2] + sigmaP * rng.gaussian();
  }

  // Noisy attitude: q_vio = q_true * [1, δθ/2]
  const halfSigma = sigmaA * 0.5;
  _pertQuat[0] = 1;
  _pertQuat[1] = halfSigma * rng.gaussian();
  _pertQuat[2] = halfSigma * rng.gaussian();
  _pertQuat[3] = halfSigma * rng.gaussian();
  qMultiply(_quatOut, state.quaternion, _pertQuat);
  qNormalize(_quatOut, _quatOut);

  return {
    timestamp: simTime,
    valid: true,
    measurement: { position: _posOut, quaternion: _quatOut, featureQuality },
  };
}

const _pCam = v3Create();

/** Project scene landmarks into camera and count visible features. */
function computeProjectedFeatureQuality(
  state: DroneState,
  config: CameraVIOSensorConfig,
  worldGeo: WorldGeometry,
): number {
  const fx = config.focalLength;
  const [cx, cy] = config.principalPoint;
  const [imgW, imgH] = config.imageSize;
  const k1 = config.distortionK1;
  const expectedFeatures = 20; // baseline for quality=1

  // Collect scene landmarks: AABB corners + floor grid (LOCAL WINDOW only)
  const LOCAL_RADIUS = 30; // only project landmarks within 30m of drone
  const droneX = state.position[0], droneY = state.position[1];
  const landmarks: Vec3[] = [];

  // Obstacle corners within local radius
  for (const obs of worldGeo.getObstacles()) {
    // Quick AABB distance check
    const nearX = clamp(droneX, obs.min[0], obs.max[0]);
    const nearY = clamp(droneY, obs.min[1], obs.max[1]);
    if ((nearX - droneX) ** 2 + (nearY - droneY) ** 2 > LOCAL_RADIUS * LOCAL_RADIUS) continue;
    for (let dx = 0; dx <= 1; dx++) {
      for (let dy = 0; dy <= 1; dy++) {
        for (let dz = 0; dz <= 1; dz++) {
          landmarks.push(v3Create(
            dx ? obs.max[0] : obs.min[0],
            dy ? obs.max[1] : obs.min[1],
            dz ? obs.max[2] : obs.min[2],
          ));
        }
      }
    }
  }

  // Floor grid: local window only (max ~900 landmarks regardless of scene size)
  const gridStep = 2;
  const xMin = Math.floor((droneX - LOCAL_RADIUS) / gridStep) * gridStep;
  const xMax = Math.ceil((droneX + LOCAL_RADIUS) / gridStep) * gridStep;
  const yMin = Math.floor((droneY - LOCAL_RADIUS) / gridStep) * gridStep;
  const yMax = Math.ceil((droneY + LOCAL_RADIUS) / gridStep) * gridStep;
  for (let x = xMin; x <= xMax; x += gridStep) {
    for (let y = yMin; y <= yMax; y += gridStep) {
      landmarks.push(v3Create(x, y, 0));
    }
  }

  // Transform each landmark to camera frame and project
  let visibleCount = 0;
  for (const lm of landmarks) {
    // World to body: p_body = R_bw^T * (p_world - p_drone)
    const dx = lm[0] - state.position[0];
    const dy = lm[1] - state.position[1];
    const dz = lm[2] - state.position[2];
    const pBody = v3Create(0, 0, 0);
    qRotateVectorInverse(pBody, state.quaternion, v3Create(dx, dy, dz));
    // Body to camera: apply extrinsics (Euler ZYX rotation + translation)
    const [rx, ry, rz] = config.extrinsicsRotation;
    const [tx, ty, tz] = config.extrinsicsTranslation;
    // Apply translation offset in body frame
    const pbx = pBody[0] - tx, pby = pBody[1] - ty, pbz = pBody[2] - tz;
    // Full Euler ZYX rotation matrix R_cb
    const cx = Math.cos(rx), sx = Math.sin(rx);
    const cy = Math.cos(ry), sy = Math.sin(ry);
    const cz = Math.cos(rz), sz = Math.sin(rz);
    _pCam[0] = (cy * cz) * pbx + (cy * sz) * pby + (-sy) * pbz;
    _pCam[1] = (sx * sy * cz - cx * sz) * pbx + (sx * sy * sz + cx * cz) * pby + (sx * cy) * pbz;
    _pCam[2] = (cx * sy * cz + sx * sz) * pbx + (cx * sy * sz - sx * cz) * pby + (cx * cy) * pbz;

    // Camera convention: z forward, x right, y down (for downward camera, body z = camera z)
    const z = _pCam[2];
    if (z <= 0.1) continue; // behind camera

    // Pinhole projection
    let u = fx * _pCam[0] / z + cx;
    let v = fx * _pCam[1] / z + cy;

    // Radial distortion
    if (k1 !== 0) {
      const r2 = ((u - cx) / fx) ** 2 + ((v - cy) / fx) ** 2;
      const distFactor = 1 + k1 * r2;
      u = (u - cx) * distFactor + cx;
      v = (v - cy) * distFactor + cy;
    }

    // Check image bounds
    if (u >= 0 && u < imgW && v >= 0 && v < imgH) {
      visibleCount++;
    }
  }

  return clamp(visibleCount / expectedFeatures, 0, 1);
}
