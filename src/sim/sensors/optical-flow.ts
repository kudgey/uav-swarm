/**
 * Optical flow sensor.
 * STATUS: simplified — translation + rotation flow model, no camera projection.
 *
 * flow_x = vx_b / h + omega_y + noise_x
 * flow_y = vy_b / h - omega_x + noise_y
 *
 * Includes angular-rate-induced image motion. Pure rotation produces nonzero flow.
 * Noise scales with height. Invalid on featureless surface, height limits, dropout.
 *
 * Not camera-model-based: no lens distortion, rolling shutter, exposure integration.
 * Excluded from future estimator acceptance until upgraded to camera-based model.
 */

import { v3Create } from '@lib/math';
import { qRotateVectorInverse } from '@sim/physics/quaternion';
import type { Vec3, DroneState, EnvironmentOutput, FlowSensorConfig } from '@sim/core/types';
import type { DeterministicRNG } from '@sim/core/rng';
import type { SensorOutput, FlowMeasurement } from './sensor-types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('sensor-flow', 'simplified',
  'Translation + rotation flow model, no camera projection', {
    simplifications: [
      'No lens distortion or camera intrinsics',
      'No rolling shutter or exposure integration',
      'Height-only depth model (no per-pixel depth)',
    ],
  });

// Pre-allocated scratch
const _vRelWorld = v3Create();
const _vRelBody = v3Create();

export function readOpticalFlow(
  state: DroneState,
  env: EnvironmentOutput,
  config: FlowSensorConfig,
  rng: DeterministicRNG,
  simTime: number,
): SensorOutput<FlowMeasurement> {
  if (!config.enabled) {
    return { timestamp: simTime, valid: false, measurement: { flowX: 0, flowY: 0 } };
  }

  const h = env.heightAboveGround;

  // Height limits
  if (h < config.minHeight || h > config.maxHeight) {
    return { timestamp: simTime, valid: false, measurement: { flowX: 0, flowY: 0 } };
  }

  // Texture quality check
  if (env.surfaceTextureQuality < config.textureThreshold) {
    return { timestamp: simTime, valid: false, measurement: { flowX: 0, flowY: 0 } };
  }

  // Dropout
  if (config.dropoutProbability > 0 && rng.next() < config.dropoutProbability) {
    return { timestamp: simTime, valid: false, measurement: { flowX: 0, flowY: 0 } };
  }

  // Body-frame velocity (air-relative)
  _vRelWorld[0] = state.velocity[0] - env.wind[0];
  _vRelWorld[1] = state.velocity[1] - env.wind[1];
  _vRelWorld[2] = state.velocity[2] - env.wind[2];
  qRotateVectorInverse(_vRelBody, state.quaternion, _vRelWorld);

  // Body angular rates
  const omegaX = state.angularVelocity[0];
  const omegaY = state.angularVelocity[1];

  // Flow model: translation / height + rotation
  const flowXTrue = _vRelBody[0] / h + omegaY;
  const flowYTrue = _vRelBody[1] / h - omegaX;

  // Height-scaled noise
  const noiseEff = config.noiseBase * (1 + config.heightScaleCoeff * h);
  const flowX = flowXTrue + noiseEff * rng.gaussian();
  const flowY = flowYTrue + noiseEff * rng.gaussian();

  return {
    timestamp: simTime,
    valid: true,
    measurement: { flowX, flowY },
  };
}
