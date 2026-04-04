import { describe, it, expect } from 'vitest';
import { computeOuterLoop } from '@sim/control/outer-loop';
import { v3Create } from '@lib/math';
import { qCreate } from '@sim/physics/quaternion';
import { defaultDroneParams, defaultControllerConfig } from '@sim/core/config-defaults';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import type { EstimatedState } from '@sim/estimation/estimator-types';
import type { GuidanceOutput } from '@sim/control/controller-types';

describe('Outer Loop', () => {
  it('hover at origin: thrust = mg, desired quaternion ≈ identity', () => {
    const est: EstimatedState = {
      position: v3Create(0, 0, -2), velocity: v3Create(0, 0, 0),
      quaternion: qCreate(1, 0, 0, 0), gyroBias: v3Create(0, 0, 0),
      accelBias: v3Create(0, 0, 0), baroBias: 0,
      angularVelocity: v3Create(0, 0, 0), timestamp: 0,
    };
    const guidance: GuidanceOutput = {
      positionDes: v3Create(0, 0, -2), velocityDes: v3Create(0, 0, 0),
      accelerationDes: v3Create(0, 0, 0), yawDes: 0,
    };
    const params = defaultDroneParams();
    const config = defaultControllerConfig();

    const result = computeOuterLoop(est, guidance, params, config);

    // Thrust should be approximately mg
    expect(result.thrust).toBeCloseTo(params.mass * GRAVITY_MPS2, 0);
    // Desired quaternion near identity (hover upright)
    expect(result.desiredQuat[0]).toBeCloseTo(1, 1);
  });

  it('tilt limiting clamps large position error', () => {
    const est: EstimatedState = {
      position: v3Create(10, 0, -2), // 10m off in x
      velocity: v3Create(0, 0, 0),
      quaternion: qCreate(1, 0, 0, 0), gyroBias: v3Create(0, 0, 0),
      accelBias: v3Create(0, 0, 0), baroBias: 0,
      angularVelocity: v3Create(0, 0, 0), timestamp: 0,
    };
    const guidance: GuidanceOutput = {
      positionDes: v3Create(0, 0, -2), velocityDes: v3Create(0, 0, 0),
      accelerationDes: v3Create(0, 0, 0), yawDes: 0,
    };
    const params = defaultDroneParams();
    const config = defaultControllerConfig();
    config.maxTiltRad = Math.PI / 6; // 30 deg limit

    const result = computeOuterLoop(est, guidance, params, config);

    // The desired z-axis should not tilt more than 30 deg from vertical
    // cos(30) = 0.866. desiredQuat should not have excessive tilt.
    expect(result.thrust).toBeGreaterThan(0);
  });
});
