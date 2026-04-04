/**
 * Gravity model. Phase 1: constant NED gravity.
 * STATUS: experimental (constant model, no altitude dependence)
 */

import type { Vec3 } from '@sim/core/types';
import { GRAVITY_MPS2 } from '@sim/core/frames';
import { v3Set } from '@lib/math';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('gravity', 'experimental', 'Constant NED gravity, no altitude variation');

/** Write gravity vector into out. NED: [0, 0, +g]. */
export function getGravity(out: Vec3): Vec3 {
  return v3Set(out, 0, 0, GRAVITY_MPS2);
}
