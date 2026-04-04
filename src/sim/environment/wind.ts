/**
 * Wind field model: mean wind + gust events + turbulence.
 * STATUS: experimental (mean + gust), turbulence plugged in from separate module
 *
 * w(p,t) = w_mean(p) + w_gust(t) + w_turb(t)
 *
 * Mean wind: constant direction + speed, optional altitude shear (power-law).
 * Gusts: one-minus-cosine envelope (FAA standard).
 * Turbulence: plugged in via setTurbulence() from turbulence.ts.
 */

import { v3Create, v3Zero, v3ScaleAdd } from '@lib/math';
import type { Vec3, WindConfig, GustEvent } from '@sim/core/types';
import { registerSubsystem } from '@sim/core/status-labels';
import type { TurbulenceGenerator } from './turbulence';

registerSubsystem('wind', 'experimental', 'Mean wind + gust events, altitude shear');

// Pre-allocated scratch
const _meanOut = v3Create();
const _gustOut = v3Create();

export class WindField {
  private config: WindConfig;
  private turbulence: TurbulenceGenerator | null = null;

  constructor(config: WindConfig) {
    this.config = config;
  }

  setTurbulence(turb: TurbulenceGenerator): void {
    this.turbulence = turb;
  }

  updateConfig(config: WindConfig): void {
    this.config = config;
  }

  /**
   * Evaluate total wind at a position and time. Writes into out.
   * Height convention: h = -position[2] (NED z=0 is floor).
   */
  evaluate(out: Vec3, position: Vec3, simTime: number): Vec3 {
    const cfg = this.config;

    // Mean wind with optional altitude shear
    const h = Math.max(0, -position[2]);
    let speedScale = cfg.meanSpeed;
    if (cfg.shearEnabled && h > 0) {
      // Power-law wind profile: V(h) = V_ref * (h / h_ref)^alpha
      // Using h_ref = 10m as reference height
      speedScale = cfg.meanSpeed * Math.pow(Math.max(h, 0.1) / 10.0, cfg.shearExponent);
    }
    _meanOut[0] = cfg.meanDirection[0] * speedScale;
    _meanOut[1] = cfg.meanDirection[1] * speedScale;
    _meanOut[2] = cfg.meanDirection[2] * speedScale;

    // Start with mean wind
    out[0] = _meanOut[0]; out[1] = _meanOut[1]; out[2] = _meanOut[2];

    // Add gust events
    for (const gust of cfg.gustEvents) {
      this.evaluateGust(_gustOut, gust, simTime);
      out[0] += _gustOut[0]; out[1] += _gustOut[1]; out[2] += _gustOut[2];
    }

    // Add turbulence (if connected)
    if (this.turbulence) {
      const turb = this.turbulence.getOutput();
      out[0] += turb[0]; out[1] += turb[1]; out[2] += turb[2];
    }

    return out;
  }

  private evaluateGust(out: Vec3, gust: GustEvent, simTime: number): Vec3 {
    const tLocal = simTime - gust.onsetTime;
    if (tLocal < 0 || tLocal > gust.duration) {
      v3Zero(out);
      return out;
    }
    // One-minus-cosine envelope: 0.5 * (1 - cos(pi * t / T))
    const envelope = 0.5 * (1 - Math.cos(Math.PI * tLocal / gust.duration));
    out[0] = gust.peakAmplitude[0] * envelope;
    out[1] = gust.peakAmplitude[1] * envelope;
    out[2] = gust.peakAmplitude[2] * envelope;
    return out;
  }
}
