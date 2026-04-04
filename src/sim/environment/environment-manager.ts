/**
 * Environment manager: orchestrates all environment subsystems.
 * STATUS: simplified — environment evaluated once per physics step,
 * held constant across 4 RK4 sub-evaluations.
 *
 * At 1000Hz with v<20 m/s, position changes <0.02mm per step.
 * Ground effect varies on ~10cm (R_prop) scale → <0.02% error.
 * If near-ground scenarios need higher fidelity, reduce physicsDt.
 */

import { v3Create } from '@lib/math';
import { isaPressure, isaTemperature, isaDensity } from './atmosphere';
import { WindField } from './wind';
import { TurbulenceGenerator } from './turbulence';
import { computeGroundEffectMultiplier } from './ground-effect';
import { MagneticFieldModel } from './magnetic-field';
import { WorldGeometry } from './world-geometry';
import type { Vec3, EnvironmentOutput, EnvironmentConfig } from '@sim/core/types';
import type { DeterministicRNG } from '@sim/core/rng';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('environment-manager', 'simplified',
  'Env constant across RK4 sub-steps; see environment-manager.ts for analysis', {
    simplifications: ['Environment not recomputed per RK4 sub-evaluation'],
  });

// Pre-allocated scratch
const _windOut = v3Create();
const _magOut = v3Create();

export class EnvironmentManager {
  private wind: WindField;
  private turbulence: TurbulenceGenerator;
  private magField: MagneticFieldModel;
  private worldGeo: WorldGeometry;
  private config: EnvironmentConfig;
  private propRadius: number;

  constructor(config: EnvironmentConfig, rng: DeterministicRNG, armLength: number) {
    this.config = config;
    this.propRadius = armLength * config.groundEffect.propRadiusFactor;
    this.wind = new WindField(config.wind);
    this.turbulence = new TurbulenceGenerator(config.turbulence, rng);
    this.wind.setTurbulence(this.turbulence);
    this.magField = new MagneticFieldModel(config.magneticField);
    this.worldGeo = new WorldGeometry(config.scene);
    // Initialize spatial lattice if enabled
    if (config.turbulence.spatialCorrelation) {
      this.turbulence.initLattice(this.worldGeo.getBounds(), rng.nextU32());
    }
  }

  getWorldGeometry(): WorldGeometry {
    return this.worldGeo;
  }

  /** Live-update environment config. Propagates to all subsystems. */
  updateConfig(config: EnvironmentConfig, armLength: number): void {
    this.config = config;
    this.propRadius = armLength * config.groundEffect.propRadiusFactor;
    this.wind.updateConfig(config.wind);
    this.turbulence.updateConfig(config.turbulence);
    this.magField.updateConfig(config.magneticField);
    this.worldGeo.updateConfig(config.scene);
  }

  /**
   * Advance temporal state (turbulence). Call ONCE per physics step.
   * For multi-drone: call this once, then sampleAt() per drone.
   */
  stepTemporal(dt: number): void {
    this.turbulence.step(dt);
  }

  /**
   * Sample environment at a specific position. Stateless w.r.t. time —
   * call stepTemporal() first. Safe to call N times for N drones.
   */
  sampleAt(
    out: EnvironmentOutput,
    position: Vec3,
    simTime: number,
  ): void {
    const h = this.worldGeo.getHeightAboveGround(position);
    out.heightAboveGround = h;

    const absoluteAltitude = Math.max(0, -position[2]);
    out.pressure = isaPressure(absoluteAltitude);
    out.temperature = isaTemperature(absoluteAltitude);
    out.airDensity = isaDensity(absoluteAltitude);

    out.gravity[0] = 0; out.gravity[1] = 0; out.gravity[2] = 9.80665;

    this.wind.evaluate(_windOut, position, simTime);
    // If spatial turbulence enabled, replace global turbulence with position-sampled
    if (this.config.turbulence.spatialCorrelation) {
      const spatialTurb = this.turbulence.sampleAt(position);
      // Wind already added global turbulence via getOutput(); subtract it and add spatial
      const globalTurb = this.turbulence.getOutput();
      _windOut[0] += spatialTurb[0] - globalTurb[0];
      _windOut[1] += spatialTurb[1] - globalTurb[1];
      _windOut[2] += spatialTurb[2] - globalTurb[2];
    }
    out.wind[0] = _windOut[0]; out.wind[1] = _windOut[1]; out.wind[2] = _windOut[2];

    out.groundEffectMultiplier = this.config.groundEffect.enabled
      ? computeGroundEffectMultiplier(h, this.propRadius) : 1.0;

    this.magField.evaluate(_magOut, position);
    out.earthMagneticField[0] = _magOut[0];
    out.earthMagneticField[1] = _magOut[1];
    out.earthMagneticField[2] = _magOut[2];

    out.surfaceTextureQuality = this.worldGeo.getSurfaceTextureQuality(position);
  }

  /**
   * Legacy single-drone evaluate: stepTemporal + sampleAt in one call.
   * Kept for backward compatibility with existing tests.
   */
  evaluate(
    out: EnvironmentOutput,
    position: Vec3,
    _velocity: Vec3,
    _motorSpeeds: Float64Array,
    _numRotors: number,
    simTime: number,
    dt: number,
  ): void {
    this.stepTemporal(dt);
    this.sampleAt(out, position, simTime);
  }

  reset(rng: DeterministicRNG): void {
    this.turbulence.reset();
    // Recreate turbulence with fresh RNG fork
    this.turbulence = new TurbulenceGenerator(this.config.turbulence, rng);
    this.wind.setTurbulence(this.turbulence);
  }
}
