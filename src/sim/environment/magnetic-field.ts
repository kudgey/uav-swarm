/**
 * Earth magnetic field model.
 * STATUS: experimental
 *
 * Outputs WORLD-FRAME field only. Motor EMI is body-frame and lives in the
 * magnetometer sensor (sensor/magnetometer.ts).
 *
 * Earth field from inclination/declination:
 *   B_n = B * cos(inc) * cos(dec)
 *   B_e = B * cos(inc) * sin(dec)
 *   B_d = B * sin(inc)
 *
 * Anomaly zones: quadratic decay with distance.
 */

import { v3Create } from '@lib/math';
import type { Vec3, MagneticFieldConfig, MagAnomaly } from '@sim/core/types';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('magnetic-field', 'experimental',
  'Earth field from inclination/declination + anomaly zones');

// Pre-allocated scratch
const _anomalyDelta = v3Create();

export class MagneticFieldModel {
  private config: MagneticFieldConfig;
  private baseField: Vec3;

  constructor(config: MagneticFieldConfig) {
    this.config = config;
    this.baseField = v3Create();
    this.computeBaseField();
  }

  updateConfig(config: MagneticFieldConfig): void {
    this.config = config;
    this.computeBaseField();
  }

  private computeBaseField(): void {
    if (this.config.wmmEnabled) {
      // Simplified WMM: dipole model approximation from lat/lon
      const wmm = computeWMMDipole(this.config.latitude, this.config.longitude, this.config.wmmDate);
      this.baseField[0] = wmm.north;
      this.baseField[1] = wmm.east;
      this.baseField[2] = wmm.down;
      return;
    }
    const { fieldStrength, inclination, declination } = this.config;
    const cosInc = Math.cos(inclination);
    // NED: x=North, y=East, z=Down
    this.baseField[0] = fieldStrength * cosInc * Math.cos(declination);  // North
    this.baseField[1] = fieldStrength * cosInc * Math.sin(declination);  // East
    this.baseField[2] = fieldStrength * Math.sin(inclination);           // Down
  }

  /**
   * Evaluate world-frame magnetic field at a position.
   * Includes earth baseline + anomaly perturbations.
   * Excludes motor EMI (that's body-frame, handled by magnetometer sensor).
   */
  evaluate(out: Vec3, position: Vec3): Vec3 {
    // Start with baseline earth field
    out[0] = this.baseField[0];
    out[1] = this.baseField[1];
    out[2] = this.baseField[2];

    // Add anomaly perturbations
    for (const anomaly of this.config.anomalies) {
      this.addAnomaly(out, position, anomaly);
    }

    return out;
  }

  private addAnomaly(out: Vec3, position: Vec3, anomaly: MagAnomaly): void {
    _anomalyDelta[0] = position[0] - anomaly.position[0];
    _anomalyDelta[1] = position[1] - anomaly.position[1];
    _anomalyDelta[2] = position[2] - anomaly.position[2];
    const dist = Math.sqrt(
      _anomalyDelta[0] * _anomalyDelta[0] +
      _anomalyDelta[1] * _anomalyDelta[1] +
      _anomalyDelta[2] * _anomalyDelta[2],
    );

    if (dist >= anomaly.radius) return;

    // Quadratic decay: (1 - dist/radius)^2
    const t = 1 - dist / anomaly.radius;
    const scale = t * t;
    out[0] += anomaly.strength[0] * scale;
    out[1] += anomaly.strength[1] * scale;
    out[2] += anomaly.strength[2] * scale;
  }
}

/**
 * Simplified WMM: tilted dipole model.
 * Returns NED field components in Tesla at sea level.
 * Accuracy: ~5-10% — sufficient for simulation, not navigation.
 */
function computeWMMDipole(latDeg: number, lonDeg: number, wmmDate = 2025.0): { north: number; east: number; down: number } {
  const DEG2RAD = Math.PI / 180;
  const lat = latDeg * DEG2RAD;

  // Earth's dipole: magnetic pole drifts with time (secular variation)
  // WMM2025 reference: ~80.5°N, 72.6°W. Drift rate: ~0.1°/yr north, ~0.3°/yr west
  const dt = wmmDate - 2025.0;
  const poleLat = (80.5 + 0.1 * dt) * DEG2RAD;
  const poleLon = (-72.6 - 0.3 * dt) * DEG2RAD;
  const lon = lonDeg * DEG2RAD;

  // Magnetic colatitude
  const cosMagColat = Math.sin(lat) * Math.sin(poleLat) + Math.cos(lat) * Math.cos(poleLat) * Math.cos(lon - poleLon);
  const sinMagColat = Math.sqrt(1 - cosMagColat * cosMagColat);
  const magColat = Math.acos(Math.max(-1, Math.min(1, cosMagColat)));

  // Dipole field at surface (approximate mean: B0 ≈ 30 µT at equator, 60 µT at poles)
  const B0 = 30e-6; // T, equatorial surface field
  const Br = -2 * B0 * Math.cos(magColat); // radial (down in NED)
  const Btheta = -B0 * Math.sin(magColat); // southward component

  // Total field strength
  const totalField = Math.sqrt(Br * Br + Btheta * Btheta);

  // Inclination and declination (simplified)
  const inclination = Math.atan2(Br, -Btheta); // positive = field points down
  // Declination: azimuth of magnetic north from geographic north
  const sinDec = Math.cos(poleLat) * Math.sin(lon - poleLon) / sinMagColat;
  const cosDec = (Math.sin(poleLat) - cosMagColat * Math.sin(lat)) / (sinMagColat * Math.cos(lat) + 1e-20);
  const declination = Math.atan2(sinDec, cosDec);

  const cosInc = Math.cos(inclination);
  return {
    north: totalField * cosInc * Math.cos(declination),
    east: totalField * cosInc * Math.sin(declination),
    down: totalField * Math.sin(inclination),
  };
}
