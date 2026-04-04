/**
 * ISA (International Standard Atmosphere) troposphere model.
 * STATUS: experimental
 *
 * Valid for altitudes 0-11000m (troposphere).
 * T(h) = T0 - L*h
 * P(h) = P0 * (T(h)/T0)^(g/(L*R))
 * rho(h) = P(h) / (R_specific * T(h))
 */

import {
  ISA_SEA_LEVEL_PRESSURE, ISA_SEA_LEVEL_TEMPERATURE,
  ISA_TEMPERATURE_LAPSE_RATE, G_STANDARD,
} from '@sim/core/units';
import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('atmosphere', 'experimental', 'ISA troposphere model (0-11km)');

const R_SPECIFIC = 287.058; // J/(kg*K) — specific gas constant for dry air
const EXPONENT = G_STANDARD / (ISA_TEMPERATURE_LAPSE_RATE * R_SPECIFIC); // ~5.2559

/** ISA temperature at geometric altitude (m above sea level). */
export function isaTemperature(altitudeM: number): number {
  const alt = Math.max(0, Math.min(altitudeM, 11000));
  return ISA_SEA_LEVEL_TEMPERATURE - ISA_TEMPERATURE_LAPSE_RATE * alt;
}

/** ISA pressure at geometric altitude (m above sea level). Returns Pa. */
export function isaPressure(altitudeM: number): number {
  const T = isaTemperature(altitudeM);
  return ISA_SEA_LEVEL_PRESSURE * Math.pow(T / ISA_SEA_LEVEL_TEMPERATURE, EXPONENT);
}

/** ISA air density at geometric altitude (m above sea level). Returns kg/m^3. */
export function isaDensity(altitudeM: number): number {
  const T = isaTemperature(altitudeM);
  const P = ISA_SEA_LEVEL_PRESSURE * Math.pow(T / ISA_SEA_LEVEL_TEMPERATURE, EXPONENT);
  return P / (R_SPECIFIC * T);
}

/**
 * Inverse: pressure (Pa) to altitude (m).
 * Inverts the ISA pressure formula for the troposphere.
 */
export function isaPressureToAltitude(pressurePa: number): number {
  const ratio = pressurePa / ISA_SEA_LEVEL_PRESSURE;
  const T = ISA_SEA_LEVEL_TEMPERATURE * Math.pow(ratio, 1 / EXPONENT);
  return (ISA_SEA_LEVEL_TEMPERATURE - T) / ISA_TEMPERATURE_LAPSE_RATE;
}
