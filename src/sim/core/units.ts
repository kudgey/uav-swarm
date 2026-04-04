/**
 * SI unit constants and conversion helpers.
 */

// ── Fundamental constants ──

export const G_STANDARD = 9.80665;  // m/s^2

// ── ISA sea-level atmosphere ──

export const ISA_SEA_LEVEL_PRESSURE = 101325.0;     // Pa
export const ISA_SEA_LEVEL_TEMPERATURE = 288.15;     // K (15°C)
export const ISA_SEA_LEVEL_DENSITY = 1.225;          // kg/m^3
export const ISA_TEMPERATURE_LAPSE_RATE = 0.0065;    // K/m

// ── Conversions ──

export const DEG_TO_RAD = Math.PI / 180;
export const RAD_TO_DEG = 180 / Math.PI;
export const RPM_TO_RADS = (2 * Math.PI) / 60;
export const RADS_TO_RPM = 60 / (2 * Math.PI);
