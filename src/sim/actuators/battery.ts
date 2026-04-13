/**
 * Simplified battery voltage sag model.
 * STATUS: simplified — lumped-parameter SOC → voltage, no temperature.
 *
 * V(SOC) = V_nom * (V_full_fraction + (1 - V_full_fraction) * SOC)
 *        - R_internal * I
 *
 * Current I is proportional to sum of rotor thrust (P = T*v_ind ≈ k_I * Ω²).
 * SOC depletion: d(SOC)/dt = -I / capacity_coulombs.
 *
 * Effect on thrust: max_thrust scales with V/V_nom (linear approximation).
 * This captures two realistic phenomena:
 *  1) Voltage sag under heavy load (transient) — reduces peak thrust capability
 *  2) SOC depletion over time — reduces baseline capability
 */

import { registerSubsystem } from '@sim/core/status-labels';

registerSubsystem('battery', 'simplified',
  'Lumped-parameter SOC → voltage with internal resistance sag', {
    simplifications: ['No temperature model', 'Linear V(SOC) curve', 'No thermal cutoff'],
  });

export interface BatteryConfig {
  enabled: boolean;
  capacityWh: number;        // total energy (Wh)
  nominalVoltage: number;    // V, e.g. 14.8 for 4S LiPo
  fullVoltageFraction: number; // V/V_nom at 100% SOC (typically 1.1)
  cutoffFraction: number;    // V/V_nom at which thrust is zero (typically 0.85)
  internalResistance: number;  // Ω (per-pack), typical 0.015-0.030
  currentPerThrustNewton: number; // A per N thrust (rough)
}

export interface BatteryState {
  soc: number;      // 0..1 state of charge
  voltage: number;  // current terminal voltage (V)
  currentA: number; // last step current draw (A)
}

export function defaultBatteryConfig(): BatteryConfig {
  return {
    enabled: false,
    capacityWh: 100,
    nominalVoltage: 14.8,
    fullVoltageFraction: 1.08,
    cutoffFraction: 0.85,
    internalResistance: 0.020,
    currentPerThrustNewton: 2.5, // ~10 A for ~4 N hover thrust
  };
}

export function createBatteryState(startSOC = 1.0): BatteryState {
  return { soc: startSOC, voltage: 14.8, currentA: 0 };
}

/**
 * Step battery one physics tick.
 * @param state          Mutable battery state
 * @param config         Battery config
 * @param totalThrustN   Sum of rotor thrust at this instant (N)
 * @param dt             Timestep (s)
 * @returns Thrust multiplier (0..1) to apply to commanded motor force
 */
export function stepBattery(
  state: BatteryState,
  config: BatteryConfig,
  totalThrustN: number,
  dt: number,
): number {
  if (!config.enabled) return 1.0;

  // Current draw
  const current = Math.max(0, totalThrustN * config.currentPerThrustNewton);
  state.currentA = current;

  // Open-circuit voltage from SOC (linear approximation)
  const socClamped = Math.max(0, Math.min(1, state.soc));
  const vocFraction = config.fullVoltageFraction - (config.fullVoltageFraction - config.cutoffFraction) * (1 - socClamped);
  const voc = vocFraction * config.nominalVoltage;

  // Terminal voltage = VOC - I*R
  const terminal = voc - current * config.internalResistance;
  state.voltage = Math.max(0, terminal);

  // SOC depletion: d(SOC)/dt = -P / (V_nom * capacity_coulombs)
  // capacity_coulombs = capacity_Wh * 3600 / V_nom
  const capacityCoulombs = (config.capacityWh * 3600) / config.nominalVoltage;
  state.soc = Math.max(0, state.soc - (current * dt) / capacityCoulombs);

  // Thrust multiplier: linear in voltage fraction above cutoff
  const voltFraction = terminal / config.nominalVoltage;
  if (voltFraction <= config.cutoffFraction) return 0;
  // Map [cutoffFraction, fullVoltageFraction] → [0, 1]
  const span = config.fullVoltageFraction - config.cutoffFraction;
  return Math.min(1.0, Math.max(0, (voltFraction - config.cutoffFraction) / span));
}
