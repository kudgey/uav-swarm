/**
 * Environment model.
 * STATUS: stub — returns zero wind, constant ISA atmosphere.
 */
import { registerSubsystem } from '@sim/core/status-labels';
registerSubsystem('environment', 'stub', 'Zero wind, constant ISA sea-level atmosphere');
