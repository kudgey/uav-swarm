/**
 * Sensor layer barrel export.
 * Individual sensors register their own status in their modules.
 */

// Side-effect imports to trigger subsystem registration
import './imu';
import './magnetometer';
import './barometer';
import './rangefinder';
import './optical-flow';
import './camera-vio';
import './uwb';
