/**
 * Telemetry data types for recording and transferring simulation state.
 */

export interface TelemetryFrame {
  time: number;
  posX: number; posY: number; posZ: number;
  velX: number; velY: number; velZ: number;
  qW: number; qX: number; qY: number; qZ: number;
  omegaX: number; omegaY: number; omegaZ: number;
  motor0: number; motor1: number; motor2: number; motor3: number;
  cmd0: number; cmd1: number; cmd2: number; cmd3: number;
  totalThrust: number;
}

export interface TelemetryBatch {
  startIndex: number;
  count: number;
  timestamps: Float64Array;
  positions: Float64Array;     // interleaved x,y,z
  velocities: Float64Array;    // interleaved x,y,z
  motorSpeeds: Float64Array;   // interleaved m0,m1,m2,m3
  motorCommands: Float64Array;
}
