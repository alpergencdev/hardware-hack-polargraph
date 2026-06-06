#pragma once

#include <stdint.h>

// One DC motor + encoder on a polargraph winch.
// Position is measured as cable length paid OUT from the anchor (mm).
struct MotorState {
  // --- Raw sensor input ---
  int32_t encoderCount;     // Total ticks since power-on (wraps at int32 limits)
  int32_t encoderOffset;    // Tick count at the calibrated zero/home position
  int32_t encoderDelta;     // Ticks since last read (for velocity)

  // --- Converted physical state ---
  float positionMm;         // Cable length paid out from anchor (mm)
  float velocityMmS;        // Current speed (mm/s, positive = paying out cable)

  // --- Actuator output ---
  int16_t power;            // PWM command, -255 (full reverse) .. +255 (full forward)
  bool enabled;             // Driver enabled / holding torque

  // --- Control target (filled by planner, consumed by PID on Arduino) ---
  float targetMm;           // Desired cable length (mm)
  bool atTarget;            // True when |positionMm - targetMm| < tolerance

  // --- Optional diagnostics (omit if you have no current sensor) ---
  float currentA;           // Motor draw in amps; NaN if not measured

  // --- Timing ---
  uint32_t lastUpdateMs;    // millis() when this snapshot was taken
};

// Encoder ticks → millimeters. Set once you know your gear ratio + spool diameter.
struct MotorCalibration {
  float ticksPerMm;         // e.g. 42.5 ticks per mm of cable
  float homePositionMm;     // Cable length at the machine's home pose
  float positionToleranceMm;  // How close counts as "at target"
};

inline float encoderToMm(int32_t ticks, int32_t offset, float ticksPerMm) {
  return (float)(ticks - offset) / ticksPerMm;
}

inline int32_t mmToEncoder(float mm, int32_t offset, float ticksPerMm) {
  return offset + (int32_t)(mm * ticksPerMm);
}
