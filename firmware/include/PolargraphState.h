#pragma once

#include "MotorState.h"
#include <math.h>

// Full machine snapshot: two winch motors + geometry needed for kinematics.
struct PolargraphState {
  MotorState left;
  MotorState right;

  // Fixed machine dimensions (measure once, store in EEPROM or flash)
  float anchorSpacingMm;    // Horizontal distance between the two motor anchors
  float minCableMm;         // Shortest safe cable length (don't over-spool)
  float maxCableMm;         // Longest safe cable length (pen still reachable)

  // Derived pen position in the drawing frame (mm).
  // Origin: midpoint between anchors. +X toward right anchor, +Y toward the pen.
  float penXMm;
  float penYMm;

  uint32_t timestampMs;
};

// Forward kinematics: cable lengths → pen (x, y).
// Left anchor at (-spacing/2, 0), right at (+spacing/2, 0).
inline void cableLengthsToPen(float leftMm, float rightMm, float spacingMm,
                              float &outX, float &outY) {
  const float half = spacingMm * 0.5f;
  outX = (rightMm * rightMm - leftMm * leftMm) / (2.0f * spacingMm);
  const float under = leftMm * leftMm - (half - outX) * (half - outX);
  outY = sqrtf(under > 0.0f ? under : 0.0f);
}

inline void updatePenPosition(PolargraphState &state) {
  cableLengthsToPen(state.left.positionMm, state.right.positionMm,
                    state.anchorSpacingMm, state.penXMm, state.penYMm);
}
