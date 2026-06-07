#pragma once

#include "Geometry.h"

// Tracks one motor's encoder: converts between tick counts and cord length (mm).
// No Arduino dependency — the actual tick counting happens in the ISR (main.cpp),
// which pushes the latest count in via setPosition().
class StepperState {
  int spool_diameter_mm;
  int steps_per_rev;
  int initial_length_mm;
  int current_position;

 public:
  StepperState(int spool_diameter_mm, int ticks_per_rev, int initial_length_mm, int initial_position)
      : spool_diameter_mm(spool_diameter_mm),
        steps_per_rev(ticks_per_rev),
        initial_length_mm(initial_length_mm),
        current_position(initial_position) {}

  float getCordLength() {
    return initial_length_mm + current_position * (MY_PI * spool_diameter_mm / steps_per_rev);
  }

  // Inverse of getCordLength(): cord length (mm) -> encoder tick count.
  int lengthToPosition(float length_mm) {
    return (int)((length_mm - initial_length_mm) / (MY_PI * spool_diameter_mm / steps_per_rev));
  }

  int getPosition() { return current_position; }

  void setPosition(int position) { current_position = position; }
};
