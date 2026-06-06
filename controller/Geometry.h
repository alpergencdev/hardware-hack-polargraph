#pragma once

// Plain data types shared across the kinematics code.
// No Arduino dependency — safe to include in off-hardware simulations/tests.

const float MY_PI = 3.14159265358979323846;

struct Coord {
  float x;
  float y;
};

struct CordLengths {
  float left;
  float right;
};

struct PositionDelta {
  int left;
  int right;
};
