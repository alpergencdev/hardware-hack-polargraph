#pragma once

#include <math.h>  // sqrt — works both on Arduino and in off-hardware sims

#include "EncoderState.h"
#include "Geometry.h"

// Max straight-line distance (mm) the pen travels per interpolated segment.
// Smaller = straighter lines, more callback invocations. ~1-2mm is typical.
const float SEGMENT_LENGTH_MM = 1.0;

// All polargraph kinematics. Pure math + state — no Arduino dependency, so this
// can be driven directly from a simulation (feed setPosition, read getCoordinates).
class PolargraphState {
  EncoderState left_encoder;
  EncoderState right_encoder;
  int anchor_spacing_mm;

  // Logical pen position the firmware is tracking, in the same (x, y)
  // convention as getCoordinates(). moveTo() interpolates from here.
  float pen_x;
  float pen_y;

 public:
  PolargraphState(EncoderState left_encoder, EncoderState right_encoder, int anchor_spacing_mm)
      : left_encoder(left_encoder),
        right_encoder(right_encoder),
        anchor_spacing_mm(anchor_spacing_mm) {
    Coord pen = getCoordinates();
    pen_x = pen.x;
    pen_y = pen.y;
  }

  void setLeftPosition(int position) { left_encoder.setPosition(position); }
  void setRightPosition(int position) { right_encoder.setPosition(position); }

  // Forward kinematics: current cord lengths -> pen position (x, y).
  // Usage:
  //   Coord pen = machine.getCoordinates();
  //   Serial.print(pen.x); Serial.print(", "); Serial.println(pen.y);
  Coord getCoordinates() {
    // Left motor at origin (0,0), right motor at (D,0).
    // x increases left->right, y increases downward (pen hangs below => y > 0).

    float left_mm  = left_encoder.getCordLength();
    float right_mm = right_encoder.getCordLength();
    float D        = anchor_spacing_mm;

    float x = (left_mm*left_mm - right_mm*right_mm + D*D) / (2.0 * D);
    float y = sqrt(left_mm*left_mm - x*x);

    return {x, y};
  }

  // Inverse kinematics: pen target (x, y) -> required cord lengths.
  // Same convention as getCoordinates():
  //   left motor at (0,0), right motor at (D,0),
  //   x increases left->right, y increases downward.
  // Usage:
  //   CordLengths c = machine.getCordLengths(120.0, 200.0);
  //   // c.left, c.right are the target cord lengths in mm
  CordLengths getCordLengths(float x, float y) {
    float D = anchor_spacing_mm;

    float left  = sqrt(x*x + y*y);
    float right = sqrt((x - D)*(x - D) + y*y);

    return {left, right};
  }

  // For a pen target (x, y), how many encoder ticks each motor must move
  // from its CURRENT position. Positive = cord must lengthen, negative = shorten
  // (sign follows current_position's convention in getCordLength()).
  // Usage:
  //   PositionDelta d = machine.getPositionDelta(120.0, 200.0);
  //   driveMotors(d.left, d.right);  // move each motor by that many ticks
  PositionDelta getPositionDelta(float x, float y) {
    CordLengths target = getCordLengths(x, y);

    int left_target  = left_encoder.lengthToPosition(target.left);
    int right_target = right_encoder.lengthToPosition(target.right);

    int left_delta  = left_target  - left_encoder.getPosition();
    int right_delta = right_target - right_encoder.getPosition();

    return {left_delta, right_delta};
  }

  // Move the pen in a STRAIGHT line from its current logical position to
  // (target_x, target_y), splitting the path into <= SEGMENT_LENGTH_MM steps
  // so the pen follows the line instead of bowing into an arc.
  //
  // For each segment, `drive` is called with the per-motor tick delta to reach
  // that segment's endpoint. The caller decides how to drive (PID, bang-bang,
  // etc.). After driving, advance the encoder positions to the segment endpoint
  // (e.g. via setLeftPosition/setRightPosition once the move completes) so the
  // next segment's delta is computed correctly.
  // Usage:
  //   void drive(PositionDelta d) {
  //     // move each motor by d.left / d.right ticks; BLOCK until arrived
  //   }
  //   machine.moveTo(120.0, 200.0, drive);  // straight line to (120, 200)
  void moveTo(float target_x, float target_y, void (*drive)(PositionDelta)) {
    float dx = target_x - pen_x;
    float dy = target_y - pen_y;
    float dist = sqrt(dx*dx + dy*dy);

    int steps = (int)(dist / SEGMENT_LENGTH_MM);
    if (steps < 1) steps = 1;  // always take at least one step to the target

    for (int i = 1; i <= steps; i++) {
      float t = (float)i / steps;            // 0..1 along the line
      float sx = pen_x + dx * t;
      float sy = pen_y + dy * t;

      drive(getPositionDelta(sx, sy));
    }

    pen_x = target_x;
    pen_y = target_y;
  }
};
