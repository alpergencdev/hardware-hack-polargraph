#include <Arduino.h>
#include <Stepper.h>

#include "StepperState.h"

const int STEPS_PER_REV = 2048;

// Spool / cord geometry — measure on the real build.
const int SPOOL_DIAMETER_MM = 12;
const int INITIAL_LENGTH_MM = 100;

const unsigned long SERIAL_BAUD = 115200;
const unsigned long REPORT_INTERVAL_MS = 50;  // 20 Hz telemetry

// 28BYJ-48 via ULN2003. Middle pin pair is swapped (10,9 / 6,5) — that's the
// standard coil ordering for these boards.
Stepper leftMotor(STEPS_PER_REV, 8, 10, 9, 11);
Stepper rightMotor(STEPS_PER_REV, 4, 6, 5, 7);

// Open-loop tracking: a Stepper has no encoder, so "position" is the running
// total of steps we have commanded. We push that count into StepperState,
// which converts it to cord length (mm).
StepperState leftState(SPOOL_DIAMETER_MM, STEPS_PER_REV, INITIAL_LENGTH_MM, 0);
StepperState rightState(SPOOL_DIAMETER_MM, STEPS_PER_REV, INITIAL_LENGTH_MM, 0);

long leftSteps = 0;
long rightSteps = 0;
unsigned long lastReportMs = 0;

void report() {
  Serial.print("left:  steps=");
  Serial.print(leftState.getPosition());
  Serial.print(" length_mm=");
  Serial.print(leftState.getCordLength());
  Serial.print("   right: steps=");
  Serial.print(rightState.getPosition());
  Serial.print(" length_mm=");
  Serial.println(rightState.getCordLength());
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  leftMotor.setSpeed(10);   // RPM
  rightMotor.setSpeed(10);
}

void loop() {
  // Mirror: left pays out a revolution while right reels one in, then reverse.
  // Stepper::step() blocks, so interleave single steps to move them together.
  for (int i = 0; i < STEPS_PER_REV; i++) {
    leftMotor.step(1);
    rightMotor.step(-1);
  }
  leftSteps += STEPS_PER_REV;
  rightSteps -= STEPS_PER_REV;
  leftState.setPosition((int)leftSteps);
  rightState.setPosition((int)rightSteps);

  delay(1000);

  for (int i = 0; i < STEPS_PER_REV; i++) {
    leftMotor.step(-1);
    rightMotor.step(1);
  }
  leftSteps -= STEPS_PER_REV;
  rightSteps += STEPS_PER_REV;
  leftState.setPosition((int)leftSteps);
  rightState.setPosition((int)rightSteps);

  delay(1000);

  unsigned long now = millis();
  if (now - lastReportMs >= REPORT_INTERVAL_MS) {
    lastReportMs = now;
    report();
  }
}
