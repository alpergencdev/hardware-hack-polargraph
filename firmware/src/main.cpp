#include <Arduino.h>

#include "EncoderState.h"
#include "Geometry.h"
#include "PolargraphState.h"

// This file is the Arduino-only layer: pin setup, encoder ISRs, and the
// setup()/loop() glue. All geometry/state lives in the headers above so it can
// be compiled and simulated off-hardware.

volatile long position = 0;

void encoderA() {
  if (digitalRead(3))
    position++;
  else
    position--;
}

void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(2), encoderA, CHANGE);

  Serial.begin(115200);
}

void loop() {
  Serial.println(position);
  delay(100);
}
