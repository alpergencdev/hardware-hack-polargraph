#include "L293D.h"

L293D::L293D(L293DPins pins) : pins_(pins), power_(0) {}

void L293D::begin() {
  pinMode(pins_.in1, OUTPUT);
  pinMode(pins_.in2, OUTPUT);
  pinMode(pins_.pwm, OUTPUT);
  stop();
}

void L293D::setPower(int16_t power) {
  power = constrain(power, -255, 255);
  power_ = power;

  if (power == 0) {
    stop();
    return;
  }

  if (power > 0) {
    digitalWrite(pins_.in1, HIGH);
    digitalWrite(pins_.in2, LOW);
    analogWrite(pins_.pwm, power);
  } else {
    digitalWrite(pins_.in1, LOW);
    digitalWrite(pins_.in2, HIGH);
    analogWrite(pins_.pwm, -power);
  }
}

void L293D::stop() {
  digitalWrite(pins_.in1, LOW);
  digitalWrite(pins_.in2, LOW);
  analogWrite(pins_.pwm, 0);
  power_ = 0;
}
