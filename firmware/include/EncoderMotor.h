#pragma once

#include "L293D.h"
#include "MotorState.h"

// Quadrature encoder + L293D channel, updates a MotorState snapshot.
struct EncoderPins {
  uint8_t pinA;
  uint8_t pinB;
};

class EncoderMotor {
 public:
  EncoderMotor(L293D &driver, EncoderPins enc, MotorCalibration cal);

  void begin();
  void update(uint32_t nowMs);
  void setTargetMm(float targetMm);
  void setPower(int16_t power);

  MotorState &state() { return state_; }
  const MotorState &state() const { return state_; }

  // Called from ISR — keep minimal
  void onEncoderA();

 private:
  void readEncoderDelta();
  void applyPower(int16_t power);

  L293D &driver_;
  EncoderPins enc_;
  MotorCalibration cal_;

  volatile int32_t rawCount_;
  int32_t lastRawCount_;

  MotorState state_;
};
