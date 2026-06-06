#include "EncoderMotor.h"

EncoderMotor::EncoderMotor(L293D &driver, EncoderPins enc, MotorCalibration cal)
    : driver_(driver),
      enc_(enc),
      cal_(cal),
      rawCount_(0),
      lastRawCount_(0) {
  state_.currentA = NAN;
  state_.encoderOffset = 0;
}

void EncoderMotor::begin() {
  driver_.begin();
  pinMode(enc_.pinA, INPUT_PULLUP);
  pinMode(enc_.pinB, INPUT_PULLUP);
  state_.enabled = true;
}

void EncoderMotor::onEncoderA() {
  // Full quadrature: 4× resolution
  const bool b = digitalRead(enc_.pinB);
  if (digitalRead(enc_.pinA) == b) {
    rawCount_++;
  } else {
    rawCount_--;
  }
}

void EncoderMotor::readEncoderDelta() {
  noInterrupts();
  const int32_t count = rawCount_;
  interrupts();

  state_.encoderCount = count;
  state_.encoderDelta = count - lastRawCount_;
  lastRawCount_ = count;

  state_.positionMm =
      encoderToMm(count, state_.encoderOffset, cal_.ticksPerMm);
}

void EncoderMotor::update(uint32_t nowMs) {
  const uint32_t dtMs = nowMs - state_.lastUpdateMs;
  readEncoderDelta();

  if (dtMs > 0) {
    const float dtS = dtMs / 1000.0f;
    state_.velocityMmS =
        (state_.encoderDelta / cal_.ticksPerMm) / dtS;
  }

  state_.power = driver_.power();
  state_.atTarget =
      fabsf(state_.positionMm - state_.targetMm) <= cal_.positionToleranceMm;
  state_.lastUpdateMs = nowMs;
}

void EncoderMotor::setTargetMm(float targetMm) { state_.targetMm = targetMm; }

void EncoderMotor::setPower(int16_t power) {
  applyPower(power);
  state_.power = driver_.power();
}

void EncoderMotor::applyPower(int16_t power) {
  driver_.setPower(power);
  state_.enabled = (power != 0);
}
