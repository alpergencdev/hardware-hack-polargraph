#pragma once

#include <Arduino.h>

// One motor channel on an L293D chip.
//
// L293D vs L298N: same control pattern (IN1/IN2 + enable PWM), but ~600 mA
// continuous per channel instead of ~2 A. Use a separate motor supply and
// expect the chip to run warm under load.
struct L293DPins {
  uint8_t in1;
  uint8_t in2;
  uint8_t pwm; // 1,2EN or 3,4EN — must be a PWM-capable pin
};

class L293D {
 public:
  explicit L293D(L293DPins pins);

  void begin();
  void setPower(int16_t power); // -255 (reverse) .. 0 (coast) .. +255 (forward)
  void stop();                  // coast — both direction pins LOW
  int16_t power() const { return power_; }

 private:
  L293DPins pins_;
  int16_t power_;
};
