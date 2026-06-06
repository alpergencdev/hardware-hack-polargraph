#pragma once

// ── Board: Elegoo Uno R3 (ATmega328P, CH340 USB) ──────────────────────────
// Same pinout as Arduino Uno R3. PlatformIO target: board = uno
//
// Do NOT use these pins — they are reserved on the Uno:
//   D0, D1  → USB serial (upload + telemetry)
//   D2, D3  → encoder channel A (only pins with hardware interrupts)
//
// Elegoo's single-motor L293D lesson uses D3/D4/D5 for direction + enable.
// With two encoders we remap motor direction off D3/D4 so D2/D3 stay free.
//
// L293D wiring (one chip, both motors):
//   Left  motor → 1,2EN + inputs 1A/2A
//   Right motor → 3,4EN + inputs 3A/4A
//   VCC1 → 5V (logic)    VCC2 → 6–12V motor supply    GND → common ground
//
// Elegoo wiki (single motor): ENABLE=D5, IN1=D3, IN2=D4
// https://wiki.elegoo.com/oshw-getting-started-kits/relay-UNO

// Left winch — L293D channels 1 & 2 (1,2EN) — same enable pin as Elegoo lesson
static const uint8_t LEFT_IN1  = 7;
static const uint8_t LEFT_IN2  = 8;
static const uint8_t LEFT_PWM  = 5;   // 1,2EN  (Elegoo: ENABLE on D5)
static const uint8_t LEFT_ENC_A = 2;  // INT0 — must stay on D2
static const uint8_t LEFT_ENC_B = 4;

// Right winch — L293D channels 3 & 4 (3,4EN)
static const uint8_t RIGHT_IN1  = 10;
static const uint8_t RIGHT_IN2  = 11;
static const uint8_t RIGHT_PWM  = 6;  // 3,4EN
static const uint8_t RIGHT_ENC_A = 3; // INT1 — must stay on D3
static const uint8_t RIGHT_ENC_B = 12;

// Machine geometry — measure on the real build
static const float ANCHOR_SPACING_MM = 800.0f;
static const float MIN_CABLE_MM      = 100.0f;
static const float MAX_CABLE_MM      = 1200.0f;

// Encoder calibration — jog 100 mm and divide tick count by 100
static const float TICKS_PER_MM           = 42.5f;
static const float POSITION_TOLERANCE_MM  = 0.5f;

// Control loop
static const uint16_t TELEMETRY_HZ = 20;
static const uint16_t CONTROL_HZ   = 100;

// Serial — Elegoo CH340 works at 115200 with our firmware (Elegoo lessons use 9600)
static const unsigned long SERIAL_BAUD = 115200;
