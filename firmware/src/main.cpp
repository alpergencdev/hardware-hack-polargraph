#include <Arduino.h>
#include <Stepper.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <HTTPClient.h>

#include <math.h>

#include "Geometry.h"
#include "StepperState.h"

// --- WiFi + path server -------------------------------------------------------
// Fill these in for your network and the machine running ui/server.py. The
// server prints the exact URL to use ("ESP32 fetches -> http://<ip>:8000/...").
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* SERVER_HOST = "192.168.1.50";  // the laptop's LAN IP from server.py
const int SERVER_PORT = 8000;

const int STEPS_PER_REV = 2048;

// --- OLED status display (SSD1306, I2C) ---------------------------------------
// Wiring on the ESP32-S3: SDA = GPIO 8, SCL = GPIO 9.
const int OLED_SDA = 8;
const int OLED_SCL = 9;
const int OLED_WIDTH = 128;
const int OLED_HEIGHT = 64;        // use 32 here if you have the short 128x32 panel
const uint8_t OLED_ADDR = 0x3C;    // the common SSD1306 I2C address (some are 0x3D)
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// Center a single line of big text and show it. Used for the status words below.
void showStatus(const char* msg) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  // Rough horizontal centering: size-2 chars are ~12 px wide.
  int16_t x = (OLED_WIDTH - (int)strlen(msg) * 12) / 2;
  if (x < 0) x = 0;
  display.setCursor(x, (OLED_HEIGHT - 16) / 2);
  display.println(msg);
  display.display();
}

void showInProgress() { showStatus("IN PROGRESS"); }
void showCompleted()  { showStatus("COMPLETED"); }

// Spool / cord geometry — measure on the real build.
const int SPOOL_DIAMETER_MM = 12;

// Hanging-V plotter. Two spools anchored at the top corners of the work area;
// the pen/gondola hangs on the two cords. Anchors sit on the y=0 line:
//   left anchor  = (0, 0)
//   right anchor = (ANCHOR_SEPARATION_MM, 0)
// +x points right, +y points DOWN (away from the anchors). A cord's length is
// just the straight-line distance from its anchor to the pen.
const float ANCHOR_SEPARATION_MM = 600.0f;  // distance between the two anchors — MEASURE

// Where the pen physically sits at power-on. The machine cannot home itself
// (open-loop steppers, no limit switches), so you must position the pen here
// by hand before running. All motion is tracked relative to this.
const Coord START = {300.0f, 400.0f};  // mm, in the anchor frame

const unsigned long SERIAL_BAUD = 115200;
const unsigned long REPORT_INTERVAL_MS = 50;  // 20 Hz telemetry

// 28BYJ-48 via ULN2003 on ESP32 DevKit C. Middle pin pair is swapped — that's
// the standard coil ordering for these boards. GPIOs 6-11 are reserved for the
// onboard flash and must NOT be used (driving them hangs/crashes the chip), so
// the left motor uses 19/21 instead of 6/7. These are all safe output-capable pins.
Stepper leftMotor(STEPS_PER_REV, 4, 5, 6, 7);
Stepper rightMotor(STEPS_PER_REV, 15, 16, 17, 18);

// Inverse kinematics: pen position (mm) -> the two cord lengths (mm).
CordLengths cordLengthsFor(Coord p) {
  float dxL = p.x - 0.0f;
  float dxR = p.x - ANCHOR_SEPARATION_MM;
  CordLengths c;
  c.left = sqrtf(dxL * dxL + p.y * p.y);
  c.right = sqrtf(dxR * dxR + p.y * p.y);
  return c;
}

// Open-loop tracking: a Stepper has no encoder, so "position" is the running
// total of steps we have commanded. StepperState converts that count to cord
// length (mm). Each state's initial length is the start cord length, so step
// count 0 == the pen sitting at START.
CordLengths startCords = cordLengthsFor(START);
StepperState leftState(SPOOL_DIAMETER_MM, STEPS_PER_REV, (int)startCords.left, 0);
StepperState rightState(SPOOL_DIAMETER_MM, STEPS_PER_REV, (int)startCords.right, 0);

long leftSteps = 0;
long rightSteps = 0;
unsigned long lastReportMs = 0;

// ---------------------------------------------------------------------------
// Hard-coded path from the painter. Each entry is an {x, y} target in mm in the
// anchor frame. The pen visits them in order. This will be replaced by points
// fetched over WiFi later, so keep the consumer (moveTo) decoupled from the
// source.
// ---------------------------------------------------------------------------
const float PATH[][2] = {
    {200.0f, 350.0f},
    {400.0f, 350.0f},
    {400.0f, 500.0f},
    {200.0f, 500.0f},
    {200.0f, 350.0f},
};
const int PATH_LEN = sizeof(PATH) / sizeof(PATH[0]);

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

void maybeReport() {
  unsigned long now = millis();
  if (now - lastReportMs >= REPORT_INTERVAL_MS) {
    lastReportMs = now;
    report();
  }
}

// Move the pen to an (x,y) target with coordinated motion: compute each motor's
// target step count from the cord lengths, then interleave the two motors'
// steps (Bresenham-style) so they start and finish together and the pen tracks
// a roughly straight line. Stepper::step() blocks, so we issue one step at a
// time and alternate between motors.
void moveTo(Coord target) {
  CordLengths cords = cordLengthsFor(target);
  int leftTarget = leftState.lengthToPosition(cords.left);
  int rightTarget = rightState.lengthToPosition(cords.right);

  int leftDir = (leftTarget >= (int)leftSteps) ? 1 : -1;
  int rightDir = (rightTarget >= (int)rightSteps) ? 1 : -1;
  long leftAbs = labs(leftTarget - (int)leftSteps);
  long rightAbs = labs(rightTarget - (int)rightSteps);

  // Bresenham over the longer axis: step the major-axis motor every iteration,
  // and advance the minor-axis motor proportionally so both reach their targets
  // together and the pen tracks a roughly straight line.
  long major = max(leftAbs, rightAbs);
  long minor = min(leftAbs, rightAbs);
  long err = major / 2;

  for (long i = 0; i < major; i++) {
    err -= minor;
    bool stepMinor = false;
    if (err < 0) {
      err += major;
      stepMinor = true;
    }

    if (leftAbs >= rightAbs) {
      leftMotor.step(leftDir);  // left is major
      leftSteps += leftDir;
      if (stepMinor) {
        rightMotor.step(rightDir);
        rightSteps += rightDir;
      }
    } else {
      rightMotor.step(rightDir);  // right is major
      rightSteps += rightDir;
      if (stepMinor) {
        leftMotor.step(leftDir);
        leftSteps += leftDir;
      }
    }

    leftState.setPosition((int)leftSteps);
    rightState.setPosition((int)rightSteps);
    maybeReport();
  }

  // Snap to the exact target to absorb any rounding in the loop accounting.
  leftSteps = leftTarget;
  rightSteps = rightTarget;
  leftState.setPosition((int)leftSteps);
  rightState.setPosition((int)rightSteps);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  leftMotor.setSpeed(10);  // RPM
  rightMotor.setSpeed(10);

  // Bring up the OLED on the chosen I2C pins. If init fails (bad wiring or wrong
  // address) we just log it and carry on — the plotter still runs without a screen.
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed — check wiring (SDA=8, SCL=9) and address (0x3C/0x3D).");
  } else {
    showInProgress();
  }

  Serial.print("Pen assumed at START (");
  Serial.print(START.x);
  Serial.print(", ");
  Serial.print(START.y);
  Serial.println(") mm. Running hard-coded path.");

  for (int i = 0; i < PATH_LEN; i++) {
    Coord target = {PATH[i][0], PATH[i][1]};
    moveTo(target);
    report();
  }

  Serial.println("Path complete.");
  showCompleted();
}

void loop() {
  // Path runs once in setup(). Nothing to do until we add a WiFi point source.
}
