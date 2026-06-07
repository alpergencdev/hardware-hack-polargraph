// Arduino IDE version of the PlatformIO project at ../../firmware.
// Logic is identical to src/main.cpp; only Arduino-IDE conventions differ
// (no explicit <Arduino.h>, headers live alongside this sketch).
//
// Board settings to set under Tools  (matches platformio.ini):
//   Board:                 "ESP32S3 Dev Module"
//   USB CDC On Boot:       "Enabled"
//   Flash Size:            "32MB (256Mb)"      (this module is 32MB)
//   PSRAM:                 "OPI PSRAM"
//   Flash Mode:            "QIO 80MHz"
//   Upload Speed:          921600
//   USB Mode:              "Hardware CDC and JTAG"
//
// Libraries needed (Tools > Manage Libraries):
//   "Stepper" by Arduino, "Adafruit SSD1306" (+ GFX, BusIO).
// WiFi.h / HTTPClient.h ship with the ESP32 core — no install needed.

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
// Fill these in for your network and the machine running ui/server.py. When you
// start the server it prints the exact URL to use:
//   "ESP32 fetches -> http://<this-machine-LAN-IP>:8000/path.ndjson"
// Put that IP in SERVER_HOST and the port in SERVER_PORT.
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* SERVER_HOST = "192.168.1.50";  // laptop LAN IP printed by server.py
const int SERVER_PORT = 8000;

const int STEPS_PER_REV = 2048;

// --- OLED status display (SSD1306, I2C) ---------------------------------------
// Wiring on the ESP32-S3: SDA = GPIO 8, SCL = GPIO 9.
const int OLED_SDA = 8;
const int OLED_SCL = 9;
const int OLED_WIDTH = 128;
const int OLED_HEIGHT = 64;        // use 32 here if you have the short 128x32 panel
const uint8_t OLED_ADDR = 0x3C;    // the common SSD1306 I2C address (some are 0x3D)
// Panel is mounted sideways, so rotate the whole display 90°. 0=landscape,
// 1=90° CW, 2=180°, 3=270°. With 1 or 3 the usable area becomes 64 wide x 128
// tall; the centering below reads the live dimensions so it adapts automatically.
// If the text comes out upside down for your mounting, use 3 instead of 1.
const uint8_t OLED_ROTATION = 1;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// Center a line of big text and show it. After setRotation the usable width is
// narrow (64 px when rotated), so a long word like "IN PROGRESS" won't fit on one
// line — we word-wrap it across rows and vertically center the block. Uses the
// GFX text-bounds API so centering is exact at any rotation.
void showStatus(const char* msg) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(true);  // let long words flow onto the next row

  const int16_t W = display.width();   // live (post-rotation) dimensions
  const int16_t H = display.height();

  // Measure the wrapped text block so we can center it vertically.
  int16_t bx, by;
  uint16_t bw, bh;
  display.getTextBounds(msg, 0, 0, &bx, &by, &bw, &bh);
  int16_t y = (H - (int16_t)bh) / 2;
  if (y < 0) y = 0;
  // Horizontal: center if it fits on one row, else start at the left margin and
  // let wrapping handle the rest.
  int16_t x = (bw <= (uint16_t)W) ? (W - (int16_t)bw) / 2 : 0;
  if (x < 0) x = 0;

  display.setCursor(x, y);
  display.print(msg);
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

// 28BYJ-48 via ULN2003 on an ESP32-S3. The middle pin pair is swapped — that's
// the standard coil ordering for these boards.
//
// ESP32-S3 reserved/unsafe pins (do NOT use for the motors):
//   - GPIO 26-37 : SPI flash + octal (OPI) PSRAM on this module — using them crashes the chip
//   - GPIO 19,20 : native USB D-/D+ (your upload + Serial port)
//   - GPIO 0,3,45,46 : strapping pins   - GPIO 43,44 : UART0 TX/RX
// The eight pins below are all safe, general-purpose, output-capable S3 GPIOs.
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

// The path now comes from the server (ui/server.py) over WiFi instead of a
// hard-coded array. The server streams the single continuous pen-down line as
// "x y\n" per point, terminated by a line "END" — see drawStreamedPath() below.
// X-Path-Version lets us long-poll for the NEXT drawing without re-drawing the
// current one. -1 means "we have drawn nothing yet" (fetch whatever is current).
long lastDrawnVersion = -1;

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

// ---------------------------------------------------------------------------
// WiFi + path streaming from the server.
// ---------------------------------------------------------------------------

// Block until we're on WiFi, showing dots on serial. Returns once connected.
void connectWiFi() {
  Serial.printf("Connecting to WiFi \"%s\"", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.print(" connected, IP ");
  Serial.println(WiFi.localIP());
}

// Tell the server how many points we've drawn so the website's progress bar
// can update. Best-effort: failures here never interrupt drawing.
void reportProgress(int drawn) {
  HTTPClient http;
  String url = String("http://") + SERVER_HOST + ":" + SERVER_PORT + "/progress";
  if (http.begin(url)) {
    http.addHeader("Content-Type", "application/json");
    http.POST(String("{\"drawn\":") + drawn + "}");
    http.end();
  }
}

// Fetch the current path and draw it as it streams in, one point per line.
// Returns the path version drawn (>=0) on success, or -1 if nothing was drawn
// (no path ready / connection problem) so the caller can retry.
//
// We deliberately read the body line-by-line and call moveTo() per point: the
// firmware holds only the current target, never the whole path, so a drawing of
// any size fits in RAM.
long drawStreamedPath() {
  HTTPClient http;
  String url = String("http://") + SERVER_HOST + ":" + SERVER_PORT +
               "/path.ndjson?since=" + lastDrawnVersion;
  if (!http.begin(url)) return -1;
  // The server long-polls (waits ~25s) for a path, so allow a generous timeout.
  http.setTimeout(30000);

  const char* hdrKeys[] = {"X-Path-Version", "X-Path-Points"};
  http.collectHeaders(hdrKeys, 2);

  int code = http.GET();
  if (code == 204) { http.end(); return -1; }   // nothing new yet — caller retries
  if (code != 200) {
    Serial.printf("path fetch failed: HTTP %d\n", code);
    http.end();
    return -1;
  }

  long version = http.header("X-Path-Version").toInt();
  long total = http.header("X-Path-Points").toInt();
  Serial.printf("Drawing path v%ld (%ld points)...\n", version, total);
  showInProgress();

  WiFiClient* stream = http.getStreamPtr();
  String line;
  int drawn = 0;
  unsigned long lastProgressMs = 0;

  // Read until the stream ends or we hit the "END" sentinel. readStringUntil
  // blocks per line; the connection stays open while the server streams.
  while (http.connected()) {
    line = stream->readStringUntil('\n');
    line.trim();
    if (line.length() == 0) {
      if (!stream->available() && !http.connected()) break;
      continue;
    }
    if (line == "END") break;

    // Parse "x y" — two floats separated by a space.
    int sp = line.indexOf(' ');
    if (sp <= 0) continue;
    float x = line.substring(0, sp).toFloat();
    float y = line.substring(sp + 1).toFloat();

    moveTo({x, y});
    drawn++;

    // Throttle progress reports to ~3/sec so we don't flood the server.
    if (millis() - lastProgressMs > 300) {
      lastProgressMs = millis();
      reportProgress(drawn);
    }
  }
  http.end();

  reportProgress(drawn);
  Serial.printf("Path v%ld complete: drew %d points.\n", version, drawn);
  showCompleted();
  return version;
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
    display.setRotation(OLED_ROTATION);  // panel is mounted sideways
    showStatus("WIFI...");
  }

  connectWiFi();

  Serial.print("Pen assumed at START (");
  Serial.print(START.x);
  Serial.print(", ");
  Serial.print(START.y);
  Serial.println(") mm. Waiting for a path from the server.");
  showStatus("READY");
}

void loop() {
  // Reconnect if WiFi dropped.
  if (WiFi.status() != WL_CONNECTED) {
    showStatus("WIFI...");
    connectWiFi();
  }

  // Fetch + draw the next published path. drawStreamedPath() long-polls, so this
  // mostly blocks here until someone hits "Send to plotter" on the website. On a
  // successful draw it returns the version, which we remember so we don't redraw
  // the same path; on timeout/no-path it returns -1 and we just loop again.
  long version = drawStreamedPath();
  if (version >= 0) {
    lastDrawnVersion = version;
  } else {
    delay(500);  // brief pause before re-polling after a timeout or error
  }
}
