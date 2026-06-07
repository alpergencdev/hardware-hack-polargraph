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
const char* WIFI_SSID = "AG";
const char* WIFI_PASS = "ALRR7P9M3CRE";
const char* SERVER_HOST = "172.20.10.2";  // laptop LAN IP printed by server.py
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
const int SPOOL_DIAMETER_MM = 11;

// Hanging-V plotter. Two spools anchored at the top corners of the work area;
// the pen/gondola hangs on the two cords. Anchors sit on the y=0 line:
//   left anchor  = (0, 0)
//   right anchor = (ANCHOR_SEPARATION_MM, 0)
// +x points right, +y points DOWN (away from the anchors). A cord's length is
// just the straight-line distance from its anchor to the pen.
const float ANCHOR_SEPARATION_MM = 450.0f;  // distance between the two anchors — MEASURE

// Where the pen physically sits at power-on. The machine cannot home itself
// (open-loop steppers, no limit switches), so you must position the pen here
// by hand before running. All motion is tracked relative to this.
//
// This is the DEFAULT; the server can override it per-run via the CFG header in
// the path stream (see applyStartConfig / drawStreamedPath). Not const anymore
// because a run may re-seed it from uploaded config.
Coord START = {100.5f, 150.0f};  // mm, in the anchor frame

// Motor speed (RPM), also overridable per-run from the CFG header. Default 10.
int motorSpeed = 10;

const unsigned long SERIAL_BAUD = 115200;
const unsigned long REPORT_INTERVAL_MS = 50;  // 20 Hz telemetry

// 28BYJ-48 via ULN2003 on an ESP32-S3. The Arduino Stepper class energizes the
// four pins in the ORDER GIVEN (p1->p2->p3->p4). For a 28BYJ-48 the coils must
// fire in the sequence IN1, IN3, IN2, IN4 — i.e. the MIDDLE PAIR IS SWAPPED
// relative to the board's pin labels. Passing them straight (IN1,IN2,IN3,IN4)
// makes the rotor rock in place and HUM instead of rotating.
//
// Wiring assumption: GPIO 4->IN1, 5->IN2, 6->IN3, 7->IN4 (and 15->IN1 ...).
// The constructor args below reflect that with the middle pair swapped so the
// fire order is IN1, IN3, IN2, IN4. If a motor still hums after this, your GPIO
// ->INx wiring differs — re-check which pin goes to which IN label.
//
// ESP32-S3 reserved/unsafe pins (do NOT use for the motors):
//   - GPIO 26-37 : SPI flash + octal (OPI) PSRAM on this module — using them crashes the chip
//   - GPIO 19,20 : native USB D-/D+ (your upload + Serial port)
//   - GPIO 0,3,45,46 : strapping pins   - GPIO 43,44 : UART0 TX/RX
// The eight pins below are all safe, general-purpose, output-capable S3 GPIOs.
Stepper leftMotor(STEPS_PER_REV, 4, 5, 6, 7);       // IN1, IN3, IN2, IN4
Stepper rightMotor(STEPS_PER_REV, 15, 16, 17, 18);  // IN1, IN3, IN2, IN4

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

// Inverse of cordLengthsFor: two cord lengths -> pen (x, y) in the anchor frame.
// Intersection of the two circles centered on the anchors. Used when the start
// position is given as cord lengths (which you can MEASURE on the real rig).
Coord coordForCords(float left, float right) {
  // Left anchor (0,0) radius=left; right anchor (D,0) radius=right.
  float D = ANCHOR_SEPARATION_MM;
  float x = (left * left - right * right + D * D) / (2.0f * D);
  float under = left * left - x * x;        // y^2; pen hangs below (y>0)
  float y = under > 0.0f ? sqrtf(under) : 0.0f;
  return {x, y};
}

// Re-seed the open-loop tracking so "the pen is physically at START right now,
// step counts = 0". Called at the start of every run after the server's CFG
// header sets START + motorSpeed, so each drawing begins from a known origin.
void applyStartConfig() {
  CordLengths c = cordLengthsFor(START);
  leftState = StepperState(SPOOL_DIAMETER_MM, STEPS_PER_REV, (int)c.left, 0);
  rightState = StepperState(SPOOL_DIAMETER_MM, STEPS_PER_REV, (int)c.right, 0);
  leftSteps = 0;
  rightSteps = 0;
  leftMotor.setSpeed(motorSpeed);
  rightMotor.setSpeed(motorSpeed);
  Serial.printf("Config applied: START=(%.1f, %.1f) mm, speed=%d RPM\n",
                START.x, START.y, motorSpeed);
}

// The path now comes from the server (ui/server.py) over WiFi instead of a
// hard-coded array. The server streams the single continuous pen-down line as
// "x y\n" per point, terminated by a line "END" — see drawStreamedPath() below.
// X-Path-Version lets us long-poll for the NEXT drawing without re-drawing the
// current one. -1 means "we have drawn nothing yet" (fetch whatever is current).
long lastDrawnVersion = -1;

// Resume-after-disconnect state. While a draw is in flight we remember which
// version we're on and how many of its points we've already committed, so if
// WiFi drops mid-draw we can re-fetch the SAME version starting at that index
// (server ?from=N) and pick up where the pen physically is — instead of losing
// the whole drawing or restarting it from the top. resumeVersion = -1 means
// "no interrupted draw to resume"; it's set when a draw begins and cleared when
// it completes.
long resumeVersion = -1;
int  resumePointsDrawn = 0;

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

// Human-readable WiFi status code, so a failure tells us WHY (wrong password vs
// network not found vs ...) instead of just spinning forever on dots.
const char* wifiStatusStr(int s) {
  switch (s) {
    case WL_IDLE_STATUS:     return "IDLE";
    case WL_NO_SSID_AVAIL:   return "NO_SSID_AVAIL (network not found — name wrong, or 5GHz-only, or hotspot off)";
    case WL_SCAN_COMPLETED:  return "SCAN_COMPLETED";
    case WL_CONNECTED:       return "CONNECTED";
    case WL_CONNECT_FAILED:  return "CONNECT_FAILED (usually a wrong password)";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED:    return "DISCONNECTED";
    default:                 return "UNKNOWN";
  }
}

// Scan and list nearby 2.4GHz networks — confirms the ESP32 can actually SEE
// your hotspot (and shows the exact SSID spelling/case to match).
void scanNetworks() {
  Serial.println("Scanning for 2.4GHz networks the ESP32 can see...");
  int n = WiFi.scanNetworks();
  if (n <= 0) {
    Serial.println("  (none found — if your hotspot is on, it may be 5GHz-only)");
    return;
  }
  for (int i = 0; i < n; i++) {
    Serial.printf("  \"%s\"  RSSI %d  %s\n",
                  WiFi.SSID(i).c_str(), WiFi.RSSI(i),
                  WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "open" : "secured");
  }
  WiFi.scanDelete();
}

// Connect to WiFi with a timeout + diagnostics. Retries forever (so the plotter
// recovers on its own), but each attempt reports the status and re-scans so you
// can see whether the network is even visible.
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);   // clear any stale state from a previous attempt
  delay(100);
  scanNetworks();

  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf("Connecting to WiFi \"%s\" ", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long start = millis();
    // Give each attempt 15s before reporting status and retrying.
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
      delay(400);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) break;
    Serial.printf("\n  failed — status: %s\n", wifiStatusStr(WiFi.status()));
    Serial.println("  retrying in 3s...");
    delay(3000);
  }
  Serial.print("WiFi connected, IP ");
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
  // If a draw was interrupted, re-ask for the SAME version (since=resumeVersion)
  // starting at the point we got to (from=resumePointsDrawn). Otherwise long-poll
  // for whatever is newer than the last version we fully drew.
  bool resuming = (resumeVersion >= 0 && resumePointsDrawn > 0);
  long sinceVersion = resuming ? resumeVersion : lastDrawnVersion;
  String url = String("http://") + SERVER_HOST + ":" + SERVER_PORT +
               "/path.ndjson?since=" + sinceVersion;
  if (resuming) url += "&from=" + String(resumePointsDrawn);
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
  Serial.printf("Got path v%ld (%ld points)...\n", version, total);

  // Absolute index of the first point in THIS stream within the full drawing.
  // 0 for a fresh draw/jog; the resume offset when picking up after a drop (the
  // server already skipped the points before this index).
  int baseIndex = resuming ? resumePointsDrawn : 0;
  // OLED status is shown at the POINTS marker, once we know if it's a draw or jog.

  WiFiClient* stream = http.getStreamPtr();
  String line;
  int drawn = 0;
  unsigned long lastProgressMs = 0;

  // Phase 1: read the CFG header (config lines) until the POINTS marker. We
  // accumulate config into these temporaries, then apply it ONCE at POINTS so
  // the run starts from the uploaded START + speed. Defaults to whatever START/
  // motorSpeed already hold, so a stream with no CFG lines is harmless.
  bool inPoints = false;
  bool sawEnd = false;     // true once we read the "END" sentinel — i.e. the
                           // server finished sending the whole (remaining) path.
                           // If the loop exits WITHOUT this, the connection was
                           // cut mid-stream and the draw is incomplete.
  String startMode = "xy";
  String kind = "draw";   // "draw" = re-seed to START then draw; "jog" = move
                          // from the CURRENT position to the target, no re-seed.
  float cfgX = START.x, cfgY = START.y;
  float cfgLeft = 0, cfgRight = 0;
  bool haveCord = false;
  float lastX = START.x, lastY = START.y;  // last point reached (for jog re-seed)

  // Read until the stream ends or we hit the "END" sentinel. readStringUntil
  // blocks per line; the connection stays open while the server streams.
  while (http.connected()) {
    line = stream->readStringUntil('\n');
    line.trim();
    if (line.length() == 0) {
      if (!stream->available() && !http.connected()) break;
      continue;
    }
    if (line == "END") { sawEnd = true; break; }

    if (!inPoints) {
      if (line == "POINTS") {
        if (kind == "jog") {
          // Jog: move from the pen's CURRENT tracked position to the target. Do
          // NOT re-seed to START first (that would zero the steps and skip the
          // move). Just set the speed; the single point is moved to in phase 2,
          // and we adopt it as the new origin AFTER the move (see below).
          leftMotor.setSpeed(motorSpeed);
          rightMotor.setSpeed(motorSpeed);
          Serial.printf("Jog: moving to target at speed %d RPM\n", motorSpeed);
          showStatus("JOG");
        } else if (kind == "resume") {
          // Resume after a disconnect: the server is replaying the tail of the
          // SAME drawing. The pen is already physically where we left off and our
          // open-loop step tracking still reflects that, so do NOT re-seed to
          // START (that would zero the steps and teleport the pen). Just set the
          // speed and continue moving through the remaining points.
          leftMotor.setSpeed(motorSpeed);
          rightMotor.setSpeed(motorSpeed);
          Serial.printf("Resuming v%ld from point %d at speed %d RPM\n",
                        version, resumePointsDrawn, motorSpeed);
          showInProgress();
        } else {
          // Draw: set START from config, then re-seed tracking to it.
          if (startMode == "cord" && haveCord) {
            START = coordForCords(cfgLeft, cfgRight);
          } else {
            START = {cfgX, cfgY};
          }
          applyStartConfig();
          showInProgress();
          // Arm resume tracking: from here on, if WiFi drops, we can re-fetch
          // this version from the point we got to. (jog runs are single-shot and
          // intentionally don't arm this.)
          resumeVersion = version;
          resumePointsDrawn = 0;
        }
        inPoints = true;
        continue;
      }
      if (line.startsWith("CFG ")) {
        // "CFG <key> <value>"
        int s1 = line.indexOf(' ', 4);
        if (s1 < 0) continue;
        String key = line.substring(4, s1);
        String val = line.substring(s1 + 1);
        if (key == "kind")            kind = val;
        else if (key == "start_mode") startMode = val;
        else if (key == "start_x")    cfgX = val.toFloat();
        else if (key == "start_y")    cfgY = val.toFloat();
        else if (key == "start_left")  { cfgLeft = val.toFloat(); haveCord = true; }
        else if (key == "start_right") { cfgRight = val.toFloat(); haveCord = true; }
        else if (key == "speed")       motorSpeed = val.toInt();
      }
      continue;  // still in the header; don't treat CFG lines as points
    }

    // Phase 2: points. Parse "x y" — two floats separated by a space.
    int sp = line.indexOf(' ');
    if (sp <= 0) continue;
    float x = line.substring(0, sp).toFloat();
    float y = line.substring(sp + 1).toFloat();

    moveTo({x, y});
    drawn++;
    lastX = x;  // remember where we ended (for jog re-seed)
    lastY = y;
    // Advance the resume cursor to the absolute index of the point we just
    // committed, so a drop right here re-fetches starting at the NEXT point.
    if (kind != "jog") resumePointsDrawn = baseIndex + drawn;

    // Throttle progress reports to ~3/sec so we don't flood the server. Report
    // the absolute index so the website's progress bar is correct across resumes.
    if (millis() - lastProgressMs > 300) {
      lastProgressMs = millis();
      reportProgress(baseIndex + drawn);
    }
  }
  http.end();
  reportProgress(baseIndex + drawn);

  if (kind == "jog") {
    // The pen is now physically at the target. Adopt it as the new origin so the
    // next drawing starts here and tracking stays consistent (jog sets START).
    if (drawn > 0) {
      START = {lastX, lastY};
      applyStartConfig();
    }
    Serial.printf("Jog v%ld done: pen now at (%.1f, %.1f).\n", version, lastX, lastY);
    showStatus("READY");
    return version;
  }

  // Draw / resume. If we never saw END the connection was cut mid-stream: the
  // draw is incomplete. Keep the resume state armed (resumeVersion/
  // resumePointsDrawn already point at where we stopped) and return -1 so loop()
  // reconnects and re-polls, which will re-fetch this version from that index.
  if (!sawEnd) {
    Serial.printf("Path v%ld interrupted at point %d — will resume after reconnect.\n",
                  version, resumePointsDrawn);
    return -1;
  }

  // Finished the whole drawing: disarm resume and mark it done.
  resumeVersion = -1;
  resumePointsDrawn = 0;
  Serial.printf("Path v%ld complete: drew %d points.\n", version, drawn);
  showCompleted();
  return version;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  applyStartConfig();  // seed tracking + motor speed from the defaults

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
