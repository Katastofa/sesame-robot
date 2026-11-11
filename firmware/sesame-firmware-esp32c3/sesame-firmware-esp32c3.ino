// ======================================================================
// Sesame Robot — ESP32-C3 Super Mini (Native PWM variant)
//
// Hardware differences from the original S2-Mini/PCA9685 build:
//   - No PCA9685 board: servo signals come directly from ESP32-C3 GPIO 0-7
//   - I2C (GPIO 8 SDA, GPIO 10 SCL) is used exclusively for the OLED display
//   - Servo power is supplied from the expansion board's dedicated battery rails
//   - ESP32-C3 is isolated from its own board (only data pins + GND shared)
//
// Servo driver: SesameServo.h (1 hardware timer, ISR-based software PWM)
//   ESP32-C3 has only 6 LEDC channels, not enough for 8 servos,
//   so we use a timer ISR that drives GPIO directly at 50 Hz.
// ======================================================================

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "SesameServo.h"
#include "face-bitmaps.h"
#include "movement-sequences.h"
#include "captive-portal.h"

// --- Access Point Configuration ---
#define AP_SSID  "Sesame-Controller"
#define AP_PASS  "12345678"

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_I2C_ADDR 0x3C

// I2C — display only (no PCA9685 on bus)
#define I2C_SDA 8
#define I2C_SCL 10

// ======================================================================
// Servo pin map — GPIO pins wired to the expansion board's servo headers.
// I2C (GPIO 8 SDA, GPIO 10 SCL) is used only for the OLED display.
//
// Ch | Label | Role              | GPIO
// ---+-------+-------------------+------
//  0 | r1    | R1 hip (front R)  | 5
//  1 | l1    | L1 hip (front L)  | 7
//  2 | r2    | R2 hip (back R)   | 6
//  3 | l2    | L2 hip (back L)   | 20
//  4 | r3    | R1 knee  [inv]    | 1
//  5 | l3    | L1 knee  [inv]    | 0
//  6 | r4    | R2 knee           | 2
//  7 | l4    | L2 knee           | 3
//
// Pulse range matches original PCA9685 calibration:
//   SERVO_MIN_TICK=150, SERVO_MAX_TICK=600 at 50Hz → 732 us / 2929 us
// ======================================================================
const uint8_t servoPins[8] = {5, 7, 6, 20, 1, 0, 2, 3};
#define SERVO_MIN_US  732
#define SERVO_MAX_US  2929

// DNS Server for Captive Portal
DNSServer dnsServer;
const byte DNS_PORT = 53;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);

// Global state for animations
String currentCommand = "";
String currentFaceName = "defualt";
const unsigned char* const* currentFaceFrames = nullptr;
uint8_t currentFaceFrameCount = 0;
uint8_t currentFaceFrameIndex = 0;
unsigned long lastFaceFrameMs = 0;
int faceFps = 8;
FaceAnimMode currentFaceMode = FACE_ANIM_LOOP;
int8_t faceFrameDirection = 1;
bool faceAnimFinished = false;
int currentFaceFps = 0;
bool idleActive = false;
bool idleBlinkActive = false;
unsigned long nextIdleBlinkMs = 0;
uint8_t idleBlinkRepeatsLeft = 0;

// WiFi Info Scrolling
unsigned long lastInputTime = 0;
bool firstInputReceived = false;
bool showingWifiInfo = false;
int wifiScrollPos = 0;
unsigned long lastWifiScrollMs = 0;
String wifiInfoText = "";

// Subtrim values for each servo (offset in degrees)
int8_t servoSubtrim[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Tracks current logical angle for each servo (used by moveServosSmooth)
int currentAngles[8] = {90, 90, 90, 90, 90, 90, 90, 90};

// Animation / motion constants
int frameDelay = 100;
int walkCycles = 10;
float poseSpeed = 1.0;

struct FaceEntry {
  const char* name;
  const unsigned char* const* frames;
  uint8_t maxFrames;
};

static const uint8_t MAX_FACE_FRAMES = 6;

#define MAKE_FACE_FRAMES(name) \
  const unsigned char* const face_##name##_frames[] = { \
    epd_bitmap_##name, epd_bitmap_##name##_1, epd_bitmap_##name##_2, \
    epd_bitmap_##name##_3, epd_bitmap_##name##_4, epd_bitmap_##name##_5 \
  };

#define X(name) MAKE_FACE_FRAMES(name)
FACE_LIST
#undef X
#undef MAKE_FACE_FRAMES

const FaceEntry faceEntries[] = {
#define X(name) { #name, face_##name##_frames, MAX_FACE_FRAMES },
  FACE_LIST
#undef X
  { "default", face_defualt_frames, MAX_FACE_FRAMES }
};

struct FaceFpsEntry {
  const char* name;
  uint8_t fps;
};

const FaceFpsEntry faceFpsEntries[] = {
  { "walk",       1 },
  { "rest",       1 },
  { "swim",       1 },
  { "dance",      1 },
  { "wave",       1 },
  { "point",      5 },
  { "stand",      1 },
  { "cute",       1 },
  { "pushup",     1 },
  { "freaky",     1 },
  { "bow",        1 },
  { "worm",       1 },
  { "shake",      1 },
  { "shrug",      1 },
  { "dead",       2 },
  { "crab",       1 },
  { "idle",       1 },
  { "idle_blink", 7 },
  { "defualt",    1 },
};

// Prototypes
void setServoAngle(uint8_t channel, int angle);
void updateFaceBitmap(const unsigned char* bitmap);
void setFace(const String& faceName);
void setFaceMode(FaceAnimMode mode);
void setFaceWithMode(const String& faceName, FaceAnimMode mode);
void updateAnimatedFace();
void delayWithFace(unsigned long ms);
void enterIdle();
void exitIdle();
void updateIdleBlink();
int  getFaceFpsForName(const String& faceName);
bool pressingCheck(String cmd, int ms);
void handleGetSettings();
void handleSetSettings();
void updateWifiInfoScroll();
void recordInput();

void handleRoot() {
  server.send(200, "text/html", index_html);
}

void handleCommandWeb() {
  if (server.hasArg("pose")) {
    currentCommand = server.arg("pose");
    recordInput();
    exitIdle();
    server.send(200, "text/plain", "OK");
  }
  else if (server.hasArg("go")) {
    currentCommand = server.arg("go");
    recordInput();
    exitIdle();
    server.send(200, "text/plain", "OK");
  }
  else if (server.hasArg("stop")) {
    currentCommand = "";
    recordInput();
    server.send(200, "text/plain", "OK");
  }
  else if (server.hasArg("motor") && server.hasArg("value")) {
    int motorNum = server.arg("motor").toInt();
    int angle    = server.arg("value").toInt();
    if (motorNum >= 1 && motorNum <= 8 && angle >= 0 && angle <= 180) {
      setServoAngle(motorNum - 1, angle);
      recordInput();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Invalid motor or angle");
    }
  }
  else {
    server.send(400, "text/plain", "Bad Args");
  }
}

void handleGetSettings() {
  String json = "{";
  json += "\"frameDelay\":"  + String(frameDelay)      + ",";
  json += "\"walkCycles\":"  + String(walkCycles)      + ",";
  json += "\"motorCurrentDelay\":0,";   // not applicable for native PWM
  json += "\"poseSpeed\":"   + String(poseSpeed, 2)    + ",";
  json += "\"faceFps\":"     + String(faceFps);
  json += "}";
  server.send(200, "application/json", json);
}

void handleSetSettings() {
  if (server.hasArg("frameDelay"))  frameDelay  = server.arg("frameDelay").toInt();
  if (server.hasArg("walkCycles"))  walkCycles  = server.arg("walkCycles").toInt();
  if (server.hasArg("poseSpeed"))   poseSpeed   = constrain(server.arg("poseSpeed").toFloat(), 0.25f, 4.0f);
  if (server.hasArg("faceFps"))     faceFps     = (int)max(1L, server.arg("faceFps").toInt());
  // motorCurrentDelay is ignored — software PWM ISR handles all channels concurrently
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  randomSeed(micros());

  // I2C — display only
  Wire.begin(I2C_SDA, I2C_SCL);

  // OLED Init
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println(F("SSD1306 allocation failed."));
    while (1);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("Creating WiFi..."));
  display.display();

  // --- Access Point ---
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP Created. IP: ");
  Serial.println(myIP);

  wifiInfoText = "Connect to WiFi: " + String(AP_SSID) + "  |  Pass: " + String(AP_PASS) + "  |  Captive Portal will auto-open!  |  ";

  lastInputTime = millis();
  firstInputReceived = false;
  showingWifiInfo = false;

  dnsServer.start(DNS_PORT, "*", myIP);

  server.on("/",           handleRoot);
  server.on("/cmd",        handleCommandWeb);
  server.on("/getSettings", handleGetSettings);
  server.on("/setSettings", handleSetSettings);
  server.onNotFound(handleRoot);
  server.begin();

  // ---------------------------------------------------------------
  // Software servo init — 1 hardware timer drives 8 GPIO pins at 50Hz.
  // Each servo is registered on its GPIO pin with the calibrated pulse
  // range (732-2929 us) that matches the original PCA9685 config.
  // ---------------------------------------------------------------
  ESP32_ISR_Servos.useTimer(1);
  for (uint8_t i = 0; i < 8; i++) {
    ESP32_ISR_Servos.setupServo(servoPins[i], SERVO_MIN_US, SERVO_MAX_US);
  }

  setFace("rest");
  Serial.println(F("HTTP server & Captive Portal started."));
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  updateAnimatedFace();
  updateIdleBlink();
  updateWifiInfoScroll();

  if (currentCommand != "") {
    String cmd = currentCommand;
    if      (cmd == "forward")  runWalkPose();
    else if (cmd == "backward") runWalkBackward();
    else if (cmd == "left")     runTurnLeft();
    else if (cmd == "right")    runTurnRight();
    else if (cmd == "rest")     { runRestPose();     if (currentCommand == "rest")  currentCommand = ""; }
    else if (cmd == "stand")    { runStandPose(1);   if (currentCommand == "stand") currentCommand = ""; }
    else if (cmd == "wave")     runWavePose();
    else if (cmd == "dance")    runDancePose();
    else if (cmd == "swim")     runSwimPose();
    else if (cmd == "point")    runPointPose();
    else if (cmd == "pushup")   runPushupPose();
    else if (cmd == "bow")      runBowPose();
    else if (cmd == "cute")     runCutePose();
    else if (cmd == "freaky")   runFreakyPose();
    else if (cmd == "worm")     runWormPose();
    else if (cmd == "shake")    runShakePose();
    else if (cmd == "shrug")    runShrugPose();
    else if (cmd == "dead")     runDeadPose();
    else if (cmd == "crab")     runCrabPose();
  }

  // Serial CLI (debug)
  if (Serial.available()) {
    static char command_buffer[32];
    static byte buffer_pos = 0;
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buffer_pos > 0) {
        command_buffer[buffer_pos] = '\0';
        int motorNum, angle;
        recordInput();
        if      (strcmp(command_buffer, "run walk") == 0 || strcmp(command_buffer, "rn wf") == 0) { currentCommand = "forward";  runWalkPose();    currentCommand = ""; }
        else if (strcmp(command_buffer, "rn wb") == 0)  { currentCommand = "backward"; runWalkBackward(); currentCommand = ""; }
        else if (strcmp(command_buffer, "rn tl") == 0)  { currentCommand = "left";     runTurnLeft();     currentCommand = ""; }
        else if (strcmp(command_buffer, "rn tr") == 0)  { currentCommand = "right";    runTurnRight();    currentCommand = ""; }
        else if (strcmp(command_buffer, "run rest") == 0 || strcmp(command_buffer, "rn rs") == 0) runRestPose();
        else if (strcmp(command_buffer, "run stand") == 0 || strcmp(command_buffer, "rn st") == 0) runStandPose(1);
        else if (strcmp(command_buffer, "rn wv") == 0) { currentCommand = "wave";   runWavePose();  }
        else if (strcmp(command_buffer, "rn dn") == 0) { currentCommand = "dance";  runDancePose(); }
        else if (strcmp(command_buffer, "rn sw") == 0) { currentCommand = "swim";   runSwimPose();  }
        else if (strcmp(command_buffer, "rn pt") == 0) { currentCommand = "point";  runPointPose(); }
        else if (strcmp(command_buffer, "rn pu") == 0) { currentCommand = "pushup"; runPushupPose(); }
        else if (strcmp(command_buffer, "rn bw") == 0) { currentCommand = "bow";    runBowPose();   }
        else if (strcmp(command_buffer, "rn ct") == 0) { currentCommand = "cute";   runCutePose();  }
        else if (strcmp(command_buffer, "rn fk") == 0) { currentCommand = "freaky"; runFreakyPose(); }
        else if (strcmp(command_buffer, "rn wm") == 0) { currentCommand = "worm";   runWormPose();  }
        else if (strcmp(command_buffer, "rn sk") == 0) { currentCommand = "shake";  runShakePose(); }
        else if (strcmp(command_buffer, "rn sg") == 0) { currentCommand = "shrug";  runShrugPose(); }
        else if (strcmp(command_buffer, "rn dd") == 0) { currentCommand = "dead";   runDeadPose();  }
        else if (strcmp(command_buffer, "rn cb") == 0) { currentCommand = "crab";   runCrabPose();  }
        else if (strcmp(command_buffer, "subtrim") == 0 || strcmp(command_buffer, "st") == 0) {
          Serial.println("Subtrim values:");
          for (int i = 0; i < 8; i++) {
            Serial.print("Motor "); Serial.print(i); Serial.print(": ");
            if (servoSubtrim[i] >= 0) Serial.print("+");
            Serial.println(servoSubtrim[i]);
          }
        }
        else if (strcmp(command_buffer, "subtrim save") == 0 || strcmp(command_buffer, "st save") == 0) {
          Serial.println("Copy and paste this into your code:");
          Serial.print("int8_t servoSubtrim[8] = {");
          for (int i = 0; i < 8; i++) {
            Serial.print(servoSubtrim[i]);
            if (i < 7) Serial.print(", ");
          }
          Serial.println("};");
        }
        else if (strncmp(command_buffer, "subtrim reset", 13) == 0 || strncmp(command_buffer, "st reset", 8) == 0) {
          for (int i = 0; i < 8; i++) servoSubtrim[i] = 0;
          Serial.println("All subtrim values reset to 0");
        }
        else if (strncmp(command_buffer, "subtrim ", 8) == 0 || strncmp(command_buffer, "st ", 3) == 0) {
          const char* params = (command_buffer[1] == 't') ? command_buffer + 3 : command_buffer + 8;
          int trimMotor, trimValue;
          if (sscanf(params, "%d %d", &trimMotor, &trimValue) == 2) {
            if (trimMotor >= 0 && trimMotor < 8) {
              if (trimValue >= -90 && trimValue <= 90) {
                servoSubtrim[trimMotor] = trimValue;
                Serial.print("Motor "); Serial.print(trimMotor); Serial.print(" subtrim set to ");
                if (trimValue >= 0) Serial.print("+");
                Serial.println(trimValue);
              } else {
                Serial.println("Subtrim value must be between -90 and +90");
              }
            } else {
              Serial.println("Invalid motor number (0-7)");
            }
          }
        }
        else if (strncmp(command_buffer, "all ", 4) == 0) {
          if (sscanf(command_buffer + 4, "%d", &angle) == 1) {
            for (int i = 0; i < 8; i++) setServoAngle(i, angle);
            Serial.print("All servos set to "); Serial.println(angle);
          }
        }
        else if (sscanf(command_buffer, "%d %d", &motorNum, &angle) == 2) {
          if (motorNum >= 0 && motorNum < 8) {
            setServoAngle(motorNum, angle);
            Serial.print("Servo "); Serial.print(motorNum); Serial.print(" set to "); Serial.println(angle);
          } else {
            Serial.println("Invalid motor number (0-7)");
          }
        }
        buffer_pos = 0;
      }
    } else if (buffer_pos < sizeof(command_buffer) - 1) {
      command_buffer[buffer_pos++] = c;
    }
  }
}

void updateFaceBitmap(const unsigned char* bitmap) {
  display.clearDisplay();
  display.drawBitmap(0, 0, bitmap, 128, 64, SSD1306_WHITE);
  display.display();
}

uint8_t countFrames(const unsigned char* const* frames, uint8_t maxFrames) {
  if (frames == nullptr || frames[0] == nullptr) return 0;
  uint8_t count = 0;
  for (uint8_t i = 0; i < maxFrames; i++) {
    if (frames[i] == nullptr) break;
    count++;
  }
  return count;
}

void setFace(const String& faceName) {
  if (faceName == currentFaceName && currentFaceFrames != nullptr) return;

  currentFaceName = faceName;
  currentFaceFrameIndex = 0;
  lastFaceFrameMs = 0;
  faceFrameDirection = 1;
  faceAnimFinished = false;
  currentFaceFps = getFaceFpsForName(faceName);

  currentFaceFrames = face_defualt_frames;
  currentFaceFrameCount = countFrames(face_defualt_frames, MAX_FACE_FRAMES);

  for (size_t i = 0; i < (sizeof(faceEntries) / sizeof(faceEntries[0])); i++) {
    if (faceName.equalsIgnoreCase(faceEntries[i].name)) {
      currentFaceFrames = faceEntries[i].frames;
      currentFaceFrameCount = countFrames(faceEntries[i].frames, faceEntries[i].maxFrames);
      break;
    }
  }

  if (currentFaceFrameCount == 0) {
    currentFaceFrames = face_defualt_frames;
    currentFaceFrameCount = countFrames(face_defualt_frames, MAX_FACE_FRAMES);
    currentFaceName = "defualt";
    currentFaceFps = getFaceFpsForName(currentFaceName);
  }

  if (currentFaceFrameCount > 0 && currentFaceFrames[0] != nullptr)
    updateFaceBitmap(currentFaceFrames[0]);
}

void setFaceMode(FaceAnimMode mode) {
  currentFaceMode = mode;
  faceFrameDirection = 1;
  faceAnimFinished = false;
}

void setFaceWithMode(const String& faceName, FaceAnimMode mode) {
  setFaceMode(mode);
  setFace(faceName);
}

int getFaceFpsForName(const String& faceName) {
  for (size_t i = 0; i < (sizeof(faceFpsEntries) / sizeof(faceFpsEntries[0])); i++) {
    if (faceName.equalsIgnoreCase(faceFpsEntries[i].name))
      return faceFpsEntries[i].fps;
  }
  return faceFps;
}

void updateAnimatedFace() {
  if (currentFaceFrames == nullptr || currentFaceFrameCount <= 1) return;
  if (currentFaceMode == FACE_ANIM_ONCE && faceAnimFinished) return;

  unsigned long now = millis();
  int fps = max(1, (currentFaceFps > 0 ? currentFaceFps : faceFps));
  unsigned long interval = 1000UL / fps;
  if (now - lastFaceFrameMs < interval) return;
  lastFaceFrameMs = now;

  if (currentFaceMode == FACE_ANIM_LOOP) {
    currentFaceFrameIndex = (currentFaceFrameIndex + 1) % currentFaceFrameCount;
  } else if (currentFaceMode == FACE_ANIM_ONCE) {
    if (currentFaceFrameIndex + 1 >= currentFaceFrameCount) {
      currentFaceFrameIndex = currentFaceFrameCount - 1;
      faceAnimFinished = true;
    } else {
      currentFaceFrameIndex++;
    }
  } else { // BOOMERANG
    if (faceFrameDirection > 0) {
      if (currentFaceFrameIndex + 1 >= currentFaceFrameCount) {
        faceFrameDirection = -1;
        if (currentFaceFrameIndex > 0) currentFaceFrameIndex--;
      } else {
        currentFaceFrameIndex++;
      }
    } else {
      if (currentFaceFrameIndex == 0) {
        faceFrameDirection = 1;
        if (currentFaceFrameCount > 1) currentFaceFrameIndex++;
      } else {
        currentFaceFrameIndex--;
      }
    }
  }
  updateFaceBitmap(currentFaceFrames[currentFaceFrameIndex]);
}

void delayWithFace(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    updateAnimatedFace();
    server.handleClient();
    dnsServer.processNextRequest();
    delay(5);
  }
}

void scheduleNextIdleBlink(unsigned long minMs, unsigned long maxMs) {
  nextIdleBlinkMs = millis() + (unsigned long)random(minMs, maxMs);
}

void enterIdle() {
  idleActive = true;
  idleBlinkActive = false;
  idleBlinkRepeatsLeft = 0;
  setFaceWithMode("idle", FACE_ANIM_BOOMERANG);
  scheduleNextIdleBlink(3000, 7000);
}

void exitIdle() {
  idleActive = false;
  idleBlinkActive = false;
}

void updateIdleBlink() {
  if (!idleActive) return;

  if (!idleBlinkActive) {
    if (millis() >= nextIdleBlinkMs) {
      idleBlinkActive = true;
      if (idleBlinkRepeatsLeft == 0 && random(0, 100) < 30)
        idleBlinkRepeatsLeft = 1;
      setFaceWithMode("idle_blink", FACE_ANIM_ONCE);
    }
    return;
  }

  if (currentFaceMode == FACE_ANIM_ONCE && faceAnimFinished) {
    idleBlinkActive = false;
    setFaceWithMode("idle", FACE_ANIM_BOOMERANG);
    if (idleBlinkRepeatsLeft > 0) {
      idleBlinkRepeatsLeft--;
      scheduleNextIdleBlink(120, 220);
    } else {
      scheduleNextIdleBlink(3000, 7000);
    }
  }
}

// ======================================================================
// Servo helpers
// Servo 4 (R1 knee) and 5 (L1 knee) are physically inverted relative to
// servos 6 (R2 knee) and 7 (L2 knee), so we flip their angle.
// ======================================================================
const bool servoInvert[8] = {false, false, false, false, true, true, false, false};

void setServoAngle(uint8_t channel, int angle) {
  if (channel >= 8) return;
  currentAngles[channel] = angle;
  int adjusted = constrain(angle + servoSubtrim[channel], 0, 180);
  if (servoInvert[channel]) adjusted = 180 - adjusted;
  ESP32_ISR_Servos.setPosition(channel, (uint16_t)adjusted);
}

bool pressingCheck(String cmd, int ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    server.handleClient();
    dnsServer.processNextRequest();
    updateAnimatedFace();
    if (currentCommand != cmd) {
      runStandPose(1);
      return false;
    }
    yield();
  }
  return true;
}

void recordInput() {
  lastInputTime = millis();
  if (!firstInputReceived) {
    firstInputReceived = true;
    showingWifiInfo = false;
  }
}

void updateWifiInfoScroll() {
  if (firstInputReceived) {
    if (showingWifiInfo) {
      showingWifiInfo = false;
      if (currentFaceFrames != nullptr && currentFaceFrameCount > 0)
        updateFaceBitmap(currentFaceFrames[currentFaceFrameIndex]);
    }
    return;
  }

  unsigned long now = millis();

  if (!showingWifiInfo && (now - lastInputTime >= 30000)) {
    showingWifiInfo = true;
    wifiScrollPos = 0;
    lastWifiScrollMs = now;
  }

  if (!showingWifiInfo) return;

  if (now - lastWifiScrollMs >= 150) {
    lastWifiScrollMs = now;
    display.clearDisplay();
    if (currentFaceFrames != nullptr && currentFaceFrameCount > 0)
      display.drawBitmap(0, 0, currentFaceFrames[currentFaceFrameIndex], 128, 64, SSD1306_WHITE);
    display.fillRect(0, 0, 128, 10, SSD1306_BLACK);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    display.setCursor(-wifiScrollPos, 1);
    display.print(wifiInfoText);
    display.setTextWrap(true);
    display.display();
    wifiScrollPos += 2;
    if (wifiScrollPos >= (int)(wifiInfoText.length() * 6))
      wifiScrollPos = 0;
  }
}
