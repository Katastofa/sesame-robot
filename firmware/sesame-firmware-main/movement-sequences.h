#pragma once

#include <Arduino.h>

enum FaceAnimMode : uint8_t {
  FACE_ANIM_LOOP = 0,
  FACE_ANIM_ONCE = 1,
  FACE_ANIM_BOOMERANG = 2
};

// External globals and helpers used by movement/pose sequences
extern int frameDelay;
extern int walkCycles;
extern float poseSpeed;
extern int currentAngles[8];
extern String currentCommand;

extern void setServoAngle(uint8_t channel, int angle);
extern void setFace(const String& faceName);
extern void setFaceMode(FaceAnimMode mode);
extern void setFaceWithMode(const String& faceName, FaceAnimMode mode);
extern void delayWithFace(unsigned long ms);
extern void enterIdle();
extern bool pressingCheck(String cmd, int ms);

// Pose-speed-aware delay: scales all animation pauses by poseSpeed
// poseSpeed < 1.0 = faster, poseSpeed > 1.0 = slower
#define pD(ms) delayWithFace((unsigned long)((ms) * poseSpeed))

// Pose/animation prototypes
void runRestPose();
void runStandPose(int face = 1);
void runWavePose();
void runDancePose();
void runSwimPose();
void runPointPose();
void runPushupPose();
void runBowPose();
void runCutePose();
void runFreakyPose();
void runWormPose();
void runShakePose();
void runShrugPose();
void runDeadPose();
void runCrabPose();
void runWalkPose();
void runWalkBackward();
void runTurnLeft();
void runTurnRight();

// ====== SMOOTH INTERPOLATION ======
// Cubic ease-in-out: starts slow, accelerates, decelerates to stop
inline float easeInOut(float t) {
  return t * t * (3.0f - 2.0f * t);
}

// Smoothly interpolates all 8 servos from their current positions to targets[].
// Use -1 in targets[] to leave a servo unchanged.
// steps: number of interpolation frames; stepDelayMs: ms per frame.
// Total duration = steps * stepDelayMs. Recommended: 8-12 steps, 15-20ms each.
inline void moveServosSmooth(const int targets[8], int steps, int stepDelayMs) {
  float startAngles[8];
  for (int i = 0; i < 8; i++) startAngles[i] = (float)currentAngles[i];

  for (int s = 1; s <= steps; s++) {
    float ease = easeInOut((float)s / (float)steps);
    for (int i = 0; i < 8; i++) {
      if (targets[i] < 0) continue;
      int angle = (int)(startAngles[i] + (targets[i] - startAngles[i]) * ease + 0.5f);
      setServoAngle(i, angle);
    }
    delayWithFace((unsigned long)stepDelayMs);
  }
}

// Convenience wrapper that applies poseSpeed to the step delay
inline void moveServosSmoothP(const int targets[8], int steps, int baseStepDelayMs) {
  moveServosSmooth(targets, steps, (int)(baseStepDelayMs * poseSpeed));
}

// ====== POSES ======
inline void runRestPose() {
  Serial.println(F("REST"));
  setFaceWithMode("rest", FACE_ANIM_BOOMERANG);
  const int restPos[8] = {90, 90, 90, 90, 90, 90, 90, 90};
  moveServosSmoothP(restPos, 10, 18);
}

inline void runStandPose(int face) {
  Serial.println(F("STAND"));
  if (face == 1) setFaceWithMode("stand", FACE_ANIM_ONCE);
  const int standPos[8] = {135, 45, 45, 135, 0, 180, 0, 180};
  moveServosSmoothP(standPos, 10, 18);
  if (face == 1) enterIdle();
}

inline void runWavePose() {
  Serial.println(F("WAVE"));
  setFaceWithMode("wave", FACE_ANIM_ONCE);
  runStandPose(0);
  pD(200);
  // Shift weight to left side, raise right arm
  const int liftPos[8] = {100, 45, 45, 60, 80, 180, 180, 180};
  moveServosSmoothP(liftPos, 8, 15);
  pD(200);
  for (int i = 0; i < 4; i++) {
    setServoAngle(6, 180); pD(300);
    setServoAngle(6, 100); pD(300);
  }
  runStandPose(1);
  if (currentCommand == "wave") currentCommand = "";
}

inline void runDancePose() {
  Serial.println(F("DANCE"));
  setFaceWithMode("dance", FACE_ANIM_LOOP);
  const int danceInit[8] = {90, 90, 90, 90, 160, 160, 10, 10};
  moveServosSmoothP(danceInit, 8, 18);
  pD(200);
  for (int i = 0; i < 5; i++) {
    const int danceA[8] = {-1, -1, -1, -1, 115, 115, 10, 10};
    moveServosSmoothP(danceA, 6, 20);
    pD(100);
    const int danceB[8] = {-1, -1, -1, -1, 160, 160, 65, 65};
    moveServosSmoothP(danceB, 6, 20);
    pD(100);
  }
  runStandPose(1);
  if (currentCommand == "dance") currentCommand = "";
}

inline void runSwimPose() {
  Serial.println(F("SWIM"));
  setFaceWithMode("swim", FACE_ANIM_ONCE);
  const int swimInit[8] = {90, 90, 90, 90, 90, 90, 90, 90};
  moveServosSmoothP(swimInit, 8, 15);
  for (int i = 0; i < 4; i++) {
    const int swimA[8] = {135, 45, 45, 135, -1, -1, -1, -1};
    moveServosSmoothP(swimA, 8, 20);
    const int swimB[8] = {90, 90, 90, 90, -1, -1, -1, -1};
    moveServosSmoothP(swimB, 8, 20);
  }
  runStandPose(1);
  if (currentCommand == "swim") currentCommand = "";
}

inline void runPointPose() {
  Serial.println(F("POINT"));
  setFaceWithMode("point", FACE_ANIM_BOOMERANG);
  const int pointPos[8] = {135, 100, 25, 60, 80, 170, 145, 180};
  moveServosSmoothP(pointPos, 10, 18);
  pD(2000);
  runStandPose(1);
  if (currentCommand == "point") currentCommand = "";
}

inline void runPushupPose() {
  Serial.println(F("PUSHUP"));
  setFaceWithMode("pushup", FACE_ANIM_ONCE);
  runStandPose(0);
  pD(200);
  const int pushupReady[8] = {180, 45, 0, 135, -1, 90, 90, -1};
  moveServosSmoothP(pushupReady, 8, 18);
  pD(400);
  for (int i = 0; i < 4; i++) {
    const int pushDown[8] = {-1, -1, -1, -1, -1, 180, 0, -1};
    moveServosSmoothP(pushDown, 6, 20);
    pD(200);
    const int pushUp[8] = {-1, -1, -1, -1, -1, 90, 90, -1};
    moveServosSmoothP(pushUp, 6, 20);
    pD(200);
  }
  runStandPose(1);
  if (currentCommand == "pushup") currentCommand = "";
}

inline void runBowPose() {
  Serial.println(F("BOW"));
  setFaceWithMode("bow", FACE_ANIM_ONCE);
  runStandPose(0);
  pD(200);
  const int bowFull[8] = {180, 0, 0, 180, 0, 180, 0, 180};
  moveServosSmoothP(bowFull, 10, 20);
  pD(400);
  const int bowHold[8] = {-1, -1, -1, -1, -1, 90, 90, -1};
  moveServosSmoothP(bowHold, 8, 20);
  pD(3000);
  runStandPose(1);
  if (currentCommand == "bow") currentCommand = "";
}

inline void runCutePose() {
  Serial.println(F("CUTE"));
  setFaceWithMode("cute", FACE_ANIM_ONCE);
  runStandPose(0);
  pD(200);
  const int cutePos[8] = {180, 20, 0, 160, 180, 0, 180, 0};
  moveServosSmoothP(cutePos, 10, 18);
  pD(200);
  for (int i = 0; i < 5; i++) {
    const int cuteA[8] = {-1, -1, -1, -1, 180, -1, -1, 45};
    moveServosSmoothP(cuteA, 5, 15);
    const int cuteB[8] = {-1, -1, -1, -1, 135, -1, -1, 0};
    moveServosSmoothP(cuteB, 5, 15);
  }
  runStandPose(1);
  if (currentCommand == "cute") currentCommand = "";
}

inline void runFreakyPose() {
  Serial.println(F("FREAKY"));
  setFaceWithMode("freaky", FACE_ANIM_ONCE);
  runStandPose(0);
  pD(200);
  const int freakyPos[8] = {180, 0, 0, 180, 90, 0, -1, -1};
  moveServosSmoothP(freakyPos, 10, 18);
  pD(200);
  for (int i = 0; i < 3; i++) {
    const int freakyA[8] = {-1, -1, -1, -1, -1, 25, -1, -1};
    moveServosSmoothP(freakyA, 5, 20);
    const int freakyB[8] = {-1, -1, -1, -1, -1, 0, -1, -1};
    moveServosSmoothP(freakyB, 5, 20);
  }
  runStandPose(1);
  if (currentCommand == "freaky") currentCommand = "";
}

inline void runWormPose() {
  Serial.println(F("WORM"));
  setFaceWithMode("worm", FACE_ANIM_ONCE);
  runStandPose(0);
  pD(200);
  const int wormFlat[8] = {180, 0, 0, 180, 90, 90, 90, 90};
  moveServosSmoothP(wormFlat, 10, 18);
  pD(200);
  for (int i = 0; i < 5; i++) {
    const int wormA[8] = {-1, -1, -1, -1, 45, 45, 135, 135};
    moveServosSmoothP(wormA, 6, 18);
    const int wormB[8] = {-1, -1, -1, -1, 135, 135, 45, 45};
    moveServosSmoothP(wormB, 6, 18);
  }
  runStandPose(1);
  if (currentCommand == "worm") currentCommand = "";
}

inline void runShakePose() {
  Serial.println(F("SHAKE"));
  setFaceWithMode("shake", FACE_ANIM_ONCE);
  runStandPose(0);
  pD(200);
  const int shakeReady[8] = {135, 90, 45, 90, 90, 90, 90, 90};
  moveServosSmoothP(shakeReady, 8, 18);
  pD(200);
  for (int i = 0; i < 5; i++) {
    const int shakeA[8] = {-1, -1, -1, -1, 45, -1, -1, 135};
    moveServosSmoothP(shakeA, 4, 15);
    const int shakeB[8] = {-1, -1, -1, -1, 0, -1, -1, 180};
    moveServosSmoothP(shakeB, 4, 15);
  }
  runStandPose(1);
  if (currentCommand == "shake") currentCommand = "";
}

inline void runShrugPose() {
  Serial.println(F("SHRUG"));
  runStandPose(0);
  setFaceWithMode("dead", FACE_ANIM_ONCE);
  const int shrugDown[8] = {-1, -1, -1, -1, 90, 90, 90, 90};
  moveServosSmoothP(shrugDown, 8, 18);
  pD(1000);
  setFaceWithMode("shrug", FACE_ANIM_ONCE);
  const int shrugUp[8] = {-1, -1, -1, -1, 180, 0, 180, 0};
  moveServosSmoothP(shrugUp, 8, 18);
  pD(1500);
  runStandPose(1);
  if (currentCommand == "shrug") currentCommand = "";
}

inline void runDeadPose() {
  Serial.println(F("DEAD"));
  runStandPose(0);
  setFaceWithMode("dead", FACE_ANIM_BOOMERANG);
  const int deadPos[8] = {-1, -1, -1, -1, 90, 90, 90, 90};
  moveServosSmoothP(deadPos, 10, 18);
  if (currentCommand == "dead") currentCommand = "";
}

inline void runCrabPose() {
  Serial.println(F("CRAB"));
  setFaceWithMode("crab", FACE_ANIM_ONCE);
  runStandPose(0);
  pD(200);
  const int crabInit[8] = {90, 90, 90, 90, 0, 180, 45, 135};
  moveServosSmoothP(crabInit, 8, 18);
  for (int i = 0; i < 5; i++) {
    const int crabA[8] = {-1, -1, -1, -1, 45, 135, 0, 180};
    moveServosSmoothP(crabA, 5, 18);
    const int crabB[8] = {-1, -1, -1, -1, 0, 180, 45, 135};
    moveServosSmoothP(crabB, 5, 18);
  }
  runStandPose(1);
  if (currentCommand == "crab") currentCommand = "";
}

// --- MOVEMENT ANIMATIONS ---
// Walk uses frameDelay for timing (user-tunable speed).
// With motorCurrentDelay=0, servo groups write in ~3ms via I2C,
// so diagonal pairs (e.g. R1+L2) effectively move simultaneously.
inline void runWalkPose() {
  Serial.println(F("WALK FWD"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);

  // Each phase takes steps*stepMs ≈ frameDelay ms total.
  // All servos in a phase move simultaneously with ease-in-out curve.
  const int steps  = 5;
  const int stepMs = max(8, frameDelay / steps);

  // Pre-position: lift pair B knees, swing hips to stride start
  { const int t[8] = {135, 100, 25, 135, 0, 135, 45, 180};
    moveServosSmooth(t, steps, stepMs); }
  if (!pressingCheck("forward", 0)) return;

  for (int i = 0; i < walkCycles; i++) {
    // 1: finish lifting R2 knee (pair B airborne)
    { const int t[8] = {-1, -1, -1, -1, -1, 135, 0, -1};
      moveServosSmooth(t, steps, stepMs); }
    if (!pressingCheck("forward", 0)) return;

    // 2: swing pair A (R1+L2) forward + lift their knees simultaneously
    { const int t[8] = {180, -1, -1, 90, 0, -1, -1, 135};
      moveServosSmooth(t, steps, stepMs); }
    if (!pressingCheck("forward", 0)) return;

    // 3: swing pair B hips + lower pair A knees (pair A lands)
    { const int t[8] = {-1, 45, 90, -1, 45, -1, -1, 180};
      moveServosSmooth(t, steps, stepMs); }
    if (!pressingCheck("forward", 0)) return;

    // 4: lift pair A knees + push with pair B hips
    { const int t[8] = {-1, 90, 0, -1, -1, 180, 45, -1};
      moveServosSmooth(t, steps, stepMs); }
    if (!pressingCheck("forward", 0)) return;

    // 5: swing pair A hips back (pair B lands)
    { const int t[8] = {90, -1, -1, 135, -1, -1, -1, -1};
      moveServosSmooth(t, steps, stepMs); }
    if (!pressingCheck("forward", 0)) return;
  }
  runStandPose(1);
}

// Logic reversed from Walk
inline void runWalkBackward() {
  Serial.println(F("WALK BACK"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);
  if (!pressingCheck("backward", frameDelay)) return;

  for (int i = 0; i < walkCycles; i++) {
    setServoAngle(5, 135); setServoAngle(6, 0);
    if (!pressingCheck("backward", frameDelay)) return;
    setServoAngle(7, 135); setServoAngle(3, 135);
    setServoAngle(4, 0); setServoAngle(0, 90);
    if (!pressingCheck("backward", frameDelay)) return;
    setServoAngle(1, 90); setServoAngle(2, 0);
    if (!pressingCheck("backward", frameDelay)) return;
    setServoAngle(4, 45); setServoAngle(7, 180);
    if (!pressingCheck("backward", frameDelay)) return;
    setServoAngle(5, 180); setServoAngle(6, 45);
    setServoAngle(1, 45); setServoAngle(2, 90);
    if (!pressingCheck("backward", frameDelay)) return;
    setServoAngle(3, 90); setServoAngle(0, 180);
    if (!pressingCheck("backward", frameDelay)) return;
  }
  runStandPose(1);
}

// Simple turn logic
inline void runTurnLeft() {
  Serial.println(F("TURN LEFT"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);
  for (int i = 0; i < walkCycles; i++) {
    //legset 1 (R1 L2)
    setServoAngle(5, 135); setServoAngle(7, 135);
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(0, 180); setServoAngle(3, 180);
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(5, 180); setServoAngle(7, 180);
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(0, 135); setServoAngle(3, 135);
    if (!pressingCheck("left", frameDelay)) return;
      //legset 2 (R2 L1)
    setServoAngle(4, 45); setServoAngle(6, 45);
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(1, 90); setServoAngle(2, 90);
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(4, 0); setServoAngle(6, 0);
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(1, 45); setServoAngle(2, 45);
    if (!pressingCheck("left", frameDelay)) return;
  }
  runStandPose(1);
}

inline void runTurnRight() {
  Serial.println(F("TURN RIGHT"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);
  for (int i = 0; i < walkCycles; i++) {
    //legset 2 (R2 L1)
    setServoAngle(4, 45); setServoAngle(6, 45);
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(1, 0); setServoAngle(2, 0);
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(4, 0); setServoAngle(6, 0);
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(1, 45); setServoAngle(2, 45);
    if (!pressingCheck("right", frameDelay)) return;
    //legset 1 (R1 L2)
    setServoAngle(5, 135); setServoAngle(7, 135);
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(0, 90); setServoAngle(3, 90);
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(5, 180); setServoAngle(7, 180);
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(0, 135); setServoAngle(3, 135);
    if (!pressingCheck("right", frameDelay)) return;
  }
  runStandPose(1);
}
