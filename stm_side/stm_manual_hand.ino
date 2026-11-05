#include <Servo.h>

// ================================================================
//  FINGER INDEX:  0=1st  1=middle  2=ring  3=lady  4=thumb
// ================================================================

#define NUM_FINGERS 5
#define THUMB  4
#define FIRST  0
#define MIDDLE 1
#define RING   2
#define LADY   3

const char* NAMES[]      = { "1st", "Middle", "Ring", "Lady", "Thumb" };
const int   SERVO_PINS[] = { PA0,    PB8,      PB9,    PA2,    PA1    };
const int   SW_PINS[]    = { PB13,   PB12,     PA4,    PB14,   PA5    };
const bool  CW_IS_PULL[] = { true,   false,    false,  true,   false  };

// ================================================================
//  WRIST  (180° servo, PA8 = TIM1_CH1)
// ================================================================
#define WRIST_PIN     PA8
#define WRIST_DEFAULT 85
Servo wristServo;
int   wristAngle = WRIST_DEFAULT;

// ================================================================
//  TUNABLE PARAMETERS
// ================================================================
int   speedUs[]     = { 100, 100, 100, 100, 100 };     // PWM offset from 1500
long  travelMs[]    = { 2000, 2000, 2000, 2000, 2000 }; // ms for full 0→maxAngle
float maxAngleDeg[] = { 90.0, 90.0, 90.0, 90.0, 90.0 }; // logical max angle

// Pause between homing steps
const int SEQ_DELAY = 600;
const int STOP_US   = 1500;

// ================================================================
//  STATE
// ================================================================
Servo servos[NUM_FINGERS];

long  posMs[NUM_FINGERS]        = { 0 };     // estimated position (ms from home)
long  targetMs[NUM_FINGERS]     = { 0 };     // move target (ms from home)
long  moveStartMs[NUM_FINGERS]  = { 0 };
bool  isMoving[NUM_FINGERS]     = { false };
bool  movingToOpen[NUM_FINGERS] = { false };
bool  isHomed[NUM_FINGERS]      = { false }; // true once homed this session

const unsigned long DEBOUNCE = 50;
unsigned long lastDebounce[NUM_FINGERS] = { 0 };
int lastRaw[NUM_FINGERS]     = { HIGH, HIGH, HIGH, HIGH, HIGH };
int stableState[NUM_FINGERS] = { HIGH, HIGH, HIGH, HIGH, HIGH };

// ================================================================
//  HELPERS
// ================================================================

bool readSwitch(int i) {
  int raw = digitalRead(SW_PINS[i]);
  if (raw != lastRaw[i]) { lastDebounce[i] = millis(); lastRaw[i] = raw; }
  if ((millis() - lastDebounce[i]) > DEBOUNCE) stableState[i] = raw;
  return (stableState[i] == LOW);
}

bool isAtHome(int i)    { return posMs[i] <= 0; }
bool isFullyOpen(int i) { return posMs[i] >= travelMs[i]; }

// Convert position (ms) ↔ angle
float posToAngle(int i) {
  if (travelMs[i] == 0) return 0;
  return ((float)posMs[i] / travelMs[i]) * maxAngleDeg[i];
}
long angleToMs(int i, float deg) {
  deg = constrain(deg, 0.0f, maxAngleDeg[i]);
  return (long)((deg / maxAngleDeg[i]) * travelMs[i]);
}

// ================================================================
//  MOTION PRIMITIVES
// ================================================================

void stopFinger(int i) {
  servos[i].writeMicroseconds(STOP_US);
  if (isMoving[i]) {
    long elapsed = millis() - moveStartMs[i];
    if (movingToOpen[i]) posMs[i] = min(posMs[i] + elapsed, travelMs[i]);
    else                 posMs[i] = max(posMs[i] - elapsed, 0L);
  }
  isMoving[i] = false;
}

void stopAll() { for (int i = 0; i < NUM_FINGERS; i++) stopFinger(i); }

// openFinger / closeFinger always set targetMs so checkSync knows when to stop
void closeFinger(int i) {
  if (isAtHome(i) && readSwitch(i)) return;
  targetMs[i] = 0; // default target: home — override after call for angle moves
  int us = CW_IS_PULL[i] ? STOP_US + speedUs[i] : STOP_US - speedUs[i];
  servos[i].writeMicroseconds(us);
  moveStartMs[i]  = millis();
  movingToOpen[i] = false;
  isMoving[i]     = true;
}

void openFinger(int i) {
  if (isFullyOpen(i)) return;
  targetMs[i] = travelMs[i]; // default target: full open
  int us = CW_IS_PULL[i] ? STOP_US - speedUs[i] : STOP_US + speedUs[i];
  servos[i].writeMicroseconds(us);
  moveStartMs[i]  = millis();
  movingToOpen[i] = true;
  isMoving[i]     = true;
}

// Move to a specific angle (non-blocking)
void moveToAngle(int i, float deg) {
  long target = angleToMs(i, deg);
  if (target == posMs[i]) return;
  if (target > posMs[i]) {
    openFinger(i);
    targetMs[i] = target; // override the "full open" default
  } else {
    closeFinger(i);
    targetMs[i] = target; // override the "home" default
  }
}

// ================================================================
//  BLOCKING HELPERS  (used only in homing sequence)
// ================================================================

void openBlocking(int i) {
  if (isFullyOpen(i)) {
    Serial.print(NAMES[i]); Serial.println(" already open.");
    return;
  }
  Serial.print("  Opening "); Serial.println(NAMES[i]);
  openFinger(i);
  long needed = travelMs[i] - posMs[i];
  delay(needed + 300);
  stopFinger(i);
}

void homeBlocking(int i) {
  Serial.print("  Homing "); Serial.println(NAMES[i]);
  closeFinger(i);
  unsigned long timeout = millis() + travelMs[i] + 1000;
  while (!readSwitch(i)) {
    if (millis() > timeout) {
      stopFinger(i);
      Serial.print("  WARNING: "); Serial.print(NAMES[i]); Serial.println(" timed out!");
      return;
    }
  }
  servos[i].writeMicroseconds(STOP_US);
  posMs[i]   = 0;
  isMoving[i]= false;
  isHomed[i] = true;
  Serial.print("  "); Serial.print(NAMES[i]); Serial.println(" homed.");
}

// Home any number of fingers simultaneously — each stops on its own switch
void homeGroupBlocking(int* fingers, int count) {
  long maxTravel = 0;
  Serial.print("  Homing: ");
  for (int j = 0; j < count; j++) {
    Serial.print(NAMES[fingers[j]]); Serial.print(" ");
    closeFinger(fingers[j]);
    if (travelMs[fingers[j]] > maxTravel) maxTravel = travelMs[fingers[j]];
  }
  Serial.println();

  bool done[NUM_FINGERS] = { false };
  unsigned long timeout = millis() + maxTravel + 1000;

  while (true) {
    bool allDone = true;
    for (int j = 0; j < count; j++) {
      int i = fingers[j];
      if (done[j]) continue;
      if (readSwitch(i)) {
        servos[i].writeMicroseconds(STOP_US);
        posMs[i] = 0; isMoving[i] = false; isHomed[i] = true; done[j] = true;
        Serial.print("  "); Serial.print(NAMES[i]); Serial.println(" homed.");
      } else {
        allDone = false;
      }
    }
    if (allDone) break;
    if (millis() > timeout) {
      for (int j = 0; j < count; j++) {
        if (!done[j]) {
          stopFinger(fingers[j]);
          Serial.print("  WARNING: "); Serial.print(NAMES[fingers[j]]); Serial.println(" timed out!");
        }
      }
      break;
    }
  }
}

// Open multiple fingers simultaneously (blocking), waits for the slowest
void openGroupBlocking(int* fingers, int count) {
  long maxNeeded = 0;
  for (int j = 0; j < count; j++) {
    int i = fingers[j];
    if (!isFullyOpen(i)) {
      openFinger(i);
      long needed = travelMs[i] - posMs[i];
      if (needed > maxNeeded) maxNeeded = needed;
    }
  }
  if (maxNeeded > 0) delay(maxNeeded + 300);
  for (int j = 0; j < count; j++) stopFinger(fingers[j]);
}

// ================================================================
//  HOMING SEQUENCE 
//
//  1. Open thumb only if it's not already clear
//  2. Home 1st + Middle + Lady + Ring simultaneously
//  3. Open those four simultaneously
//  4. Home thumb
// ================================================================

void homeAll() {
  Serial.println("\n=== Homing sequence start ===");

  // Step 1: clear thumb only if needed
  if (!isFullyOpen(THUMB)) {
    Serial.println("[1] Thumb not clear — opening...");
    openBlocking(THUMB);
    delay(SEQ_DELAY);
  } else {
    Serial.println("[1] Thumb already clear.");
  }

  // Step 2: home all four fingers at once
  Serial.println("[2] Homing 1st + Middle + Lady + Ring...");
  int group[] = { FIRST, MIDDLE, LADY, RING };
  homeGroupBlocking(group, 4);
  delay(SEQ_DELAY);

  // Step 3: open all four to clear thumb path
  Serial.println("[3] Opening all fingers to clear thumb...");
  openGroupBlocking(group, 4);
  delay(SEQ_DELAY);

  // Step 4: home thumb
  Serial.println("[4] Homing thumb...");
  homeBlocking(THUMB);

  Serial.println("=== Homing complete ===\n");
}

// ================================================================
//  OPEN / CLOSE ALL
// ================================================================

void openAll() {
  Serial.println("\n--- Opening all ---");
  openBlocking(THUMB);
  delay(SEQ_DELAY);
  for (int i = 0; i < NUM_FINGERS; i++)
    if (i != THUMB) openFinger(i);
  Serial.println("Opening remaining fingers.");
}

void closeAll() {
  Serial.println("\n--- Closing all ---");
  openBlocking(THUMB);
  delay(SEQ_DELAY);
  closeFinger(LADY);  closeFinger(RING);
  delay(SEQ_DELAY);
  closeFinger(FIRST); closeFinger(MIDDLE);
  delay(SEQ_DELAY);
  closeFinger(THUMB);
  Serial.println("Closing sequence done.");
}

// ================================================================
//  SYNC CHECK  — runs every loop, handles limit switches + angle targets
// ================================================================

void checkSync() {
  for (int i = 0; i < NUM_FINGERS; i++) {
    if (!isMoving[i]) continue;

    long elapsed = millis() - moveStartMs[i];
    long estPos  = movingToOpen[i]
                     ? posMs[i] + elapsed
                     : posMs[i] - elapsed;
    estPos = constrain(estPos, 0L, travelMs[i]);

    // Limit switch hit while closing → re-sync to home
    if (!movingToOpen[i] && readSwitch(i)) {
      stopFinger(i);
      posMs[i]   = 0;
      isHomed[i] = true;
      Serial.print(NAMES[i]); Serial.println(" re-synced at home.");
      continue;
    }

    // Reached open target
    if (movingToOpen[i] && estPos >= targetMs[i]) {
      stopFinger(i);
      posMs[i] = targetMs[i];
      if (targetMs[i] >= travelMs[i]) {
        Serial.print(NAMES[i]); Serial.println(" fully open.");
      } else {
        Serial.print(NAMES[i]); Serial.print(" at ");
        Serial.print(posToAngle(i), 1); Serial.println("°");
      }
    }

    // Reached close target (angle stop, not necessarily home)
    if (!movingToOpen[i] && estPos <= targetMs[i]) {
      stopFinger(i);
      posMs[i] = targetMs[i];
      if (targetMs[i] == 0) {
        Serial.print(NAMES[i]); Serial.println(" at home.");
      } else {
        Serial.print(NAMES[i]); Serial.print(" at ");
        Serial.print(posToAngle(i), 1); Serial.println("°");
      }
    }
  }
}

// ================================================================
//  WRIST
// ================================================================

void setWrist(int angle) {
  wristAngle = constrain(angle, 0, 180);
  wristServo.write(wristAngle);
  Serial.print("Wrist → "); Serial.print(wristAngle); Serial.println("°");
}

// ================================================================
//  SERIAL COMMANDS
// ================================================================
//  h          → home all (safe sequence)
//  o          → open all
//  c          → close all
//  o0–o4      → open single finger (full)
//  c0–c4      → close single finger (to home)
//  s0–s4      → stop single finger
//  S          → STOP ALL immediately
//  a0 45.0    → move finger 0 to 45° (uses angle+speed, stops at target)
//  t0 2000    → set travel time for finger 0 (ms)
//  v0 120     → set speed offset for finger 0 (µs)
//  m0 90      → set max angle for finger 0 (°)
//  w 90       → set wrist to 90°
//  p          → print positions, angles, homed status

void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "h") { homeAll();  return; }
  if (cmd == "o") { openAll();  return; }
  if (cmd == "c") { closeAll(); return; }
  if (cmd == "S") { stopAll();  Serial.println("STOP ALL."); return; }

  if (cmd == "p") {
    Serial.println("\n-- Positions --");
    for (int i = 0; i < NUM_FINGERS; i++) {
      Serial.print("  "); Serial.print(NAMES[i]);
      Serial.print(": "); Serial.print(posToAngle(i), 1); Serial.print("°");
      Serial.print("  ("); Serial.print(posMs[i]); Serial.print("/");
      Serial.print(travelMs[i]); Serial.print(" ms, max ");
      Serial.print(maxAngleDeg[i], 0); Serial.print("°)");
      Serial.println(isHomed[i] ? "  [homed]" : "  [not homed]");
    }
    Serial.print("  Wrist: "); Serial.print(wristAngle); Serial.println("°");
    return;
  }

  char action = cmd.charAt(0);

  // Wrist: "w 90"
  if (action == 'w') {
    int val = cmd.substring(2).toInt();
    setWrist(val);
    return;
  }

  // All other commands: char + digit index
  int idx = cmd.charAt(1) - '0';
  if (idx < 0 || idx >= NUM_FINGERS) { Serial.println("Unknown command."); return; }

  if (action == 'o') { openFinger(idx);  Serial.print("Opening ");  Serial.println(NAMES[idx]); return; }
  if (action == 'c') { closeFinger(idx); Serial.print("Closing ");  Serial.println(NAMES[idx]); return; }
  if (action == 's') { stopFinger(idx);  Serial.print("Stopping "); Serial.println(NAMES[idx]); return; }

  if (action == 'a') {
    float deg = cmd.substring(3).toFloat();
    moveToAngle(idx, deg);
    Serial.print(NAMES[idx]); Serial.print(" → "); Serial.print(deg, 1); Serial.println("°");
    return;
  }
  if (action == 't') {
    long val = cmd.substring(3).toInt();
    if (val > 0) { travelMs[idx] = val; Serial.print(NAMES[idx]); Serial.print(" travelMs="); Serial.println(val); }
    return;
  }
  if (action == 'v') {
    int val = cmd.substring(3).toInt();
    if (val > 0 && val <= 400) { speedUs[idx] = val; Serial.print(NAMES[idx]); Serial.print(" speed="); Serial.println(val); }
    return;
  }
  if (action == 'm') {
    float val = cmd.substring(3).toFloat();
    if (val > 0 && val <= 360) { maxAngleDeg[idx] = val; Serial.print(NAMES[idx]); Serial.print(" maxAngle="); Serial.print(val, 1); Serial.println("°"); }
    return;
  }

  Serial.println("Unknown command.");
}

// ================================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  for (int i = 0; i < NUM_FINGERS; i++) {
    pinMode(SW_PINS[i], INPUT_PULLUP);
    servos[i].attach(SERVO_PINS[i]);
    servos[i].writeMicroseconds(STOP_US);
  }

  wristServo.attach(WRIST_PIN);
  wristServo.write(WRIST_DEFAULT);
  delay(500);

  Serial.println("=== Bionic Hand v4 ===");
  Serial.println("h=home | o=open | c=close | S=STOP");
  Serial.println("o0-o4 | c0-c4 | s0-s4");
  Serial.println("a0 45.0=angle | m0 90=maxAngle");
  Serial.println("t0 2000=travel | v0 120=speed");
  Serial.println("w 90=wrist | p=print");
  Serial.println("======================\n");
  // Positions unknown until homed — send 'h' to start
}

void loop() {
  checkSync();
  handleSerial();
}
