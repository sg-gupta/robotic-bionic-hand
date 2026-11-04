#include <Servo.h>

// ── Curl servo pins ──────────────────────────────────────────────
// 0=1st  1=mid  2=ring  3=lady  4=thumb
#define NUM_FINGERS 5
const int   SERVO_PINS[] = { PA0,  PB8,   PB9,  PA2,  PA1  };
const int   SW_PINS[]    = { PB13, PB12,  PA4,  PB14, PA5  };
const bool  CW_IS_PULL[] = { false, true, true, false, true };

// ── Wrist ────────────────────────────────────────────────────────
#define WRIST_PIN     PA8
#define WRIST_DEFAULT 90

// ── Knuckle servo pins (abduction/adduction, 180° position servos)
// 0=index 1=middle 2=ring 3=pinky 4=thumb
// Note: PA6/PA7/PB0/PB1 are TIM3 — verify no conflict on your build
const int KNUCKLE_PINS[]  = { PA3, PA6, PA7, PB0, PB1 };
#define KNUCKLE_NEUTRAL  90   // degrees — fingers together
#define KNUCKLE_STEP_MS  12   // ms between each step (lower = faster)
#define KNUCKLE_STEP_DEG  1   // degrees per step

// ── Curl tuning ──────────────────────────────────────────────────
const int   SPEED_US  = 120;
const long  TRAVEL_MS = 2000;
const float MAX_ANGLE = 90.0f;
const int   STOP_US   = 1500;

// ── State ────────────────────────────────────────────────────────
Servo servos[NUM_FINGERS];
Servo wristServo;
Servo knuckles[NUM_FINGERS];

// Curl
long posMs[NUM_FINGERS]       = {0};
long targetMs[NUM_FINGERS]    = {0};
long moveStartMs[NUM_FINGERS] = {0};
bool isMoving[NUM_FINGERS]    = {false};
bool toOpen[NUM_FINGERS]      = {false};

// Knuckle
int  knuckleCurrent[NUM_FINGERS];
int  knuckleTarget[NUM_FINGERS];
unsigned long lastKnuckleStep = 0;

// ── Curl helpers ─────────────────────────────────────────────────
long angleToMs(float deg) {
  return (long)(constrain(deg, 0.0f, MAX_ANGLE) / MAX_ANGLE * TRAVEL_MS);
}

void stopFinger(int i) {
  servos[i].writeMicroseconds(STOP_US);
  if (isMoving[i]) {
    long elapsed = millis() - moveStartMs[i];
    if (toOpen[i]) posMs[i] = min(posMs[i] + elapsed, TRAVEL_MS);
    else           posMs[i] = max(posMs[i] - elapsed, 0L);
  }
  isMoving[i] = false;
}

void moveTo(int i, long target) {
  if (abs(target - posMs[i]) < 30) return;
  if (isMoving[i]) stopFinger(i);

  targetMs[i]    = constrain(target, 0L, (long)TRAVEL_MS);
  moveStartMs[i] = millis();
  toOpen[i]      = (targetMs[i] > posMs[i]);

  int us = toOpen[i]
    ? (CW_IS_PULL[i] ? STOP_US - SPEED_US : STOP_US + SPEED_US)
    : (CW_IS_PULL[i] ? STOP_US + SPEED_US : STOP_US - SPEED_US);

  servos[i].writeMicroseconds(us);
  isMoving[i] = true;
}

// ── Curl sync check ───────────────────────────────────────────────
void checkSync() {
  for (int i = 0; i < NUM_FINGERS; i++) {
    if (!isMoving[i]) continue;

    long elapsed = millis() - moveStartMs[i];
    long est     = toOpen[i] ? posMs[i] + elapsed : posMs[i] - elapsed;
    est          = constrain(est, 0L, (long)TRAVEL_MS);

    if (!toOpen[i] && digitalRead(SW_PINS[i]) == LOW) {
      servos[i].writeMicroseconds(STOP_US);
      posMs[i]    = 0;
      isMoving[i] = false;
      continue;
    }

    bool arrived = toOpen[i] ? (est >= targetMs[i]) : (est <= targetMs[i]);
    if (arrived) {
      servos[i].writeMicroseconds(STOP_US);
      posMs[i]    = targetMs[i];
      isMoving[i] = false;
    }
  }
}

// ── Knuckle smooth stepping ───────────────────────────────────────
void updateKnuckles() {
  if (millis() - lastKnuckleStep < KNUCKLE_STEP_MS) return;
  lastKnuckleStep = millis();

  for (int i = 0; i < NUM_FINGERS; i++) {
    if (knuckleCurrent[i] == knuckleTarget[i]) continue;
    if (knuckleCurrent[i] < knuckleTarget[i])
      knuckleCurrent[i] = min(knuckleCurrent[i] + KNUCKLE_STEP_DEG, knuckleTarget[i]);
    else
      knuckleCurrent[i] = max(knuckleCurrent[i] - KNUCKLE_STEP_DEG, knuckleTarget[i]);
    knuckles[i].write(knuckleCurrent[i]);
  }
}

// ── Parse "P f0 f1 f2 f3 f4 w k0 k1 k2 k3 k4\n" ─────────────────
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.charAt(0) != 'P') return;

  float val[11] = {0};
  int   idx = 0, pos = 2;
  while (idx < 11 && pos < (int)line.length()) {
    int sp = line.indexOf(' ', pos);
    if (sp == -1) sp = line.length();
    val[idx++] = line.substring(pos, sp).toFloat();
    pos = sp + 1;
  }

  // Curl fingers (val[0..4])
  for (int i = 0; i < NUM_FINGERS; i++)
    moveTo(i, angleToMs(val[i]));

  // Wrist (val[5])
  wristServo.write((int)constrain(val[5], 0, 180));

  // Knuckles (val[6..9] = index, middle, ring, pinky)
  for (int i = 0; i < 4; i++)
    knuckleTarget[i] = (int)constrain(val[6 + i], 0, 180);

  // Thumb knuckle (val[10]) — mechanically inverted
  knuckleTarget[4] = (int)constrain(180 - val[10], 0, 180);
}

// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_FINGERS; i++) {
    pinMode(SW_PINS[i], INPUT_PULLUP);
    servos[i].attach(SERVO_PINS[i]);
    servos[i].writeMicroseconds(STOP_US);
  }

  wristServo.attach(WRIST_PIN);
  wristServo.write(WRIST_DEFAULT);

  for (int i = 0; i < NUM_FINGERS; i++) {
    knuckles[i].attach(KNUCKLE_PINS[i]);
    knuckleCurrent[i] = KNUCKLE_NEUTRAL;
    knuckleTarget[i]  = KNUCKLE_NEUTRAL;
    knuckles[i].write(KNUCKLE_NEUTRAL);
  }

  Serial.println("Ready.");
}

void loop() {
  checkSync();
  updateKnuckles();
  handleSerial();
}
