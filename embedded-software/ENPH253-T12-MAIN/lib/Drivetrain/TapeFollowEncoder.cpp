#include "TapeFollowEncoder.h"

#include <Pose.h>

#include <pins.h>
#include <types.h>

// distance in cm from center of tape sensors
constexpr float POS_OUTER_LEFT{-3.2f};
constexpr float POS_INNER_LEFT{-0.7f};
constexpr float POS_INNER_RIGHT{0.7f};
constexpr float POS_OUTER_RIGHT{3.2f};

// Tape deteced high or low?
constexpr int TAPE_DETECTED_LEVEL{HIGH};

// change constexpr after we tune
float baseRpm{80.0f}; // might be a bit low
float approachRpm{40.0f};
float kp{2.5f};
float kd{1.0f};

volatile bool hasNotCaughtTapeYet{true};

constexpr float lastKnownBoost{22.5f};

float lastError{0.0f};
float lastKnownDirection{0.0f};

unsigned long setDoubleTapeCheckTime{};
volatile unsigned long doubleTapeStartTime{};

volatile int tapeDetectedLeft{};
volatile int tapeDetectedRight{};

// --- Side tape blip watch ---
// Debounce window
// constexpr unsigned long sideBlipDebounceMs{10};

// volatile bool sideTapeWatchActive{false};
// volatile bool sideTapeWatchDone{false};
// volatile int sideTapeBlipCount{0};
// volatile unsigned long lastSideBlipTime{0};
// volatile unsigned long sideBlipMaxGapMs{defaultSideBlipMaxGapMs};

void tapeFollowReset() {
  hasNotCaughtTapeYet = true;
  lastError = 0.0f;
  lastKnownDirection = 0.0f;
}

void IRAM_ATTR outerLeftISR() {
  tapeDetectedLeft += 1;

  // if (sideTapeWatchActive && !sideTapeWatchDone) {
  //   // only trust this as a real side blip if we're still tracking the main
  //   // line (an inner sensor on tape) rather than genuinely weaving off it -
  //   // that's what should distinguish the side marks from ordinary steering
  //   bool innerLeftOn = (digitalRead(PIN_GPIO_TAPE_2) == TAPE_DETECTED_LEVEL);
  //   bool innerRightOn = (digitalRead(PIN_GPIO_TAPE_3) ==
  //   TAPE_DETECTED_LEVEL);

  //   if (innerLeftOn || innerRightOn) {
  //     unsigned long now = millis();

  //     // debounce: ignore re-triggers that belong to the same physical blip
  //     if ((now - lastSideBlipTime) > sideBlipDebounceMs) {
  //       // if too much time has passed since the previous blip, this is a
  //       // fresh "first" blip rather than a "second" one
  //       if (sideTapeBlipCount > 0 &&
  //           (now - lastSideBlipTime) > sideBlipMaxGapMs) {
  //         sideTapeBlipCount = 0;
  //       }

  //       sideTapeBlipCount += 1;
  //       lastSideBlipTime = now;

  //       if (sideTapeBlipCount >= 2) {
  //         sideTapeWatchDone = true;
  //         sideTapeWatchActive = false;
  //       }
  //     }
  //   }
  // }
}
void IRAM_ATTR outerRightISR() { tapeDetectedRight += 1; }

bool tapeCheckStart(int pin, unsigned long timeInterval) {
  setDoubleTapeCheckTime = timeInterval;

  // Reset pin counts
  tapeDetectedLeft = 0;
  tapeDetectedRight = 0;

  bool usingLeft{pin == PIN_GPIO_TAPE_1};

  while (!((usingLeft) ? static_cast<bool>(tapeDetectedLeft)
                       : static_cast<bool>(tapeDetectedRight))) {
    vTaskDelay(1);
  }

  doubleTapeStartTime = micros();

  // keep checking till we either time out or get second tape
  while (((usingLeft) ? (tapeDetectedLeft < 2) : (tapeDetectedRight < 2)) &&
         ((micros() - doubleTapeStartTime) < setDoubleTapeCheckTime)) {
    vTaskDelay(1);
  }

  // if we time out then return false
  if ((micros() - doubleTapeStartTime) >= setDoubleTapeCheckTime)
    return false;

  // default return true
  return true;
}

void waitForTapeCrossing(unsigned long crossingWindowMs) {
  bool seen1{false};
  bool seen2{false};
  bool seen3{false};
  bool seen4{false};
  unsigned long firstSeenTime{0};

  for (;;) {
    if (digitalRead(PIN_GPIO_TAPE_1) == TAPE_DETECTED_LEVEL)
      seen1 = true;
    if (digitalRead(PIN_GPIO_TAPE_2) == TAPE_DETECTED_LEVEL)
      seen2 = true;
    if (digitalRead(PIN_GPIO_TAPE_3) == TAPE_DETECTED_LEVEL)
      seen3 = true;
    if (digitalRead(PIN_GPIO_TAPE_4) == TAPE_DETECTED_LEVEL)
      seen4 = true;

    if (firstSeenTime == 0 && (seen1 || seen2 || seen3 || seen4)) {
      firstSeenTime = millis();
    }

    if (seen1 && seen2 && seen3 && seen4) {
      return;
    }

    if (firstSeenTime != 0 && (millis() - firstSeenTime) > crossingWindowMs) {
      // partial trigger, so discard and wait for next
      seen1 = seen2 = seen3 = seen4 = false;
      firstSeenTime = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

float computeWeightedError() {
  bool outerLeftOn = (digitalRead(PIN_GPIO_TAPE_1) == TAPE_DETECTED_LEVEL);
  bool innerLeftOn = (digitalRead(PIN_GPIO_TAPE_2) == TAPE_DETECTED_LEVEL);
  bool innerRightOn = (digitalRead(PIN_GPIO_TAPE_3) == TAPE_DETECTED_LEVEL);
  bool outerRightOn = (digitalRead(PIN_GPIO_TAPE_4) == TAPE_DETECTED_LEVEL);

  float weightedSum{0.0f};
  int triggeredCount{0};

  // first if ignores far tape if inner is on to prevent too much jerk driving
  // by double tape
  if ((outerLeftOn && innerRightOn) || (outerRightOn && innerLeftOn)) {
    weightedSum += (innerRightOn) ? POS_INNER_RIGHT : POS_INNER_LEFT;
    triggeredCount++;
  } else { // general weighted sums
    if (outerLeftOn) {
      weightedSum += 12 * POS_OUTER_LEFT;
      triggeredCount++;
    }
    if (innerLeftOn) {
      weightedSum += POS_INNER_LEFT;
      triggeredCount++;
    }
    if (innerRightOn) {
      weightedSum += POS_INNER_RIGHT;
      triggeredCount++;
    }
    if (outerRightOn) {
      weightedSum += 12 * POS_OUTER_RIGHT;
      triggeredCount++;
    }
  }
  if (triggeredCount == 0) {
    return (lastKnownDirection >= 0.0f) ? lastKnownBoost : -lastKnownBoost;
  }

  float error = weightedSum / triggeredCount;
  lastKnownDirection = error;
  return error;
}

Wheels::RpmTargets tapeFollow() {
  if (hasNotCaughtTapeYet) {
    bool anyTapeDetected =
        (digitalRead(PIN_GPIO_TAPE_1) == TAPE_DETECTED_LEVEL) ||
        (digitalRead(PIN_GPIO_TAPE_2) == TAPE_DETECTED_LEVEL) ||
        (digitalRead(PIN_GPIO_TAPE_3) == TAPE_DETECTED_LEVEL) ||
        (digitalRead(PIN_GPIO_TAPE_4) == TAPE_DETECTED_LEVEL);

    if (!anyTapeDetected) {
      return Wheels::RpmTargets{approachRpm, approachRpm};
    }
    hasNotCaughtTapeYet =
        false; // caught tape — hand off to real steering from here on
  }

  float error = computeWeightedError();
  float p = kp * error;
  float d = kd * (error - lastError);
  float turnEffort = p + d;

  lastError = error;

  Wheels::RpmTargets targets{};

  targets.rpmLeft = baseRpm + turnEffort;
  targets.rpmRight = baseRpm - turnEffort;

  return targets;
}

void tapeFollowSetKp(float newKp) { kp = newKp; }
void tapeFollowSetKd(float newKd) { kd = newKd; }
void setApproachRpm(float newApproachRpm) { approachRpm = newApproachRpm; }
void setBaseSpeedRpm(float newBaseSpeed) { baseRpm = newBaseSpeed; }

void tapeFollow_init() {
  // looking from the top, with the top edge as the front of the bot
  pinMode(PIN_GPIO_TAPE_1, INPUT); // outer left
  pinMode(PIN_GPIO_TAPE_2, INPUT); // inner left
  pinMode(PIN_GPIO_TAPE_3, INPUT); // inner right
  pinMode(PIN_GPIO_TAPE_4, INPUT); // outer right

  // For scanning double tape off outer tape sensors
  attachInterrupt(digitalPinToInterrupt(PIN_GPIO_TAPE_1), outerLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_GPIO_TAPE_4), outerRightISR,
                  RISING);
}

// void startSideTapeWatch(unsigned long maxGapMs) {
//   sideBlipMaxGapMs = maxGapMs;
//   sideTapeBlipCount = 0;
//   lastSideBlipTime = 0;
//   sideTapeWatchDone = false;
//   sideTapeWatchActive = true; // arm last so the ISR never sees a half-reset
//   state
// }

// void stopSideTapeWatch() { sideTapeWatchActive = false; }

// bool sideTapeWatchState() { return sideTapeWatchDone; }

// int getSideTapeBlipCount() { return sideTapeBlipCount; }

TapeSensorStates getTapeSensorStates() {
  TapeSensorStates s;
  s.s1 = (digitalRead(PIN_GPIO_TAPE_1) == TAPE_DETECTED_LEVEL);
  s.s2 = (digitalRead(PIN_GPIO_TAPE_2) == TAPE_DETECTED_LEVEL);
  s.s3 = (digitalRead(PIN_GPIO_TAPE_3) == TAPE_DETECTED_LEVEL);
  s.s4 = (digitalRead(PIN_GPIO_TAPE_4) == TAPE_DETECTED_LEVEL);
  s.s5 = (digitalRead(PIN_GPIO_TAPE_5) == TAPE_DETECTED_LEVEL);
  return s;
}