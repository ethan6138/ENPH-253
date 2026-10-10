#include "Encoder.h"

#include <helpers.h>
#include <pins.h>
#include <types.h>

#include <Arduino.h>

namespace {
constexpr float RPUSToRPM{60000000.0f};

constexpr unsigned long
    deBounceWindow(1000); // us debounce. 8300us per tick at 100rpm
constexpr TickType_t encoderTaskPeriodMs{
    100}; // how often the RPM state will update

constexpr int8_t quadratureTable[16] = {0,  -1, +1, 0,  +1, 0,  0,  -1,
                                        -1, 0,  0,  +1, 0,  +1, -1, 0};

struct EncoderState {
  int pinA{};
  int pinB{};

  volatile long tickCount{};
  volatile uint8_t lastState{};
  volatile unsigned long lastValidEdgeMicro{};

  float currentRpm{};

  SemaphoreHandle_t rpmMutex;
  portMUX_TYPE tickMux = portMUX_INITIALIZER_UNLOCKED; // per-encoder
};

EncoderState leftEncoder{};
EncoderState rightEncoder{};

void IRAM_ATTR processEncoder(EncoderState *encoder) {
  uint8_t a = digitalRead(encoder->pinA);
  uint8_t b = digitalRead(encoder->pinB);
  uint8_t currentState = (a << 1) | b;

  uint8_t index = (encoder->lastState << 2) | currentState;
  int8_t direction{quadratureTable[index]};

  encoder->lastState = currentState;

  if (direction == 0) {
    return;
  }

  unsigned long now{micros()};

  if (now - encoder->lastValidEdgeMicro < deBounceWindow) {
    return;
  }

  portENTER_CRITICAL_ISR(&encoder->tickMux);
  encoder->tickCount += direction;
  portEXIT_CRITICAL_ISR(&encoder->tickMux);
  encoder->lastValidEdgeMicro = now;
}

void IRAM_ATTR leftEncoderISR() { processEncoder(&leftEncoder); }

void IRAM_ATTR rightEncoderISR() { processEncoder(&rightEncoder); }

void initEncoderState(EncoderState &encoder, int pinA, int pinB) {
  encoder.pinA = pinA;
  encoder.pinB = pinB;
  encoder.tickCount = 0;
  encoder.currentRpm = 0.0f;

  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);

  uint8_t a = digitalRead(pinA);
  uint8_t b = digitalRead(pinB);
  encoder.lastState = (a << 1) | b;
  encoder.lastValidEdgeMicro = 0;

  encoder.rpmMutex = xSemaphoreCreateMutex();
}

float computeRpmFromDelta(long deltaTicks, unsigned long elapsedUs) {
  float revsPerUs =
      static_cast<float>(deltaTicks) / TICKS_PER_WHEEL_REV / elapsedUs;
  return revsPerUs * RPUSToRPM;
}

long readTickCount(EncoderState &encoder) {
  long result{};
  portENTER_CRITICAL(&encoder.tickMux);
  result = encoder.tickCount;
  portEXIT_CRITICAL(&encoder.tickMux);
  return result;
}

void encoderTask(void *pvParameters) {
  unsigned long lastCalcMicros{micros()};
  long lastLeftSnapshot{readTickCount(leftEncoder)};
  long lastRightSnapshot{readTickCount(rightEncoder)};

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(encoderTaskPeriodMs));

    unsigned long now{micros()};
    unsigned long elapsedUs{now - lastCalcMicros};

    long currentLeft = readTickCount(leftEncoder);
    long currentRight = readTickCount(rightEncoder);

    float rpmLeft =
        computeRpmFromDelta(currentLeft - lastLeftSnapshot, elapsedUs);
    float rpmRight =
        computeRpmFromDelta(currentRight - lastRightSnapshot, elapsedUs);

    if (xSemaphoreTake(leftEncoder.rpmMutex, portMAX_DELAY) == pdTRUE) {
      leftEncoder.currentRpm = rpmLeft;
      xSemaphoreGive(leftEncoder.rpmMutex);
    }
    if (xSemaphoreTake(rightEncoder.rpmMutex, portMAX_DELAY) == pdTRUE) {
      rightEncoder.currentRpm = rpmRight;
      xSemaphoreGive(rightEncoder.rpmMutex);
    }

    lastCalcMicros = now;
    lastLeftSnapshot = currentLeft;
    lastRightSnapshot = currentRight;
  }
}
} // namespace

void encoder_init() {
  initEncoderState(leftEncoder, PIN_GPIO_LMD_B, PIN_GPIO_LMD_A);
  initEncoderState(rightEncoder, PIN_GPIO_RMD_B, PIN_GPIO_RMD_A);

  attachInterrupt(digitalPinToInterrupt(leftEncoder.pinA), leftEncoderISR,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoder.pinB), leftEncoderISR,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoder.pinA), rightEncoderISR,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoder.pinB), rightEncoderISR,
                  CHANGE);

  xTaskCreatePinnedToCore(encoderTask, "encoder Task", 4096, nullptr, 1,
                          nullptr, 0);
}

float getEncoderRpmLeft() {
  float result = 0.0f;
  if (xSemaphoreTake(leftEncoder.rpmMutex, portMAX_DELAY) == pdTRUE) {
    result = leftEncoder.currentRpm;
    xSemaphoreGive(leftEncoder.rpmMutex);
  }
  return result;
}

float getEncoderRpmRight() {
  float result = 0.0f;
  if (xSemaphoreTake(rightEncoder.rpmMutex, portMAX_DELAY) == pdTRUE) {
    result = rightEncoder.currentRpm;
    xSemaphoreGive(rightEncoder.rpmMutex);
  }
  return result;
}

long getEncoderTicksLeft() { return readTickCount(leftEncoder); }
long getEncoderTicksRight() { return readTickCount(rightEncoder); }