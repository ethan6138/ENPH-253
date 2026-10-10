#include <Arduino.h>

//  -------------------- DRIVETRAIN  -----------------
// Rotary Encoders
constexpr int PIN_GPIO_LMD_A{7};
constexpr int PIN_GPIO_LMD_B{8};
constexpr int PIN_GPIO_RMD_A{9};
constexpr int PIN_GPIO_RMD_B{10};

constexpr int TICKS_PER_WHEEL_REV{static_cast<int>(96 * 2.5)};
constexpr int convertUsToMs{1000};
constexpr float RPMSToRPM{60000.0f};

constexpr unsigned long deBounceWindow(
    250); // us debounce. 8300us per tick at 100rpm. so 4x longer
constexpr TickType_t encoderTaskPeriodMs{
    20}; // how often the RPM state will update

const int8_t quadratureTable[16] = {0,  -1, +1, 0,  +1, 0,  0,  -1,
                                    -1, 0,  0,  +1, 0,  +1, -1, 0};

volatile long tickCount{0L};
volatile uint8_t lastState{0};
volatile unsigned long lastValidEdgeMicro{0};

void IRAM_ATTR encoderISR() {
  uint8_t a = digitalRead(PIN_GPIO_LMD_A);
  uint8_t b = digitalRead(PIN_GPIO_LMD_B);
  uint8_t currentState{(a << 1) | b};

  uint8_t index{(lastState << 2) | currentState};
  int8_t direction{quadratureTable[index]};

  lastState = currentState;

  if (direction == 0) {
    return;
  }

  unsigned long now{micros()};

  if (now - lastValidEdgeMicro < deBounceWindow) {
    return;
  }

  tickCount += direction;
  lastValidEdgeMicro = now;
}

float currentRPM{0.0f};
SemaphoreHandle_t rpmMutex;

void encoderTask(void *pvParameters) {
  unsigned long lastCalcMicros{micros()};
  long lastTickSnapshot{tickCount};

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(encoderTaskPeriodMs));

    unsigned long now{micros()};
    unsigned long elapsedUs{now - lastCalcMicros};

    long currentTicks{tickCount};
    long deltaTicks{currentTicks - lastTickSnapshot};

    float revsPerMs{(float)deltaTicks / TICKS_PER_WHEEL_REV / elapsedUs *
                    convertUsToMs};
    float rpm {revsPerMs * RPMSToRPM};

    if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
      currentRPM = rpm;
      xSemaphoreGive(rpmMutex);
    }

    lastCalcMicros = now;
    lastTickSnapshot = currentTicks;
  }
}
float getEncoderRpm() {
  float result{0.0f};

  if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
    result = currentRPM;
    xSemaphoreGive(rpmMutex);
  }

  return result;
}

long getEncoderTicks() { return tickCount; }

void setup() {
  Serial.begin(115200);

  pinMode(PIN_GPIO_LMD_A, INPUT_PULLUP);
  pinMode(PIN_GPIO_LMD_B, INPUT_PULLUP);

  uint8_t a{digitalRead(PIN_GPIO_LMD_A)};
  uint8_t b{digitalRead(PIN_GPIO_LMD_B)};

  lastState = (a << 1) | b;

  attachInterrupt(digitalPinToInterrupt(PIN_GPIO_LMD_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_GPIO_LMD_B), encoderISR, CHANGE);

  rpmMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(encoderTask, "encoder Task", 4096, nullptr, 1,
                          nullptr, 0);
}

void loop() {
  Serial.print("Ticks: ");
  Serial.println(getEncoderTicks());
  Serial.print("RPM: ");
  Serial.println(getEncoderRpm());

  delay(100);
}


// #include <Arduino.h>

// constexpr int PIN_GPIO_LMD_A{7};
// constexpr int PIN_GPIO_LMD_B{8};

// constexpr int TICKS_PER_REV{96 * 3};
// constexpr float RPMSToRPM{60000.0f};

// const int8_t quadratureTable[16] = {0,  -1, +1, 0,  +1, 0,  0,  -1,
//                                      -1, 0,  0,  +1, 0,  +1, -1, 0};

// volatile long tickCount{0L};
// volatile uint8_t lastState{0};

// void IRAM_ATTR encoderISR() {
//     uint8_t a = digitalRead(PIN_GPIO_LMD_A);
//     uint8_t b = digitalRead(PIN_GPIO_LMD_B);
//     uint8_t currentState{(a << 1) | b};

//     uint8_t index{(lastState << 2) | currentState};
//     tickCount += quadratureTable[index];

//     lastState = currentState;
// }

// void setup() {
//     Serial.begin(115200);

//     pinMode(PIN_GPIO_LMD_A, INPUT_PULLUP);
//     pinMode(PIN_GPIO_LMD_B, INPUT_PULLUP);

//     uint8_t a{digitalRead(PIN_GPIO_LMD_A)};
//     uint8_t b{digitalRead(PIN_GPIO_LMD_B)};
//     lastState = (a << 1) | b;

//     attachInterrupt(digitalPinToInterrupt(PIN_GPIO_LMD_A), encoderISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(PIN_GPIO_LMD_B), encoderISR, CHANGE);
// }

// unsigned long lastCalcMicros{0};
// long lastTickSnapshot{0};

// void loop() {
//     unsigned long now = micros();
//     unsigned long elapsedUs = now - lastCalcMicros;

//     long currentTicks = tickCount;
//     long deltaTicks = currentTicks - lastTickSnapshot;

//     float revsPerUs = (float)deltaTicks / TICKS_PER_REV / elapsedUs;
//     float rpm = revsPerUs * 60000000.0f;  // convert revs/us to revs/min

//     lastCalcMicros = now;
//     lastTickSnapshot = currentTicks;

//     Serial.print("Ticks: ");
//     Serial.print(currentTicks);
//     Serial.print("  RPM: ");
//     Serial.println(rpm);

//     delay(100);
// }