#include <Arduino.h>

constexpr int trigPin{38};
constexpr int echoPin{39};

constexpr float speedOfSoundCmPerUs{0.0343f}; // cm per microsecond
constexpr TickType_t triggerPeriodMs{20};

volatile unsigned long echoRiseUs{0};
volatile unsigned long echoFallUs{0};
volatile bool echoCapture{false};

float currentDistance{-1.0f};
SemaphoreHandle_t distanceMutex;

void IRAM_ATTR echoISR() {
  if (digitalRead(echoPin) == HIGH)
    echoRiseUs = micros();

  else {
    echoFallUs = micros();
    echoCapture = true;
  }
}

void sonarTask(void *pvParameters) {
  while (true) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    vTaskDelay(pdMS_TO_TICKS(triggerPeriodMs));

    if (echoCapture) {
      unsigned long durationUs{echoFallUs - echoRiseUs};
      float distanceCm{(durationUs * speedOfSoundCmPerUs) / 2.0f};

      if (xSemaphoreTake(distanceMutex, portMAX_DELAY) == pdTRUE) {
        currentDistance = distanceCm;
        xSemaphoreGive(distanceMutex);
      }

      Serial.print("Distance: ");
      Serial.print(distanceCm);
      Serial.println("cm");
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  distanceMutex = xSemaphoreCreateMutex();
  attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);

  xTaskCreatePinnedToCore(sonarTask, "sonarTask", 2048, nullptr, 1, nullptr, 1);
}

void loop() {}