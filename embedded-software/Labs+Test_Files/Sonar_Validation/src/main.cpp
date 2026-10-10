#include <Arduino.h>

constexpr int trigPin = 38;
constexpr int echoPin = 39;

constexpr float speedOfSoundCmPerUs = 0.0343f; // cm per microsecond
constexpr unsigned long echoTimeoutUs = 30000; // ~5m max range cutoff

float readDistanceCm() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long durationUs = pulseIn(echoPin, HIGH, echoTimeoutUs);

  if (durationUs == 0) {
    return -1.0f; // no echo received / out of range
  }

  return (durationUs * speedOfSoundCmPerUs) / 2.0f;
}

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
}

void loop() {
  float distanceCm = readDistanceCm();

  if (distanceCm < 0) {
    Serial.println("No echo received (out of range or timeout)");
  } else {
    Serial.print("Distance: ");
    Serial.print(distanceCm);
    Serial.println(" cm");
  }

  delay(60);
}