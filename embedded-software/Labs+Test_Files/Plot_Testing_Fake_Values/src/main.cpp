// fake_imu_test.cpp
// Standalone sketch with NO real IMU hardware involved.
// Just prints fake wiggling roll/pitch values in the same
// "RollDeg:xx PitchDeg:xx" format the real IMU test uses,
// so the Python plotting pipeline can be validated independently.

#include <Arduino.h>

float fakeTimeSeconds = 0.0f;

void setup() {
    Serial.begin(115200);
    delay(1000);  // give serial monitor/plotter time to connect
    Serial.println("Fake IMU test starting...");
}

void loop() {
    // Generate smooth fake wiggling values using sine waves,
    // just so the plot has something meaningful-looking to draw
    float fakeRoll = 30.0f * sin(fakeTimeSeconds);        // oscillates -30 to +30 deg
    float fakePitch = 15.0f * sin(fakeTimeSeconds * 0.5f); // oscillates -15 to +15 deg, slower

    Serial.print("RollDeg:");
    Serial.print(fakeRoll);
    Serial.print(" PitchDeg:");
    Serial.println(fakePitch);

    fakeTimeSeconds += 0.05f;

    delay(50);  // matches the 50ms print rate the real test uses
}