#include <Adafruit_GFX.h>     //oled board
#include <Adafruit_SSD1306.h> //oled board
#include <Arduino.h>
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define I2C_SDA 15
#define I2C_SCL 16
#define OLED_RESET -1

#define PIN_PWM_SERVO_1 3
#define PIN_PWM_SERVO_2 2
#define FREQUENCY 100
#define RESOLUTION 12 // range is 0-65535

Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire,
                                 OLED_RESET);

uint32_t anglePos(float angle) {
  uint32_t pulse_us = 500 + (uint32_t)(angle * 2000.0f / 270.0f);
  return pulse_us * 4095UL / 10000UL;
}

void servo1SweepTask(void *pvParameters) {
  float angle1{0.0f};
  int direction1{1};

  while (true) {

    angle1 += direction1;

    if (angle1 >= 270) {
      angle1 = 270;
      direction1 = -1;
    }

    if (angle1 < 0) {
      angle1 = 0;
      direction1 = 1;
    }

    ledcWrite(PIN_PWM_SERVO_1, anglePos(angle1));
    Serial.printf("[%lu mus] Servo 1 -> %.0f deg \n", micros(), angle1);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void servo2SweepTask(void *pvParameters) {
  float angle{0.0f};
  int direction{1};

  while (true) {

    angle += direction;

    if (angle >= 270) {
      angle = 270;
      direction = -1;
    }

    if (angle < 0) {
      angle = 0;
      direction = 1;
    }

    ledcWrite(PIN_PWM_SERVO_2, anglePos(angle));
    Serial.printf("[%lu mus] Servo 2 -> %.0f deg \n", micros(), angle);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void OledSetup() {
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0, 0);
  display_handler.println("Servo motor bullshit");
  display_handler.display();
  delay(500);
}

void setup() {
  Serial.begin(115200);
  // Wire.begin(I2C_SDA, I2C_SCL); // moved here — was a global before

  ledcAttach(PIN_PWM_SERVO_1, FREQUENCY, RESOLUTION);
  ledcAttach(PIN_PWM_SERVO_2, FREQUENCY, RESOLUTION);

  ledcWrite(PIN_PWM_SERVO_1, anglePos(0));
  ledcWrite(PIN_PWM_SERVO_2, anglePos(0));

  // OledSetup();
  delay(5000);

  xTaskCreatePinnedToCore(servo1SweepTask, "servo1", 8192, nullptr, 1, nullptr,
                          0);
  xTaskCreatePinnedToCore(servo2SweepTask, "servo2", 8192, nullptr, 1, nullptr,
                          1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(10000));

  // float targetAngle1{90};
  // float targetAngle2{90};

  // uint32_t angle1 = anglePos(targetAngle1);
  // uint32_t angle2 = anglePos(targetAngle2);

  // ledcWrite(PIN_PWM_SERVO_1, angle1);
  // ledcWrite(PIN_PWM_SERVO_2, angle2);

  // display_handler.clearDisplay();
  // display_handler.setTextSize(1);
  // display_handler.setTextColor(SSD1306_WHITE);
  // display_handler.setCursor(0, 0);

  // display_handler.print("Servo 1: ");
  // display_handler.print(angle1);
  // display_handler.print(", ");
  // display_handler.println(targetAngle1);

  // display_handler.print("Servo 2: ");
  // display_handler.print(angle2);
  // display_handler.print(", ");
  // display_handler.println(targetAngle2);

  // display_handler.display();
  // delay(2000);

  // angle = anglePos(0);
  // ledcWrite(PIN_PWM_SERVO_1, angle);
  // display_handler.clearDisplay();
  // display_handler.setTextSize(1);
  // display_handler.setTextColor(SSD1306_WHITE);
  // display_handler.setCursor(0, 0);
  // display_handler.println(angle);
  // display_handler.display();
  // delay(2000);
}
