#include <Adafruit_GFX.h>     //oled board
#include <Adafruit_SSD1306.h> //oled board
#include <Arduino.h>
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define I2C_SDA 15
#define I2C_SCL 16
#define OLED_RESET -1

#define PIN_PWM1 2
#define PIN_PWM 3
#define FREQUENCY 100
#define RESOLUTION 12 // range is 0-4095

Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire,
                                 OLED_RESET);

uint32_t anglePos(float angle) {
  float pulseUs = 500 + (angle * (2500.0f - 500.0f) / 270.0f);
  return static_cast<uint32_t>(pulseUs * 4095.0f / 10000.0f + 0.5f);
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
  Wire.begin(I2C_SDA, I2C_SCL); // moved here — was a global before

  ledcAttach(PIN_PWM, FREQUENCY, RESOLUTION);
  ledcAttach(PIN_PWM1, FREQUENCY, RESOLUTION);
  ledcWrite(PIN_PWM, anglePos(100));
  ledcWrite(PIN_PWM1, anglePos(120));

  OledSetup();

  delay(5000);
}

void loop() {
  uint32_t angle = anglePos(135);
  ledcWrite(PIN_PWM, angle);
  ledcWrite(PIN_PWM1, angle);

  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0, 0);
  display_handler.println(angle);
  display_handler.display();
  delay(4000);

  // angle = anglePos(70);
  // ledcWrite(PIN_PWM, angle);
  // display_handler.clearDisplay();
  // display_handler.setTextSize(1);
  // display_handler.setTextColor(SSD1306_WHITE);
  // display_handler.setCursor(0, 0);
  // display_handler.println(angle);
  // display_handler.display();
  // delay(4000);
}