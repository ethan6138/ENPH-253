#include <Adafruit_GFX.h>     //oled board
#include <Adafruit_SSD1306.h> //oled board
#include <Arduino.h>
#include <Wire.h> //oled board
// #include <driver/adc.h> //adcs //adc but outdated version
#include <esp_adc/adc_oneshot.h>

// #define SCREEN_WIDTH 128
// #define SCREEN_HEIGHT 64

// #define I2C_SDA 15
// #define I2C_SCL 16

constexpr int PIN_PWM_LMD_0{11};
constexpr int PIN_PWM_LMD_1{12};
constexpr int PIN_PWM_RMD_0{13};
constexpr int PIN_PWM_RMD_1{14};

#define pwmOut1 11
#define pwmOut2 12
#define DUTY_CYCLE 3500
// #define OLED_RESET -1

// this is for setting up adc one shot
//  static adc_oneshot_unit_handle_t adc1;

// Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire,
//                                  OLED_RESET);

// int adc_val{0};
bool failed{};
// static adc_oneshot_unit_handle_t adc1;

// void OledSetup() {
//   display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
//   display_handler.clearDisplay();
//   display_handler.setTextSize(1);
//   display_handler.setTextColor(SSD1306_WHITE);
//   display_handler.setCursor(0, 0);
//   display_handler.println("L4Q1 Motor Control");
//   display_handler.display();
//   delay(500);
// }

void setup() {
  Serial.begin(115200);
  // Wire.begin(I2C_SDA, I2C_SCL); // moved here — was a global before

  // adc_oneshot_unit_init_cfg_t unit_cfg{ADC_UNIT_1};
  // adc_oneshot_new_unit(&unit_cfg, &adc1);
  // adc_oneshot_chan_cfg_t chan_cfg{ADC_ATTEN_DB_12, ADC_BITWIDTH_12};
  // adc_oneshot_config_channel(adc1, ADC_CHANNEL_0, &chan_cfg);

  if (!ledcAttach(PIN_PWM_LMD_0, 5000, 12))
    failed = true; // pin, 5000 Hz, 12-bit
  ledcAttach(PIN_PWM_LMD_1, 5000, 12);
  ledcAttach(PIN_PWM_RMD_0, 5000, 12);
  ledcAttach(PIN_PWM_RMD_1, 5000, 12);

  // OledSetup();
}

void loop() {
  // adc oneshot reading through adc1, updated version
  // adc_oneshot_read(adc1, ADC_CHANNEL_0, &adc_val);

  ledcWrite(PIN_PWM_LMD_0, 0); // 3.x: write to the pin, not a channel
  ledcWrite(PIN_PWM_LMD_1, DUTY_CYCLE);

  ledcWrite(PIN_PWM_RMD_0, 0);
  ledcWrite(PIN_PWM_RMD_1, DUTY_CYCLE + 80); 

  // adc_oneshot_read(adc1, ADC_CHANNEL_0, &adc_val);

  // // display oled updating adc values
  // display_handler.clearDisplay();
  // display_handler.setTextSize(1);
  // display_handler.setTextColor(SSD1306_WHITE);
  // display_handler.setCursor(0, 0);
  // display_handler.println(adc_val);
  // display_handler.display();
}