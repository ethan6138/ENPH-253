#include <Adafruit_GFX.h>     //oled board
#include <Adafruit_SSD1306.h> //oled board
#include <Arduino.h>
#include <Wire.h> //oled board
// #include <driver/adc.h> //adcs //adc but outdated version
#include "esp_adc/adc_continuous.h" //analog read (legacy, still works on 3.x)
#include <esp_adc/adc_oneshot.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define I2C_SDA 15
#define I2C_SCL 16

#define pwmOut46 46
#define pwmOut2 2
#define DUTY_CYCLE 128
#define OLED_RESET -1

#define SAMPLE_RATE 50000
#define BUF_SAMPLES 512
#define RESULT_BYTES SOC_ADC_DIGI_RESULT_BYTES
#define BUF_BYTES (BUF_SAMPLES * RESULT_BYTES)

static adc_continuous_handle_t handle = NULL;
static volatile bool ready = false;

int adc_val1{-1};
int adc_val2{-1};
int adc_val3{-1};

// this is for setting up adc one shot
//  static adc_oneshot_unit_handle_t adc1;

Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire,
                                 OLED_RESET);

static bool IRAM_ATTR on_full(adc_continuous_handle_t handle,
                              const adc_continuous_evt_data_t *edata,
                              void *user_data) {

  ready = true;
  return false;
}

void OledSetup() {
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0, 0);
  display_handler.println("L3Q1 ADC with DMA");
  display_handler.display();
  delay(500);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL); // moved here — was a global before

  // adc1_config_width(ADC_WIDTH_BIT_12);
  // adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);

  // set up for a oneshot adc
  // adc_oneshot_unit_init_cfg_t unit_cfg {ADC_UNIT_1};
  // adc_oneshot_new_unit(&unit_cfg, &adc1);
  // adc_oneshot_chan_cfg_t chan_cfg { ADC_ATTEN_DB_12, ADC_BITWIDTH_12};
  // adc_oneshot_config_channel(adc1, ADC_CHANNEL_0, &chan_cfg);

  adc_continuous_handle_cfg_t hcfg{.max_store_buf_size = 2 * BUF_BYTES,
                                   .conv_frame_size = BUF_BYTES,
                                   .flags = 0};

  ESP_ERROR_CHECK(adc_continuous_new_handle(&hcfg, &handle));

  adc_digi_pattern_config_t pat{.atten = ADC_ATTEN_DB_12,
                                .channel = ADC_CHANNEL_0,
                                .unit = ADC_UNIT_1,
                                .bit_width = ADC_BITWIDTH_12};

  adc_continuous_config_t ccfg{.pattern_num = 1,
                               .adc_pattern = &pat,
                               .sample_freq_hz = SAMPLE_RATE,
                               .conv_mode = ADC_CONV_SINGLE_UNIT_1,
                               .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2};

  ESP_ERROR_CHECK(adc_continuous_config(handle, &ccfg));

  adc_continuous_evt_cbs_t cbs{.on_conv_done = on_full, .on_pool_ovf = NULL};
  adc_continuous_register_event_callbacks(handle, &cbs, NULL);
  ESP_ERROR_CHECK(adc_continuous_start(handle));

  ledcAttach(pwmOut46, 1000, 8); // pin, 1000 Hz, 8-bit
  // ledcAttach(pwmOut2, 1000, 8);
  OledSetup();
}

void loop() {
  // adc_val = adc1_get_raw(ADC1_CHANNEL_0); outdated version

  // adc oneshot reading through adc1, updated version
  //    adc_oneshot_read(adc1, ADC_CHANNEL_0, &adc_val);

  // 3.29V
  // ledcWrite(pwmOut46, DUTY_CYCLE); // 3.x: write to the pin, not a channel

  if (ready) {
    ready = false;

    ledcWrite(pwmOut46, DUTY_CYCLE);

    static adc_continuous_data_t parsed[BUF_SAMPLES];
    uint32_t n = 0;

    if (adc_continuous_read_parse(handle, parsed, BUF_SAMPLES, &n, 0) ==
        ESP_OK) {
      // do the auto correlation here someda please future ethan
      adc_val1 = parsed[0].raw_data;
      adc_val2 = parsed[255].raw_data;
      adc_val3 = parsed[511].raw_data;

      display_handler.clearDisplay();
      display_handler.setTextSize(1);
      display_handler.setTextColor(SSD1306_WHITE);
      display_handler.setCursor(0, 0);
      display_handler.println(adc_val1);
      display_handler.println(adc_val2);
      display_handler.println(adc_val3);
      display_handler.display();

      delay(100);
    }
  }
  // display oled updating adc values
  //    display_handler.clearDisplay();
  //    display_handler.setTextSize(1);
  //    display_handler.setTextColor(SSD1306_WHITE);
  //    display_handler.setCursor(0, 0);
  //    display_handler.println(adc_val);
  //    display_handler.display();
}