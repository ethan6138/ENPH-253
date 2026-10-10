#include "Oscillator.h"

#include <Arduino.h>
#include <esp_err.h>

#include <driver/pulse_cnt.h>

#include <pins.h>

namespace {
constexpr uint32_t kOscillatorSampleWindowMs = 50;
constexpr float kOscillatorEdgesPerCycle = 2.0f;

volatile float kMetalThresholdHz = 171900.0f;

// PCNT's hardware counter register is 16-bit signed (max 32767),
// so arm watch point so that larger sampling windows work (see
// onPcntWatchPointReached)
constexpr int kPcntHighLimit = 30000;
constexpr int kPcntLowLimit = -1;

pcnt_unit_handle_t g_pcntUnit = nullptr;
pcnt_channel_handle_t g_pcntChan = nullptr;

portMUX_TYPE g_overflowMux = portMUX_INITIALIZER_UNLOCKED;
int64_t g_overflowAccumulator = 0;

volatile float g_oscillatorFrequencyHz = 0.0f;

bool IRAM_ATTR onPcntWatchPointReached(pcnt_unit_handle_t /*unit*/,
                                       const pcnt_watch_event_data_t *edata,
                                       void * /*userCtx*/) {
  portENTER_CRITICAL_ISR(&g_overflowMux);
  g_overflowAccumulator += edata->watch_point_value;
  portEXIT_CRITICAL_ISR(&g_overflowMux);
  return false; // no higher-priority task to wake here
}

constexpr uint32_t kOscillatorTaskPeriodMs{100};

volatile bool g_samplingEnabled = false;
volatile bool g_oscillatorFrequencyDetected = false;

bool oscillatorDetectionConditionMet() { return g_samplingEnabled; }

} // namespace

void oscillatorTask(void *pvParameters) {
  for (;;) {
    if (oscillatorDetectionConditionMet()) {
      float freq = detectOscillatorFrequency(); // blocks ~50ms, updates
                                                // g_oscillatorFrequencyHz
      g_oscillatorFrequencyDetected = (freq > kMetalThresholdHz);
    } else {
      g_oscillatorFrequencyDetected = false; // no signal while not sampling
    }
    vTaskDelay(pdMS_TO_TICKS(kOscillatorTaskPeriodMs));
  }
}

void oscillator_init() {
  pcnt_unit_config_t unitConfig = {};
  unitConfig.high_limit = kPcntHighLimit;
  unitConfig.low_limit = kPcntLowLimit;
  ESP_ERROR_CHECK(pcnt_new_unit(&unitConfig, &g_pcntUnit));

  pcnt_chan_config_t chanConfig = {};
  chanConfig.edge_gpio_num = PIN_GPIO_OSCILLATOR;
  chanConfig.level_gpio_num = -1; // no control/level input, just count edges
  ESP_ERROR_CHECK(pcnt_new_channel(g_pcntUnit, &chanConfig, &g_pcntChan));

  // We count on both edges
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
      g_pcntChan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
      PCNT_CHANNEL_EDGE_ACTION_INCREASE));

  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(g_pcntUnit, kPcntHighLimit));
  pcnt_event_callbacks_t callbacks = {};
  callbacks.on_reach = onPcntWatchPointReached;
  ESP_ERROR_CHECK(
      pcnt_unit_register_event_callbacks(g_pcntUnit, &callbacks, nullptr));

  ESP_ERROR_CHECK(pcnt_unit_enable(g_pcntUnit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcntUnit));

  xTaskCreatePinnedToCore(oscillatorTask, "oscillatorTask", 4096, nullptr, 1,
                          nullptr, 1);
}

float detectOscillatorFrequency() {
  if (g_pcntUnit == nullptr) {
    return 0.0f; // oscillator_init() wasn't called
  }

  portENTER_CRITICAL(&g_overflowMux);
  g_overflowAccumulator = 0;
  portEXIT_CRITICAL(&g_overflowMux);

  ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcntUnit));
  ESP_ERROR_CHECK(pcnt_unit_start(g_pcntUnit));

  const unsigned long windowStartUs = micros();
  vTaskDelay(pdMS_TO_TICKS(kOscillatorSampleWindowMs));
  const unsigned long elapsedUs = micros() - windowStartUs;

  ESP_ERROR_CHECK(pcnt_unit_stop(g_pcntUnit));

  int rawCount = 0;
  ESP_ERROR_CHECK(pcnt_unit_get_count(g_pcntUnit, &rawCount));

  portENTER_CRITICAL(&g_overflowMux);
  const int64_t totalEdges = g_overflowAccumulator + rawCount;
  portEXIT_CRITICAL(&g_overflowMux);

  const float elapsedSeconds = elapsedUs / 1000000.0f;
  const float frequencyHz = elapsedSeconds > 0.0f
                                ? static_cast<float>(totalEdges) /
                                      kOscillatorEdgesPerCycle / elapsedSeconds
                                : 0.0f;

  g_oscillatorFrequencyHz = frequencyHz;
  return frequencyHz;
}

void setOscillatorSampling(bool enabled) { g_samplingEnabled = enabled; }

void setOscillatorThreshold(float threshold) { kMetalThresholdHz = threshold; }

bool getOscillatorDetected() { return g_oscillatorFrequencyDetected; }

float getOscillatorFrequency() { return g_oscillatorFrequencyHz; }