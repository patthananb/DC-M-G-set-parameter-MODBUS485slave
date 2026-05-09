#pragma once

#include <Arduino.h>
#include "esp_timer.h"
#include "Config.h"
#include "Measurements.h"

namespace config { namespace dcmotormeasurement {

// 100 Hz ADC + pulse sampler with 8-sample moving average.
// Single-instance: ISR + esp_timer trampolines bind to the first instance
// whose begin() is called. Constructing a second Sampler is undefined.
struct Sampler {
  void begin();
  bool readIfNew(Measurements& out);   // true once per 10 ms tick
  void readLatest(Measurements& out);

private:
  void onTimer();
  void onPulse();

  static void IRAM_ATTR isrTrampoline();
  static void timerTrampoline(void* arg);

  esp_timer_handle_t timerHandle_ = nullptr;

  portMUX_TYPE stateMux_ = portMUX_INITIALIZER_UNLOCKED;
  portMUX_TYPE pulseMux_ = portMUX_INITIALIZER_UNLOCKED;

  volatile uint32_t pulseCount_ = 0;

  Measurements latest_{};
  volatile bool newSample_ = false;

  // MA filter state — owned by onTimer()
  float   vmotBuf_[settings::MA_WINDOW]  = {};
  float   vgenBuf_[settings::MA_WINDOW]  = {};
  int16_t speedBuf_[settings::MA_WINDOW] = {};
  float   vmotSum_  = 0.0f;
  float   vgenSum_  = 0.0f;
  int32_t speedSum_ = 0;
  uint8_t idx_      = 0;
  uint8_t filled_   = 0;
};

}  // namespace dcmotormeasurement
}  // namespace config
