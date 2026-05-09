#pragma once

#include <Arduino.h>
#include "esp_timer.h"
#include "Config.h"

namespace dcmotor {

struct Measurements {
  int16_t speed    = 0;     // pulses/s, 8-sample MA
  float   vmot     = 0.0f;  // motor V, 8-sample MA
  float   vgen     = 0.0f;  // motor V, 8-sample MA
  float   rpm      = 0.0f;  // speed * 60 / PULSES_PER_REV
  float   vmotRaw  = 0.0f;  // latest raw ADC V
  float   vgenRaw  = 0.0f;
};

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
  float   vmotBuf_[config::MA_WINDOW]  = {};
  float   vgenBuf_[config::MA_WINDOW]  = {};
  int16_t speedBuf_[config::MA_WINDOW] = {};
  float   vmotSum_  = 0.0f;
  float   vgenSum_  = 0.0f;
  int32_t speedSum_ = 0;
  uint8_t idx_      = 0;
  uint8_t filled_   = 0;
};

}  // namespace dcmotor
