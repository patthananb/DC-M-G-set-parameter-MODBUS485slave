#pragma once

#include <Arduino.h>
#include "esp_timer.h"
#include "Config.h"

namespace config { namespace motor {

struct Measurements {
  int16_t speed   = 0;     // pulses/s, 8-sample MA
  float   vmot    = 0.0f;  // motor V, 8-sample MA
  float   vgen    = 0.0f;  // generator V, 8-sample MA
  float   rpm     = 0.0f;  // speed * 60 / PULSES_PER_REV
  float   vmotRaw = 0.0f;  // latest raw ADC V
  float   vgenRaw = 0.0f;
};

struct MotorMeasurement {
  void begin(bool enableSimulator = true);
  bool readIfNew(Measurements& out);
  void readLatest(Measurements& out);
  void stopSimulator();

private:
  void onSampleTimer();
  void onSpeedPulse();
  void startSimulator();

  static void IRAM_ATTR speedIsr();
  static void timerCallback(void* arg);

  esp_timer_handle_t timerHandle_ = nullptr;

  portMUX_TYPE stateMux_ = portMUX_INITIALIZER_UNLOCKED;
  portMUX_TYPE pulseMux_ = portMUX_INITIALIZER_UNLOCKED;

  volatile uint32_t pulseCount_ = 0;

  Measurements latest_{};
  volatile bool newSample_ = false;

  float   vmotBuf_[settings::MA_WINDOW]  = {};
  float   vgenBuf_[settings::MA_WINDOW]  = {};
  int16_t speedBuf_[settings::MA_WINDOW] = {};
  float   vmotSum_  = 0.0f;
  float   vgenSum_  = 0.0f;
  int32_t speedSum_ = 0;
  uint8_t idx_      = 0;
  uint8_t filled_   = 0;

  bool simulatorEnabled_ = false;
};

float adcToMotor(float adcV);

}  // namespace motor
}  // namespace config
