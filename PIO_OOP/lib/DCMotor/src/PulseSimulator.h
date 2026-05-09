#pragma once

#include <Arduino.h>
#include "Config.h"

namespace dcmotor {

// LEDC hardware PWM square wave for bench testing.
// Jumper SIM_PIN -> SPEED_PIN to feed simulated pulses into the Sampler.
struct PulseSimulator {
  void begin(uint8_t pin = config::SIM_PIN,
             uint32_t hz = config::SIM_PULSE_HZ,
             uint8_t resolutionBits = 8,
             uint8_t dutyOf255 = 128);
  void stop();

private:
  uint8_t pin_       = 0;
  bool    attached_  = false;
};

}  // namespace dcmotor
