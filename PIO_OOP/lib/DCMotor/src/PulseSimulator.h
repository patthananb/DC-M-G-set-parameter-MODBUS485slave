#pragma once

#include <Arduino.h>
#include "Config.h"

namespace config { namespace dcmotormeasurement {

// LEDC hardware PWM square wave for bench testing.
// Jumper SIM_PIN -> SPEED_PIN to feed simulated pulses into the Sampler.
struct PulseSimulator {
  void begin(uint8_t pin = settings::SIM_PIN,
             uint32_t hz = settings::SIM_PULSE_HZ,
             uint8_t resolutionBits = settings::SIM_LEDC_BITS,
             uint8_t dutyOf255 = settings::SIM_LEDC_DUTY);
  void stop();

private:
  uint8_t pin_       = 0;
  bool    attached_  = false;
};

}  // namespace dcmotormeasurement
}  // namespace config
