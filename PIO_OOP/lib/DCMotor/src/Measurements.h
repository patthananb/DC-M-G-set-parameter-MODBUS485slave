#pragma once

#include <Arduino.h>

namespace config { namespace dcmotormeasurement {

struct Measurements {
  int16_t speed   = 0;     // pulses/s, 8-sample MA
  float   vmot    = 0.0f;  // motor V, 8-sample MA
  float   vgen    = 0.0f;  // generator V, 8-sample MA
  float   rpm     = 0.0f;  // speed * 60 / PULSES_PER_REV
  float   vmotRaw = 0.0f;  // latest raw ADC V
  float   vgenRaw = 0.0f;
};

}  // namespace dcmotormeasurement
}  // namespace config
