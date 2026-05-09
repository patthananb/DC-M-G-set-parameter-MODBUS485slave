#pragma once

#include "Measurements.h"
#include "PulseSimulator.h"
#include "Sampler.h"

namespace config { namespace dcmotormeasurement {

struct DCMotorMeasuring {
public:
  void begin(bool enableSimulator = true);
  bool readIfNew(Measurements& out);
  void readLatest(Measurements& out);
  void stopSimulator();

private:
  Sampler        sampler_;
  PulseSimulator simulator_;
  bool           simulatorEnabled_ = false;
};

}  // namespace dcmotormeasurement
}  // namespace config
