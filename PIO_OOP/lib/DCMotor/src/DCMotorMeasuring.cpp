#include "DCMotorMeasuring.h"

namespace config { namespace dcmotormeasurement {

void DCMotorMeasuring::begin(bool enableSimulator) {
  simulatorEnabled_ = enableSimulator;
  if (simulatorEnabled_) simulator_.begin();
  sampler_.begin();
}

bool DCMotorMeasuring::readIfNew(Measurements& out) {
  return sampler_.readIfNew(out);
}

void DCMotorMeasuring::readLatest(Measurements& out) {
  sampler_.readLatest(out);
}

void DCMotorMeasuring::stopSimulator() {
  if (!simulatorEnabled_) return;
  simulator_.stop();
  simulatorEnabled_ = false;
}

}  // namespace dcmotormeasurement
}  // namespace config
