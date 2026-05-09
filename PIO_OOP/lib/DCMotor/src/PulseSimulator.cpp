#include "PulseSimulator.h"

namespace dcmotor {

void PulseSimulator::begin(uint8_t pin, uint32_t hz, uint8_t resolutionBits, uint8_t dutyOf255) {
  pin_ = pin;
  ledcAttach(pin_, hz, resolutionBits);
  ledcWrite(pin_, dutyOf255);
  attached_ = true;
}

void PulseSimulator::stop() {
  if (attached_) {
    ledcDetach(pin_);
    attached_ = false;
  }
}

}  // namespace dcmotor
