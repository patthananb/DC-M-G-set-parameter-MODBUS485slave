#pragma once

namespace dcmotor {

struct VoltageMap {
  static float toMotor(float adcV);
};

}  // namespace dcmotor
