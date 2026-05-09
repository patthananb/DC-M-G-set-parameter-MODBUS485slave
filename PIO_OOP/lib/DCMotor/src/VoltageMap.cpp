#include "VoltageMap.h"
#include "Config.h"

namespace dcmotor {

float VoltageMap::toMotor(float v) {
  using namespace config;
  float motorV;
  if (v >= ZERO_CROSS_V) {
    motorV = ((v - ZERO_CROSS_V) / (MAX_SENSOR_V - ZERO_CROSS_V)) * MOTOR_V_MAX;
  } else {
    motorV = ((v - ZERO_CROSS_V) / (ZERO_CROSS_V - MIN_SENSOR_V)) * MOTOR_V_MAX;
  }
  if (motorV >  MOTOR_V_MAX) motorV =  MOTOR_V_MAX;
  if (motorV < -MOTOR_V_MAX) motorV = -MOTOR_V_MAX;
  return motorV;
}

}  // namespace dcmotor
