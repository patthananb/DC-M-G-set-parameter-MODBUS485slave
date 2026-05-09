#pragma once

#include <Arduino.h>
#include "Config.h"
#include "MotorMeasurement.h"

namespace config { namespace modbus {

using motor::Measurements;

void writeMeasurementsToHoldingRegisters(uint16_t* regs,
                                         uint8_t regCount,
                                         const Measurements& m);

}  // namespace modbus
}  // namespace config
