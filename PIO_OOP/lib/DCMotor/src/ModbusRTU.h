#pragma once

#include <Arduino.h>
#include <ModbusRTUSlave.h>
#include "Config.h"
#include "MotorMeasurement.h"

namespace config { namespace modbus {

using motor::Measurements;

// Wrapper around ModbusRTUSlave that holds the holding-register array
// and packs Measurements into the wire layout used by the Python master:
//   reg[0]    = speed   INT16   pulses/s
//   reg[1..2] = vmot    FLOAT32 (low word first)
//   reg[3..4] = vgen    FLOAT32 (low word first)
//   reg[5..6] = rpm     FLOAT32 (low word first)
struct ModbusRTU {
  explicit ModbusRTU(HardwareSerial& port);

  void begin(uint8_t  addr     = settings::MODBUS_ADDR,
             uint32_t baud     = settings::MODBUS_BAUD,
             uint8_t  regCount = settings::REG_COUNT);

  void update(const Measurements& m);
  void poll();

private:
  HardwareSerial& serial_;
  ModbusRTUSlave  modbus_;
  uint16_t        regs_[settings::REG_COUNT] = {};
  uint8_t         regCount_                  = settings::REG_COUNT;
};

using RTU = ModbusRTU;

}  // namespace modbus
}  // namespace config
