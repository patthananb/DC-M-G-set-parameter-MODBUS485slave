#include "ModbusRTU.h"
#include "RegisterMap.h"

namespace config { namespace modbus {

ModbusRTU::ModbusRTU(HardwareSerial& port)
    : serial_(port), modbus_(port) {}

void ModbusRTU::begin(uint8_t addr, uint32_t baud, uint8_t regCount) {
  regCount_ = (regCount > settings::REG_COUNT) ? settings::REG_COUNT : regCount;

  serial_.begin(baud, SERIAL_8N1, settings::RS485_RX, settings::RS485_TX);

  modbus_.begin(addr, baud);
  modbus_.configureHoldingRegisters(regs_, regCount_);
}

void ModbusRTU::update(const Measurements& m) {
  writeMeasurementsToHoldingRegisters(regs_, regCount_, m);
}

void ModbusRTU::poll() {
  modbus_.poll();
}

}  // namespace modbus
}  // namespace config
