#include "ModbusServer.h"

#include <string.h>

namespace dcmotor {

ModbusServer::ModbusServer(HardwareSerial& port)
    : serial_(port), modbus_(port) {}

void ModbusServer::begin(uint8_t addr, uint32_t baud, uint8_t regCount) {
  regCount_ = (regCount > config::REG_COUNT) ? config::REG_COUNT : regCount;

  serial_.begin(baud, SERIAL_8N1, config::RS485_RX, config::RS485_TX);

  modbus_.begin(addr, baud);
  modbus_.configureHoldingRegisters(regs_, regCount_);
}

void ModbusServer::update(const Measurements& m) {
  regs_[0] = static_cast<uint16_t>(m.speed);
  if (regCount_ >= 3) memcpy(&regs_[1], &m.vmot, sizeof(m.vmot));  // 1..2
  if (regCount_ >= 5) memcpy(&regs_[3], &m.vgen, sizeof(m.vgen));  // 3..4
  if (regCount_ >= 7) memcpy(&regs_[5], &m.rpm,  sizeof(m.rpm));   // 5..6
}

void ModbusServer::poll() {
  modbus_.poll();
}

}  // namespace dcmotor
