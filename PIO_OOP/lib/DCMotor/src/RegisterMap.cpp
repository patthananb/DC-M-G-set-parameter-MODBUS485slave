#include "RegisterMap.h"

#include <string.h>

namespace config { namespace modbus {

void writeMeasurementsToHoldingRegisters(uint16_t* regs,
                                         uint8_t regCount,
                                         const Measurements& m) {
  if (!regs || regCount == 0) return;

  regs[0] = static_cast<uint16_t>(m.speed);
  if (regCount >= 3) memcpy(&regs[1], &m.vmot, sizeof(m.vmot));  // 1..2
  if (regCount >= 5) memcpy(&regs[3], &m.vgen, sizeof(m.vgen));  // 3..4
  if (regCount >= 7) memcpy(&regs[5], &m.rpm,  sizeof(m.rpm));   // 5..6
}

}  // namespace modbus
}  // namespace config
