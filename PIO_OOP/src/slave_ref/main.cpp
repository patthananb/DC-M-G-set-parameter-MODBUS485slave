// env: slave_ref
// Modbus RTU slave with the same 7-register layout as slave_full but
// terse serial debug output, mirroring the original esp32slave/ sketch.
// 1000 Hz pulse simulator on GPIO20 — jumper to GPIO14 for bench testing.

// this is also an example of using namespace aliases to keep the sketch short.

#include <Arduino.h>
#include "Config.h"
#include "Debug.h"
#include "DCMotorMeasuring.h"
#include "ModbusRTU.h"

namespace measure = config::dcmotormeasurement;
namespace modbus  = config::modbus;

HardwareSerial          RS485Serial(1);
measure::DCMotorMeasuring motor;
modbus::ModbusRTU slave(RS485Serial);

void setup() {
  Serial.begin(config::settings::DEBUG_BAUD);

  motor.begin();
  slave.begin();
}

void loop() {
  measure::Measurements m;
  const bool fresh = motor.readIfNew(m);
  if (!fresh) motor.readLatest(m);

  slave.update(m);
  slave.poll();

  if (fresh) {
    dbg->printf("speed=%d, vmot=%.3f V, vgen=%.3f V\n",
                         m.speed, m.vmot, m.vgen);
  }
}
