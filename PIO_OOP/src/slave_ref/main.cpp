// env: slave_ref
// Modbus RTU slave with the same 7-register layout as slave_full but
// terse serial debug output, mirroring the original esp32slave/ sketch.
// 1000 Hz pulse simulator on GPIO20 — jumper to GPIO14 for bench testing.

// this is also an example of using namespace aliases to keep the sketch short.

#include <Arduino.h>
#include "Config.h"
#include "Debug.h"
#include "MotorMeasurement.h"
#include "ModbusRTU.h"

namespace motor  = config::motor;
namespace modbus  = config::modbus;

HardwareSerial          RS485Serial(1);
motor::MotorMeasurement motorReader;
modbus::ModbusRTU slave(RS485Serial);

void setup() {
  Serial.begin(config::settings::DEBUG_BAUD);

  motorReader.begin();
  slave.begin();
}

void loop() {
  motor::Measurements m;
  const bool fresh = motorReader.readIfNew(m);
  if (!fresh) motorReader.readLatest(m);

  slave.update(m);
  slave.poll();

  if (fresh) {
    dbg->printf("speed=%d, vmot=%.3f V, vgen=%.3f V\n",
                         m.speed, m.vmot, m.vgen);
  }
}
