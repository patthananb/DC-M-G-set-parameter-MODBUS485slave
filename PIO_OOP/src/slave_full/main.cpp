// env: slave_full
// Full Modbus RTU slave. 7 holding registers (speed, vmot, vgen, rpm).
// Verbose serial debug includes raw ADC voltages.
// 1000 Hz pulse simulator on GPIO20 — jumper to GPIO14 for bench testing.

#include <Arduino.h>
#include "Config.h"
#include "Debug.h"
#include "MotorMeasurement.h"
#include "ModbusRTU.h"

HardwareSerial          RS485Serial(1);
config::motor::MotorMeasurement motor;
config::modbus::ModbusRTU slave(RS485Serial);

void setup() {
  Serial.begin(config::settings::DEBUG_BAUD);

  motor.begin();
  slave.begin();

  dbg->printf("ESP32-C6 Modbus slave ready — address %u, %lu baud\n",
              config::settings::MODBUS_ADDR,
              static_cast<unsigned long>(config::settings::MODBUS_BAUD));
  dbg->printf("speed (pulse/s) | rpm (RPM) | vmot (V) | vgen (V) | vmot_raw (V) | vgen_raw (V)\n");
}

void loop() {
  config::motor::Measurements m;
  const bool fresh = motor.readIfNew(m);
  if (!fresh) motor.readLatest(m);

  slave.update(m);
  slave.poll();

  if (fresh) {
    dbg->printf("%d pulse/s | %.1f RPM | %.3f V | %.3f V | %.3f V raw | %.3f V raw\n",
                         m.speed, m.rpm, m.vmot, m.vgen, m.vmotRaw, m.vgenRaw);
  }
}
