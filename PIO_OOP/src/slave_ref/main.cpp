// env: slave_ref
// Modbus RTU slave with the same 7-register layout as slave_full but
// terse serial debug output, mirroring the original esp32slave/ sketch.
// 1000 Hz pulse simulator on GPIO20 — jumper to GPIO14 for bench testing.

#include <Arduino.h>
#include "Config.h"
#include "Sampler.h"
#include "PulseSimulator.h"
#include "ModbusServer.h"

HardwareSerial          RS485Serial(1);
dcmotor::Sampler        sampler;
dcmotor::PulseSimulator simulator;
dcmotor::ModbusServer   slave(RS485Serial);

void setup() {
  Serial.begin(115200);

  simulator.begin();
  sampler.begin();
  slave.begin();
}

void loop() {
  dcmotor::Measurements m;
  const bool fresh = sampler.readIfNew(m);
  if (!fresh) sampler.readLatest(m);

  slave.update(m);
  slave.poll();

  if (fresh) {
    Serial.print("speed=");
    Serial.print(m.speed);
    Serial.print(", vmot=");
    Serial.print(m.vmot, 3);
    Serial.print(" V, vgen=");
    Serial.print(m.vgen, 3);
    Serial.println(" V");
  }
}
