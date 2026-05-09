// env: slave_ref
// Modbus RTU slave with the same 7-register layout as slave_full but
// terse serial debug output, mirroring the original esp32slave/ sketch.
// 1000 Hz pulse simulator on GPIO20 — jumper to GPIO14 for bench testing.

// this is also a example of "using namespace" to avoid having to prefix everything with dcmotor::

#include <Arduino.h>
#include "Config.h"
#include "Debug.h"
#include "Sampler.h"
#include "PulseSimulator.h"
#include "ModbusServer.h"

using namespace dcmotor;

HardwareSerial          RS485Serial(1);
Sampler        sampler;
PulseSimulator simulator;
ModbusServer   slave(RS485Serial);

void setup() {
  Serial.begin(115200);

  simulator.begin();
  sampler.begin();
  slave.begin();
}

void loop() {
Measurements m;
  const bool fresh = sampler.readIfNew(m);
  if (!fresh) sampler.readLatest(m);

  slave.update(m);
  slave.poll();

  if (fresh) {
    dbg->printf("speed=%d, vmot=%.3f V, vgen=%.3f V\n",
                         m.speed, m.vmot, m.vgen);
  }
}
