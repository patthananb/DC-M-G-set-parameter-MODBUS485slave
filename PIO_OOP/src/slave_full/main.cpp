// env: slave_full
// Full Modbus RTU slave. 7 holding registers (speed, vmot, vgen, rpm).
// Verbose serial debug includes raw ADC voltages.
// 1000 Hz pulse simulator on GPIO20 — jumper to GPIO14 for bench testing.

#include <Arduino.h>
#include "Config.h"
#include "Debug.h"
#include "Sampler.h"
#include "PulseSimulator.h"
#include "ModbusServer.h"

HardwareSerial          RS485Serial(1);
dcmotor::Sampler        sampler;
dcmotor::PulseSimulator simulator;
dcmotor::ModbusServer   slave(RS485Serial);

void setup() {
  Serial0.begin(115200);

  simulator.begin();
  sampler.begin();
  slave.begin();   // addr=1, 9600 baud, 7 regs

  dcmotor::dbg->printf("ESP32-C6 Modbus slave ready — address 1, 9600 baud\n");
  dcmotor::dbg->printf("speed (pulse/s) | rpm (RPM) | vmot (V) | vgen (V) | vmot_raw (V) | vgen_raw (V)\n");
}

void loop() {
  dcmotor::Measurements m;
  const bool fresh = sampler.readIfNew(m);
  if (!fresh) sampler.readLatest(m);

  slave.update(m);
  slave.poll();

  if (fresh) {
    dcmotor::dbg->printf("%d pulse/s | %.1f RPM | %.3f V | %.3f V | %.3f V raw | %.3f V raw\n",
                         m.speed, m.rpm, m.vmot, m.vgen, m.vmotRaw, m.vgenRaw);
  }
}
