// env: slave_full
// Full Modbus RTU slave. 7 holding registers (speed, vmot, vgen, rpm).
// Verbose serial debug includes raw ADC voltages.
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
  slave.begin();   // addr=1, 9600 baud, 7 regs

  Serial.println("ESP32-C6 Modbus slave ready — address 1, 9600 baud");
  Serial.println("speed (pulse/s) | rpm (RPM) | vmot (V) | vgen (V) | vmot_raw (V) | vgen_raw (V)");
}

void loop() {
  dcmotor::Measurements m;
  const bool fresh = sampler.readIfNew(m);
  if (!fresh) sampler.readLatest(m);

  slave.update(m);
  slave.poll();

  if (fresh) {
    Serial.print(m.speed);       Serial.print(" pulse/s | ");
    Serial.print(m.rpm, 1);      Serial.print(" RPM | ");
    Serial.print(m.vmot, 3);     Serial.print(" V | ");
    Serial.print(m.vgen, 3);     Serial.print(" V | ");
    Serial.print(m.vmotRaw, 3);  Serial.print(" V raw | ");
    Serial.print(m.vgenRaw, 3);  Serial.println(" V raw");
  }
}
