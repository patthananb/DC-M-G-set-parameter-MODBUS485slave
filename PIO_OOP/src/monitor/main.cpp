// env: monitor
// Standalone test sketch — no Modbus, no RS-485.
// Prints speed, RPM, vmot, vgen to USB Serial at 100 Hz.
// Includes 1000 Hz pulse simulator on GPIO20 (jumper to GPIO14 to test).

#include <Arduino.h>
#include "Config.h"
#include "Sampler.h"
#include "PulseSimulator.h"

dcmotor::Sampler        sampler;
dcmotor::PulseSimulator simulator;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  simulator.begin();   // 1000 Hz on GPIO20 — jumper to GPIO14
  sampler.begin();

  Serial.println("Ready — 100 Hz sampling started");
  Serial.println("speed (pulse/s) | rpm (RPM) | vmot (V) | vgen (V)");
}

void loop() {
  dcmotor::Measurements m;
  if (!sampler.readIfNew(m)) return;

  Serial.print(m.speed);    Serial.print(" pulse/s | ");
  Serial.print(m.rpm, 1);   Serial.print(" RPM | ");
  Serial.print(m.vmot, 3);  Serial.print(" V | ");
  Serial.print(m.vgen, 3);  Serial.println(" V");
}
