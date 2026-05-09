// env: monitor
// Standalone test sketch — no Modbus, no RS-485.
// Prints speed, RPM, vmot, vgen to USB Serial at 100 Hz.
// Includes 1000 Hz pulse simulator on GPIO20 (jumper to GPIO14 to test).

#include <Arduino.h>
#include "Config.h"
#include "Debug.h"
#include "Sampler.h"
#include "PulseSimulator.h"

dcmotor::Sampler        sampler;
dcmotor::PulseSimulator simulator;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);   // wait for USB-CDC host

  simulator.begin();   // 1000 Hz on GPIO20 — jumper to GPIO14
  sampler.begin();

  dbg->printf("Ready — 100 Hz sampling started\n");
  dbg->printf("speed (pulse/s) | rpm (RPM) | vmot (V) | vgen (V)\n");
}

void loop() {
  dcmotor::Measurements m;
  if (!sampler.readIfNew(m)) return;

  dbg->printf("%d pulse/s | %.1f RPM | %.3f V | %.3f V\n",
                       m.speed, m.rpm, m.vmot, m.vgen);
}
