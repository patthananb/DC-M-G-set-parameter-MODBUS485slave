// env: monitor
// Standalone test sketch — no Modbus, no RS-485.
// Prints speed, RPM, vmot, vgen to USB Serial at 100 Hz.
// Includes 1000 Hz pulse simulator on GPIO20 (jumper to GPIO14 to test).

#include <Arduino.h>
#include "Config.h"
#include "Debug.h"
#include "DCMotorMeasuring.h"

config::dcmotormeasurement::DCMotorMeasuring motor;

void setup() {
  Serial.begin(config::settings::DEBUG_BAUD);
  while (!Serial) delay(config::settings::USB_SERIAL_WAIT_MS);   // wait for USB-CDC host

  motor.begin();   // 1000 Hz on GPIO20 — jumper to GPIO14

  dbg->printf("Ready — %lu Hz sampling started\n",
              static_cast<unsigned long>(config::settings::SAMPLE_HZ));
  dbg->printf("speed (pulse/s) | rpm (RPM) | vmot (V) | vgen (V)\n");
}

void loop() {
  config::dcmotormeasurement::Measurements m;
  if (!motor.readIfNew(m)) return;

  dbg->printf("%d pulse/s | %.1f RPM | %.3f V | %.3f V\n",
                       m.speed, m.rpm, m.vmot, m.vgen);
}
