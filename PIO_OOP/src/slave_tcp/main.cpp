// env: slave_tcp
// Modbus TCP slave with the same 7-register layout as the RTU sketches.
// Configure Wi-Fi via DCMOTOR_WIFI_SSID / DCMOTOR_WIFI_PASSWORD build flags.
// 1000 Hz pulse simulator on GPIO20 - jumper to GPIO14 for bench testing.

#include <Arduino.h>
#include "Config.h"
#include "Debug.h"
#include "DCMotorMeasuring.h"
#include "ModbusTCP.h"

config::dcmotormeasurement::DCMotorMeasuring motor;
config::modbus::ModbusTCP slave;

void setup() {
  Serial.begin(config::settings::DEBUG_BAUD);

  motor.begin();

  const bool wifiReady = slave.begin();
  if (wifiReady) {
    dbg->printf("ESP32-C6 Modbus TCP slave ready at %s:%u, unit id %u\n",
                slave.localIP().toString().c_str(),
                config::settings::MODBUS_TCP_PORT,
                config::settings::MODBUS_ADDR);
  } else {
    dbg->printf("Wi-Fi connection failed; set DCMOTOR_WIFI_SSID and DCMOTOR_WIFI_PASSWORD\n");
  }

  dbg->printf("speed (pulse/s) | rpm (RPM) | vmot (V) | vgen (V)\n");
}

void loop() {
  config::dcmotormeasurement::Measurements m;
  const bool fresh = motor.readIfNew(m);
  if (!fresh) motor.readLatest(m);

  slave.update(m);
  slave.poll();

  if (fresh) {
    dbg->printf("%d pulse/s | %.1f RPM | %.3f V | %.3f V\n",
                m.speed, m.rpm, m.vmot, m.vgen);
  }
}
