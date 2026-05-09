#pragma once

#include <Arduino.h>

namespace dcmotor {

// Debug sink. Default points to Serial0 (hardware UART0 on ESP32-C6, the
// USB-UART bridge on the devkit). Reassign in setup() if you want to send
// debug output somewhere else (e.g. &Serial for the USB-CDC interface, or
// nullptr to mute — caller must guard the calls in that case).
//
// Usage:
//     dcmotor::dbg->printf("speed=%d rpm=%.1f\n", m.speed, m.rpm);
extern HardwareSerial* dbg;

}  // namespace dcmotor
