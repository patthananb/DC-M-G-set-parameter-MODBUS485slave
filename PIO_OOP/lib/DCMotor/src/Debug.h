#pragma once

#include <Arduino.h>

// Global debug sink. Default points to Serial — on the ESP32-C6 with
// ARDUINO_USB_CDC_ON_BOOT=1 (set in platformio.ini), Serial is the native
// USB-Serial-JTAG (HWCDC) class. Type is Print* so the same pointer can
// hold any concrete sink (HWCDC, HardwareSerial, USBCDC, custom Print).
//
// To redirect, reassign in setup() — for example to send debug out of the
// on-board USB-UART bridge instead:
//     dbg = &Serial0;
//
// To mute, assign nullptr (caller must guard the calls in that case).
//
// Usage:
//     dbg->printf("speed=%d rpm=%.1f\n", m.speed, m.rpm);
extern Print* dbg;
