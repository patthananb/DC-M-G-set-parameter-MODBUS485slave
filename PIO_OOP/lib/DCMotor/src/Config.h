#pragma once

#include <Arduino.h>

#ifndef DCMOTOR_ENABLE_MODBUS_TCP
#define DCMOTOR_ENABLE_MODBUS_TCP 0
#endif

namespace config {
    namespace settings {

    inline constexpr uint8_t  VMOT_PIN       = 4;
    inline constexpr uint8_t  VGEN_PIN       = 5;
    inline constexpr uint8_t  SPEED_PIN      = 14;
    inline constexpr uint8_t  RS485_TX       = 6;
    inline constexpr uint8_t  RS485_RX       = 7;

    inline constexpr uint8_t  SIM_PIN        = 20;
    inline constexpr uint32_t SIM_PULSE_HZ   = 1000;
    inline constexpr uint8_t  SIM_LEDC_BITS  = 8;
    inline constexpr uint8_t  SIM_LEDC_DUTY  = 128;

    inline constexpr uint32_t SAMPLE_HZ      = 100;
    inline constexpr uint32_t SAMPLE_US      = 1000000UL / SAMPLE_HZ;
    inline constexpr uint8_t  ADC_RESOLUTION_BITS = 12;
    inline constexpr float    MILLIVOLTS_PER_VOLT = 1000.0f;

    inline constexpr float    ZERO_CROSS_V   = 1.65f;
    inline constexpr float    MAX_SENSOR_V   = 2.85f;
    inline constexpr float    MIN_SENSOR_V   = 0.45f;
    inline constexpr float    MOTOR_V_MAX    = 12.0f;

    inline constexpr uint8_t  MA_WINDOW      = 8;
    inline constexpr uint8_t  PULSES_PER_REV = 20;

    inline constexpr uint8_t  MODBUS_ADDR    = 1;
    inline constexpr uint32_t MODBUS_BAUD    = 9600;
    inline constexpr uint16_t MODBUS_TCP_PORT = 502;
    inline constexpr uint8_t  REG_COUNT      = 7;
    inline constexpr uint8_t  MODBUS_TCP_READ_HOLDING_REGISTERS = 0x03;
    inline constexpr uint8_t  MODBUS_TCP_EXCEPTION_FLAG = 0x80;
    inline constexpr uint8_t  MODBUS_TCP_ILLEGAL_FUNCTION = 0x01;
    inline constexpr uint8_t  MODBUS_TCP_ILLEGAL_DATA_ADDRESS = 0x02;
    inline constexpr uint8_t  MODBUS_TCP_ILLEGAL_DATA_VALUE = 0x03;
    inline constexpr uint8_t  MODBUS_TCP_MBAP_HEADER_BYTES = 7;
    inline constexpr size_t   MODBUS_TCP_MAX_PDU_BYTES = 253;
    inline constexpr uint8_t  MODBUS_TCP_MAX_READ_REGISTERS = 125;
    inline constexpr uint32_t MODBUS_TCP_CLIENT_TIMEOUT_MS = 50;

    inline constexpr uint32_t DEBUG_BAUD = 115200;
    inline constexpr uint32_t USB_SERIAL_WAIT_MS = 10;

    #ifndef DCMOTOR_WIFI_SSID
    #define DCMOTOR_WIFI_SSID "YOUR_WIFI_SSID"
    #endif

    #ifndef DCMOTOR_WIFI_PASSWORD
    #define DCMOTOR_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
    #endif

    inline constexpr const char* WIFI_SSID     = DCMOTOR_WIFI_SSID;
    inline constexpr const char* WIFI_PASSWORD = DCMOTOR_WIFI_PASSWORD;
    inline constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
    inline constexpr uint32_t WIFI_CONNECT_POLL_MS = 100;

    }  // namespace settings
}  // namespace config
