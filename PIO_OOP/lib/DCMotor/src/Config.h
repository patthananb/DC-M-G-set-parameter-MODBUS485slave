#pragma once

#include <Arduino.h>

namespace dcmotor {
namespace config {

inline constexpr uint8_t  VMOT_PIN       = 4;
inline constexpr uint8_t  VGEN_PIN       = 5;
inline constexpr uint8_t  SPEED_PIN      = 14;
inline constexpr uint8_t  RS485_TX       = 6;
inline constexpr uint8_t  RS485_RX       = 7;

inline constexpr uint8_t  SIM_PIN        = 20;
inline constexpr uint32_t SIM_PULSE_HZ   = 1000;

inline constexpr uint32_t SAMPLE_HZ      = 100;
inline constexpr uint32_t SAMPLE_US      = 1000000UL / SAMPLE_HZ;

inline constexpr float    ZERO_CROSS_V   = 1.65f;
inline constexpr float    MAX_SENSOR_V   = 2.85f;
inline constexpr float    MIN_SENSOR_V   = 0.45f;
inline constexpr float    MOTOR_V_MAX    = 12.0f;

inline constexpr uint8_t  MA_WINDOW      = 8;
inline constexpr uint8_t  PULSES_PER_REV = 20;

inline constexpr uint8_t  MODBUS_ADDR    = 1;
inline constexpr uint32_t MODBUS_BAUD    = 9600;
inline constexpr uint8_t  REG_COUNT      = 7;

}  // namespace config
}  // namespace dcmotor
