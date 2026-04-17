# DC Motor Reading — CLAUDE.md

## Project Overview
ESP32-C6 Modbus RTU slave that reads motor voltage (vmot), generator voltage (vgen), and motor speed, then exposes them over RS-485 to a Python Modbus master.

## Hardware
- **MCU**: ESP32-C6 (Arduino core 3.x / IDF 5.x)
- **RS-485**: UART1 — TX=GPIO6, RX=GPIO7, 9600 baud, 8-N-1, Modbus address 1
- **vmot**: GPIO4 (ADC1_CH4, 11 dB attenuation, ~0–3.9 V)
- **vgen**: GPIO5 (ADC1_CH5, 11 dB attenuation, ~0–3.9 V)
- **speed**: GPIO14 (digital pulse input, rising-edge counting — NOT an ADC pin)

## Voltage Mapping
Piecewise linear, asymmetric around the zero-cross:

| ADC voltage | Motor voltage      |
|-------------|--------------------|
| 1.45 V      | −12 V              |
| 1.65 V      |   0 V (zero-cross) |
| 3.80 V      | +12 V              |

## Sampling
- Rate: **100 Hz** (10 ms period) via `esp_timer` periodic timer
- Speed stored as **pulses per second** (Hz); convert to RPM: `RPM = speed_Hz * 60 / pulses_per_rev`
- 8-sample moving average applied to vmot, vgen, and speed

## Modbus Registers (Holding, slave address 1)
| Register | Content | Type                          |
|----------|---------|-------------------------------|
| 0        | speed   | int16 (pulses/s)              |
| 1–2      | vmot    | float32 (little-endian word swap) |
| 3–4      | vgen    | float32 (little-endian word swap) |

## Key Rules for AI Assistance
- **Do NOT use** `timerBegin(num, prescaler, up)` — removed in Arduino core 3.x. Use `esp_timer`.
- **Do NOT use** `analogRead(14)` — GPIO14 is not an ADC pin on ESP32-C6.
- ADC attenuation must be set via `analogSetPinAttenuation(pin, ADC_11db)`.
- All shared state between the timer task and `loop()` must be protected with `portMUX_TYPE` / `portENTER_CRITICAL`.
- Speed ISR must use `portENTER_CRITICAL_ISR`, timer task code uses `portENTER_CRITICAL`.
- `analogReadMilliVolts()` is safe to call from `ESP_TIMER_TASK` context (task, not ISR).

## File Layout
```
esp32slave_combined/
  esp32slave_combined.ino   # RECOMMENDED — single-file Modbus slave
                            # Sampling + Modbus in one self-contained sketch

esp32slave/
  esp32slave.ino            # Modbus RTU slave: setup/loop + Modbus polling
  read_paramters.ino        # Sampling helper: 100 Hz timer, ISR, MA filter
                            # (no setup/loop — works together with esp32slave.ino)

dc_motor_reader/
  dc_motor_reader.ino       # Standalone: sampling + Serial output, no Modbus
                            # Use for hardware validation without RS-485

python485master/
  read_rs485.py             # Python Modbus master
  requirements.txt
```

## Sketch Selection Guide
| Goal | Open this sketch |
|------|-----------------|
| Deploy full Modbus slave (simplest) | `esp32slave_combined/` |
| Keep sampling and transport in separate files | `esp32slave/` |
| Test ADC + speed sensor without Modbus | `dc_motor_reader/` |
