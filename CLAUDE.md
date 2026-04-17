# DC Motor Reading — CLAUDE.md

## Project Overview
ESP32-C6 Modbus RTU slave that reads motor voltage (vmot), generator voltage (vgen),
and motor speed, calculates RPM, then exposes all values over RS-485 to a Python
Modbus master.

## Hardware
- **MCU**: ESP32-C6 (Arduino core 3.x / IDF 5.x)
- **RS-485**: UART1 — TX=GPIO6, RX=GPIO7, 9600 baud, 8-N-1, Modbus address 1
- **vmot**: GPIO4 (ADC1_CH4, 11 dB attenuation, ~0–3.9 V)
- **vgen**: GPIO5 (ADC1_CH5, 11 dB attenuation, ~0–3.9 V)
- **speed**: GPIO14 (digital pulse input, rising-edge counting — NOT an ADC pin)
- **sim out**: GPIO20 (LEDC 1000 Hz square wave for bench testing)

## Voltage Mapping
Piecewise linear, asymmetric around the zero-cross:

| ADC voltage | Motor voltage      |
|-------------|--------------------|
| 0.45 V      | −12 V              |
| 1.65 V      |   0 V (zero-cross) |
| 2.85 V      | +12 V              |

## Sampling
- Rate: **100 Hz** (10 ms period) via `esp_timer` periodic timer
- Speed stored as **pulses per second** (Hz); RPM calculated automatically
- 8-sample moving average applied to vmot, vgen, and speed before RPM is derived

## Encoder
- **Spec**: 20 pulses per revolution (`PULSES_PER_REV = 20`)
- **RPM formula**: `RPM = speed_Hz * 60.0f / PULSES_PER_REV`
- Computed inside `onSampleTimer()` from the already-smoothed `speedAvg`

## Pulse Simulator
- **Pin**: GPIO20 (`SIM_PIN = 20`)
- **Frequency**: 1000 Hz (`SIM_PULSE_HZ = 1000`) → 3000 RPM expected
- **Method**: LEDC hardware PWM — zero CPU overhead
- **To use**: jumper GPIO20 → GPIO14
- **To disable**: comment out `ledcAttach` / `ledcWrite` in `setup()`

## Modbus Registers (Holding, slave address 1)
| Register | Content | Type                              |
|----------|---------|-----------------------------------|
| 0        | speed   | int16 (pulses/s)                  |
| 1–2      | vmot    | float32 (little-endian word swap) |
| 3–4      | vgen    | float32 (little-endian word swap) |
| 5–6      | rpm     | float32 (little-endian word swap) |

## Key Rules for AI Assistance
- **Do NOT use** `timerBegin(num, prescaler, up)` — removed in Arduino core 3.x. Use `esp_timer`.
- **Do NOT use** `analogRead(14)` — GPIO14 is not an ADC pin on ESP32-C6.
- ADC attenuation must be set via `analogSetPinAttenuation(pin, ADC_11db)`.
- All shared state between the timer task and `loop()` must be protected with `portMUX_TYPE` / `portENTER_CRITICAL`.
- Speed ISR must use `portENTER_CRITICAL_ISR`; timer task code uses `portENTER_CRITICAL`.
- `analogReadMilliVolts()` is safe to call from `ESP_TIMER_TASK` context (task, not ISR).
- LEDC API for Arduino core 3.x: `ledcAttach(pin, freq, bits)` + `ledcWrite(pin, duty)`.

## File Layout
```
modbus_slave/
  modbus_slave.ino          # RECOMMENDED — single-file Modbus RTU slave
                            # Sampling + RPM + Modbus in one self-contained sketch
                            # Includes 1000 Hz pulse simulator on GPIO20

serial_monitor/
  serial_monitor.ino        # Standalone test sketch — no Modbus, no RS-485
                            # Prints speed, RPM, vmot, vgen to USB Serial at 100 Hz
                            # Includes 1000 Hz pulse simulator on GPIO20

esp32slave/                 # Two-file reference implementation (do not edit)
  esp32slave.ino            # setup/loop + Modbus polling
  read_paramters.ino        # 100 Hz sampling helper (no setup/loop)

python485master/
  modbus_master.py          # Python Modbus master — reads all 7 registers
  requirements.txt
```

## Sketch Selection Guide
| Goal | Open this sketch |
|------|-----------------|
| Deploy full Modbus slave (simplest) | `modbus_slave/` |
| Test ADC + speed without Modbus/RS-485 | `serial_monitor/` |
| Two-file reference (layers separated) | `esp32slave/` |
