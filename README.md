# DC Motor Reading

ESP32-C6 firmware that samples motor voltage, generator voltage, and motor speed
at 100 Hz, calculates RPM, and exposes all values over RS-485 Modbus RTU.
A companion Python script acts as the Modbus master.

---

## Project Layout

```
DC_Motor_Reading/
├── modbus_slave/
│   └── modbus_slave.ino        ← Full Modbus RTU slave (single file) ★ use this
│
├── serial_monitor/
│   └── serial_monitor.ino      ← Standalone test: prints values to USB Serial
│
├── esp32slave/                 ← Two-file reference implementation
│   ├── esp32slave.ino
│   └── read_paramters.ino
│
└── python485master/
    ├── modbus_master.py        ← Python Modbus master (reads all registers)
    └── requirements.txt
```

---

## Hardware Connections

| Signal       | GPIO | Notes                                              |
|--------------|------|----------------------------------------------------|
| vmot         | 4    | ADC1_CH4 — sensor output 0.45–2.85 V              |
| vgen         | 5    | ADC1_CH5 — sensor output 0.45–2.85 V              |
| speed input  | 14   | Digital pulse input (encoder / hall / tachometer)  |
| RS-485 TX    | 6    | UART1                                              |
| RS-485 RX    | 7    | UART1                                              |
| sim output   | 20   | Pulse simulator output — jumper to GPIO14 to test  |

> **GPIO14 is digital-only** on ESP32-C6 (not an ADC pin). Speed is measured
> by counting rising edges per 10 ms window × 100 = pulses/second.

---

## Voltage Mapping

Analog sensors produce a DC-biased output. The firmware maps piecewise-linearly
to real motor voltage:

```
ADC voltage   Motor voltage
──────────    ─────────────
  0.45 V  →    −12 V
  1.65 V  →      0 V   (zero-cross / DC bias)
  2.85 V  →    +12 V
```

Both ADC channels use 11 dB attenuation (`ADC_11db`) to cover the 0–3.9 V range.

---

## RPM Calculation

Encoder spec: **20 pulses per revolution**

```
RPM = speed_Hz × 60 / 20
```

RPM is derived inside the 100 Hz timer callback from the 8-sample moving-average
speed, so it inherits the same smoothing as the raw pulse count.

**Pulse simulator (for bench testing):**
GPIO20 outputs a hardware 1000 Hz square wave via LEDC (zero CPU overhead).
→ Expected reading: **1000 pulse/s → 3000 RPM**
To use: connect GPIO20 to GPIO14 with a jumper wire.
To disable: comment out the two `ledcAttach` / `ledcWrite` lines in `setup()`.

---

## Modbus RTU Configuration

| Parameter     | Value |
|---------------|-------|
| Slave address | 1     |
| Baud rate     | 9600  |
| Framing       | 8-N-1 |
| UART          | UART1 — TX=GPIO6, RX=GPIO7 |

### Holding Registers

| Address | Signal | Type    | Unit            |
|---------|--------|---------|-----------------|
| 0       | speed  | INT16   | pulses / second |
| 1–2     | vmot   | FLOAT32 | volts           |
| 3–4     | vgen   | FLOAT32 | volts           |
| 5–6     | rpm    | FLOAT32 | RPM             |

All FLOAT32 values use **little-endian word order** (low word at lower address).

---

## Sampling Architecture

| Component | Detail |
|-----------|--------|
| Timer | `esp_timer` periodic, 10 000 µs → 100 Hz |
| ADC reads | `analogReadMilliVolts()` inside timer task (safe, not ISR) |
| Speed | GPIO14 ISR counts rising edges; timer task snapshots + resets each tick |
| Filter | 8-sample moving average on vmot, vgen, and speed |
| Thread safety | `portMUX_TYPE` spinlocks: ISR ↔ timer-task, timer-task ↔ loop() |

---

## Which Sketch to Open

| Goal | Sketch |
|------|--------|
| Deploy full Modbus slave — simplest | `modbus_slave/modbus_slave.ino` ★ |
| Test ADC + speed without RS-485 | `serial_monitor/serial_monitor.ino` |
| Two-file reference (sampling separate from Modbus) | `esp32slave/esp32slave.ino` |

### Building

1. **Arduino IDE** → File → Open → select the `.ino` inside the sketch folder.
2. Board: **ESP32C6 Dev Module**
3. Install [ModbusRTUSlave](https://github.com/CMB27/ModbusRTUSlave) library
   *(not needed for `serial_monitor`)*.
4. Upload.

---

## Python Master

```bash
cd python485master
pip install -r requirements.txt
python modbus_master.py
```

Auto-detects USB-RS485 adapters and reads speed, RPM, vmot, and vgen.
Pass `--port /dev/cu.usbserial-XXXX` to skip the port selection prompt.

**Output:**
```
Speed: 1000 pulse/s
RPM:   3000.0 RPM
V_mot: 3.25 V
V_gen: 1.82 V
```
