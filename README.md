# DC Motor Reading

ESP32-C6 firmware that samples motor voltage, generator voltage, and motor speed at 100 Hz and exposes the values over RS-485 Modbus RTU. A companion Python script acts as the Modbus master.

## Hardware Connections

| Signal | GPIO | Notes |
|--------|------|-------|
| vmot   | 4    | ADC1_CH4 — sensor output 1.45–3.80 V |
| vgen   | 5    | ADC1_CH5 — sensor output 1.45–3.80 V |
| speed  | 14   | Digital pulse input (encoder / hall / tachometer) |
| RS-485 TX | 6 | UART1 |
| RS-485 RX | 7 | UART1 |

> **Note:** GPIO14 is a digital-only pin on ESP32-C6. Motor speed is measured by counting rising edges per 10 ms window (×100 = pulses/second). To convert to RPM: `RPM = speed_Hz × 60 / pulses_per_revolution`.

## Voltage Mapping

The analog sensors produce a biased output centred at 1.65 V. The firmware maps this piecewise-linearly to motor voltage:

```
1.45 V  →  −12 V
1.65 V  →    0 V  (zero-cross)
3.80 V  →  +12 V
```

Both channels use 11 dB ADC attenuation to cover the full 0–3.9 V input range.

## Modbus RTU Configuration

| Parameter | Value |
|-----------|-------|
| Slave address | 1 |
| Baud rate | 9600 |
| Framing | 8-N-1 |

### Holding Registers

| Address | Signal | Type |
|---------|--------|------|
| 0       | speed  | INT16 — pulses/second |
| 1–2     | vmot   | FLOAT32 (two 16-bit words, low word first) |
| 3–4     | vgen   | FLOAT32 (two 16-bit words, low word first) |

## Sampling Architecture

- **Timer**: IDF `esp_timer` periodic, 10 000 µs period → 100 Hz
- **ADC**: `analogReadMilliVolts()` called inside the timer task (safe, not in ISR)
- **Speed**: GPIO interrupt counts rising edges; the timer task snapshots and resets the counter each tick
- **Filtering**: 8-sample moving average on all three signals
- **Thread safety**: `portMUX_TYPE` spinlocks separate ISR ↔ timer-task and timer-task ↔ loop() access

## Building

Open `nano_slave/` as an Arduino sketch. Select board **ESP32C6 Dev Module** and install:

- [ModbusRTUSlave](https://github.com/CMB27/ModbusRTUSlave) library

## Python Master

```bash
cd python485master
pip install -r requirements.txt
python read_rs485.py
```

Reads all five Modbus registers and prints decoded vmot, vgen, and speed values.
