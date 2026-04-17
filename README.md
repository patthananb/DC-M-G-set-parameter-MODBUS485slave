# DC Motor Reading

ESP32-C6 firmware that samples motor voltage, generator voltage, and motor speed at 100 Hz and exposes the values over RS-485 Modbus RTU. A companion Python script acts as the Modbus master.

## Hardware Connections

| Signal    | GPIO | Notes                                             |
|-----------|------|---------------------------------------------------|
| vmot      | 4    | ADC1_CH4 — sensor output 1.45–3.80 V             |
| vgen      | 5    | ADC1_CH5 — sensor output 1.45–3.80 V             |
| speed     | 14   | Digital pulse input (encoder / hall / tachometer) |
| RS-485 TX | 6    | UART1                                             |
| RS-485 RX | 7    | UART1                                             |

> **Note:** GPIO14 is a digital-only pin on ESP32-C6. Motor speed is measured by
> counting rising edges per 10 ms window (×100 = pulses/second).
> Convert to RPM: `RPM = speed_Hz × 60 / pulses_per_revolution`

## Voltage Mapping

The analog sensors produce a biased output centred at 1.65 V. The firmware maps
this piecewise-linearly to motor voltage:

```
1.45 V  →  −12 V
1.65 V  →    0 V  (zero-cross)
3.80 V  →  +12 V
```

Both channels use 11 dB ADC attenuation to cover the full 0–3.9 V input range.

## Modbus RTU Configuration

| Parameter     | Value |
|---------------|-------|
| Slave address | 1     |
| Baud rate     | 9600  |
| Framing       | 8-N-1 |

### Holding Registers

| Address | Signal | Type                                  |
|---------|--------|---------------------------------------|
| 0       | speed  | INT16 — pulses/second                 |
| 1–2     | vmot   | FLOAT32 (two 16-bit words, low first) |
| 3–4     | vgen   | FLOAT32 (two 16-bit words, low first) |

## Arduino Sketches

Three sketches are provided — pick whichever suits your use case:

### `esp32slave_combined/` ← **recommended for most users**
Single self-contained `.ino` file. Combines parameter sampling and Modbus RTU
transmission in one place — no dependencies between files. Open this sketch to
build the full Modbus slave in one step.

### `esp32slave/`
Two-file version of the same Modbus slave:
- `esp32slave.ino` — `setup()`, `loop()`, and Modbus polling
- `read_paramters.ino` — 100 Hz sampling helper (timer, ISR, MA filter)

Useful if you want to keep the sampling and transport layers separate.

### `dc_motor_reader/`
Standalone sketch with no Modbus dependency. Prints vmot, vgen, and speed over
USB Serial at 100 Hz. Use this for quick hardware validation without RS-485.

## Sampling Architecture

- **Timer**: IDF `esp_timer` periodic, 10 000 µs period → 100 Hz
- **ADC**: `analogReadMilliVolts()` called inside the timer task (not in ISR)
- **Speed**: GPIO interrupt counts rising edges; timer task snapshots and resets the counter each tick
- **Filtering**: 8-sample moving average on all three signals
- **Thread safety**: `portMUX_TYPE` spinlocks guard ISR ↔ timer-task and timer-task ↔ loop() access

## Building

1. Open the sketch folder in Arduino IDE (File → Open → select the `.ino` file).
2. Select board **ESP32C6 Dev Module**.
3. Install the [ModbusRTUSlave](https://github.com/CMB27/ModbusRTUSlave) library
   *(not required for `dc_motor_reader`)*.
4. Upload.

## Python Master

```bash
cd python485master
pip install -r requirements.txt
python read_rs485.py
```

Auto-detects USB-RS485 adapters and reads speed, vmot, and vgen from the slave.
Pass `--port /dev/cu.usbserial-XXXX` to skip port selection.
