# PIO_OOP — DC Motor Reading (PlatformIO + OOP refactor)

PlatformIO port of the original Arduino IDE sketches in `../modbus_slave/`,
`../serial_monitor/`, and `../esp32slave/`. Same hardware behavior, refactored
into `struct` types inside `namespace dcmotor` with a shared library under
`lib/DCMotor/`. Three build envs target the three deployment scenarios.

The original sketches are unchanged — this folder is a parallel implementation.

## Layout

```
PIO_OOP/
├── platformio.ini
├── lib/
│   └── DCMotor/                    shared library, namespace dcmotor
│       └── src/
│           ├── Config.h            pin / timing / voltage / Modbus constants
│           ├── VoltageMap.{h,cpp}  ADC volts -> motor volts (piecewise linear)
│           ├── Sampler.{h,cpp}     100 Hz esp_timer + ISR + 8-sample MA
│           ├── PulseSimulator.{h,cpp}  LEDC 1 kHz square wave on GPIO20
│           └── ModbusServer.{h,cpp}    wraps ModbusRTUSlave (gated by __has_include)
└── src/
    ├── monitor/main.cpp            USB serial only, no Modbus
    ├── slave_full/main.cpp         Modbus slave + verbose debug
    └── slave_ref/main.cpp          Modbus slave + terse debug (esp32slave-style)
```

## Build envs

| Env          | Purpose                                       | Equivalent old sketch |
|--------------|-----------------------------------------------|------------------------|
| `monitor`    | USB serial only, 100 Hz, no RS-485            | `serial_monitor/`      |
| `slave_full` | Modbus RTU slave + verbose serial debug       | `modbus_slave/`        |
| `slave_ref`  | Modbus RTU slave + terse serial debug         | `esp32slave/`          |

`ModbusRTUSlave` is declared as a dependency of the `DCMotor` library in
`lib/DCMotor/library.json`, so PlatformIO pulls it for every env. The `monitor`
env never references the `ModbusServer` symbols, so the linker drops them via
`-ffunction-sections` / `-fdata-sections` GC.

## Platform pin

Arduino-core 3.x on ESP32-C6 comes from
[pioarduino/platform-espressif32](https://github.com/pioarduino/platform-espressif32),
not Espressif mainline. `platformio.ini` pins a fixed release tag for
reproducible builds. Bump as needed.

## Build

```bash
pio run -d PIO_OOP -e monitor
pio run -d PIO_OOP -e slave_full
pio run -d PIO_OOP -e slave_ref
```

## Flash + monitor

```bash
pio run -d PIO_OOP -e <env> -t upload
pio device monitor -d PIO_OOP -e <env>
```

Default env (`pio run -d PIO_OOP` with no `-e`) is `slave_full`.

## OOP design

All shared code lives in `namespace dcmotor`. Hardware singletons are exposed
as plain `struct` types instantiated at file scope in each `main.cpp`:

```cpp
dcmotor::Sampler        sampler;
dcmotor::PulseSimulator simulator;
dcmotor::ModbusServer   slave(RS485Serial);   // takes serial via ctor
```

`Sampler` owns the `esp_timer` handle, the pulse counter, the 8-sample
moving-average buffers, and two `portMUX_TYPE` spinlocks. ISR + timer
trampolines are static members that forward to the singleton instance set
by `begin()`. Caller pulls fresh measurements as `dcmotor::Measurements`
via `readIfNew` / `readLatest`.

`ModbusServer::update(const Measurements&)` packs the snapshot into
holding registers using the same `memcpy` layout as the original
`modbus_slave.ino` so `python485master/modbus_master.py` reads it
without any wire-level changes.

`dcmotor::adcToMotor(float)` is a free function (not a struct) — pure
piecewise-linear math, no state, no need for a class wrapper.

## Debug output

All sketches print debug via a shared `HardwareSerial*` pointer in
[`lib/DCMotor/src/Debug.h`](lib/DCMotor/src/Debug.h):

```cpp
namespace dcmotor {
extern HardwareSerial* dbg;   // defined in Debug.cpp as &Serial0
}
```

Sketches call `Serial0.begin(115200)` in `setup()` then use `printf`-style
formatting at every print site:

```cpp
dcmotor::dbg->printf("speed=%d, vmot=%.3f V\n", m.speed, m.vmot);
```

`Serial0` is UART0 on the ESP32-C6, routed through the on-board USB-UART
bridge (not the native USB-CDC port — that one is `Serial`). Reassign
`dcmotor::dbg` in `setup()` if you need a different sink.

## Hardware (unchanged from parent project)

- ESP32-C6 dev board, GPIO4=vmot, GPIO5=vgen, GPIO14=speed, GPIO6/7=RS-485,
  GPIO20=LEDC simulator output.
- Bench testing: jumper GPIO20 → GPIO14, expect `1000 pulse/s` and `3000.0 RPM`.

## Modbus map (slave_full and slave_ref — identical)

| Address | Signal | Type    | Unit            |
|---------|--------|---------|-----------------|
| 0       | speed  | INT16   | pulses / second |
| 1–2     | vmot   | FLOAT32 | volts           |
| 3–4     | vgen   | FLOAT32 | volts           |
| 5–6     | rpm    | FLOAT32 | RPM             |

FLOAT32 values use little-endian word order (low word at lower address).

## Testing

With simulator jumper installed (GPIO20 → GPIO14):

`monitor` env serial output (115200 baud):
```
1000 pulse/s | 3000.0 RPM | <vmot> V | <vgen> V
```

`slave_full` env serial output:
```
1000 pulse/s | 3000.0 RPM | <vmot> V | <vgen> V | <vmot_raw> V raw | <vgen_raw> V raw
```

`slave_ref` env serial output:
```
speed=1000, vmot=<...> V, vgen=<...> V
```

Modbus master (from `../python485master/`):
```
Speed: 1000 pulse/s
RPM:   3000.0 RPM
V_mot: <vmot> V
V_gen: <vgen> V
```

## Resource usage

Target: ESP32-C6 (`esp32-c6-devkitc-1`), `pioarduino` 55.03.30-1, default Release build.
Totals: flash 1 310 720 B, RAM 327 680 B (DRAM heap reported by linker).

| Env          | Flash (used / total)   | Flash %  | RAM (used / total)  | RAM %   |
|--------------|------------------------|----------|---------------------|---------|
| `monitor`    | 278 885 / 1 310 720 B  | 21.3 %   | 14 392 / 327 680 B  | 4.4 %   |
| `slave_full` | 303 683 / 1 310 720 B  | 23.2 %   | 15 496 / 327 680 B  | 4.7 %   |
| `slave_ref`  | 303 397 / 1 310 720 B  | 23.1 %   | 15 496 / 327 680 B  | 4.7 %   |

Reproduce:

```bash
pio run -d PIO_OOP -e <env>
```

(`-t size` invokes `esp_idf_size --ng`, which the bundled `esp-idf-size` rejects;
use the basic build summary above instead.)

If `pio` aborts on bootloader.bin with `ModuleNotFoundError: No module named
'esptool'`, a `pyenv` shim is shadowing PlatformIO's bundled `esptool`. Prefix
the command with PlatformIO's penv on the PATH:

```bash
PATH="$HOME/.platformio/penv/bin:$PATH" pio run -d PIO_OOP -e <env>
```
