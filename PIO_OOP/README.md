# PIO_OOP — DC Motor Reading (PlatformIO + OOP refactor)

PlatformIO port of the original Arduino IDE sketches in `../modbus_slave/`,
`../serial_monitor/`, and `../esp32slave/`. Same hardware behavior, refactored
into `struct` types and small helper functions under outer `namespace config`,
with measuring code in `config::dcmotormeasurement` and Modbus code in
`config::modbus`. Build envs target the deployment scenarios.

The original sketches are unchanged — this folder is a parallel implementation.

## Layout

```
PIO_OOP/
├── platformio.ini
├── lib/
│   └── DCMotor/                    shared library, namespace config
│       └── src/
│           ├── Config.h            config::settings constants
│           ├── VoltageMap.{h,cpp}  ADC volts -> motor volts (piecewise linear)
│           ├── Measurements.h      shared measurement snapshot type
│           ├── RegisterMap.{h,cpp} packs measurements into Modbus registers
│           ├── Sampler.{h,cpp}     100 Hz esp_timer + ISR + 8-sample MA
│           ├── DCMotorMeasuring.{h,cpp} high-level motor measuring struct
│           ├── PulseSimulator.{h,cpp}  LEDC 1 kHz square wave on GPIO20
│           ├── ModbusRTU.{h,cpp}       wraps ModbusRTUSlave
│           └── ModbusTCP.{h,cpp}       Modbus TCP server over Wi-Fi
└── src/
    ├── monitor/main.cpp            USB serial only, no Modbus
    ├── slave_full/main.cpp         Modbus slave + verbose debug
    ├── slave_ref/main.cpp          Modbus slave + terse debug (esp32slave-style)
    └── slave_tcp/main.cpp          Modbus TCP slave over Wi-Fi
```

## Build envs

| Env          | Purpose                                       | Equivalent old sketch |
|--------------|-----------------------------------------------|------------------------|
| `monitor`    | USB serial only, 100 Hz, no RS-485            | `serial_monitor/`      |
| `slave_full` | Modbus RTU slave + verbose serial debug       | `modbus_slave/`        |
| `slave_ref`  | Modbus RTU slave + terse serial debug         | `esp32slave/`          |
| `slave_tcp`  | Modbus TCP slave over Wi-Fi                   | new PIO-only sketch    |

`ModbusRTUSlave` is declared as a dependency of the `DCMotor` library in
`lib/DCMotor/library.json`, so PlatformIO can build the RTU envs. The TCP source
is gated by `DCMOTOR_ENABLE_MODBUS_TCP`, so `monitor` and RTU envs do not pull
in the Wi-Fi stack.

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
pio run -d PIO_OOP -e slave_tcp
```

## Flash + monitor

```bash
pio run -d PIO_OOP -e <env> -t upload
pio device monitor -d PIO_OOP -e <env>
```

Default env (`pio run -d PIO_OOP` with no `-e`) is `slave_full`.

## OOP design

All shared code lives under outer `namespace config`. Constants are in
`config::settings`, motor measuring is in `config::dcmotormeasurement`, and
Modbus servers are in `config::modbus`. Hardware-facing objects are instantiated
at file scope in each `main.cpp`:

```cpp
config::dcmotormeasurement::DCMotorMeasuring motor;
config::modbus::ModbusRTU rtuSlave(RS485Serial);  // RTU, takes serial via ctor
config::modbus::ModbusTCP tcpSlave;               // TCP, owns Wi-Fi server
```

`DCMotorMeasuring` is the public motor measuring struct. It owns the `Sampler`
and optional `PulseSimulator`, so sketches do not need to wire sampling
internals directly. `Sampler` owns the `esp_timer` handle, pulse counter,
8-sample moving-average buffers, and two `portMUX_TYPE` spinlocks. ISR + timer
trampolines are static members that forward to the singleton instance set by
`begin()`. Caller pulls fresh measurements as
`config::dcmotormeasurement::Measurements` via `readIfNew` / `readLatest`.

`RegisterMap` owns the seven-register packing function. `config::modbus::ModbusRTU`
and `config::modbus::ModbusTCP` both call that same function, so the register
layout stays identical. `ModbusRTU::update(const Measurements&)` packs the
snapshot into holding registers using the same `memcpy` layout as the original
`modbus_slave.ino` so `python485master/modbus_master.py` reads it without any
wire-level changes.

`ModbusTCP` implements Modbus TCP function `0x03` (read holding registers) on
port `502`. It uses the same unit id/address (`1`) and holding
register layout as the RTU sketches.

## Modbus TCP Wi-Fi config

`slave_tcp` uses placeholder Wi-Fi credentials by default. The env already
enables TCP support; add the two credential flags to the existing
`[env:slave_tcp]` `build_flags` before flashing:

```ini
build_flags =
  ${env.build_flags}
  -DDCMOTOR_ENABLE_MODBUS_TCP=1
  -I$PROJECT_PACKAGES_DIR/framework-arduinoespressif32/libraries/Network/src
  -DDCMOTOR_WIFI_SSID=\"your-ssid\"
  -DDCMOTOR_WIFI_PASSWORD=\"your-password\"
```

Then build and upload:

```bash
pio run -d PIO_OOP -e slave_tcp -t upload
```

`config::dcmotormeasurement::adcToMotor(float)` is a free function (not a
struct) — pure piecewise-linear math, no state, no need for a wrapper.

## Debug output

All sketches print debug via a global `Print*` pointer declared in
[`lib/DCMotor/src/Debug.h`](lib/DCMotor/src/Debug.h):

```cpp
extern Print* dbg;   // defined in Debug.cpp as &Serial
```

The pointer is global, outside the `config` namespace, so call sites stay
short. Type is `Print*` so the same pointer can hold any `Print`-derived
sink (`HWCDC`, `HardwareSerial`, `USBCDC`, custom subclass).

Sketches call `Serial.begin(config::settings::DEBUG_BAUD)` in `setup()` then use `printf`-style
formatting at every print site:

```cpp
dbg->printf("speed=%d, vmot=%.3f V\n", m.speed, m.vmot);
```

`Serial` is mapped to the ESP32-C6's native **USB-Serial-JTAG** (HWCDC)
peripheral by two build flags in [`platformio.ini`](platformio.ini):

```ini
build_flags =
  -DARDUINO_USB_MODE=1          # select HWCDC (not TinyUSB USBCDC)
  -DARDUINO_USB_CDC_ON_BOOT=1   # bind Serial to the HWCDC instance
```

Plug the host PC into the **USB-C** connector labelled "USB" (not "UART")
on the ESP32-C6-DevKitC-1 to see debug output. To redirect to the
USB-UART bridge instead, reassign `dbg = &Serial0;` in `setup()`.

## Hardware (unchanged from parent project)

- ESP32-C6 dev board, GPIO4=vmot, GPIO5=vgen, GPIO14=speed, GPIO6/7=RS-485,
  GPIO20=LEDC simulator output.
- Bench testing: jumper GPIO20 → GPIO14, expect `1000 pulse/s` and `3000.0 RPM`.

## Modbus map (slave_full, slave_ref, and slave_tcp — identical)

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
| `monitor`    | 292 443 / 1 310 720 B  | 22.3 %   | 15 208 / 327 680 B  | 4.6 %   |
| `slave_full` | 308 859 / 1 310 720 B  | 23.6 %   | 15 608 / 327 680 B  | 4.8 %   |
| `slave_ref`  | 308 571 / 1 310 720 B  | 23.5 %   | 15 608 / 327 680 B  | 4.8 %   |
| `slave_tcp`  | 1 006 765 / 1 310 720 B | 76.8 %   | 42 940 / 327 680 B  | 13.1 %  |

The USB-CDC stack (HWCDC + linker-pulled USB peripheral driver) accounts
for the ~13 KB flash / ~800 B RAM increase over the previous Serial0-based
build. `slave_tcp` is larger because it links the ESP32 Networking and Wi-Fi
libraries.

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
