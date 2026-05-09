# PIO_OOP — DC Motor Reading

PlatformIO version of the ESP32-C6 motor reader and Modbus slave. The firmware
keeps the everyday structure small:

```cpp
config::motor::MotorMeasurement motor;
config::modbus::ModbusRTU modbus(RS485Serial);
```

All firmware pins and tuning values live in one file:
`lib/DCMotor/src/Config.h`.

## Layout

```text
PIO_OOP/
├── platformio.ini
├── lib/DCMotor/src/
│   ├── Config.h               all pins, ADC, sampling, Modbus, Wi-Fi config
│   ├── MotorMeasurement.h     config::motor::Measurements + MotorMeasurement
│   ├── MotorMeasurement.cpp   ADC reading, speed ISR, RPM, moving average,
│   │                          and optional GPIO20 pulse simulator
│   ├── RegisterMap.h
│   ├── RegisterMap.cpp        packs Measurements into 7 Modbus registers
│   ├── ModbusRTU.h
│   ├── ModbusRTU.cpp          RS-485 Modbus slave
│   ├── ModbusTCP.h
│   ├── ModbusTCP.cpp          optional Wi-Fi Modbus TCP slave
│   ├── Debug.h
│   └── Debug.cpp
└── src/
    ├── monitor/main.cpp       USB serial only, no Modbus
    ├── slave_full/main.cpp    Modbus RTU + verbose debug
    ├── slave_ref/main.cpp     Modbus RTU + terse debug
    └── slave_tcp/main.cpp     optional Modbus TCP over Wi-Fi
```

## Namespaces

- `config::settings`: pins and constants from `Config.h`
- `config::motor`: motor measurement structs and ADC voltage mapping
- `config::modbus`: RTU and optional TCP Modbus structs

The main RTU sketch is intentionally plain:

```cpp
HardwareSerial RS485Serial(1);
config::motor::MotorMeasurement motor;
config::modbus::ModbusRTU slave(RS485Serial);

void setup() {
  Serial.begin(config::settings::DEBUG_BAUD);
  motor.begin();
  slave.begin();
}

void loop() {
  config::motor::Measurements m;
  const bool fresh = motor.readIfNew(m);
  if (!fresh) motor.readLatest(m);

  slave.update(m);
  slave.poll();
}
```

## Build Envs

| Env          | Purpose                                 |
|--------------|------------------------------------------|
| `monitor`    | USB serial only, no RS-485 or Modbus     |
| `slave_full` | Modbus RTU slave + verbose serial debug  |
| `slave_ref`  | Modbus RTU slave + terse serial debug    |
| `slave_tcp`  | Modbus TCP slave over Wi-Fi              |

Default env is `slave_full`.

```bash
pio run -d PIO_OOP -e slave_full
```

If a `pyenv` shim hides PlatformIO's bundled tools, use:

```bash
PATH="$HOME/.platformio/penv/bin:$PATH" pio run -d PIO_OOP -e slave_full
```

## Hardware

- ESP32-C6 DevKitC-1
- vmot: GPIO4, ADC 11 dB attenuation
- vgen: GPIO5, ADC 11 dB attenuation
- speed: GPIO14, digital rising-edge pulse input
- RS-485 UART1: TX GPIO6, RX GPIO7, 9600 baud, Modbus address 1
- bench simulator: GPIO20 LEDC 1000 Hz square wave

For bench testing, jumper GPIO20 to GPIO14. Expected speed is `1000 pulse/s`
and expected RPM is `3000.0` with `PULSES_PER_REV = 20`.

## Modbus Registers

Holding registers are identical for RTU and TCP.

| Address | Signal | Type    | Unit            |
|---------|--------|---------|-----------------|
| 0       | speed  | INT16   | pulses / second |
| 1-2     | vmot   | FLOAT32 | volts           |
| 3-4     | vgen   | FLOAT32 | volts           |
| 5-6     | rpm    | FLOAT32 | RPM             |

FLOAT32 values use little-endian word order, with the low word at the lower
register address.

## Optional Modbus TCP

The RTU build is the main path. TCP is still available, but only the `slave_tcp`
env enables it with `DCMOTOR_ENABLE_MODBUS_TCP=1`.

Add Wi-Fi credentials in `[env:slave_tcp]` before flashing:

```ini
build_flags =
  ${env.build_flags}
  -DDCMOTOR_ENABLE_MODBUS_TCP=1
  -I$PROJECT_PACKAGES_DIR/framework-arduinoespressif32/libraries/Network/src
  -DDCMOTOR_WIFI_SSID=\"your-ssid\"
  -DDCMOTOR_WIFI_PASSWORD=\"your-password\"
```

## Notes

- Do not use `timerBegin(num, prescaler, up)` with Arduino core 3.x; the code
  uses `esp_timer`.
- Do not use `analogRead(14)`; GPIO14 is a pulse input, not an ESP32-C6 ADC pin.
- Timer and ISR shared state is protected with `portMUX_TYPE`.
- `ledcAttach(pin, freq, bits)` and `ledcWrite(pin, duty)` are used for the
  simulator.
