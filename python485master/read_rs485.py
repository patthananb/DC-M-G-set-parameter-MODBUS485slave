import argparse

from pymodbus.client import ModbusSerialClient
from serial.tools import list_ports

# Serial and Modbus settings
BAUDRATE = 9600
PARITY = "N"
STOPBITS = 1
BYTESIZE = 8
TIMEOUT = 1

SLAVE_ID = 1
START_ADDRESS = 0
REGISTER_COUNT = 5


def create_client(port: str) -> ModbusSerialClient:
    return ModbusSerialClient(
        port=port,
        baudrate=BAUDRATE,
        parity=PARITY,
        stopbits=STOPBITS,
        bytesize=BYTESIZE,
        timeout=TIMEOUT,
    )


def get_candidate_ports() -> list[str]:
    candidates: list[str] = []

    for p in list_ports.comports():
        text = " ".join(
            [
                p.device or "",
                p.description or "",
                p.manufacturer or "",
                p.product or "",
                p.hwid or "",
            ]
        ).lower()

        # Keep this broad so common USB-RS485 adapters are detected on macOS/Linux/Windows.
        if any(
            key in text
            for key in (
                "usb",
                "rs485",
                "ch340",
                "ch341",
                "ftdi",
                "cp210",
                "pl2303",
                "usbserial",
                "ttyusb",
                "tty.usb",
            )
        ):
            if "bluetooth" in text:
                continue
            candidates.append(p.device)

    # If filter was too strict, fall back to all serial ports.
    if not candidates:
        candidates = [p.device for p in list_ports.comports()]

    # De-duplicate while preserving order.
    seen = set()
    ordered_candidates: list[str] = []
    for port in candidates:
        if port not in seen:
            seen.add(port)
            ordered_candidates.append(port)
    return ordered_candidates


def decode_values(registers: list[int]) -> tuple[int, float, float]:
    # Match dc_motor_read.ino mapping:
    # register 0 -> int16 speed
    # registers 1-2 -> float32 vmot (little word order)
    # registers 3-4 -> float32 v_gen (little word order)
    speed = ModbusSerialClient.convert_from_registers(
        [registers[0]],
        data_type=ModbusSerialClient.DATATYPE.INT16,
    )
    v_mot = ModbusSerialClient.convert_from_registers(
        registers[1:3],
        data_type=ModbusSerialClient.DATATYPE.FLOAT32,
        word_order="little",
    )
    v_gen = ModbusSerialClient.convert_from_registers(
        registers[3:5],
        data_type=ModbusSerialClient.DATATYPE.FLOAT32,
        word_order="little",
    )
    return speed, v_mot, v_gen


def select_ports(ports: list[str], requested_port: str | None) -> list[str]:
    if requested_port:
        return [requested_port]

    print("Detected serial ports:")
    for i, port in enumerate(ports, start=1):
        print(f"  {i}. {port}")

    choice = input(
        "Select a port number, or type 'a' to scan all ports [a]: "
    ).strip().lower()

    if choice in ("", "a"):
        return ports

    if choice.isdigit():
        idx = int(choice)
        if 1 <= idx <= len(ports):
            return [ports[idx - 1]]

    print("Invalid selection. Scanning all ports.")
    return ports


def main() -> None:
    parser = argparse.ArgumentParser(description="Read Modbus values over USB-RS485")
    parser.add_argument(
        "--port",
        help="Use a specific serial port (for example: /dev/cu.usbserial-1140)",
    )
    args = parser.parse_args()

    ports = get_candidate_ports()
    if not ports:
        print("No serial ports detected. Plug in USB-RS485 and retry.")
        return

    selected_ports = select_ports(ports, args.port)
    print(f"Selected serial ports: {', '.join(selected_ports)}")

    for port in selected_ports:
        print(f"Trying port: {port}")
        client = create_client(port)

        if not client.connect():
            print(f"Connect failed on {port}")
            continue

        print(f"Connected on {port}")
        try:
            result = client.read_holding_registers(
                address=START_ADDRESS,
                count=REGISTER_COUNT,
                device_id=SLAVE_ID,
            )
        except Exception as exc:
            print(f"Read exception on {port}: {exc}")
            client.close()
            continue

        if result.isError():
            print(f"Read failed on {port}: {result}")
            client.close()
            continue

        speed, v_mot, v_gen = decode_values(result.registers)
        print(f"Speed: {speed}")
        print(f"V_mot: {v_mot:.2f}")
        print(f"V_gen: {v_gen:.2f}")
        client.close()
        return

    print("No responding Modbus slave found on detected ports.")


if __name__ == "__main__":
    main()