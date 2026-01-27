import argparse
import math
import struct
import unittest
from typing import Iterable, List, Sequence, Tuple


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(data: bytes) -> bytes:
    crc = crc16_modbus(data)
    return data + struct.pack("<H", crc)


def build_read_holding_registers_request(
    device_id: int,
    start_addr: int,
    quantity: int,
    func_id: int = 0x03,
) -> bytes:
    if not (0 <= device_id <= 0xFF):
        raise ValueError("device_id out of range (0-255)")
    if not (0 <= start_addr <= 0xFFFF):
        raise ValueError("start_addr out of range (0-65535)")
    if not (1 <= quantity <= 0x7D):
        raise ValueError("quantity out of range (1-125)")
    frame = struct.pack(">B B H H", device_id, func_id, start_addr, quantity)
    return append_crc(frame)


def parse_read_holding_registers_response(frame: bytes) -> Tuple[int, int, bytes]:
    if len(frame) < 5:
        raise ValueError("frame too short")
    body = frame[:-2]
    recv_crc = struct.unpack("<H", frame[-2:])[0]
    calc_crc = crc16_modbus(body)
    if recv_crc != calc_crc:
        raise ValueError(f"crc mismatch: recv=0x{recv_crc:04X} calc=0x{calc_crc:04X}")
    device_id = frame[0]
    func_id = frame[1]
    byte_count = frame[2]
    data = frame[3:-2]
    if byte_count != len(data):
        raise ValueError(
            f"byte_count mismatch: header={byte_count} actual={len(data)}"
        )
    return device_id, func_id, data


def decode_float16_registers(data: bytes, byteorder: str = "big") -> List[float]:
    if len(data) % 2 != 0:
        raise ValueError("float16 decode needs even number of bytes")
    if byteorder not in ("big", "little"):
        raise ValueError("byteorder must be 'big' or 'little'")
    fmt = (">" if byteorder == "big" else "<") + ("e" * (len(data) // 2))
    return list(struct.unpack(fmt, data))


def decode_float32_registers(
    data: bytes, byteorder: str = "big", wordorder: str = "big"
) -> List[float]:
    if len(data) % 4 != 0:
        raise ValueError("float32 decode needs byte count multiple of 4")
    if byteorder not in ("big", "little"):
        raise ValueError("byteorder must be 'big' or 'little'")
    if wordorder not in ("big", "little"):
        raise ValueError("wordorder must be 'big' or 'little'")
    registers = [data[i : i + 2] for i in range(0, len(data), 2)]
    if byteorder == "little":
        registers = [reg[::-1] for reg in registers]
    values: List[float] = []
    for i in range(0, len(registers), 2):
        pair = registers[i : i + 2]
        if wordorder == "little":
            pair = pair[::-1]
        raw = pair[0] + pair[1]
        values.append(struct.unpack(">f", raw)[0])
    return values


def decode_quaternion(
    data: bytes,
    float_format: str = "float16",
    byteorder: str = "big",
    wordorder: str = "big",
) -> Tuple[float, float, float, float]:
    if float_format == "float16":
        values = decode_float16_registers(data, byteorder=byteorder)
    elif float_format == "float32":
        values = decode_float32_registers(
            data, byteorder=byteorder, wordorder=wordorder
        )
    else:
        raise ValueError("float_format must be 'float16' or 'float32'")
    if len(values) != 4:
        raise ValueError(
            f"expected 4 values, got {len(values)}. "
            "For float32, read 8 registers (16 bytes)."
        )
    return tuple(values)  # type: ignore[return-value]


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def read_rtu_frame_from_serial(
    port: str, baudrate: int, timeout: float, request: bytes
) -> bytes:
    try:
        import serial
    except ImportError as exc:  # pragma: no cover - optional path
        raise RuntimeError("pyserial is required for serial access") from exc

    with serial.Serial(port, baudrate=baudrate, timeout=timeout) as ser:
        ser.reset_input_buffer()
        ser.write(request)
        header = ser.read(3)
        if len(header) < 3:
            raise RuntimeError("timeout waiting for response header")
        byte_count = header[2]
        payload = ser.read(byte_count + 2)
        if len(payload) < byte_count + 2:
            raise RuntimeError("timeout waiting for response payload")
        return header + payload


def format_values(values: Sequence[float]) -> str:
    parts = []
    for name, value in zip(("qw", "qx", "qy", "qz"), values):
        if isinstance(value, float) and math.isnan(value):
            parts.append(f"{name}=nan")
        else:
            parts.append(f"{name}={value}")
    return ", ".join(parts)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Modbus RTU test case for quaternion decoding"
    )
    parser.add_argument("--port", help="serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--device-id", type=lambda x: int(x, 0), default=0x02)
    parser.add_argument("--start-addr", type=lambda x: int(x, 0), default=0x0046)
    parser.add_argument("--quantity", type=int, default=4)
    parser.add_argument(
        "--float-format", choices=("float16", "float32"), default="float16"
    )
    parser.add_argument("--byteorder", choices=("big", "little"), default="big")
    parser.add_argument("--wordorder", choices=("big", "little"), default="big")
    args = parser.parse_args()

    request = build_read_holding_registers_request(
        args.device_id, args.start_addr, args.quantity
    )
    print(f"TX: {hex_bytes(request)}")

    if args.port:
        response = read_rtu_frame_from_serial(
            args.port, args.baudrate, args.timeout, request
        )
    else:
        response = bytes.fromhex("02 03 08 26 FC FF E8 FF C7 02 E8 D0 71")
    print(f"RX: {hex_bytes(response)}")

    device_id, func_id, data = parse_read_holding_registers_response(response)
    if device_id != args.device_id or func_id != 0x03:
        raise ValueError("unexpected device_id or func_id in response")

    quaternion = decode_quaternion(
        data,
        float_format=args.float_format,
        byteorder=args.byteorder,
        wordorder=args.wordorder,
    )
    print(format_values(quaternion))
    return 0


class ModbusRtuSampleTest(unittest.TestCase):
    def test_crc_and_parse_sample(self) -> None:
        request = build_read_holding_registers_request(0x02, 0x0046, 0x0004)
        self.assertEqual(hex_bytes(request), "02 03 00 46 00 04 A5 EF")

        response = bytes.fromhex("02 03 08 26 FC FF E8 FF C7 02 E8 D0 71")
        device_id, func_id, data = parse_read_holding_registers_response(response)
        self.assertEqual(device_id, 0x02)
        self.assertEqual(func_id, 0x03)
        self.assertEqual(hex_bytes(data), "26 FC FF E8 FF C7 02 E8")

        values = decode_quaternion(data, float_format="float16")
        self.assertEqual(len(values), 4)


if __name__ == "__main__":
    raise SystemExit(main())
