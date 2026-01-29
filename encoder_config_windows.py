import argparse
import serial
import time


class SimpleEncoderReader:
    def __init__(self, port="COM35", baudrate=9600, timeout=0.002):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_data = {f"J{i+1}": 0.0 for i in range(6)}

    def crc16_modbus(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return ((crc & 0xFF) << 8) | ((crc >> 8) & 0xFF)

    def _append_crc(self, frame):
        crc = self.crc16_modbus(frame)
        return frame + bytes([(crc >> 8) & 0xFF, crc & 0xFF])

    def _validate_crc(self, response):
        if len(response) < 3:
            return False
        crc = self.crc16_modbus(response[:-2])
        return response[-2] == ((crc >> 8) & 0xFF) and response[-1] == (crc & 0xFF)

    def _send_and_receive(self, ser, frame, expected_length, delay=0.02):
        ser.reset_input_buffer()
        ser.write(frame)
        time.sleep(delay)
        response = ser.read(expected_length)
        return response

    def read_encoder_value(self, slave_addr):
        frame = bytes([slave_addr, 0x03, 0x00, 0x01, 0x00, 0x01])
        frame = self._append_crc(frame)
        with serial.Serial(self.port, self.baudrate, timeout=self.timeout) as ser:
            response = self._send_and_receive(ser, frame, expected_length=7)

        if (
            len(response) == 7
            and response[0] == slave_addr
            and response[1] == 0x03
            and response[2] == 0x02
            and self._validate_crc(response)
        ):
            register_value = (response[3] << 8) | response[4]
            angle = round((register_value / 65536.0) * 360.0, 2)
            return angle
        return None

    def read_all_encoders_fast(self):
        try:
            with serial.Serial(self.port, self.baudrate, timeout=self.timeout) as ser:
                for i in range(6):
                    slave_addr = 0x01 + i
                    encoder_id = f"J{i+1}"

                    frame = bytes([slave_addr, 0x03, 0x00, 0x01, 0x00, 0x01])
                    frame = self._append_crc(frame)
                    response = self._send_and_receive(ser, frame, expected_length=7)

                    if (
                        len(response) == 7
                        and response[0] == slave_addr
                        and response[1] == 0x03
                        and response[2] == 0x02
                        and self._validate_crc(response)
                    ):
                        register_value = (response[3] << 8) | response[4]
                        angle = round((register_value / 65536.0) * 360.0, 2)
                        self.encoder_data[encoder_id] = angle
        except Exception as exc:
            print(f"Read error: {exc}")

        return self.encoder_data

    def set_baudrate(self, slave_addr, baudrate_value):
        if not (0 <= baudrate_value <= 0xFFFFFFFF):
            raise ValueError("baudrate_value must fit in 32 bits")

        baud_bytes = baudrate_value.to_bytes(4, byteorder="big")
        frame = bytes([slave_addr, 0x10, 0x00, 0x65, 0x00, 0x02, 0x04]) + baud_bytes
        frame = self._append_crc(frame)

        with serial.Serial(self.port, self.baudrate, timeout=self.timeout) as ser:
            response = self._send_and_receive(ser, frame, expected_length=8)

        return (
            len(response) == 8
            and response[0] == slave_addr
            and response[1] == 0x10
            and response[2:6] == bytes([0x00, 0x65, 0x00, 0x02])
            and self._validate_crc(response)
        )

    def set_slave_id(self, current_id, new_id):
        if not (1 <= new_id <= 247):
            raise ValueError("new_id must be between 1 and 247")

        frame = bytes([current_id, 0x06, 0x00, 0x68, 0x00, new_id])
        frame = self._append_crc(frame)

        with serial.Serial(self.port, self.baudrate, timeout=self.timeout) as ser:
            response = self._send_and_receive(ser, frame, expected_length=8)

        return (
            len(response) == 8
            and response[0] == current_id
            and response[1] == 0x06
            and response[2:6] == bytes([0x00, 0x68, 0x00, new_id])
            and self._validate_crc(response)
        )

    def save_settings(self, slave_addr, function_code=0x06):
        frame = bytes([slave_addr, function_code, 0x00, 0x0A, 0x00, 0x01])
        frame = self._append_crc(frame)

        with serial.Serial(self.port, self.baudrate, timeout=self.timeout) as ser:
            response = self._send_and_receive(ser, frame, expected_length=8)

        return (
            len(response) == 8
            and response[0] == slave_addr
            and response[1] == function_code
            and response[2:6] == bytes([0x00, 0x0A, 0x00, 0x01])
            and self._validate_crc(response)
        )

    def configure_device(self, current_id, new_id, new_baudrate):
        angle = self.read_encoder_value(current_id)
        if angle is None:
            print("Communication check failed, no response to read.")
            return False

        print(f"Communication OK, current angle: {angle} deg")

        if not self.set_baudrate(current_id, new_baudrate):
            print("Failed to set baudrate.")
            return False

        if not self.set_slave_id(current_id, new_id):
            print("Failed to set slave ID.")
            return False

        if not self.save_settings(current_id):
            print("Failed to save settings.")
            return False

        print("Settings saved. Power cycle the device to apply changes.")
        return True


def continuous_reading(port, baudrate):
    reader = SimpleEncoderReader(port=port, baudrate=baudrate)
    print("Start continuous reading...")
    count = 0

    try:
        while True:
            count += 1
            data = reader.read_all_encoders_fast()
            formatted = [f"{k}: {v} deg" for k, v in data.items()]
            print(f"#{count}: {{{', '.join(formatted)}}}")
    except KeyboardInterrupt:
        print("\nStopped.")


def main():
    parser = argparse.ArgumentParser(description="Encoder configuration for Windows.")
    parser.add_argument("--port", default="COM35", help="Serial port (default: COM35)")
    parser.add_argument("--baudrate", type=int, default=9600, help="Current baudrate")
    parser.add_argument("--read", action="store_true", help="Read one encoder value")
    parser.add_argument("--continuous", action="store_true", help="Continuous reading")
    parser.add_argument("--current-id", type=lambda x: int(x, 0), default=0x01)
    parser.add_argument("--new-id", type=lambda x: int(x, 0), default=0x04)
    parser.add_argument("--new-baudrate", type=int, default=921600)
    args = parser.parse_args()

    reader = SimpleEncoderReader(port=args.port, baudrate=args.baudrate)

    if args.read:
        angle = reader.read_encoder_value(args.current_id)
        print(f"Angle: {angle}" if angle is not None else "No response.")
        return

    if args.continuous:
        continuous_reading(args.port, args.baudrate)
        return

    reader.configure_device(
        current_id=args.current_id,
        new_id=args.new_id,
        new_baudrate=args.new_baudrate,
    )


if __name__ == "__main__":
    main()
