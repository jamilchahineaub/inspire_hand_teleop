"""Inspire RH56 dexterous-hand RS485/RS232 driver.

Trimmed from the hardware-tested diagnostic script (~/Downloads/test.py) — the
CLI/argparse layer is dropped, the field-validated `InspireHand` transaction
class is kept verbatim. Frame checksums were cross-checked against the RH56
datasheet worked examples (ANGLE_SET write → 0x70, ANGLE_ACT read → 0x32).

Serial: 115200 8N1. Default hand_id is 2 (the ID our physical hand ships with).

Register map (subset):
    CLEAR_ERROR  0x03EC   ANGLE_SET 0x05CE   ANGLE_ACT 0x060A
    ERROR        0x0646   STATUS    0x064C
"""

import time

import serial


TX_HEADER = bytes([0xEB, 0x90])
RX_HEADER = bytes([0x90, 0xEB])

CMD_READ_REGISTER = 0x11
CMD_WRITE_REGISTER = 0x12

REG_CLEAR_ERROR = 0x03EC
REG_ANGLE_SET = 0x05CE
REG_ANGLE_ACT = 0x060A
REG_ERROR = 0x0646
REG_STATUS = 0x064C


class InspireHandError(RuntimeError):
    pass


class InspireHand:
    """Synchronous register read/write interface to one RH56 hand."""

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=115200,
        hand_id=2,
        serial_timeout=0.02,
        response_timeout=0.5,
        debug=False,
    ):
        self.id = hand_id
        self.response_timeout = response_timeout
        self.debug = debug
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=serial_timeout,
            write_timeout=1.0,
        )

    @staticmethod
    def hex_bytes(data):
        return " ".join(f"{byte:02X}" for byte in data) or "<none>"

    @staticmethod
    def checksum(frame):
        return sum(frame[2:-1]) & 0xFF

    @staticmethod
    def words_to_bytes(values):
        data = bytearray()
        for value in values:
            value = int(value)
            data.append(value & 0xFF)
            data.append((value >> 8) & 0xFF)
        return data

    @staticmethod
    def bytes_to_words(data):
        if len(data) % 2:
            raise InspireHandError(f"Register payload length is odd: {len(data)}")
        return [data[i] | (data[i + 1] << 8) for i in range(0, len(data), 2)]

    def close(self):
        self.ser.close()

    def build_frame(self, cmd, address, payload=b""):
        payload = bytes(payload)
        data_len = 3 + len(payload)
        frame = bytearray(
            [
                TX_HEADER[0],
                TX_HEADER[1],
                self.id,
                data_len,
                cmd,
                address & 0xFF,
                (address >> 8) & 0xFF,
            ]
        )
        frame.extend(payload)
        frame.append(0)
        frame[-1] = self.checksum(frame)
        return frame

    def _debug(self, prefix, data):
        if self.debug:
            print(f"{prefix}: {self.hex_bytes(data)}")

    def _read_exact(self, length, deadline):
        data = bytearray()
        while len(data) < length:
            if time.monotonic() > deadline:
                raise InspireHandError(
                    f"Timed out while reading {length} bytes; "
                    f"got {len(data)}: {self.hex_bytes(data)}"
                )
            chunk = self.ser.read(length - len(data))
            if chunk:
                data.extend(chunk)
        return bytes(data)

    def read_response_frame(self):
        deadline = time.monotonic() + self.response_timeout
        seen = bytearray()

        while True:
            if time.monotonic() > deadline:
                raise InspireHandError(
                    "Timed out waiting for response header 90 EB; "
                    f"saw: {self.hex_bytes(seen)}"
                )

            byte = self.ser.read(1)
            if not byte:
                continue

            seen.extend(byte)
            if len(seen) >= 2 and bytes(seen[-2:]) == RX_HEADER:
                ignored = bytes(seen[:-2])
                if ignored:
                    self._debug("RX ignored", ignored)
                break

            if len(seen) > 64:
                seen = seen[-2:]

        id_and_len = self._read_exact(2, deadline)
        hand_id = id_and_len[0]
        data_len = id_and_len[1]
        payload_and_checksum = self._read_exact(data_len + 1, deadline)
        frame = RX_HEADER + id_and_len + payload_and_checksum
        self._debug("RX frame", frame)

        expected_checksum = self.checksum(frame)
        actual_checksum = frame[-1]
        if actual_checksum != expected_checksum:
            raise InspireHandError(
                f"Bad checksum: expected {expected_checksum:02X}, "
                f"got {actual_checksum:02X}; frame: {self.hex_bytes(frame)}"
            )

        if hand_id != self.id:
            raise InspireHandError(
                f"Response ID {hand_id} does not match requested ID {self.id}; "
                f"frame: {self.hex_bytes(frame)}"
            )

        return frame

    def transact(self, frame):
        self.ser.reset_input_buffer()
        self._debug("TX frame", frame)
        self.ser.write(frame)
        self.ser.flush()
        return self.read_response_frame()

    def write_register_bytes(self, address, payload):
        frame = self.build_frame(CMD_WRITE_REGISTER, address, payload)
        response = self.transact(frame)
        payload = response[4:-1]

        if len(response) != 9:
            raise InspireHandError(
                f"Write ACK should be 9 bytes, got {len(response)}: "
                f"{self.hex_bytes(response)}"
            )
        if payload[:3] != bytes([CMD_WRITE_REGISTER, address & 0xFF, (address >> 8) & 0xFF]):
            raise InspireHandError(
                f"Write ACK address/command mismatch: {self.hex_bytes(response)}"
            )
        if payload[3] != 0x01:
            raise InspireHandError(
                f"Write rejected with status {payload[3]:02X}: {self.hex_bytes(response)}"
            )

        return response

    def write_register_words(self, address, values):
        return self.write_register_bytes(address, self.words_to_bytes(values))

    def read_register_bytes(self, address, byte_length):
        if not 0 <= byte_length <= 255:
            raise ValueError("byte_length must fit in one byte")

        request = self.build_frame(CMD_READ_REGISTER, address, bytes([byte_length]))
        response = self.transact(request)
        payload = response[4:-1]

        expected_len = byte_length + 8
        if len(response) != expected_len:
            raise InspireHandError(
                f"Read response should be {expected_len} bytes, got {len(response)}: "
                f"{self.hex_bytes(response)}"
            )
        if payload[:3] != bytes([CMD_READ_REGISTER, address & 0xFF, (address >> 8) & 0xFF]):
            raise InspireHandError(
                f"Read response address/command mismatch: {self.hex_bytes(response)}"
            )

        return payload[3:]

    def read_register_words(self, address, word_count):
        return self.bytes_to_words(self.read_register_bytes(address, word_count * 2))

    # ── High-level helpers ─────────────────────────────────────────────────

    def set_position_raw(self, values):
        """Command six DOF in raw register units [0, 1000] (RH56 DOF order)."""
        if len(values) != 6:
            raise ValueError("set_position_raw expects six values")
        values = [max(0, min(1000, int(value))) for value in values]
        return self.write_register_words(REG_ANGLE_SET, values)

    def get_position_raw(self):
        """Read six actual angles ANGLE_ACT in raw units [0, 1000]."""
        return self.read_register_words(REG_ANGLE_ACT, 6)

    def get_error_codes(self):
        return list(self.read_register_bytes(REG_ERROR, 6))

    def get_status_codes(self):
        return list(self.read_register_bytes(REG_STATUS, 6))

    def clear_error(self):
        return self.write_register_bytes(REG_CLEAR_ERROR, bytes([1]))
