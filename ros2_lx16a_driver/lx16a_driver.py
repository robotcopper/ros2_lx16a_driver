import serial
import struct

class LX16ADriver:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)

    def send_command(self, servo_id, command, params):
        length = 3 + len(params)
        packet = [0x55, 0x55, servo_id, length, command] + params
        checksum = ~sum(packet[2:]) & 0xFF
        packet.append(checksum)
        self.ser.write(bytearray(packet))

    def move(self, servo_id, position):
        pos = int(position * 1000 / 240)
        params = list(struct.pack('<H', pos))
        self.send_command(servo_id, 1, params)

    def get_physical_angle(self, servo_id):
        self.send_command(servo_id, 28, [])
        response = self.ser.read(7)
        if len(response) == 7:
            angle = struct.unpack('<H', response[5:7])[0]
            return angle * 240 / 1000
        return None

    def close(self):
        self.ser.close()
