import os
import termios

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


def nmea_to_decimal(coord: str, direction: str) -> float:
    if not coord:
        return 0.0

    dot = coord.find('.')
    if dot == -1 or dot < 2:
        return 0.0

    degrees = float(coord[: dot - 2])
    minutes = float(coord[dot - 2 :])
    decimal = degrees + minutes / 60.0
    if direction in {'S', 'W'}:
        decimal = -decimal
    return decimal


def strip_checksum(value: str) -> str:
    star = value.find('*')
    return value[:star] if star != -1 else value


def sentence_type(identifier: str) -> str:
    stripped = strip_checksum(identifier)
    if stripped.startswith('$'):
        stripped = stripped[1:]
    if len(stripped) < 3:
        return 'UNKNOWN'
    return stripped[-3:]


class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.drone_id = self.declare_parameter(
            'drone_id',
            os.getenv('AETHER_DRONE_ID', 'AE-01'),
        ).value
        self.port = self.declare_parameter(
            'gps_port',
            os.getenv('GPS_PORT', '/dev/ttyUSB0'),
        ).value
        self.baud_rate = int(
            self.declare_parameter(
                'baud_rate',
                int(os.getenv('AETHER_GPS_BAUD_RATE', '9600')),
            ).value
        )
        self.fix_topic = self.declare_parameter(
            'fix_topic',
            f'/{self.drone_id}/gps/fix',
        ).value
        self.status_topic = self.declare_parameter(
            'status_topic',
            f'/{self.drone_id}/gps/status',
        ).value

        self.fix_publisher = self.create_publisher(NavSatFix, self.fix_topic, 10)
        self.status_publisher = self.create_publisher(
            NavSatStatus,
            self.status_topic,
            10,
        )

        self.serial_file = None
        self.connect_serial()
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info(
            f'GPS node started for {self.drone_id} on {self.port}'
        )

    def connect_serial(self):
        try:
            raw = open(self.port, 'rb', buffering=0)
            attrs = termios.tcgetattr(raw.fileno())
            attrs[0] &= ~(
                termios.IXON
                | termios.IXOFF
                | termios.IXANY
                | termios.IGNBRK
                | termios.BRKINT
                | termios.PARMRK
                | termios.ISTRIP
                | termios.INLCR
                | termios.IGNCR
                | termios.ICRNL
            )
            attrs[1] &= ~termios.OPOST
            attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE | termios.CRTSCTS)
            attrs[2] |= termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
            attrs[6][termios.VMIN] = 1
            attrs[6][termios.VTIME] = 0

            speed_attr = self.baud_constant(self.baud_rate)
            termios.cfsetispeed(attrs, speed_attr)
            termios.cfsetospeed(attrs, speed_attr)
            termios.tcsetattr(raw.fileno(), termios.TCSANOW, attrs)
            self.serial_file = raw
        except Exception as exc:
            self.serial_file = None
            self.get_logger().error(f'Failed to open GPS serial port {self.port}: {exc}')

    def baud_constant(self, baud_rate: int):
        mapping = {
            4800: termios.B4800,
            9600: termios.B9600,
            19200: termios.B19200,
            38400: termios.B38400,
            57600: termios.B57600,
            115200: termios.B115200,
        }
        return mapping.get(baud_rate, termios.B9600)

    def loop(self):
        if self.serial_file is None:
            return

        try:
            line = self.serial_file.readline().decode('ascii', errors='ignore').strip()
        except Exception as exc:
            self.get_logger().error(f'GPS read error: {exc}')
            return

        if not line or not line.startswith('$'):
            return

        fields = line.split(',')
        if not fields:
            return

        message_type = sentence_type(fields[0])
        if message_type == 'GGA':
            self.publish_gga(fields)
        elif message_type == 'RMC':
            self.publish_rmc(fields)
        elif message_type == 'GSA':
            self.publish_gsa(fields)

    def publish_rmc(self, fields):
        status = fields[2] if len(fields) > 2 else ''
        if status != 'A':
            return

        msg = NavSatFix()
        msg.latitude = nmea_to_decimal(fields[3] if len(fields) > 3 else '', fields[4] if len(fields) > 4 else '')
        msg.longitude = nmea_to_decimal(fields[5] if len(fields) > 5 else '', fields[6] if len(fields) > 6 else '')
        msg.status.status = NavSatStatus.STATUS_FIX
        self.fix_publisher.publish(msg)

    def publish_gga(self, fields):
        fix = fields[6] if len(fields) > 6 else '0'
        if fix == '0':
            return

        msg = NavSatFix()
        msg.latitude = nmea_to_decimal(fields[2] if len(fields) > 2 else '', fields[3] if len(fields) > 3 else '')
        msg.longitude = nmea_to_decimal(fields[4] if len(fields) > 4 else '', fields[5] if len(fields) > 5 else '')
        msg.altitude = float(fields[9]) if len(fields) > 9 and fields[9] else 0.0
        msg.status.status = NavSatStatus.STATUS_FIX
        self.fix_publisher.publish(msg)

    def publish_gsa(self, fields):
        fix_mode = fields[2] if len(fields) > 2 else '1'
        msg = NavSatStatus()
        msg.status = (
            NavSatStatus.STATUS_NO_FIX
            if fix_mode == '1'
            else NavSatStatus.STATUS_FIX
        )
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
