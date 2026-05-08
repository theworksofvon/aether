from config import get_config
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from common import dumps_json
from std_msgs.msg import String

from .adapters import MavlinkAdapter
from .services import FlightCommandService
from .telemetry import (
    attitude_message,
    battery_message,
    gps_message,
    gps_status_message,
    mode_message,
)


class FlightNode(Node):
    def __init__(self):
        super().__init__('flight_node')
        config = get_config()
        self.drone_id = self.declare_parameter(
            'drone_id',
            config.drone.AETHER_DRONE_ID,
        ).value
        serial_port = self.declare_parameter(
            'serial_port',
            config.flight.AETHER_FLIGHT_PORT,
        ).value
        baud_rate = int(
            self.declare_parameter(
                'baud_rate',
                config.flight.AETHER_FLIGHT_BAUD_RATE,
            ).value
        )
        self.flight_topic_prefix = self.declare_parameter(
            'flight_topic_prefix',
            config.topic('flight'),
        ).value

        self.gps_publisher_ = self.create_publisher(
            NavSatFix,
            f'{self.flight_topic_prefix}/gps',
            10,
        )
        self.gps_status_publisher_ = self.create_publisher(
            NavSatStatus,
            f'{self.flight_topic_prefix}/gps_status',
            10,
        )
        self.attitude_publisher_ = self.create_publisher(
            Vector3,
            f'{self.flight_topic_prefix}/attitude',
            10,
        )
        self.battery_publisher_ = self.create_publisher(
            BatteryState,
            f'{self.flight_topic_prefix}/battery',
            10,
        )
        self.mode_publisher_ = self.create_publisher(
            String,
            f'{self.flight_topic_prefix}/mode',
            10,
        )
        self.event_publisher_ = self.create_publisher(
            String,
            f'{self.flight_topic_prefix}/events',
            10,
        )
        self.command_subscription_ = self.create_subscription(
            String,
            f'{self.flight_topic_prefix}/commands',
            self.handle_command,
            10,
        )

        self.mavlink = MavlinkAdapter(serial_port, baud_rate, self.get_logger())
        self.mavlink.connect()
        self.command_service = FlightCommandService(
            drone_id=self.drone_id,
            mavlink_adapter=self.mavlink,
            logger=self.get_logger(),
        )
        self.timer_ = self.create_timer(0.2, self.update)
        self.get_logger().info(
            f'Flight node started for {self.drone_id} on {self.flight_topic_prefix}'
        )

    def handle_command(self, msg: String):
        event = self.command_service.handle_routed_command(msg.data)
        if event is None:
            return

        event_message = String()
        event_message.data = dumps_json(event)
        self.event_publisher_.publish(event_message)

    def update(self):
        mavlink_message = self.mavlink.receive_message()
        if mavlink_message is None:
            return

        try:
            message_type = mavlink_message.get_type()
            if message_type == 'ATTITUDE':
                self.attitude_publisher_.publish(attitude_message(mavlink_message))
            elif message_type == 'GLOBAL_POSITION_INT':
                self.gps_publisher_.publish(gps_message(mavlink_message))
            elif message_type == 'GPS_RAW_INT':
                self.gps_status_publisher_.publish(gps_status_message(mavlink_message))
            elif message_type == 'SYS_STATUS':
                self.battery_publisher_.publish(battery_message(mavlink_message))
            elif message_type == 'HEARTBEAT':
                self.mode_publisher_.publish(
                    mode_message(self.mavlink.mode_string(mavlink_message))
                )
        except Exception as exc:
            self.get_logger().error(f'Error reading message: {exc}')
