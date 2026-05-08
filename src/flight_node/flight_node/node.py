import traceback

from geometry_msgs.msg import Vector3
from common.common.publishers import JsonPublisher
from common.common.types import CommandEvent
from config import config
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
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
from .types import PendingModeCommand


class FlightNode(Node):
    def __init__(self):
        super().__init__('flight_node')
        self.drone_id = config.drone.AETHER_DRONE_ID
        self.flight_topic_prefix = config.topic('flight')

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
        self.event_publisher_ = JsonPublisher(
            self.create_publisher(
                String,
                f'{self.flight_topic_prefix}/events',
                10,
            )
        )
        self.command_subscription_ = self.create_subscription(
            String,
            f'{self.flight_topic_prefix}/commands',
            self.handle_command,
            10,
        )

        self.mavlink = MavlinkAdapter(
            config.flight.AETHER_FLIGHT_PORT,
            config.flight.AETHER_FLIGHT_BAUD_RATE,
            self.get_logger(),
        )
        self.mavlink.connect()
        self.command_service = FlightCommandService(
            drone_id=self.drone_id,
            mavlink_adapter=self.mavlink,
            logger=self.get_logger(),
            auth_token=config.require_auth_token('flight_node'),
        )
        self.pending_mode_command: PendingModeCommand | None = None
        self.mode_confirm_timeout_ns = int(
            config.flight.AETHER_FLIGHT_MODE_CONFIRM_TIMEOUT_S * 1_000_000_000
        )
        self.timer_ = self.create_timer(0.2, self.update)
        self.get_logger().info(
            f'Flight node started for {self.drone_id} on {self.flight_topic_prefix}'
        )

    def handle_command(self, msg: String):
        event, pending_mode_command = self.command_service.handle_routed_command(msg.data)
        if pending_mode_command is not None:
            pending_mode_command.requested_at_ns = self.get_clock().now().nanoseconds
            self.pending_mode_command = pending_mode_command
        if event is None:
            return

        self.publish_event(event)

    def publish_event(self, event: CommandEvent):
        self.event_publisher_.publish(event)

    def check_pending_mode_timeout(self):
        if self.pending_mode_command is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.pending_mode_command.requested_at_ns < self.mode_confirm_timeout_ns:
            return

        event = self.command_service.mode_timeout_event(self.pending_mode_command)
        self.pending_mode_command = None
        self.publish_event(event)

    def update(self):
        self.check_pending_mode_timeout()
        mavlink_message = self.mavlink.receive_message()
        if mavlink_message is None:
            return

        message_type = 'unknown'
        try:
            message_type = mavlink_message.get_type()
            if message_type == 'HEARTBEAT':
                self._handle_heartbeat(mavlink_message)
            else:
                self._publish_telemetry(message_type, mavlink_message)
        except Exception as exc:
            self.get_logger().error(
                f'Error handling MAVLink message type={message_type}: {exc}'
            )
            self.get_logger().error(traceback.format_exc())

    def _handle_heartbeat(self, mavlink_message):
        observed_mode = self.mavlink.mode_string(mavlink_message)
        self.mode_publisher_.publish(mode_message(observed_mode))
        if self.pending_mode_command is None:
            return

        event = self.command_service.confirm_mode_change(
            self.pending_mode_command,
            observed_mode,
        )
        if event is not None:
            self.pending_mode_command = None
            self.publish_event(event)

    def _publish_telemetry(self, message_type: str, mavlink_message):
        if message_type == 'ATTITUDE':
            self.attitude_publisher_.publish(attitude_message(mavlink_message))
        elif message_type == 'GLOBAL_POSITION_INT':
            self.gps_publisher_.publish(gps_message(mavlink_message))
        elif message_type == 'GPS_RAW_INT':
            self.gps_status_publisher_.publish(gps_status_message(mavlink_message))
        elif message_type == 'SYS_STATUS':
            self.battery_publisher_.publish(battery_message(mavlink_message))
