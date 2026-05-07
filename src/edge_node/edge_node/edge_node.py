import json
import os
from datetime import datetime, timezone
from typing import Optional

from config import get_config
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import String


ENV_FLEET_COMMANDS_TOPIC = 'AETHER_FLEET_COMMANDS_TOPIC'
ENV_FLEET_ACKS_TOPIC = 'AETHER_FLEET_ACKS_TOPIC'
ENV_FLEET_TELEMETRY_TOPIC = 'AETHER_FLEET_TELEMETRY_TOPIC'

TARGET_TYPE_DRONE = 'drone'
TARGET_TYPE_GROUP = 'group'
TARGET_TYPE_BROADCAST = 'broadcast'
TARGET_TYPE_MULTI = 'multi'

ACK_STATUS_ACCEPTED = 'accepted'
ACK_STATUS_IGNORED = 'ignored'
ACK_STATUS_COMPLETED = 'completed'
ACK_STATUS_FAILED = 'failed'

ROUTE_FLIGHT = 'flight'
ROUTE_AUTONOMY = 'autonomy'
ROUTE_VISION = 'vision'
ROUTE_MISSION = 'mission'
ROUTE_SYSTEM = 'system'

DEFAULT_COMMAND_ROUTES = {
    'arm': ROUTE_FLIGHT,
    'disarm': ROUTE_FLIGHT,
    'takeoff': ROUTE_FLIGHT,
    'land': ROUTE_FLIGHT,
    'rtl': ROUTE_FLIGHT,
    'hold': ROUTE_FLIGHT,
    'set_mode': ROUTE_FLIGHT,
    'goto_waypoint': ROUTE_FLIGHT,
    'ping': ROUTE_SYSTEM,
    'health_check': ROUTE_SYSTEM,
    'start_mission': ROUTE_MISSION,
    'pause_mission': ROUTE_MISSION,
    'resume_mission': ROUTE_MISSION,
    'abort_mission': ROUTE_MISSION,
    'set_autonomy_state': ROUTE_AUTONOMY,
    'start_scan': ROUTE_VISION,
    'stop_scan': ROUTE_VISION,
}

DEFAULT_ROUTE_TOPICS = {
    ROUTE_FLIGHT: 'flight/commands',
    ROUTE_AUTONOMY: 'autonomy/commands',
    ROUTE_VISION: 'vision/commands',
    ROUTE_MISSION: 'mission/commands',
    ROUTE_SYSTEM: 'system/commands',
}

DEFAULT_COMPLETION_TOPICS = {
    ROUTE_FLIGHT: 'flight/events',
    ROUTE_AUTONOMY: 'autonomy/events',
    ROUTE_VISION: 'vision/events',
    ROUTE_MISSION: 'mission/events',
    ROUTE_SYSTEM: 'system/events',
}


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_csv_env(value: str) -> list[str]:
    return [item.strip() for item in value.split(',') if item.strip()]


def json_or_default(value: str, default: dict) -> dict:
    if not value:
        return dict(default)
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError:
        return dict(default)
    if not isinstance(parsed, dict):
        return dict(default)
    return {str(key): str(parsed[key]) for key in parsed}


class EdgeNode(Node):
    def __init__(self):
        super().__init__('edge_node')
        config = get_config()
        self.drone_id = self.declare_parameter(
            'drone_id',
            config.drone.AETHER_DRONE_ID,
        ).value
        self.groups = list(
            self.declare_parameter(
                'groups',
                config.drone.AETHER_DRONE_GROUPS,
            ).value
        )
        self.disconnect_timeout_s = float(
            self.declare_parameter(
                'disconnect_timeout_s',
                config.edge.AETHER_EDGE_DISCONNECT_TIMEOUT_S,
            ).value
        )
        self.status_period_s = float(
            self.declare_parameter(
                'status_period_s',
                config.edge.AETHER_EDGE_STATUS_PERIOD_S,
            ).value
        )

        self.fleet_commands_topic = self.declare_parameter(
            'fleet_commands_topic',
            config.fleet.AETHER_FLEET_COMMANDS_TOPIC,
        ).value
        self.fleet_acks_topic = self.declare_parameter(
            'fleet_acks_topic',
            config.fleet.AETHER_FLEET_ACKS_TOPIC,
        ).value
        self.fleet_telemetry_topic = self.declare_parameter(
            'fleet_telemetry_topic',
            config.fleet.AETHER_FLEET_TELEMETRY_TOPIC,
        ).value

        self.command_route_map = DEFAULT_COMMAND_ROUTES
        self.route_topics = self.load_route_topics()
        self.completion_topics = self.load_completion_topics()

        self.flight_gps_topic = config.topic('flight/gps')
        self.flight_battery_topic = config.topic('flight/battery')
        self.flight_mode_topic = config.topic('flight/mode')
        self.autonomy_state_topic = config.topic('autonomy/state')
        self.mission_status_topic = config.topic('mission/status')
        self.edge_ack_topic = config.topic('edge/acks')

        self.route_publishers = {
            route_name: self.create_publisher(String, topic_name, 10)
            for route_name, topic_name in self.route_topics.items()
        }
        self.edge_ack_publisher = self.create_publisher(String, self.edge_ack_topic, 10)
        self.fleet_ack_publisher = self.create_publisher(String, self.fleet_acks_topic, 10)
        self.fleet_telemetry_publisher = self.create_publisher(
            String,
            self.fleet_telemetry_topic,
            10,
        )

        self.create_subscription(
            String,
            self.fleet_commands_topic,
            self.handle_command,
            10,
        )
        self.create_subscription(NavSatFix, self.flight_gps_topic, self.handle_flight_gps, 10)
        self.create_subscription(BatteryState, self.flight_battery_topic, self.handle_battery, 10)
        self.create_subscription(String, self.flight_mode_topic, self.handle_mode, 10)
        self.create_subscription(String, self.autonomy_state_topic, self.handle_autonomy_state, 10)
        self.create_subscription(String, self.mission_status_topic, self.handle_mission_status, 10)

        self.completion_subscriptions = []
        for route_name, topic_name in self.completion_topics.items():
            subscription = self.create_subscription(
                String,
                topic_name,
                lambda msg, route=route_name: self.handle_route_completion(route, msg),
                10,
            )
            self.completion_subscriptions.append(subscription)

        self.last_contact_monotonic: Optional[int] = None
        self.last_command_id = ''
        self.last_command_type = ''
        self.latest_position = None
        self.latest_battery_voltage = None
        self.latest_mode = 'unknown'
        self.latest_autonomy_state = 'idle'
        self.latest_mission_status = 'idle'

        self.status_timer = self.create_timer(self.status_period_s, self.publish_status_summary)
        self.get_logger().info(
            f'Edge node started for {self.drone_id} with routes {self.route_topics}'
        )

    def load_route_topics(self) -> dict[str, str]:
        config = get_config()
        topics = {}
        for route_name, route_suffix in DEFAULT_ROUTE_TOPICS.items():
            env_name = f'AETHER_{route_name.upper()}_COMMAND_TOPIC'
            default_topic = config.topic(route_suffix)
            topics[route_name] = self.declare_parameter(
                f'{route_name}_command_topic',
                os.getenv(env_name, default_topic),
            ).value
        return topics

    def load_completion_topics(self) -> dict[str, str]:
        config = get_config()
        topics = {}
        for route_name, route_suffix in DEFAULT_COMPLETION_TOPICS.items():
            env_name = f'AETHER_{route_name.upper()}_EVENT_TOPIC'
            default_topic = config.topic(route_suffix)
            topics[route_name] = self.declare_parameter(
                f'{route_name}_event_topic',
                os.getenv(env_name, default_topic),
            ).value
        return topics

    def handle_flight_gps(self, msg: NavSatFix):
        self.latest_position = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
        }

    def handle_battery(self, msg: BatteryState):
        self.latest_battery_voltage = msg.voltage

    def handle_mode(self, msg: String):
        self.latest_mode = msg.data or 'unknown'

    def handle_autonomy_state(self, msg: String):
        self.latest_autonomy_state = msg.data or 'idle'

    def handle_mission_status(self, msg: String):
        self.latest_mission_status = msg.data or 'idle'

    def handle_command(self, msg: String):
        command = self.parse_json_message(msg.data, log_prefix='fleet command')
        if command is None:
            return

        if not self.matches_target(command):
            return

        self.last_contact_monotonic = self.get_clock().now().nanoseconds
        self.last_command_id = self.command_id(command)
        self.last_command_type = self.command_type(command)

        route_name = self.resolve_route_name(command)
        if route_name is None:
            ack = self.build_ack(command, ACK_STATUS_FAILED, route_name='unknown', reason='unknown_command_type')
            self.publish_ack(ack)
            return

        self.apply_command_state(command)
        routed_command = {
            'received_at': utc_timestamp(),
            'drone_id': self.drone_id,
            'route': route_name,
            'route_topic': self.route_topics[route_name],
            'command': command,
        }
        self.publish_json(self.route_publishers[route_name], routed_command)

        ack = self.build_ack(command, ACK_STATUS_ACCEPTED, route_name=route_name)
        self.publish_ack(ack)

    def handle_route_completion(self, route_name: str, msg: String):
        payload = self.parse_json_message(msg.data, log_prefix=f'{route_name} completion')
        if payload is None:
            return

        command_id = str(payload.get('command_id', ''))
        status = str(payload.get('status', ACK_STATUS_COMPLETED)).lower()
        if status not in {ACK_STATUS_COMPLETED, ACK_STATUS_FAILED}:
            status = ACK_STATUS_COMPLETED

        ack = {
            'ack_at': utc_timestamp(),
            'drone_id': self.drone_id,
            'command_id': command_id,
            'command_type': str(payload.get('command_type', 'unknown')),
            'route': route_name,
            'status': status,
            'source_topic': self.completion_topics[route_name],
            'details': payload,
            'coordinator_link': self.coordinator_link_state(),
        }
        self.publish_ack(ack)

    def matches_target(self, command: dict) -> bool:
        target_type = self.command_target_type(command)
        target = command.get('target')

        if target_type == TARGET_TYPE_BROADCAST:
            return True

        if target_type == TARGET_TYPE_DRONE:
            return isinstance(target, str) and target == self.drone_id

        if target_type == TARGET_TYPE_GROUP:
            return isinstance(target, str) and target in self.groups

        if target_type == TARGET_TYPE_MULTI:
            targets = command.get('targets', [])
            return isinstance(targets, list) and self.drone_id in targets

        return False

    def command_target_type(self, command: dict) -> str:
        value = str(command.get('target_type', TARGET_TYPE_DRONE)).lower()
        if value in {
            TARGET_TYPE_DRONE,
            TARGET_TYPE_GROUP,
            TARGET_TYPE_BROADCAST,
            TARGET_TYPE_MULTI,
        }:
            return value
        return TARGET_TYPE_DRONE

    def resolve_route_name(self, command: dict) -> Optional[str]:
        command_type = self.command_type(command)
        route_name = self.command_route_map.get(command_type)
        if route_name not in self.route_publishers:
            return None
        return route_name

    def command_id(self, command: dict) -> str:
        return str(command.get('command_id', ''))

    def command_type(self, command: dict) -> str:
        return str(command.get('command_type', 'unknown')).lower()

    def apply_command_state(self, command: dict):
        command_type = self.command_type(command)
        if command_type == 'start_mission':
            self.latest_mission_status = 'active'
        elif command_type == 'pause_mission':
            self.latest_mission_status = 'paused'
        elif command_type == 'resume_mission':
            self.latest_mission_status = 'active'
        elif command_type in {'hold', 'rtl', 'land', 'abort_mission'}:
            self.latest_mission_status = command_type

    def publish_status_summary(self):
        summary = {
            'timestamp': utc_timestamp(),
            'drone_id': self.drone_id,
            'groups': self.groups,
            'coordinator_link': self.coordinator_link_state(),
            'last_command_id': self.last_command_id,
            'last_command_type': self.last_command_type,
            'mode': self.latest_mode,
            'autonomy_state': self.latest_autonomy_state,
            'mission_status': self.latest_mission_status,
            'battery_voltage': self.latest_battery_voltage,
            'position': self.latest_position,
            'route_topics': self.route_topics,
        }
        self.publish_json(self.fleet_telemetry_publisher, summary)

    def coordinator_link_state(self) -> str:
        if self.last_contact_monotonic is None:
            return 'unknown'

        now_ns = self.get_clock().now().nanoseconds
        elapsed_s = (now_ns - self.last_contact_monotonic) / 1_000_000_000
        if elapsed_s <= self.disconnect_timeout_s:
            return 'connected'
        return 'stale'

    def build_ack(
        self,
        command: dict,
        status: str,
        route_name: str,
        reason: str = '',
    ) -> dict:
        ack = {
            'ack_at': utc_timestamp(),
            'drone_id': self.drone_id,
            'command_id': self.command_id(command),
            'command_type': self.command_type(command),
            'route': route_name,
            'status': status,
            'coordinator_link': self.coordinator_link_state(),
        }
        if reason:
            ack['reason'] = reason
        return ack

    def publish_ack(self, payload: dict):
        self.publish_json(self.edge_ack_publisher, payload)
        self.publish_json(self.fleet_ack_publisher, payload)

    def parse_json_message(self, raw_value: str, log_prefix: str) -> Optional[dict]:
        try:
            payload = json.loads(raw_value)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring malformed {log_prefix}: {exc}')
            return None
        if not isinstance(payload, dict):
            self.get_logger().warn(f'Ignoring malformed {log_prefix}: expected object')
            return None
        return payload

    def publish_json(self, publisher, payload: dict):
        msg = String()
        msg.data = json.dumps(payload)
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EdgeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
