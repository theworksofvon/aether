import os
from typing import Optional

from common import dumps_json, parse_json_object, utc_timestamp
from config import get_config

TARGET_TYPE_DRONE = 'drone'
TARGET_TYPE_GROUP = 'group'
TARGET_TYPE_BROADCAST = 'broadcast'
TARGET_TYPE_MULTI = 'multi'

ACK_STATUS_ACCEPTED = 'accepted'
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
    'ping': ROUTE_FLIGHT,
    'health_check': ROUTE_FLIGHT,
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


class EdgeRoutingService:
    def __init__(
        self,
        drone_id: str,
        groups: list[str],
        route_topics: dict[str, str],
        completion_topics: dict[str, str],
        disconnect_timeout_s: float,
        logger,
        clock_now_ns,
    ):
        self.drone_id = drone_id
        self.groups = groups
        self.route_topics = route_topics
        self.completion_topics = completion_topics
        self.disconnect_timeout_s = disconnect_timeout_s
        self.logger = logger
        self.clock_now_ns = clock_now_ns
        self.command_route_map = dict(DEFAULT_COMMAND_ROUTES)

        self.last_contact_monotonic: Optional[int] = None
        self.last_command_id = ''
        self.last_command_type = ''
        self.latest_position = None
        self.latest_battery_voltage = None
        self.latest_mode = 'unknown'
        self.latest_autonomy_state = 'idle'
        self.latest_mission_status = 'idle'

    def handle_command(self, raw_value: str) -> tuple[str | None, dict | None, dict | None]:
        command = parse_json_object(raw_value, logger=self.logger, log_prefix='fleet command')
        if command is None:
            return None, None, None

        if not self.matches_target(command):
            return None, None, None

        self.last_contact_monotonic = self.clock_now_ns()
        self.last_command_id = self.command_id(command)
        self.last_command_type = self.command_type(command)

        route_name = self.resolve_route_name(command)
        if route_name is None:
            ack = self.build_ack(
                command,
                ACK_STATUS_FAILED,
                route_name='unknown',
                reason='unknown_command_type',
            )
            return None, None, ack

        self.apply_command_state(command)
        routed_command = {
            'received_at': utc_timestamp(),
            'drone_id': self.drone_id,
            'route': route_name,
            'route_topic': self.route_topics[route_name],
            'command': command,
        }
        ack = self.build_ack(command, ACK_STATUS_ACCEPTED, route_name=route_name)
        return route_name, routed_command, ack

    def handle_route_completion(self, route_name: str, raw_value: str) -> dict | None:
        payload = parse_json_object(
            raw_value,
            logger=self.logger,
            log_prefix=f'{route_name} completion',
        )
        if payload is None:
            return None

        command_id = str(payload.get('command_id', ''))
        status = str(payload.get('status', ACK_STATUS_COMPLETED)).lower()
        if status not in {ACK_STATUS_COMPLETED, ACK_STATUS_FAILED}:
            status = ACK_STATUS_COMPLETED

        return {
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

    def handle_flight_gps(self, latitude: float, longitude: float, altitude: float):
        self.latest_position = {
            'latitude': latitude,
            'longitude': longitude,
            'altitude': altitude,
        }

    def handle_battery(self, voltage: float):
        self.latest_battery_voltage = voltage

    def handle_mode(self, mode: str):
        self.latest_mode = mode or 'unknown'

    def handle_autonomy_state(self, value: str):
        self.latest_autonomy_state = value or 'idle'

    def handle_mission_status(self, value: str):
        self.latest_mission_status = value or 'idle'

    def build_status_summary(self) -> dict:
        return {
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

    def coordinator_link_state(self) -> str:
        if self.last_contact_monotonic is None:
            return 'unknown'

        elapsed_s = (self.clock_now_ns() - self.last_contact_monotonic) / 1_000_000_000
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

    def publish_json(self, publisher, payload: dict):
        from std_msgs.msg import String

        message = String()
        message.data = dumps_json(payload)
        publisher.publish(message)

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

    def resolve_route_name(self, command: dict) -> str | None:
        route_name = self.command_route_map.get(self.command_type(command))
        if route_name not in self.route_topics:
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


def load_route_topics(node) -> dict[str, str]:
    config = get_config()
    topics = {}
    for route_name, route_suffix in DEFAULT_ROUTE_TOPICS.items():
        env_name = f'AETHER_{route_name.upper()}_COMMAND_TOPIC'
        default_topic = config.topic(route_suffix)
        topics[route_name] = node.declare_parameter(
            f'{route_name}_command_topic',
            os.getenv(env_name, default_topic),
        ).value
    return topics


def load_completion_topics(node) -> dict[str, str]:
    config = get_config()
    topics = {}
    for route_name, route_suffix in DEFAULT_COMPLETION_TOPICS.items():
        env_name = f'AETHER_{route_name.upper()}_EVENT_TOPIC'
        default_topic = config.topic(route_suffix)
        topics[route_name] = node.declare_parameter(
            f'{route_name}_event_topic',
            os.getenv(env_name, default_topic),
        ).value
    return topics
