from common.common.commands import parse_json_model
from common.common.events import sign_payload, utc_timestamp, verify_signed_payload
from common.common.types import CommandEvent, CommandModel, Position, RoutedCommandEnvelope
from config import config

from .types import EdgeAck, OutstandingCommand, RouteTopics, StatusSummary

TARGET_TYPE_DRONE = 'drone'
TARGET_TYPE_GROUP = 'group'
TARGET_TYPE_BROADCAST = 'broadcast'
TARGET_TYPE_MULTI = 'multi'
VALID_TARGET_TYPES = {
    TARGET_TYPE_DRONE,
    TARGET_TYPE_GROUP,
    TARGET_TYPE_BROADCAST,
    TARGET_TYPE_MULTI,
}

ACK_STATUS_ACCEPTED = 'accepted'
ACK_STATUS_COMPLETED = 'completed'
ACK_STATUS_FAILED = 'failed'
REASON_INVALID_COMMAND = 'invalid_command'
REASON_INVALID_TARGET_TYPE = 'invalid_target_type'
ROUTE_FLIGHT = 'flight'
ROUTE_AUTONOMY = 'autonomy'
ROUTE_VISION = 'vision'
ROUTE_MISSION = 'mission'
ROUTE_SYSTEM = 'system'

DEFAULT_ROUTE_TOPICS = RouteTopics(
    flight='flight/commands',
    autonomy='autonomy/commands',
    vision='vision/commands',
    mission='mission/commands',
    system='system/commands',
)

DEFAULT_COMPLETION_TOPICS = RouteTopics(
    flight='flight/events',
    autonomy='autonomy/events',
    vision='vision/events',
    mission='mission/events',
    system='system/events',
)


class EdgeRoutingService:
    def __init__(
        self,
        drone_id: str,
        groups: list[str],
        route_topics: RouteTopics,
        completion_topics: RouteTopics,
        disconnect_timeout_s: float,
        auth_token: str,
        logger,
        clock_now_ns,
    ):
        self.drone_id = drone_id
        self.groups = groups
        self.route_topics = route_topics
        self.completion_topics = completion_topics
        self.disconnect_timeout_s = disconnect_timeout_s
        self.auth_token = auth_token
        self.logger = logger
        self.clock_now_ns = clock_now_ns

        self.last_contact_monotonic: int | None = None
        self.last_command_id = ''
        self.last_command_type = ''
        self.latest_position = None
        self.latest_battery_voltage = None
        self.latest_mode = 'unknown'
        self.latest_autonomy_state = 'idle'
        self.latest_mission_status = 'idle'
        self.outstanding_commands: list[OutstandingCommand] = []

    def handle_command(
        self,
        raw_value: str,
    ) -> tuple[str | None, RoutedCommandEnvelope | None, EdgeAck | None]:
        command = parse_json_model(
            raw_value,
            CommandModel,
            logger=self.logger,
            log_prefix='fleet command',
        )
        if command is None:
            return None, None, None

        validation_reason = self.validate_command(command)
        if validation_reason:
            ack = self.build_ack(
                command,
                ACK_STATUS_FAILED,
                route_name='unknown',
                reason=validation_reason,
            )
            return None, None, ack

        if not self.matches_target(command):
            return None, None, None

        self.last_contact_monotonic = self.clock_now_ns()
        self.last_command_id = command.command_id
        self.last_command_type = command.command_type

        route_name = self.resolve_route_name(command)
        if route_name is None:
            ack = self.build_ack(
                command,
                ACK_STATUS_FAILED,
                route_name='unknown',
                reason='unknown_command_type',
            )
            return None, None, ack

        routed_command = RoutedCommandEnvelope(
            received_at=utc_timestamp(),
            drone_id=self.drone_id,
            route=route_name,
            route_topic=self.route_topics.topic_for(route_name),
            command=command,
        )
        routed_command = sign_payload(routed_command, self.auth_token)
        self.outstanding_commands.append(OutstandingCommand(
            command_id=command.command_id,
            route=route_name,
            command_type=command.command_type,
        ))
        ack = self.build_ack(command, ACK_STATUS_ACCEPTED, route_name=route_name)
        return route_name, routed_command, ack

    def handle_route_completion(self, route_name: str, raw_value: str) -> EdgeAck | None:
        payload = parse_json_model(
            raw_value,
            CommandEvent,
            logger=self.logger,
            log_prefix=f'{route_name} completion',
        )
        if payload is None:
            return None
        if not verify_signed_payload(payload, self.auth_token):
            self.logger.warn(f'Ignoring unsigned or invalidly signed {route_name} completion')
            return None

        if not payload.command_id:
            self.logger.warn(
                f'Ignoring {route_name} completion without command_id'
            )
            return None
        if payload.drone_id != self.drone_id:
            self.logger.warn(
                f'Ignoring {route_name} completion for drone_id={payload.drone_id}'
            )
            return None

        outstanding = self.find_outstanding_command(payload.command_id)
        if outstanding is None:
            self.logger.warn(
                f'Ignoring unexpected {route_name} completion for command_id={payload.command_id}'
            )
            return None
        if outstanding.route != route_name or outstanding.command_type != payload.command_type:
            self.logger.warn(
                f'Ignoring mismatched {route_name} completion for command_id={payload.command_id}'
            )
            return None

        status = payload.status.lower()
        if status not in {ACK_STATUS_COMPLETED, ACK_STATUS_FAILED}:
            status = ACK_STATUS_COMPLETED
        self.remove_outstanding_command(payload.command_id)

        return EdgeAck(
            ack_at=utc_timestamp(),
            drone_id=self.drone_id,
            command_id=payload.command_id,
            command_type=payload.command_type,
            route=route_name,
            status=status,
            source_topic=self.completion_topics.topic_for(route_name),
            details=payload,
            coordinator_link=self.coordinator_link_state(),
        )

    def handle_flight_gps(self, latitude: float, longitude: float, altitude: float):
        self.latest_position = Position(
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
        )

    def handle_battery(self, voltage: float):
        self.latest_battery_voltage = voltage

    def handle_mode(self, mode: str):
        self.latest_mode = mode or 'unknown'

    def handle_autonomy_state(self, value: str):
        self.latest_autonomy_state = value or 'idle'

    def handle_mission_status(self, value: str):
        self.latest_mission_status = value or 'idle'

    def build_status_summary(self) -> StatusSummary:
        return StatusSummary(
            timestamp=utc_timestamp(),
            drone_id=self.drone_id,
            groups=self.groups,
            coordinator_link=self.coordinator_link_state(),
            last_command_id=self.last_command_id,
            last_command_type=self.last_command_type,
            mode=self.latest_mode,
            autonomy_state=self.latest_autonomy_state,
            mission_status=self.latest_mission_status,
            battery_voltage=self.latest_battery_voltage,
            position=self.latest_position,
            route_topics=self.route_topics,
        )

    def coordinator_link_state(self) -> str:
        if self.last_contact_monotonic is None:
            return 'unknown'

        elapsed_s = (self.clock_now_ns() - self.last_contact_monotonic) / 1_000_000_000
        if elapsed_s <= self.disconnect_timeout_s:
            return 'connected'
        return 'stale'

    def build_ack(
        self,
        command: CommandModel,
        status: str,
        route_name: str,
        reason: str = '',
    ) -> EdgeAck:
        return EdgeAck(
            ack_at=utc_timestamp(),
            drone_id=self.drone_id,
            command_id=command.command_id,
            command_type=command.command_type,
            route=route_name,
            status=status,
            coordinator_link=self.coordinator_link_state(),
            reason=reason or None,
        )

    def matches_target(self, command: CommandModel) -> bool:
        target_type = self.command_target_type(command)
        target = command.target

        if target_type == TARGET_TYPE_BROADCAST:
            return True
        if target_type == TARGET_TYPE_DRONE:
            return isinstance(target, str) and target == self.drone_id
        if target_type == TARGET_TYPE_GROUP:
            return isinstance(target, str) and target in self.groups
        if target_type == TARGET_TYPE_MULTI:
            return self.drone_id in command.targets
        return False

    def command_target_type(self, command: CommandModel) -> str | None:
        value = command.target_type
        if value in VALID_TARGET_TYPES:
            return value
        return None

    def resolve_route_name(self, command: CommandModel) -> str | None:
        command_type = command.command_type
        if command_type in {
            'arm',
            'disarm',
            'takeoff',
            'land',
            'rtl',
            'hold',
            'set_mode',
            'goto_waypoint',
            'ping',
            'health_check',
        }:
            return ROUTE_FLIGHT
        if command_type in {
            'start_mission',
            'pause_mission',
            'resume_mission',
            'abort_mission',
        }:
            return ROUTE_MISSION
        if command_type == 'set_autonomy_state':
            return ROUTE_AUTONOMY
        if command_type in {'start_scan', 'stop_scan'}:
            return ROUTE_VISION
        return None

    def validate_command(self, command: CommandModel) -> str:
        if not command.command_id:
            return REASON_INVALID_COMMAND
        if not command.command_type or command.command_type == 'unknown':
            return REASON_INVALID_COMMAND
        if self.command_target_type(command) is None:
            return REASON_INVALID_TARGET_TYPE
        return ''

    def find_outstanding_command(self, command_id: str) -> OutstandingCommand | None:
        for outstanding_command in self.outstanding_commands:
            if outstanding_command.command_id == command_id:
                return outstanding_command
        return None

    def remove_outstanding_command(self, command_id: str):
        self.outstanding_commands = [
            outstanding_command
            for outstanding_command in self.outstanding_commands
            if outstanding_command.command_id != command_id
        ]


def load_route_topics() -> RouteTopics:
    return RouteTopics(
        flight=config.flight_command_topic,
        autonomy=config.autonomy_command_topic,
        vision=config.vision_command_topic,
        mission=config.mission_command_topic,
        system=config.system_command_topic,
    )


def load_completion_topics() -> RouteTopics:
    return RouteTopics(
        flight=config.flight_event_topic,
        autonomy=config.autonomy_event_topic,
        vision=config.vision_event_topic,
        mission=config.mission_event_topic,
        system=config.system_event_topic,
    )
