from common import (
    REASON_INVALID_COMMAND,
    REASON_MAVLINK_SEND_FAILED,
    REASON_NO_CONNECTION,
    REASON_UNSUPPORTED_COMMAND,
    REASON_UNKNOWN_MODE,
    STATUS_COMPLETED,
    STATUS_FAILED,
    extract_nested_command,
    parse_json_object,
)

from .events import flight_event

UNSUPPORTED_COMMANDS = {
    'arm',
    'disarm',
    'takeoff',
    'land',
    'goto_waypoint',
}

MODE_ALIASES = {
    'hold': 'LOITER',
    'rtl': 'RTL',
}


class FlightCommandService:
    def __init__(self, drone_id: str, mavlink_adapter, logger):
        self.drone_id = drone_id
        self.mavlink_adapter = mavlink_adapter
        self.logger = logger

    def handle_routed_command(self, raw_value: str) -> dict | None:
        envelope = parse_json_object(
            raw_value,
            logger=self.logger,
            log_prefix='flight command',
        )
        if envelope is None:
            return None

        command = extract_nested_command(
            envelope,
            logger=self.logger,
            log_prefix='flight command',
        )
        if command is None:
            return flight_event(
                drone_id=self.drone_id,
                command_id='',
                command_type='unknown',
                status=STATUS_FAILED,
                reason=REASON_INVALID_COMMAND,
                details=self.mavlink_adapter.details(),
            )

        command_id = str(command.get('command_id', ''))
        command_type = str(command.get('command_type', 'unknown')).lower()

        if command_type in {'ping', 'health_check'}:
            return self._connection_result(command_id, command_type)

        if command_type in MODE_ALIASES:
            return self._set_mode(command_id, command_type, MODE_ALIASES[command_type])

        if command_type == 'set_mode':
            requested_mode = self._requested_mode(command.get('payload', {}))
            return self._set_mode(command_id, command_type, requested_mode)

        if command_type in UNSUPPORTED_COMMANDS:
            return self._failed_event(
                command_id,
                command_type,
                REASON_UNSUPPORTED_COMMAND,
            )

        return self._failed_event(
            command_id,
            command_type,
            REASON_INVALID_COMMAND,
        )

    def _connection_result(self, command_id: str, command_type: str) -> dict:
        if self.mavlink_adapter.is_connected():
            return flight_event(
                drone_id=self.drone_id,
                command_id=command_id,
                command_type=command_type,
                status=STATUS_COMPLETED,
                details=self.mavlink_adapter.details(),
            )
        return self._failed_event(command_id, command_type, REASON_NO_CONNECTION)

    def _set_mode(self, command_id: str, command_type: str, requested_mode: str) -> dict:
        details = self.mavlink_adapter.details()
        if requested_mode:
            details['requested_mode'] = requested_mode

        if not self.mavlink_adapter.is_connected():
            return flight_event(
                drone_id=self.drone_id,
                command_id=command_id,
                command_type=command_type,
                status=STATUS_FAILED,
                reason=REASON_NO_CONNECTION,
                details=details,
            )

        if not requested_mode:
            return flight_event(
                drone_id=self.drone_id,
                command_id=command_id,
                command_type=command_type,
                status=STATUS_FAILED,
                reason=REASON_UNKNOWN_MODE,
                details=details,
            )

        sent, reason = self.mavlink_adapter.send_mode_change(requested_mode)
        if sent:
            return flight_event(
                drone_id=self.drone_id,
                command_id=command_id,
                command_type=command_type,
                status=STATUS_COMPLETED,
                details=details,
            )

        normalized_reason = reason or REASON_MAVLINK_SEND_FAILED
        if normalized_reason not in {
            REASON_NO_CONNECTION,
            REASON_UNKNOWN_MODE,
            REASON_MAVLINK_SEND_FAILED,
        }:
            normalized_reason = REASON_MAVLINK_SEND_FAILED

        return flight_event(
            drone_id=self.drone_id,
            command_id=command_id,
            command_type=command_type,
            status=STATUS_FAILED,
            reason=normalized_reason,
            details=details,
        )

    def _failed_event(self, command_id: str, command_type: str, reason: str) -> dict:
        return flight_event(
            drone_id=self.drone_id,
            command_id=command_id,
            command_type=command_type,
            status=STATUS_FAILED,
            reason=reason,
            details=self.mavlink_adapter.details(),
        )

    def _requested_mode(self, payload: dict) -> str:
        for key in ('mode', 'target_mode'):
            value = payload.get(key)
            if isinstance(value, str) and value.strip():
                return value.strip().upper()
        return ''
