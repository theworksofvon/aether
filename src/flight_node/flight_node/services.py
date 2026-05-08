from common import (
    REASON_COMMAND_IN_PROGRESS,
    REASON_INVALID_COMMAND,
    REASON_MAVLINK_SEND_FAILED,
    REASON_MODE_CHANGE_TIMEOUT,
    REASON_NO_CONNECTION,
    REASON_UNSUPPORTED_COMMAND,
    REASON_UNKNOWN_MODE,
    STATUS_COMPLETED,
    STATUS_FAILED,
    extract_nested_command,
    parse_json_object,
    verify_signed_payload,
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
    def __init__(self, drone_id: str, mavlink_adapter, logger, auth_token: str):
        self.drone_id = drone_id
        self.mavlink_adapter = mavlink_adapter
        self.logger = logger
        self.auth_token = auth_token
        self._pending_mode_command: dict | None = None
        self._new_pending_mode_command: dict | None = None

    def handle_routed_command(self, raw_value: str) -> dict | None:
        envelope = parse_json_object(
            raw_value,
            logger=self.logger,
            log_prefix='flight command',
        )
        if envelope is None:
            return None
        if not verify_signed_payload(envelope, self.auth_token):
            self.logger.warn('Ignoring unsigned or invalidly signed flight command')
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
                secret=self.auth_token,
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
                secret=self.auth_token,
                details=self.mavlink_adapter.details(),
            )
        return self._failed_event(command_id, command_type, REASON_NO_CONNECTION)

    def _set_mode(self, command_id: str, command_type: str, requested_mode: str) -> dict:
        details = self.mavlink_adapter.details()
        if requested_mode:
            details['requested_mode'] = requested_mode
        if self._pending_mode_command is not None:
            details['pending_command_id'] = self._pending_mode_command['command_id']
            details['pending_requested_mode'] = self._pending_mode_command['requested_mode']
            return flight_event(
                drone_id=self.drone_id,
                command_id=command_id,
                command_type=command_type,
                status=STATUS_FAILED,
                secret=self.auth_token,
                reason=REASON_COMMAND_IN_PROGRESS,
                details=details,
            )

        if not self.mavlink_adapter.is_connected():
            return flight_event(
                drone_id=self.drone_id,
                command_id=command_id,
                command_type=command_type,
                status=STATUS_FAILED,
                secret=self.auth_token,
                reason=REASON_NO_CONNECTION,
                details=details,
            )

        if not requested_mode:
            return flight_event(
                drone_id=self.drone_id,
                command_id=command_id,
                command_type=command_type,
                status=STATUS_FAILED,
                secret=self.auth_token,
                reason=REASON_UNKNOWN_MODE,
                details=details,
            )

        sent, reason = self.mavlink_adapter.send_mode_change(requested_mode)
        if sent:
            self._pending_mode_command = {
                'command_id': command_id,
                'command_type': command_type,
                'requested_mode': requested_mode,
                'details': details,
            }
            self._new_pending_mode_command = dict(self._pending_mode_command)
            return None

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
            secret=self.auth_token,
            reason=normalized_reason,
            details=details,
        )

    def _failed_event(self, command_id: str, command_type: str, reason: str) -> dict:
        return flight_event(
            drone_id=self.drone_id,
            command_id=command_id,
            command_type=command_type,
            status=STATUS_FAILED,
            secret=self.auth_token,
            reason=reason,
            details=self.mavlink_adapter.details(),
        )

    def _requested_mode(self, payload: dict) -> str:
        for key in ('mode', 'target_mode'):
            value = payload.get(key)
            if isinstance(value, str) and value.strip():
                return value.strip().upper()
        return ''

    def take_new_pending_mode_command(self) -> dict | None:
        pending_command = self._new_pending_mode_command
        self._new_pending_mode_command = None
        return pending_command

    def confirm_mode_change(self, pending_command: dict, observed_mode: str) -> dict | None:
        if observed_mode.strip().upper() != pending_command['requested_mode']:
            return None

        details = dict(pending_command['details'])
        details['observed_mode'] = observed_mode
        details['result_scope'] = 'flight_controller_mode_confirmed'
        self._pending_mode_command = None
        return flight_event(
            drone_id=self.drone_id,
            command_id=pending_command['command_id'],
            command_type=pending_command['command_type'],
            status=STATUS_COMPLETED,
            secret=self.auth_token,
            details=details,
        )

    def mode_timeout_event(self, pending_command: dict) -> dict:
        details = dict(pending_command['details'])
        details['result_scope'] = 'flight_controller_mode_not_confirmed'
        self._pending_mode_command = None
        return flight_event(
            drone_id=self.drone_id,
            command_id=pending_command['command_id'],
            command_type=pending_command['command_type'],
            status=STATUS_FAILED,
            secret=self.auth_token,
            reason=REASON_MODE_CHANGE_TIMEOUT,
            details=details,
        )
