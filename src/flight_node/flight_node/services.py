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

    def handle_routed_command(self, raw_value: str) -> tuple[dict | None, dict | None]:
        envelope = parse_json_object(
            raw_value,
            logger=self.logger,
            log_prefix='flight command',
        )
        if envelope is None:
            return None, None
        if not verify_signed_payload(envelope, self.auth_token):
            self.logger.warn('Ignoring unsigned or invalidly signed flight command')
            return None, None

        command = extract_nested_command(
            envelope,
            logger=self.logger,
            log_prefix='flight command',
        )
        if command is None:
            return self._event(
                command_id='',
                command_type='unknown',
                status=STATUS_FAILED,
                reason=REASON_INVALID_COMMAND,
                details=self.mavlink_adapter.details(),
            ), None

        command_id = str(command.get('command_id', ''))
        command_type = str(command.get('command_type', 'unknown')).lower()

        if command_type in UNSUPPORTED_COMMANDS:
            return self._failed_event(
                command_id,
                command_type,
                REASON_UNSUPPORTED_COMMAND,
            ), None

        if command_type in {'ping', 'health_check'}:
            return self._connection_result(command_id, command_type), None

        if command_type in MODE_ALIASES:
            return self._set_mode(command_id, command_type, MODE_ALIASES[command_type])

        if command_type == 'set_mode':
            requested_mode = self._requested_mode(command.get('payload', {}))
            return self._set_mode(command_id, command_type, requested_mode)

        return self._failed_event(
            command_id,
            command_type,
            REASON_INVALID_COMMAND,
        ), None

    def _connection_result(self, command_id: str, command_type: str) -> dict:
        if self.mavlink_adapter.is_connected():
            return self._event(
                command_id=command_id,
                command_type=command_type,
                status=STATUS_COMPLETED,
                details=self.mavlink_adapter.details(),
            )
        return self._failed_event(command_id, command_type, REASON_NO_CONNECTION)

    def _set_mode(
        self,
        command_id: str,
        command_type: str,
        requested_mode: str,
    ) -> tuple[dict | None, dict | None]:
        details = self.mavlink_adapter.details()
        if requested_mode:
            details['requested_mode'] = requested_mode

        rejection = self._reject_set_mode_preconditions(
            command_id,
            command_type,
            requested_mode,
            details,
        )
        if rejection is not None:
            return rejection, None

        sent, reason = self.mavlink_adapter.send_mode_change(requested_mode)
        if sent:
            self._pending_mode_command = {
                'command_id': command_id,
                'command_type': command_type,
                'requested_mode': requested_mode,
                'details': details,
            }
            return None, dict(self._pending_mode_command)

        return self._event(
            command_id=command_id,
            command_type=command_type,
            status=STATUS_FAILED,
            reason=reason or REASON_MAVLINK_SEND_FAILED,
            details=details,
        ), None

    def _failed_event(self, command_id: str, command_type: str, reason: str) -> dict:
        return self._event(
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

    def confirm_mode_change(self, pending_command: dict, observed_mode: str) -> dict | None:
        if observed_mode.strip().upper() != pending_command['requested_mode']:
            return None

        details = dict(pending_command['details'])
        details['observed_mode'] = observed_mode
        details['result_scope'] = 'flight_controller_mode_confirmed'
        self._pending_mode_command = None
        return self._event(
            command_id=pending_command['command_id'],
            command_type=pending_command['command_type'],
            status=STATUS_COMPLETED,
            details=details,
        )

    def mode_timeout_event(self, pending_command: dict) -> dict:
        details = dict(pending_command['details'])
        details['result_scope'] = 'flight_controller_mode_not_confirmed'
        self._pending_mode_command = None
        return self._event(
            command_id=pending_command['command_id'],
            command_type=pending_command['command_type'],
            status=STATUS_FAILED,
            reason=REASON_MODE_CHANGE_TIMEOUT,
            details=details,
        )

    def _reject_set_mode_preconditions(
        self,
        command_id: str,
        command_type: str,
        requested_mode: str,
        details: dict,
    ) -> dict | None:
        if self._pending_mode_command is not None:
            details['pending_command_id'] = self._pending_mode_command['command_id']
            details['pending_requested_mode'] = self._pending_mode_command['requested_mode']
            return self._event(
                command_id=command_id,
                command_type=command_type,
                status=STATUS_FAILED,
                reason=REASON_COMMAND_IN_PROGRESS,
                details=details,
            )
        if not self.mavlink_adapter.is_connected():
            return self._event(
                command_id=command_id,
                command_type=command_type,
                status=STATUS_FAILED,
                reason=REASON_NO_CONNECTION,
                details=details,
            )
        if not requested_mode:
            return self._event(
                command_id=command_id,
                command_type=command_type,
                status=STATUS_FAILED,
                reason=REASON_UNKNOWN_MODE,
                details=details,
            )
        return None

    def _event(
        self,
        command_id: str,
        command_type: str,
        status: str,
        *,
        reason: str = '',
        details: dict | None = None,
    ) -> dict:
        return flight_event(
            drone_id=self.drone_id,
            secret=self.auth_token,
            command_id=command_id,
            command_type=command_type,
            status=status,
            reason=reason,
            details=details,
        )
