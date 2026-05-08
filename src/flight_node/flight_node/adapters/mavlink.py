from typing import cast

from pymavlink import mavutil
from common.common.types import EventDetails
from .typing import MavlinkConnection


class MavlinkAdapter:
    def __init__(self, serial_port: str, baud_rate: int, logger):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.logger = logger
        self.connection: MavlinkConnection | None = None

    def connect(self):
        try:
            connection = self._open_connection()
            connection.wait_heartbeat(timeout=10)
            self.connection = connection
            self.logger.info(
                f'Heartbeat received on {self.serial_port} for flight control link'
            )
        except Exception as exc:
            self.logger.error(f'Failed to get heartbeat: {exc}')
            self.connection = None

    def _open_connection(self) -> MavlinkConnection:
        return cast(
            MavlinkConnection,
            mavutil.mavlink_connection(
                self.serial_port,
                baud=self.baud_rate,
            ),
        )

    def is_connected(self) -> bool:
        return self.connection is not None

    def receive_message(self):
        if self.connection is None:
            return None
        return self.connection.recv_match(blocking=False)

    def resolve_mode(self, mode_name: str) -> int | None:
        if self.connection is None:
            return None

        requested = mode_name.upper()
        try:
            mapping = self._normalized_mode_mapping(self.connection.mode_mapping())
        except Exception as exc:
            self.logger.error(f'Failed to load mode mapping: {exc}')
            return None

        for name, custom_mode in mapping.items():
            if name == requested:
                return custom_mode
        return None

    def send_mode_change(self, mode_name: str) -> tuple[bool, str]:
        if self.connection is None:
            return False, 'no_connection'

        custom_mode = self.resolve_mode(mode_name)
        if custom_mode is None:
            return False, 'unknown_mode'

        try:
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode,
            )
        except Exception as exc:
            self.logger.error(f'Failed to send mode change to {mode_name}: {exc}')
            return False, 'mavlink_send_failed'

        return True, ''

    def details(self) -> EventDetails:
        return EventDetails(
            connection_state='connected' if self.is_connected() else 'disconnected',
            serial_port=self.serial_port,
            baud_rate=self.baud_rate,
        )

    def mode_string(self, message) -> str:
        return mavutil.mode_string_v10(message)

    def _normalized_mode_mapping(
        self,
        raw_mapping: dict[object, object] | None,
    ) -> dict[str, int]:
        normalized_mapping: dict[str, int] = {}
        if raw_mapping is None:
            return normalized_mapping

        for name, custom_mode in raw_mapping.items():
            normalized_value = self._normalize_custom_mode(custom_mode)
            if normalized_value is None:
                continue
            normalized_mapping[str(name).upper()] = normalized_value
        return normalized_mapping

    def _normalize_custom_mode(self, value: object) -> int | None:
        if isinstance(value, bool):
            return int(value)
        if isinstance(value, int):
            return value
        if isinstance(value, float):
            return int(value)
        if isinstance(value, str):
            try:
                return int(value)
            except ValueError:
                return None
        return None
