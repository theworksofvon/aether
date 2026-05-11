from common.common.events import build_command_event, sign_payload
from common.common.types import CommandEvent, EventDetails


def flight_event(
    drone_id: str,
    command_id: str,
    command_type: str,
    status: str,
    secret: str,
    reason: str = '',
    details: EventDetails | None = None,
) -> CommandEvent:
    return sign_payload(
        build_command_event(
            drone_id=drone_id,
            command_id=command_id,
            command_type=command_type,
            status=status,
            reason=reason,
            details=details,
        ),
        secret,
    )
