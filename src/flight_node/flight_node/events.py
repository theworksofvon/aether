from common import build_command_event, sign_payload


def flight_event(
    drone_id: str,
    command_id: str,
    command_type: str,
    status: str,
    secret: str,
    reason: str = '',
    details: dict | None = None,
) -> dict:
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
