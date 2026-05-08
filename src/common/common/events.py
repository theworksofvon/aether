import json
from datetime import datetime, timezone


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def build_command_event(
    drone_id: str,
    command_id: str,
    command_type: str,
    status: str,
    reason: str = '',
    details: dict | None = None,
) -> dict:
    event = {
        'timestamp': utc_timestamp(),
        'drone_id': drone_id,
        'command_id': command_id,
        'command_type': command_type,
        'status': status,
    }
    if reason:
        event['reason'] = reason
    if details:
        event['details'] = details
    return event


def dumps_json(payload: dict) -> str:
    return json.dumps(payload)
