import json
from typing import Any


def parse_json_object(raw_value: str, logger=None, log_prefix: str = 'message') -> dict | None:
    try:
        payload = json.loads(raw_value)
    except json.JSONDecodeError as exc:
        if logger is not None:
            logger.warn(f'Ignoring malformed {log_prefix}: {exc}')
        return None
    if not isinstance(payload, dict):
        if logger is not None:
            logger.warn(f'Ignoring malformed {log_prefix}: expected object')
        return None
    return payload


def extract_nested_command(
    payload: dict[str, Any],
    logger=None,
    log_prefix: str = 'routed command',
) -> dict | None:
    command = payload.get('command')
    if not isinstance(command, dict):
        if logger is not None:
            logger.warn(f'Ignoring malformed {log_prefix}: missing command object')
        return None

    command_type = str(command.get('command_type', '')).strip().lower()
    command_id = str(command.get('command_id', '')).strip()
    nested_payload = command.get('payload', {})

    if nested_payload is None:
        nested_payload = {}
    if not isinstance(nested_payload, dict):
        if logger is not None:
            logger.warn(f'Ignoring malformed {log_prefix}: command payload must be an object')
        return None
    if not command_id:
        if logger is not None:
            logger.warn(f'Ignoring malformed {log_prefix}: missing command_id')
        return None
    if not command_type:
        if logger is not None:
            logger.warn(f'Ignoring malformed {log_prefix}: missing command_type')
        return None

    normalized = dict(command)
    normalized['command_id'] = command_id
    normalized['command_type'] = command_type
    normalized['payload'] = nested_payload
    return normalized
