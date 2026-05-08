import hmac
import json
from hashlib import sha256
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


def canonical_json(payload: dict) -> str:
    return json.dumps(payload, sort_keys=True, separators=(',', ':'))


def sign_payload(payload: dict, secret: str) -> dict:
    signed_payload = dict(payload)
    signed_payload.pop('signature', None)
    digest = hmac.new(
        secret.encode('utf-8'),
        canonical_json(signed_payload).encode('utf-8'),
        sha256,
    ).hexdigest()
    signed_payload['signature'] = digest
    return signed_payload


def verify_signed_payload(payload: dict, secret: str) -> bool:
    signature = str(payload.get('signature', ''))
    if not signature:
        return False

    unsigned_payload = dict(payload)
    unsigned_payload.pop('signature', None)
    expected = hmac.new(
        secret.encode('utf-8'),
        canonical_json(unsigned_payload).encode('utf-8'),
        sha256,
    ).hexdigest()
    return hmac.compare_digest(signature, expected)
