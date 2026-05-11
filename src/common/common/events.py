import hmac
import json
from hashlib import sha256
from datetime import datetime, timezone
from typing import TypeVar

from pydantic import BaseModel

from .types import CommandEvent, EventDetails, SignedModel

SignedModelT = TypeVar('SignedModelT', bound=SignedModel)


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def build_command_event(
    drone_id: str,
    command_id: str,
    command_type: str,
    status: str,
    reason: str = '',
    details: EventDetails | None = None,
) -> CommandEvent:
    return CommandEvent(
        timestamp=utc_timestamp(),
        drone_id=drone_id,
        command_id=command_id,
        command_type=command_type,
        status=status,
        reason=reason or None,
        details=details if details is not None else None,
    )


def dumps_json(payload: BaseModel) -> str:
    return canonical_json(payload)


# keeps signing and verification stable by forcing one byte representation.
def canonical_json(payload: BaseModel) -> str:
    return json.dumps(
        payload_to_data(payload),
        sort_keys=True,
        separators=(',', ':'),
    )


def sign_payload(payload: SignedModelT, secret: str) -> SignedModelT:
    unsigned_payload = payload.model_copy(update={'signature': None})
    digest = hmac.new(
        secret.encode('utf-8'),
        canonical_json(unsigned_payload).encode('utf-8'),
        sha256,
    ).hexdigest()
    return payload.model_copy(update={'signature': digest})


def verify_signed_payload(payload: SignedModel, secret: str) -> bool:
    signature = payload.signature
    if not isinstance(signature, str) or not signature:
        return False

    unsigned_payload = payload.model_copy(update={'signature': None})
    expected = hmac.new(
        secret.encode('utf-8'),
        canonical_json(unsigned_payload).encode('utf-8'),
        sha256,
    ).hexdigest()
    return hmac.compare_digest(signature, expected)


def payload_to_data(payload: BaseModel) -> object:
    return payload.model_dump(mode='json', exclude_none=True)
