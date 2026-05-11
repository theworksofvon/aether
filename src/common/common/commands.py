from typing import TypeVar

from pydantic import BaseModel, ValidationError

T = TypeVar('T', bound=BaseModel)

def parse_json_model(
    raw_value: str,
    model_type: type[T],
    logger=None,
    log_prefix: str = 'message',
) -> T | None:
    try:
        return model_type.model_validate_json(raw_value)
    except ValidationError as exc:
        if logger is not None:
            logger.warn(f'Ignoring malformed {log_prefix}: {exc}')
        return None
