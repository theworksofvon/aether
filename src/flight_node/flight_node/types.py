from common.common.types import EventDetails
from pydantic import BaseModel


class PendingModeCommand(BaseModel):
    command_id: str
    command_type: str
    requested_mode: str
    details: EventDetails
    requested_at_ns: int | None = None
