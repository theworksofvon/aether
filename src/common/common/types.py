from collections.abc import Mapping

from pydantic import BaseModel, ConfigDict, Field, field_validator


class FlexibleModel(BaseModel):
    model_config = ConfigDict(extra='allow')


class CommandPayload(FlexibleModel):
    pass


class EventDetails(FlexibleModel):
    requested_mode: str | None = None
    observed_mode: str | None = None
    connection_state: str | None = None
    serial_port: str | None = None
    baud_rate: int | None = None
    result_scope: str | None = None
    pending_command_id: str | None = None
    pending_requested_mode: str | None = None


class Position(FlexibleModel):
    latitude: float
    longitude: float
    altitude: float


class SignedModel(BaseModel):
    signature: str | None = None


class CommandModel(FlexibleModel):
    command_id: str
    command_type: str
    target_type: str = 'drone'
    target: str | None = None
    targets: list[str] = Field(default_factory=list)
    payload: CommandPayload = Field(default_factory=CommandPayload)

    @field_validator('command_id', mode='before')
    @classmethod
    def normalize_command_id(cls, value: object) -> str:
        return str(value or '').strip()

    @field_validator('command_type', mode='before')
    @classmethod
    def normalize_command_type(cls, value: object) -> str:
        return str(value or '').strip().lower()

    @field_validator('target_type', mode='before')
    @classmethod
    def normalize_target_type(cls, value: object) -> str:
        return str(value or 'drone').strip().lower()

    @field_validator('payload', mode='before')
    @classmethod
    def normalize_payload(cls, value: object) -> CommandPayload:
        if value is None:
            return CommandPayload()
        if isinstance(value, CommandPayload):
            return value
        if isinstance(value, Mapping):
            return CommandPayload.model_validate(value)
        raise ValueError('command payload must be an object')

    @field_validator('targets', mode='before')
    @classmethod
    def normalize_targets(cls, value: object) -> list[str]:
        if value is None:
            return []
        if isinstance(value, list):
            return [str(item).strip() for item in value if str(item).strip()]
        raise ValueError('targets must be a list')


class RoutedCommandEnvelope(SignedModel):
    received_at: str
    drone_id: str
    route: str
    route_topic: str
    command: CommandModel


class CommandEvent(SignedModel):
    timestamp: str
    drone_id: str
    command_id: str
    command_type: str
    status: str
    reason: str | None = None
    details: EventDetails | None = None
