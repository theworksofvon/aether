from pydantic import BaseModel, ConfigDict, Field

from common.common.publishers import JsonPublisher
from common.common.types import CommandEvent, Position, RoutedCommandEnvelope


class RouteTopics(BaseModel):
    model_config = ConfigDict(extra='forbid')

    flight: str
    autonomy: str
    vision: str
    mission: str
    system: str

    def topic_for(self, route_name: str) -> str:
        return getattr(self, route_name)

    def items(self) -> list[tuple[str, str]]:
        return [
            ('flight', self.flight),
            ('autonomy', self.autonomy),
            ('vision', self.vision),
            ('mission', self.mission),
            ('system', self.system),
        ]


class RoutePublishers(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True, extra='forbid')

    flight: JsonPublisher[RoutedCommandEnvelope]
    autonomy: JsonPublisher[RoutedCommandEnvelope]
    vision: JsonPublisher[RoutedCommandEnvelope]
    mission: JsonPublisher[RoutedCommandEnvelope]
    system: JsonPublisher[RoutedCommandEnvelope]

    def publisher_for(self, route_name: str) -> JsonPublisher[RoutedCommandEnvelope]:
        return getattr(self, route_name)


class EdgeAck(BaseModel):
    ack_at: str
    drone_id: str
    command_id: str
    command_type: str
    route: str
    status: str
    coordinator_link: str
    reason: str | None = None
    source_topic: str | None = None
    details: CommandEvent | None = None


class StatusSummary(BaseModel):
    model_config = ConfigDict(extra='forbid')

    timestamp: str
    drone_id: str
    groups: list[str]
    coordinator_link: str
    last_command_id: str
    last_command_type: str
    mode: str
    autonomy_state: str
    mission_status: str
    battery_voltage: float | None = None
    position: Position | None = None
    route_topics: RouteTopics


class OutstandingCommand(BaseModel):
    command_id: str = Field(min_length=1)
    route: str
    command_type: str
