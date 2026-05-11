from functools import lru_cache

from pydantic import Field, field_validator
from pydantic_settings import BaseSettings

COMMAND_AUTH_TOKEN_PLACEHOLDER = 'change-me-before-production'


class DroneConfig(BaseSettings):
    AETHER_DRONE_ID: str = Field(
        description='Drone identifier',
        default='AE-01',
    )
    AETHER_DRONE_GROUPS: list[str] = Field(
        description='Comma-separated drone groups',
        default_factory=list,
    )

    @field_validator('AETHER_DRONE_GROUPS', mode='before')
    @classmethod
    def parse_drone_groups(cls, value):
        if value is None:
            return []
        if isinstance(value, str):
            if not value.strip():
                return []
            return [item.strip() for item in value.split(',') if item.strip()]
        return value


class FlightConfig(BaseSettings):
    AETHER_FLIGHT_PORT: str = Field(
        description='Flight controller serial device',
        default='/dev/ttyUSB0',
    )
    AETHER_FLIGHT_BAUD_RATE: int = Field(
        description='Flight controller serial baud rate',
        default=115200,
    )
    AETHER_FLIGHT_MODE_CONFIRM_TIMEOUT_S: float = Field(
        description='Seconds to wait for FC mode confirmation after a mode change request',
        default=5.0,
    )
    AETHER_FLIGHT_COMMAND_TOPIC: str = Field(
        description='Flight command topic override',
        default='',
    )
    AETHER_FLIGHT_EVENT_TOPIC: str = Field(
        description='Flight event topic override',
        default='',
    )


class SensorConfig(BaseSettings):
    AETHER_SENSOR_DHT_PIN: str = Field(
        description='Sensor GPIO pin identifier',
        default='D4',
    )


class VisionConfig(BaseSettings):
    AETHER_VISION_MODEL_PATH: str = Field(
        description='Vision model path',
        default='yolo26n.pt',
    )
    AETHER_VISION_CAMERA_INDEX: int = Field(
        description='Camera device index',
        default=0,
    )
    AETHER_VISION_COMMAND_TOPIC: str = Field(
        description='Vision command topic override',
        default='',
    )
    AETHER_VISION_EVENT_TOPIC: str = Field(
        description='Vision event topic override',
        default='',
    )


class EdgeConfig(BaseSettings):
    AETHER_EDGE_DISCONNECT_TIMEOUT_S: float = Field(
        description='Seconds before coordinator link is treated as stale',
        default=15.0,
    )
    AETHER_EDGE_STATUS_PERIOD_S: float = Field(
        description='Seconds between edge telemetry publications',
        default=1.0,
    )


class FleetConfig(BaseSettings):
    AETHER_FLEET_COMMANDS_TOPIC: str = Field(
        description='Fleet command ingress topic',
        default='/fleet/commands',
    )
    AETHER_FLEET_ACKS_TOPIC: str = Field(
        description='Fleet acknowledgement topic',
        default='/fleet/acks',
    )
    AETHER_FLEET_TELEMETRY_TOPIC: str = Field(
        description='Fleet telemetry topic',
        default='/fleet/telemetry',
    )


class AutonomyConfig(BaseSettings):
    AETHER_AUTONOMY_COMMAND_TOPIC: str = Field(
        description='Autonomy command topic override',
        default='',
    )
    AETHER_AUTONOMY_EVENT_TOPIC: str = Field(
        description='Autonomy event topic override',
        default='',
    )


class MissionConfig(BaseSettings):
    AETHER_MISSION_COMMAND_TOPIC: str = Field(
        description='Mission command topic override',
        default='',
    )
    AETHER_MISSION_EVENT_TOPIC: str = Field(
        description='Mission event topic override',
        default='',
    )


class SystemConfig(BaseSettings):
    AETHER_SYSTEM_COMMAND_TOPIC: str = Field(
        description='System command topic override',
        default='',
    )
    AETHER_SYSTEM_EVENT_TOPIC: str = Field(
        description='System event topic override',
        default='',
    )


class SecurityConfig(BaseSettings):
    AETHER_COMMAND_AUTH_TOKEN: str = Field(
        description='Shared HMAC secret for routed commands and completion events',
        default='',
    )


class Config:
    def __init__(self):
        self.drone = DroneConfig()
        self.flight = FlightConfig()
        self.sensor = SensorConfig()
        self.vision = VisionConfig()
        self.edge = EdgeConfig()
        self.fleet = FleetConfig()
        self.autonomy = AutonomyConfig()
        self.mission = MissionConfig()
        self.system = SystemConfig()
        self.security = SecurityConfig()

    def topic(self, suffix: str) -> str:
        return f'/{self.drone.AETHER_DRONE_ID}/{suffix}'

    def resolved_topic(self, override: str, suffix: str) -> str:
        return override if override else self.topic(suffix)

    @property
    def flight_command_topic(self) -> str:
        return self.resolved_topic(
            self.flight.AETHER_FLIGHT_COMMAND_TOPIC,
            'flight/commands',
        )

    @property
    def flight_event_topic(self) -> str:
        return self.resolved_topic(
            self.flight.AETHER_FLIGHT_EVENT_TOPIC,
            'flight/events',
        )

    @property
    def autonomy_command_topic(self) -> str:
        return self.resolved_topic(
            self.autonomy.AETHER_AUTONOMY_COMMAND_TOPIC,
            'autonomy/commands',
        )

    @property
    def autonomy_event_topic(self) -> str:
        return self.resolved_topic(
            self.autonomy.AETHER_AUTONOMY_EVENT_TOPIC,
            'autonomy/events',
        )

    @property
    def vision_command_topic(self) -> str:
        return self.resolved_topic(
            self.vision.AETHER_VISION_COMMAND_TOPIC,
            'vision/commands',
        )

    @property
    def vision_event_topic(self) -> str:
        return self.resolved_topic(
            self.vision.AETHER_VISION_EVENT_TOPIC,
            'vision/events',
        )

    @property
    def mission_command_topic(self) -> str:
        return self.resolved_topic(
            self.mission.AETHER_MISSION_COMMAND_TOPIC,
            'mission/commands',
        )

    @property
    def mission_event_topic(self) -> str:
        return self.resolved_topic(
            self.mission.AETHER_MISSION_EVENT_TOPIC,
            'mission/events',
        )

    @property
    def system_command_topic(self) -> str:
        return self.resolved_topic(
            self.system.AETHER_SYSTEM_COMMAND_TOPIC,
            'system/commands',
        )

    @property
    def system_event_topic(self) -> str:
        return self.resolved_topic(
            self.system.AETHER_SYSTEM_EVENT_TOPIC,
            'system/events',
        )

    def require_auth_token(self, source: str, token: str | None = None) -> str:
        token = self.security.AETHER_COMMAND_AUTH_TOKEN if token is None else token
        if not isinstance(token, str) or not token.strip():
            raise RuntimeError(
                f'AETHER_COMMAND_AUTH_TOKEN must be set for {source} startup'
            )
        if token == COMMAND_AUTH_TOKEN_PLACEHOLDER:
            raise RuntimeError(
                'AETHER_COMMAND_AUTH_TOKEN cannot use the example placeholder value'
            )
        return token


@lru_cache(maxsize=1)
def get_config() -> Config:
    return Config()


config = get_config()
