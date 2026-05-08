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
        self.security = SecurityConfig()

    def topic(self, suffix: str) -> str:
        return f'/{self.drone.AETHER_DRONE_ID}/{suffix}'

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
