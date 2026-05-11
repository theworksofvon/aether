from typing import Generic, Protocol, TypeVar

from pydantic import BaseModel
from std_msgs.msg import String

from .events import dumps_json

ModelT = TypeVar('ModelT', bound=BaseModel)


class RosStringPublisher(Protocol):
    def publish(self, message: String) -> None:
        ...


class JsonPublisher(Generic[ModelT]):
    def __init__(self, publisher: RosStringPublisher):
        self._publisher = publisher

    def publish(self, payload: ModelT) -> None:
        message = String()
        message.data = dumps_json(payload)
        self._publisher.publish(message)
