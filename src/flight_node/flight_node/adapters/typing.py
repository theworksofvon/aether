from typing import Protocol


class MavlinkSender(Protocol):
    def set_mode_send(self, target_system: int, base_mode: int, custom_mode: int) -> None:
        ...


class MavlinkConnection(Protocol):
    target_system: int
    mav: MavlinkSender

    def wait_heartbeat(self, blocking: bool = True, timeout: float | None = None) -> None:
        ...

    def recv_match(self, blocking: bool = False):
        ...

    def mode_mapping(self) -> dict[object, object] | None:
        ...
