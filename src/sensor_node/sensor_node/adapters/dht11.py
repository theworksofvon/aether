import adafruit_dht
import board


class Dht11Adapter:
    def __init__(self, pin_name: str, logger):
        self.logger = logger
        self.pin_name = pin_name
        self.dht = None

        try:
            dht_pin = getattr(board, pin_name)
            self.dht = adafruit_dht.DHT11(dht_pin, use_pulseio=False)
        except Exception as exc:
            self.logger.error(f'Failed to initialize DHT11 on {pin_name}: {exc}')

    def read(self) -> tuple[float, float] | None:
        if self.dht is None:
            return None
        try:
            return float(self.dht.temperature), float(self.dht.humidity)
        except RuntimeError as exc:
            self.logger.warn(f'Sensor read failed: {exc}')
            return None
