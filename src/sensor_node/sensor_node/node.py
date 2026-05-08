from config import get_config
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .adapters import Dht11Adapter
from .services import sensor_reading_message


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        config = get_config()
        self.drone_id = self.declare_parameter(
            'drone_id',
            config.drone.AETHER_DRONE_ID,
        ).value
        topic = self.declare_parameter(
            'sensor_topic',
            config.topic('sensors/dht11'),
        ).value
        pin_name = self.declare_parameter(
            'dht_pin',
            config.sensor.AETHER_SENSOR_DHT_PIN,
        ).value

        self.publisher_ = self.create_publisher(Float32MultiArray, topic, 10)
        self.adapter = Dht11Adapter(pin_name=pin_name, logger=self.get_logger())
        self.timer_ = self.create_timer(2.0, self.read_sensor)
        self.get_logger().info(f'Sensor node started for {self.drone_id} on {topic}')

    def read_sensor(self):
        reading = self.adapter.read()
        if reading is None:
            return

        temperature_c, humidity_percent = reading
        self.publisher_.publish(
            sensor_reading_message(temperature_c, humidity_percent)
        )
        self.get_logger().info(
            f'Temp:{temperature_c} C, Humidity: {humidity_percent}%'
        )
