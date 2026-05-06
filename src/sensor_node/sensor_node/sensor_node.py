import os

import board
import adafruit_dht
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.drone_id = self.declare_parameter(
            'drone_id',
            os.getenv('AETHER_DRONE_ID', 'AE-01'),
        ).value
        topic = self.declare_parameter(
            'sensor_topic',
            f'/{self.drone_id}/sensors/dht11',
        ).value
        pin_name = self.declare_parameter(
            'dht_pin',
            os.getenv('AETHER_SENSOR_DHT_PIN', 'D4'),
        ).value
        self.publisher_ = self.create_publisher(Float32MultiArray, topic, 10)
        self.timer_ = self.create_timer(2.0, self.read_sensor)
        self.dht = None

        try:
            dht_pin = getattr(board, pin_name)
            self.dht = adafruit_dht.DHT11(dht_pin, use_pulseio=False)
        except Exception as e:
            self.get_logger().error(f'Failed to initialize DHT11 on {pin_name}: {e}')

        self.get_logger().info(
            f'Sensor node started for {self.drone_id} on {topic}'
        )

    def read_sensor(self):
        if self.dht is None:
            return
        try:
            temp = self.dht.temperature
            humidity = self.dht.humidity
            msg = Float32MultiArray()
            msg.data = [float(temp), float(humidity)]
            self.publisher_.publish(msg)
            self.get_logger().info(f"Temp:{temp} C, Humidity: {humidity}%")
        except RuntimeError as e:
                self.get_logger().warn(f"Sensor read failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
