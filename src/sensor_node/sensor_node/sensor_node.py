import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import adafruit_dht
import board


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/sensors/dht11', 10)
        self.timer_ = self.create_timer(2.0, self.read_sensor)
        self.dht = adafruit_dht.DHT11(board.D4, use_pulseio=False)
        self.get_logger().info('Sensor node started')

    def read_sensor(self):
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