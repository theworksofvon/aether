import rclpy

from .node import FlightNode


def main(args=None):
    rclpy.init(args=args)
    node = FlightNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
