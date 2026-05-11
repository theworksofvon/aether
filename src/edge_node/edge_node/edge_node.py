import rclpy

from .node import EdgeNode


def main(args=None):
    rclpy.init(args=args)
    node = EdgeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
