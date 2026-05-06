import os

import rclpy
from geometry_msgs.msg import Vector3
from pymavlink import mavutil
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import String

class FlightNode(Node):
    def __init__(self):
        super().__init__('flight_node')
        self.drone_id = self.declare_parameter(
            'drone_id',
            os.getenv('AETHER_DRONE_ID', 'AE-01'),
        ).value
        serial_port = self.declare_parameter(
            'serial_port',
            os.getenv('AETHER_FLIGHT_PORT', '/dev/ttyAMA0'),
        ).value
        baud_rate = int(self.declare_parameter('baud_rate', 115200).value)
        self.flight_topic_prefix = self.declare_parameter(
            'flight_topic_prefix',
            f'/{self.drone_id}/flight',
        ).value

        try:
            self.connection = mavutil.mavlink_connection(serial_port, baud=baud_rate)
            self.connection.wait_heartbeat(timeout=10)
            self.get_logger().info(
                f'Heartbeat received for {self.drone_id} on {serial_port}'
            )
        except Exception as e:
            self.get_logger().error(f"Failed to get heartbeat: {e}")
            self.connection = None

        self.gps_publisher_ = self.create_publisher(
            NavSatFix,
            f'{self.flight_topic_prefix}/gps',
            10,
        )
        self.attitude_publisher_ = self.create_publisher(
            Vector3,
            f'{self.flight_topic_prefix}/attitude',
            10,
        )
        self.battery_publisher_ = self.create_publisher(
            BatteryState,
            f'{self.flight_topic_prefix}/battery',
            10,
        )
        self.mode_publisher_ = self.create_publisher(
            String,
            f'{self.flight_topic_prefix}/mode',
            10,
        )
        self.timer_ = self.create_timer(0.2, self.update)
        self.get_logger().info(
            f'Flight node started for {self.drone_id} on {self.flight_topic_prefix}'
        )

    def update(self):
        if self.connection is None:
            self.get_logger().info(f"No mavlink connection established...")
            return
        try:
            msg = self.connection.recv_match(blocking=False)
            if msg is None:
                return
            msg_type = msg.get_type()
            
            if msg_type == "ATTITUDE":
                att_msg = Vector3()
                att_msg.x = msg.roll
                att_msg.y = msg.pitch
                att_msg.z = msg.yaw
                self.attitude_publisher_.publish(att_msg)
                self.get_logger().info(f"roll: {msg.roll}, pitch: {msg.pitch}, yaw: {msg.yaw}")
            elif msg_type == "GLOBAL_POSITION_INT":
                gps_msg = NavSatFix()
                gps_msg.latitude = msg.lat / 1e7
                gps_msg.longitude = msg.lon /1e7
                gps_msg.altitude = msg.alt / 1000.0
                self.gps_publisher_.publish(gps_msg)
                self.get_logger().info(f"lat: {msg.lat}, lon: {msg.lon}, alt: {msg.alt}")
            elif msg_type == "SYS_STATUS":
                battery_msg = BatteryState()
                battery_msg.voltage = msg.voltage_battery / 1000.0
                self.battery_publisher_.publish(battery_msg)
                self.get_logger().info(f"battery_voltage( in mV ): {msg.voltage_battery}")
            elif msg_type == 'HEARTBEAT':
                mode_msg = String()
                mode_msg.data = mavutil.mode_string_v10(msg)
                self.mode_publisher_.publish(mode_msg)
        except Exception as e:
            self.get_logger().error(f"Error reading messsage: {e}")
        
        

def main(args=None):
    rclpy.init(args=args)
    node = FlightNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
