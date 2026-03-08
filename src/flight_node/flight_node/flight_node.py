import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import BatteryState

class FlightNode(Node):
    def __init__(self):
        super().__init__('flight_node')
        try:
            self.connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
            self.connection.wait_heartbeat(timeout=10)
            self.get_logger().info("Heartbeat recieved from f405 wing")
        except Exception as e:
            self.get_logger().error(f"Failed to get heartbeat: {e}")
            self.connection = None

        self.gps_publisher_ = self.create_publisher(NavSatFix, '/drone/gps', 10)
        self.alt_publisher_ = self.create_publisher(Vector3, '/drone/attitude', 10)
        self.battery_publisher_ = self.create_publisher(BatteryState, '/drone/battery', 10)
        self.timer_ = self.create_timer(2.0, self.update)
        self.get_logger().info('Flight node started....')

    def update(self):
        if self.connection is None:
            self.get_logger().info(f"No mavlink connection established...")
            return
        try:
            msg = self.connection.recv_match( blocking=False)
            if msg is None:
                return
            msg_type = msg.get_type()
            
            if msg_type == "ATTITUDE":
                att_msg = Vector3()
                att_msg.x = msg.roll
                att_msg.y = msg.pitch
                att_msg.z = msg.yaw
                self.alt_publisher_.publish(att_msg)
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
        except Exception as e:
            self.get_logger().error(f"Error reading messsage: {e}")
        
        

def main(args=None):
    rclpy.init(args=args)
    node = FlightNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()