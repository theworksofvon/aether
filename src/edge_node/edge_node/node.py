from common.common.publishers import JsonPublisher
from config import config
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import String

from .services import EdgeRoutingService, load_completion_topics, load_route_topics
from .types import EdgeAck, RoutePublishers


class EdgeNode(Node):
    def __init__(self):
        super().__init__('edge_node')
        self.drone_id = config.drone.AETHER_DRONE_ID
        self.groups = list(config.drone.AETHER_DRONE_GROUPS)
        self.route_topics = load_route_topics()
        self.completion_topics = load_completion_topics()
        self.flight_gps_topic = config.topic('flight/gps')
        self.flight_battery_topic = config.topic('flight/battery')
        self.flight_mode_topic = config.topic('flight/mode')
        self.autonomy_state_topic = config.topic('autonomy/state')
        self.mission_status_topic = config.topic('mission/status')
        self.edge_ack_topic = config.topic('edge/acks')
        self.route_publishers = self._build_route_publishers()
        self.edge_ack_publisher = JsonPublisher(
            self.create_publisher(String, self.edge_ack_topic, 10)
        )
        self.fleet_ack_publisher = JsonPublisher(
            self.create_publisher(String, config.fleet.AETHER_FLEET_ACKS_TOPIC, 10)
        )
        self.fleet_telemetry_publisher = JsonPublisher(
            self.create_publisher(String, config.fleet.AETHER_FLEET_TELEMETRY_TOPIC, 10)
        )

        self.service = EdgeRoutingService(
            drone_id=self.drone_id,
            groups=self.groups,
            route_topics=self.route_topics,
            completion_topics=self.completion_topics,
            disconnect_timeout_s=config.edge.AETHER_EDGE_DISCONNECT_TIMEOUT_S,
            auth_token=config.require_auth_token('edge_node'),
            logger=self.get_logger(),
            clock_now_ns=lambda: self.get_clock().now().nanoseconds,
        )

        self._register_core_subscriptions()
        self.completion_subscriptions = self._register_completion_subscriptions()
        self.status_timer = self.create_timer(
            config.edge.AETHER_EDGE_STATUS_PERIOD_S,
            self.publish_status_summary,
        )
        self.get_logger().info(
            f'Edge node started for {self.drone_id} with routes {self.route_topics}'
        )

    def _build_route_publishers(self) -> RoutePublishers:
        return RoutePublishers(
            flight=JsonPublisher(self.create_publisher(String, self.route_topics.flight, 10)),
            autonomy=JsonPublisher(
                self.create_publisher(String, self.route_topics.autonomy, 10)
            ),
            vision=JsonPublisher(self.create_publisher(String, self.route_topics.vision, 10)),
            mission=JsonPublisher(self.create_publisher(String, self.route_topics.mission, 10)),
            system=JsonPublisher(self.create_publisher(String, self.route_topics.system, 10)),
        )

    def _register_core_subscriptions(self):
        self.create_subscription(
            String,
            config.fleet.AETHER_FLEET_COMMANDS_TOPIC,
            self.handle_command,
            10,
        )
        self.create_subscription(NavSatFix, self.flight_gps_topic, self.handle_flight_gps, 10)
        self.create_subscription(BatteryState, self.flight_battery_topic, self.handle_battery, 10)
        self.create_subscription(String, self.flight_mode_topic, self.handle_mode, 10)
        self.create_subscription(String, self.autonomy_state_topic, self.handle_autonomy_state, 10)
        self.create_subscription(String, self.mission_status_topic, self.handle_mission_status, 10)

    def _register_completion_subscriptions(self):
        subscriptions = []
        for route_name, topic_name in self.completion_topics.items():
            subscriptions.append(
                self.create_subscription(
                    String,
                    topic_name,
                    lambda msg, route=route_name: self.handle_route_completion(route, msg),
                    10,
                )
            )
        return subscriptions

    def handle_command(self, msg: String):
        route_name, routed_command, ack = self.service.handle_command(msg.data)
        if route_name and routed_command:
            self.route_publishers.publisher_for(route_name).publish(routed_command)
        if ack:
            self.publish_ack(ack)

    def handle_route_completion(self, route_name: str, msg: String):
        ack = self.service.handle_route_completion(route_name, msg.data)
        if ack:
            self.publish_ack(ack)

    def handle_flight_gps(self, msg: NavSatFix):
        self.service.handle_flight_gps(msg.latitude, msg.longitude, msg.altitude)

    def handle_battery(self, msg: BatteryState):
        self.service.handle_battery(msg.voltage)

    def handle_mode(self, msg: String):
        self.service.handle_mode(msg.data)

    def handle_autonomy_state(self, msg: String):
        self.service.handle_autonomy_state(msg.data)

    def handle_mission_status(self, msg: String):
        self.service.handle_mission_status(msg.data)

    def publish_status_summary(self):
        self.fleet_telemetry_publisher.publish(self.service.build_status_summary())

    def publish_ack(self, payload: EdgeAck):
        self.edge_ack_publisher.publish(payload)
        self.fleet_ack_publisher.publish(payload)
