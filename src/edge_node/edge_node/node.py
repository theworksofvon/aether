from config import COMMAND_AUTH_TOKEN_PLACEHOLDER, config
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import String

from .services import EdgeRoutingService, load_completion_topics, load_route_topics


class EdgeNode(Node):
    def __init__(self):
        super().__init__('edge_node')
        self.drone_id = self.declare_parameter(
            'drone_id',
            config.drone.AETHER_DRONE_ID,
        ).value
        self.groups = list(
            self.declare_parameter(
                'groups',
                config.drone.AETHER_DRONE_GROUPS,
            ).value
        )
        disconnect_timeout_s = float(
            self.declare_parameter(
                'disconnect_timeout_s',
                config.edge.AETHER_EDGE_DISCONNECT_TIMEOUT_S,
            ).value
        )
        status_period_s = float(
            self.declare_parameter(
                'status_period_s',
                config.edge.AETHER_EDGE_STATUS_PERIOD_S,
            ).value
        )
        auth_token = self.declare_parameter(
            'command_auth_token',
            config.security.AETHER_COMMAND_AUTH_TOKEN,
        ).value
        if not isinstance(auth_token, str) or not auth_token.strip():
            raise RuntimeError(
                'AETHER_COMMAND_AUTH_TOKEN must be set for edge_node startup'
            )
        if auth_token == COMMAND_AUTH_TOKEN_PLACEHOLDER:
            raise RuntimeError(
                'AETHER_COMMAND_AUTH_TOKEN cannot use the example placeholder value'
            )

        self.fleet_commands_topic = self.declare_parameter(
            'fleet_commands_topic',
            config.fleet.AETHER_FLEET_COMMANDS_TOPIC,
        ).value
        self.fleet_acks_topic = self.declare_parameter(
            'fleet_acks_topic',
            config.fleet.AETHER_FLEET_ACKS_TOPIC,
        ).value
        self.fleet_telemetry_topic = self.declare_parameter(
            'fleet_telemetry_topic',
            config.fleet.AETHER_FLEET_TELEMETRY_TOPIC,
        ).value

        self.route_topics = load_route_topics(self)
        self.completion_topics = load_completion_topics(self)

        self.flight_gps_topic = config.topic('flight/gps')
        self.flight_battery_topic = config.topic('flight/battery')
        self.flight_mode_topic = config.topic('flight/mode')
        self.autonomy_state_topic = config.topic('autonomy/state')
        self.mission_status_topic = config.topic('mission/status')
        self.edge_ack_topic = config.topic('edge/acks')

        self.route_publishers = {
            route_name: self.create_publisher(String, topic_name, 10)
            for route_name, topic_name in self.route_topics.items()
        }
        self.edge_ack_publisher = self.create_publisher(String, self.edge_ack_topic, 10)
        self.fleet_ack_publisher = self.create_publisher(String, self.fleet_acks_topic, 10)
        self.fleet_telemetry_publisher = self.create_publisher(
            String,
            self.fleet_telemetry_topic,
            10,
        )

        self.service = EdgeRoutingService(
            drone_id=self.drone_id,
            groups=self.groups,
            route_topics=self.route_topics,
            completion_topics=self.completion_topics,
            disconnect_timeout_s=disconnect_timeout_s,
            auth_token=auth_token,
            logger=self.get_logger(),
            clock_now_ns=lambda: self.get_clock().now().nanoseconds,
        )

        self.create_subscription(String, self.fleet_commands_topic, self.handle_command, 10)
        self.create_subscription(NavSatFix, self.flight_gps_topic, self.handle_flight_gps, 10)
        self.create_subscription(BatteryState, self.flight_battery_topic, self.handle_battery, 10)
        self.create_subscription(String, self.flight_mode_topic, self.handle_mode, 10)
        self.create_subscription(String, self.autonomy_state_topic, self.handle_autonomy_state, 10)
        self.create_subscription(String, self.mission_status_topic, self.handle_mission_status, 10)

        self.completion_subscriptions = []
        for route_name, topic_name in self.completion_topics.items():
            subscription = self.create_subscription(
                String,
                topic_name,
                lambda msg, route=route_name: self.handle_route_completion(route, msg),
                10,
            )
            self.completion_subscriptions.append(subscription)

        self.status_timer = self.create_timer(status_period_s, self.publish_status_summary)
        self.get_logger().info(
            f'Edge node started for {self.drone_id} with routes {self.route_topics}'
        )

    def handle_command(self, msg: String):
        route_name, routed_command, ack = self.service.handle_command(msg.data)
        if route_name and routed_command:
            self.service.publish_json(self.route_publishers[route_name], routed_command)
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
        self.service.publish_json(
            self.fleet_telemetry_publisher,
            self.service.build_status_summary(),
        )

    def publish_ack(self, payload: dict):
        self.service.publish_json(self.edge_ack_publisher, payload)
        self.service.publish_json(self.fleet_ack_publisher, payload)
