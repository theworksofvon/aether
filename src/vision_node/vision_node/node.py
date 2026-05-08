from config import config
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

from .adapters import VisionAdapter
from .services import detections_message


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.drone_id = self.declare_parameter(
            'drone_id',
            config.drone.AETHER_DRONE_ID,
        ).value
        model_path = self.declare_parameter(
            'model_path',
            config.vision.AETHER_VISION_MODEL_PATH,
        ).value
        camera_index = int(
            self.declare_parameter(
                'camera_index',
                config.vision.AETHER_VISION_CAMERA_INDEX,
            ).value
        )
        topic = self.declare_parameter(
            'detections_topic',
            config.topic('vision/detections'),
        ).value

        self.detected_publisher_ = self.create_publisher(Detection2DArray, topic, 10)
        self.adapter = VisionAdapter(model_path=model_path, camera_index=camera_index)
        self.timer_ = self.create_timer(0.1, self.update)
        self.get_logger().info(f'Vision node started for {self.drone_id} on {topic}')

    def update(self):
        results = self.adapter.detect()
        if results is None:
            self.get_logger().warn('Failed to grab frame...')
            return
        self.detected_publisher_.publish(detections_message(results))
