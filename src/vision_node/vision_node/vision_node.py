import os

import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose


class VisionNode(Node):
    def __init__(self):   
        super().__init__("vision_node")
        self.drone_id = self.declare_parameter(
            'drone_id',
            os.getenv('AETHER_DRONE_ID', 'AE-01'),
        ).value
        model_path = self.declare_parameter(
            'model_path',
            os.getenv('AETHER_VISION_MODEL_PATH', 'yolo26n.pt'),
        ).value
        camera_index = int(
            self.declare_parameter(
                'camera_index',
                int(os.getenv('AETHER_VISION_CAMERA_INDEX', '0')),
            ).value
        )
        topic = self.declare_parameter(
            'detections_topic',
            f'/{self.drone_id}/vision/detections',
        ).value

        self.vision_model = YOLO(model_path)
        self.capture = cv2.VideoCapture(camera_index)
        self.detected_publisher_ = self.create_publisher(Detection2DArray, topic, 10)
        self.timer_ = self.create_timer(0.1, self.update)
        self.get_logger().info(
            f'Vision node started for {self.drone_id} on {topic}'
        )


    def update(self):
        ret, frame = self.capture.read()

        if not ret:
            self.get_logger().warn("Failed to grab frame...")
            return
        
        results = self.vision_model.predict(source=frame, verbose=False)

        msg = Detection2DArray()

        for result in results:
            for box in result.boxes:
                detection = Detection2D()


                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(int(box.cls[0]))
                hypothesis.hypothesis.score = float(box.conf[0])
                detection.results.append(hypothesis)

                x1, y1, x2, y2 = box.xyxy[0]
                detection.bbox.center.position.x = float((x1 + x2) / 2)
                detection.bbox.center.position.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                msg.detections.append(detection)


        self.detected_publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
