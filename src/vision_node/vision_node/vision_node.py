import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from ultralytics import YOLO
import cv2



class VisionNode(Node):
    def __init__(self):   
        super().__init__("vision_node")
        self.vision_model = YOLO("yolo26n.pt")
        self.capture = cv2.VideoCapture(0)
        self.detected_publisher_ = self.create_publisher(Detection2DArray, "/drone/detections", 10)
        self.timer_ = self.create_timer(0.1, self.update)
        self.get_logger().info("Vision node started....")


    def update(self):
        self.get_logger().info("update called...")
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


        self.get_logger().info(f"Detected {len(msg.detections)} objects")
        self.detected_publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()