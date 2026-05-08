from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


def detections_message(results) -> Detection2DArray:
    message = Detection2DArray()
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

            message.detections.append(detection)
    return message
