import cv2
from ultralytics import YOLO


class VisionAdapter:
    def __init__(self, model_path: str, camera_index: int):
        self.vision_model = YOLO(model_path)
        self.capture = cv2.VideoCapture(camera_index)

    def detect(self):
        ret, frame = self.capture.read()
        if not ret:
            return None
        return self.vision_model.predict(source=frame, verbose=False)
