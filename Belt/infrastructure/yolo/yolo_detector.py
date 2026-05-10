from ultralytics import YOLO
from domain.domain.entities import Product
from ultils.config.settings import Settings
class YOLODetector:
    def __init__(self):
        self.model = YOLO(Settings.YOLO_MODEL_PATH)

    def detect(self, frame):
        results = self.model(frame)[0]
        products = []

        for r in results.boxes.data.cpu().numpy():
            x1, y1, x2, y2, conf, cls_id = r
            label = self.model.names[int(cls_id)]
            products.append(Product(label, (x1, y1, x2, y2), conf))

        return products