from sahi.predict import get_sliced_prediction
from domain.domain import entities
class SahiDetector:
    def __init__(self, detection_model):
        self.model = detection_model

    def detect(self, image):
        result = get_sliced_prediction(
            image,
            self.model,
            slice_height=256,
            slice_width=256,
            overlap_height_ratio=0.2,
            overlap_width_ratio=0.2,
            verbose=False
        )

        objects = []
        for obj in result.object_prediction_list:
            label = obj.category.name
            bbox = obj.bbox.to_xyxy()
            objects.append(entities.Product(label, bbox, obj.score.value))

        return objects