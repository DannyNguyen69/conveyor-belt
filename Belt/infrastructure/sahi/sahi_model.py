from ultils.config.settings import Settings
from sahi import AutoDetectionModel
class BuildSahiModel:
    def __init__(self):
        pass

    def build_sahi_model(self):
        return AutoDetectionModel.from_pretrained(
            model_type="ultralytics",
            model_path=Settings.YOLO_MODEL_PATH,
            confidence_threshold=Settings.SAHI_CONFIDENCE,
            device=Settings.DEVICE
        )