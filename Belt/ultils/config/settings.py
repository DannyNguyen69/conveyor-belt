
class Settings:
    # ================== MODEL ==================
    YOLO_MODEL_PATH = r"D:\Project\Model\Weights\bangChuyen\best1.pt"

    SAHI_ENABLED = True
    SAHI_SLICE_HEIGHT = 256
    SAHI_SLICE_WIDTH = 256
    SAHI_OVERLAP = 0.2
    SAHI_CONFIDENCE = 0.35

    YOLO_CONFIDENCE = 0.6

    DEVICE = "cuda"  # "cpu" hoặc "cuda"

    # ================== ESP32 ==================
    ESP32_IP = "192.168.1.100"
    REQUEST_TIMEOUT = 3

    # ================== CONVEYOR ==================
    SCAN_DELAY = 2.0
    SERVO_DELAY = 0.2

    # ================== PHOTO ==================
    CAPTURE_ENABLED = True
    CAPTURE_COOLDOWN = 5.0
    PHOTO_PATH = "./cap_bc"

    # ================== SYSTEM ==================
    LOG_MAX_LINES = 50

    # ================== Camera ==================
    CAMERA_INDEX = 1
    SKIP_FRAME = 1