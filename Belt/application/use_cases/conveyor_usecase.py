import time
from ultils.geometry import PointPolygon
class ConveyorUseCase:
    def __init__(self, state, yolo, sahi, motor, servo):
        self.state = state
        self.yolo = yolo
        self.sahi = sahi
        self.motor = motor
        self.servo = servo

        self.captured_image = None

    def process(self, frame, polygon):
        if len(polygon) < 4:
            return

        # ================= STEP 1: YOLO =================
        if not self.state.conveyor_running:
            return
        yolo_products = self.yolo.detect(frame)
        target_product = None
        for product in yolo_products:
            center = product.center()
            inside = PointPolygon.point_in_polygon(center, polygon)

            if inside and product.is_square:

                target_product = product
                break
        if target_product is None:
            return
        # ================= STEP 2: STOP + CAPTURE =================
        if self.state.conveyor_running:
            self.motor.stop()
            self.state.conveyor_running = False
            self.captured_image = frame.copy()
            self.state.pred_done = False
            self.state.triggered = False
            self.state.action_done = False
        # ================= STEP 3: SAHI =================
        if not self.state.conveyor_running and not self.state.pred_done:
            sahi_products = self.sahi.detect(self.captured_image)
            for product in sahi_products:
                if product.is_wrong:
                    self.triggered = True
                    break
            self.state.pred_done = True
        # ================= STEP 4: ACTION =================
        if self.state.pred_done and not self.state.action_done:
            if self.state.triggered:
                self.servo.push()
                time.sleep(0.2)
                self.servo.Pulling_out()
                time.sleep(0.2)
            self.motor.start()
            self.state.conveyor_running = True
            self.state.action_done = True