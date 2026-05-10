class CameraController:
    def __init__(self, camera_service):
        self.camera_service = camera_service
    def on_start_clicked(self):
        self.camera_service.start_camera()
        print("[Controller] Start camera")

    def on_stop_clicked(self):
        self.camera_service.stop_camera()
        print("[Controller] Stop camera")
