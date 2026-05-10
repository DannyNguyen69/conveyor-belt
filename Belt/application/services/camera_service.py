class CameraService:
    def __init__(self, camera, state):
        self.camera = camera
        self.state = state
        self.camera.frame_callback = self.on_new_frame


    def on_new_frame(self, frame):

        with self.state.frame_lock:

            self.state.current_frame = frame

    def start_camera(self):
        
        self.camera.start()
        self.state.camera_running = True

    def stop_camera(self):
        self.camera.stop()
        self.state.camera_running = False