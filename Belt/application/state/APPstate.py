# state.py
import threading

class AppState:
    def __init__(self):
        # CAMERA
        self.frame_lock = threading.Lock()
        self.camera_running = False
        self.current_frame = None
        # CONVEYOR
        self.esp32_connected = False
        self.conveyor_running = False
        #DETECT
        self.pred_done = False
        self.triggered = False
        self.action_done = False
        #ARENA
        self.is_setting = False
        self.beset = False