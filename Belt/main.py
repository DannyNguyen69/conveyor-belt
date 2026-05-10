import tkinter as tk
from presentation.controller_camera import CameraController
from presentation.controller_esp import ESP32Controller
from presentation.main_controller import MainController
from presentation.arena_selector import AreaSelector
from presentation.gui import GUI
from application.state.APPstate import AppState
from infrastructure.camera.camera_stream import CameraStream
from infrastructure.esp32.esp32_client import ESP32Client
from infrastructure.esp32.esp32_client import Motor
from infrastructure.esp32.esp32_client import Servo
from infrastructure.sahi.sahi_model import BuildSahiModel
from infrastructure.sahi.sahi_detector import SahiDetector
from infrastructure.yolo.yolo_detector import YOLODetector
from application.services.camera_service import CameraService
from application.services.esp32_service import ConveyorService
from application.use_cases.conveyor_usecase import ConveyorUseCase
from ultils.config.settings import Settings
from application.state.APPstate import AppState

root = tk.Tk()

state = AppState()

# =========================== INFRASTRUCTURE =========================
#=======================================================================
camera = CameraStream(Settings.CAMERA_INDEX)
esp32_client = ESP32Client(Settings.ESP32_IP, Settings.REQUEST_TIMEOUT)
motor = Motor(esp32_client)
servo = Servo(esp32_client)
buildSahi = BuildSahiModel()
sahiDetector = SahiDetector(buildSahi)
yoloDetector = YOLODetector()
# =========================== SERVICES =========================
#==================================================================
camera_service = CameraService(camera ,state)
esp32_service = ConveyorService(state, esp32_client, motor, servo)
usecase = ConveyorUseCase(state, yoloDetector, sahiDetector, motor, servo)
# =========================== CONTROLLERS =========================
#==================================================================
controller_camera = CameraController(camera_service=camera_service)
controller_conveyor =  ESP32Controller(esp32_service = esp32_service)
main_controller = MainController(camera_controller= controller_camera,conveyor_controller= controller_conveyor)
# =========================== UI =========================
#==================================================================
gui = GUI(
    root=root,
    controller=main_controller,
    usecase = usecase,
    state=state
)
gui.start_ui_loop()


root.mainloop()