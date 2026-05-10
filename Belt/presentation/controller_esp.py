class ESP32Controller:
    def __init__(self, esp32_service):
        self.esp32_service = esp32_service
    def on_connect_clicked(self):
        self.esp32_service.esp32_client.connect_esp32()
        print("[Controller] conn esp")
    def on_disconnect_clicked(self):
        self.esp32_service.esp32_client.disconnect_esp32()
        print("[Controller] dis esp")
    def on_start_clicked(self):
        self.esp32_service.motor.start_conveyor()
        print("[Controller] Start motor")
    def on_stop_clicked(self):
        self.esp32_service.motorstop_conveyor()
        print("[Controller] Stop motor")