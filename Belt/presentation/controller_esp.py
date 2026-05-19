class ESP32Controller:
    def __init__(self, esp32_service):
        self.esp32_service = esp32_service
    def on_connect_clicked(self, ip):
        self.ip = ip
        self.esp32_service.connect_esp32(self.ip)
        print("[Controller] conn esp")
    def on_disconnect_clicked(self):
        self.esp32_service.disconnect_esp32()
        print("[Controller] dis esp")
    def on_start_clicked(self):
        self.esp32_service.start_conveyor()
        print("[Controller] Start motor")
    def on_stop_clicked(self):
        self.esp32_service.stop_conveyor()
        print("[Controller] Stop motor")