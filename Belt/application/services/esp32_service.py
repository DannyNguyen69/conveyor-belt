import time


class ConveyorService:

    def __init__(self, state, esp32_client, motor, servo):

        self.motor = motor

        self.servo = servo

        self.esp32_client = esp32_client

        self.state = state

    # =====================================================
    # CONNECT ESP32
    # =====================================================

    def connect_esp32(self):

        success = self.esp32_client.ping()

        self.state.esp32_connected = success

        return success

    # =====================================================
    # DISCONNECT ESP32
    # =====================================================

    def disconnect_esp32(self):

        self.state.esp32_connected = False

    # =====================================================
    # START CONVEYOR
    # =====================================================

    def start_conveyor(self):

        if not self.state.esp32_connected:
            return False

        success = self.motor.start()

        if success:

            self.state.conveyor_running = True

        return success

    # =====================================================
    # STOP CONVEYOR
    # =====================================================

    def stop_conveyor(self):

        success = self.motor.stop()

        if success:

            self.state.conveyor_running = False

        return success
