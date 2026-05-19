import requests
class ESP32Client:
    def __init__(self, IP, timeout):
        self.ip = IP
        self.timeout = timeout
    
    def set_ip(self, ip):

        self.ip = ip

    def send(self, path):
        try:
            response  = requests.get(f"http://{self.ip}/{path}", timeout=self.timeout)
            response.raise_for_status()
            return True
        except requests.RequestException as ex:
            print(f"ESP32 Error:{ex}")
            return False
    def ping(self):
        return self.send('')

class Motor:
    def __init__(self, client):
        self.client = client

    def start_conveyor(self):
        print("start conveyor")
        return self.client.send("motorControl?state=1")

    def stop_conveyor(self):
        return self.client.send("motorControl?state=0")


class Servo:
    def __init__(self, client):
        self.client = client

    def Push(self):
        return self.client.send(f"setServo?angle={180}")
    def Pulling_out(self):
        return self.client.send(f"setServo?angle={0}")