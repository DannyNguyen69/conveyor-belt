#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// kich thuoc oled
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64

const char* ssid = "DUY";
const char* password = "12345678";

const int IN1 = 26;
const int IN2 = 27;
const int ENA = 25;
const int SERVO_PIN = 14;
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;

Servo myServo;
// khai bao bien oleb
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

AsyncWebServer server(80);

String motorStatus = "STOPPED";
String conveyorStatus = "STOPPED";
int servoAngle = 90;
int pwmSpeed = 0;

unsigned long lastDisplayToggle = 0;
const long displayInterval = 2000;
bool displayScreen1 = true;

void setup() {
  Serial.begin(115200);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  
  Serial.println(F("SSD1306 allocation failed"));
  for(;;);
  }
  display.clearDisplay();
  display.display();
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL);

  myServo.attach(SERVO_PIN);
  myServo.write(servoAngle);

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10); 
  display.print("System Starting...");
  display.display();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10); 
  display.print("WiFi Connected");
  display.setCursor(0, 20);
  display.print(WiFi.localIP());
  display.display();
  delay(2000);
 // html 
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Control</title>
  <style>
    body { font-family: Arial; text-align: center; }
    .button { padding: 10px 20px; margin: 5px; font-size: 16px; }
    input[type=range] { width: 300px; }
  </style>
</head>
<body>
  <h1>ESP32 Conveyor & Servo</h1>
  <p>Motor status: <span id="motorStatus">)rawliteral" + motorStatus + R"rawliteral(</span></p>
  <button class="button" onclick="controlMotor('HIGH')">Start Motor</button>
  <button class="button" onclick="controlMotor('LOW')">Stop Motor</button>
  
  <p>Servo angle: <span id="servoAngle">)rawliteral" + String(servoAngle) + R"rawliteral(</span>°</p>
  <input type="range" min="0" max="180" value=")rawliteral" + String(servoAngle) + R"rawliteral(" id="servoSlider" onchange="setServo(this.value)">
  
  <script>
    function controlMotor(signal){
      fetch('/motorControl?signal=' + signal)
      .then(response => {
        updateStatus();
      });
    }

    function setServo(angle){
      fetch('/setServo?angle=' + angle)
      .then(response => {
        document.getElementById('servoAngle').innerText = angle;
      });
    }

    function updateStatus(){
      // Cập nhật trạng thái motor mỗi giây
      fetch('/status')
      .then(response => response.json())
      .then(data => {
        document.getElementById('motorStatus').innerText = data.motor;
        document.getElementById('servoAngle').innerText = data.servo;
        document.getElementById('servoSlider').value = data.servo;
      });
    }

    setInterval(updateStatus, 1000);
  </script>
</body>
</html>
)rawliteral";
  request->send(200, "text/html", html);
});
//-----------------------------------------------------------------------------------------------------
  server.on("/motorControl", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("signal")) {
      String signal = request->getParam("signal")->value();
      if (signal == "HIGH") {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        ledcWrite(PWM_CHANNEL, 128);
        motorStatus = "RUNNING";
        conveyorStatus = "RUNNING";
        pwmSpeed = 50;
      } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        ledcWrite(PWM_CHANNEL, 0);
        motorStatus = "STOPPED";
        conveyorStatus = "STOPPED";
        pwmSpeed = 0;
      }}
    request->send(200, "text/plain", "OK");
  });
  server.on("/setServo", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("angle")) {
      servoAngle = request->getParam("angle")->value().toInt();
      myServo.write(servoAngle);
    }
    request->send(200, "text/plain", "OK");
  });
  server.begin();
}
void loop() {
  updateOLED();   
}

void updateOLED() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastDisplayToggle >= displayInterval) {
    displayScreen1 = !displayScreen1;
    lastDisplayToggle = currentMillis;
    display.clearDisplay();

    if (displayScreen1) {
      display.setCursor(0, 0);
      display.print("Conv:");
      display.print(conveyorStatus);
      display.setCursor(0, 10);
      display.print("Speed:");
      display.print(pwmSpeed);
      display.print("%");
      display.setCursor(0, 0);
      display.print("Servo:");
      display.print(servoAngle);
      display.print(" deg");
    }
    else {
      display.setTextSize(3);
      display.setTextColor(WHITE);
      display.setCursor(0, 10); 
      display.print("WiFi Connected");
      display.setCursor(0, 20);
      display.print(WiFi.localIP());
    }
    display.display();
  }
}

