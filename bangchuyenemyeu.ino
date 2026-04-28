#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>

/* ====================== Summary ======================
      ====================== 1. Config ======================
          1.1. IN1,IN2: State of motor
              LOW,LOW: Brake / Stop
              HIGH,LOW: Forward
              LOW,HIGH: Backward
              HIGH,HIGH: Brake
          1.2. ENA: PMW, config speed motor
*/
const int IN1 = 26;
const int IN2 = 27;
const int ENA = 25;       
const int SERVO_PIN = 14;

// ====================== wifi ======================
const char* ssid = "DUY";
const char* password = "12345678";


Servo myServo;
AsyncWebServer server(80);


// ====================== var ======================
String motorStatus = "STOPPED";
int servoAngle = 90;
int pwmSpeed = 0;

void setup() {
  Serial.begin(115200);
// ======== setup =========
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  myServo.attach(SERVO_PIN);
  myServo.write(servoAngle);
// ====================================

  // Kết nối WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // ====================== WEB SERVER ROUTES ======================

  // Trang chính
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Control - Test Response</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin-top: 30px; }
    button { padding: 12px 24px; margin: 10px; font-size: 18px; cursor: pointer; }
    input[type=range] { width: 80%; margin: 15px 0; }
    .response { 
      margin: 20px auto; 
      padding: 15px; 
      background: #f0f0f0; 
      border: 1px solid #ccc; 
      border-radius: 8px; 
      max-width: 600px; 
      text-align: left;
      white-space: pre-wrap;
    }
  </style>
</head>
<body>
  <h1>ESP32 Conveyor & Servo Control</h1>
  
  <h2>Motor Control</h2>
  <button onclick="controlMotor(1)">BẬT MOTOR</button>
  <button onclick="controlMotor(0)">TẮT MOTOR</button>

  <h2>Servo Control</h2>
  <p>Servo Angle: <span id="servoVal">90</span>°</p>
  <input type="range" min="0" max="180" value="90" id="slider" 
         oninput="setServo(this.value)">

  <h2>Phản hồi từ ESP32</h2>
  <div class="response" id="response">Chưa có phản hồi...</div>

  <script>
    function controlMotor(state) {
      fetch(`/motorControl?state=${state}`)
        .then(res => res.json())
        .then(data => {
          document.getElementById('response').innerText = 
            `Motor Response:\n` +
            `Status: ${data.status}\n` +
            `Message: ${data.message}\n` +
            `Motor: ${data.motor}`;
        })
        .catch(err => {
          document.getElementById('response').innerText = "Lỗi: " + err;
        });
    }

    function setServo(angle) {
      fetch(`/setServo?angle=${angle}`)
        .then(res => res.json())
        .then(data => {
          document.getElementById('response').innerText = 
            `Servo Response:\n` +
            `Status: ${data.status}\n` +
            `Message: ${data.message}`;
          document.getElementById('servoVal').innerText = angle;
        })
        .catch(err => {
          document.getElementById('response').innerText = "Lỗi khi điều khiển Servo";
        });
    }
  </script>
</body>
</html>
)rawliteral";

    request->send(200, "text/html", html);
  });

  server.on("/motorControl", HTTP_GET, [](AsyncWebServerRequest *request) {
    String message = "";
    
    if (request->hasParam("state")) {
      int state = request->getParam("state")->value().toInt();
      
      if (state == 1) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 180);     
        motorStatus = "RUNNING";
        pwmSpeed = 70;
        message = "Motor đã chạy";
      } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 0);
        motorStatus = "STOPPED";
        pwmSpeed = 0;
        message = "Motor đã dừng";
      }
    }

    String json = "{\"status\":\"success\",\"message\":\"" + message + "\",\"motor\":\"" + motorStatus + "\"}";
    request->send(200, "application/json", json);

    Serial.println("Motor: " + motorStatus);
  });

  server.on("/setServo", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("angle")) {
      servoAngle = request->getParam("angle")->value().toInt();
      servoAngle = constrain(servoAngle, 0, 180);
      myServo.write(servoAngle);

      String json = "{\"status\":\"success\",\"message\":\"Servo góc " + String(servoAngle) + "°\"}";
      request->send(200, "application/json", json);

      Serial.printf("Servo → %d°\n", servoAngle);
    } else {
      request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Thiếu tham số angle\"}");
    }
  });

  server.begin();
  Serial.println("Connected");
}

void loop() {
}