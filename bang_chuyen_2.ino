#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>

// WiFi credentials
const char* ssid = "DUY";
const char* password = "12345678";

// Định nghĩa chân
const int IN1 = 26;
const int IN2 = 27;
const int ENA = 25;
const int SERVO_PIN = 13;
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;

LiquidCrystal_I2C lcd(0x27, 16, 2); 


Servo myServo;

WebServer server(80);


String motorStatus = "STOPPED";
String conveyorStatus = "STOPPED";
int servoAngle = 90;
String detectionStatus = "No detect";
int pwmSpeed = 0;

// Biến cho hiển thị luân phiên
unsigned long lastDisplayToggle = 0;
const long displayInterval = 2000; // Chuyển đổi mỗi 2 giây
bool displayScreen1 = true;

void setup() {
  Serial.begin(115200);

  // Khởi tạo chân motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL);

  // Khởi tạo servo
  myServo.attach(SERVO_PIN);
  myServo.write(servoAngle);

  lcd.begin(21, 22); // SDA = 21, SCL = 22
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Starting...");
  delay(1000);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());

  // Hiển thị IP trên LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi Connected");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  delay(2000);

  // Thiết lập các endpoint HTTP
  server.on("/motorControl", handleMotorControl);
  server.on("/setServo", handleServoControl);
  server.on("/setWrongHole", handleWrongHole);
  server.on("/setScanStatus", handleScanStatus);
  server.begin();
}

void loop() {
  server.handleClient();
  updateLCD();
}

void handleMotorControl() {
  String signal = server.arg("signal");
  if (signal == "HIGH") {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM_CHANNEL, 128);  // Tốc độ 50%
    motorStatus = "RUNNING";
    conveyorStatus = "RUNNING";
    pwmSpeed = 50;
  } else if (signal == "LOW") {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM_CHANNEL, 0);
    motorStatus = "STOPPED";
    conveyorStatus = "STOPPED";
    pwmSpeed = 0;
  }
  server.send(200, "text/plain", "OK");
}

void handleServoControl() {
  String angleStr = server.arg("angle");
  servoAngle = angleStr.toInt();
  myServo.write(servoAngle);
  server.send(200, "text/plain", "OK");
}

void handleWrongHole() {
  String products = server.arg("products");
  detectionStatus = products == "none" ? "No detect" : String("Wrong:") + products;
  server.send(200, "text/plain", "OK");
}

void handleScanStatus() {
  float scanTime = server.arg("time").toFloat();
  conveyorStatus = "SCANNING";
  detectionStatus = String("Scan:") + String(scanTime, 1) + "s";
  server.send(200, "text/plain", "OK");
}

void updateLCD() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastDisplayToggle >= displayInterval) {
    displayScreen1 = !displayScreen1;
    lastDisplayToggle = currentMillis;
    lcd.clear();
  }

  if (displayScreen1) {
    lcd.setCursor(0, 0);
    String conveyorText = "Conv:" + conveyorStatus;
    if (conveyorText.length() > 16) conveyorText = conveyorText.substring(0, 16);
    lcd.print(conveyorText);
    
    lcd.setCursor(0, 1);
    lcd.print("Speed:");
    lcd.print(pwmSpeed);
    lcd.print("%");
  } else {
 
    lcd.setCursor(0, 0);
    lcd.print("Servo:");
    lcd.print(servoAngle);
    lcd.print(" deg");
    
    lcd.setCursor(0, 1);
    String detectText = detectionStatus;
    if (detectText.length() > 16) detectText = detectText.substring(0, 16);
    lcd.print(detectText);
  }
}