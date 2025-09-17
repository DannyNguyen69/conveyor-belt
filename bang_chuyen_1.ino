#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
const char* ssid = "DUY";
const char* password = "12345678";

const int IN1 = 26;
const int IN2 = 27;
const int ENA = 25;
const int SERVO_PIN = 13;
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;

LiquidCrystal_I2C lcd(0x27, 16, 2); 
Servo myServo;

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
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL);

  myServo.attach(SERVO_PIN);
  myServo.write(servoAngle);

  Wire.begin(21, 22);
  lcd.init();
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

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi Connected");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  delay(2000);
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
  updateLCD();   
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
  }
}
