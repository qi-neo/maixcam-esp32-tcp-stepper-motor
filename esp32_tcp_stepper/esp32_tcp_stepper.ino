// ESP32 端（TCP 客户端）：接收 Maixcam 下发的 "像素X,像素Y,类别"，驱动两台步进电机。
// 说明与接线见仓库 README。

#include <WiFi.h>
#include <WiFiClient.h>
#include <AccelStepper.h>

// 电机引脚
#define En_Pin2     17   // 电机1使能
#define Stp_Pin2    5    // 电机1步进脉冲
#define Dir_Pin2    18   // 电机1方向
#define En_Pin1     27   // 电机2使能
#define Stp_Pin1    25   // 电机2步进脉冲
#define Dir_Pin1    26   // 电机2方向

#define CIRCLE_Puls 3200   // 电机转一圈的脉冲数
#define MAX_PIXEL_X 2560   // X 轴最大像素坐标
#define MAX_PIXEL_Y 2560   // Y 轴最大像素坐标
#define LED         2

AccelStepper stepper1(AccelStepper::DRIVER, Stp_Pin1, Dir_Pin1);
AccelStepper stepper2(AccelStepper::DRIVER, Stp_Pin2, Dir_Pin2);

long targetPosition1 = 0;
long targetPosition2 = 0;
int category = 1;
String receivedData = "";
unsigned long lastDataProcessTime = 0;
const unsigned long dataProcessInterval = 10000;  // 数据处理间隔 (ms)

// 改成自己的网络与 Maixcam 的 IP
const char* ssid = "205-03";
const char* password = "<YOUR_WIFI_PASSWORD>";
const char* targetIP = "192.168.10.5";
const uint16_t targetPort = 8080;

WiFiClient client;

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  delay(200);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);

  Serial.begin(115200);
  delay(10);

  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());

  pinMode(En_Pin1, OUTPUT);
  pinMode(En_Pin2, OUTPUT);
  digitalWrite(En_Pin1, 1);
  digitalWrite(En_Pin2, 1);

  stepper1.setMaxSpeed(CIRCLE_Puls * 1000);
  stepper1.setSpeed(CIRCLE_Puls * 1.0);
  stepper1.setAcceleration(CIRCLE_Puls * 6);
  stepper2.setMaxSpeed(CIRCLE_Puls * 1000);
  stepper2.setSpeed(CIRCLE_Puls * 1.0);
  stepper2.setAcceleration(CIRCLE_Puls * 6);

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);

  delay(2000);

  Serial.print("Connecting to ");
  Serial.print(targetIP);
  Serial.print(":");
  Serial.println(targetPort);
  while (!client.connect(targetIP, targetPort)) {
    Serial.println("Connection to host failed");
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to host");
  digitalWrite(LED, HIGH);
}

void loop() {
  if (client.connected()) {
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
      receivedData += c;
      if (c == '\n') {
        unsigned long currentTime = millis();
        if (currentTime - lastDataProcessTime >= dataProcessInterval) {
          processReceivedData(receivedData);
          receivedData = "";
          lastDataProcessTime = currentTime;
        }
      }
    }
    delay(100);
  } else {
    Serial.println("Disconnected from host, trying to reconnect...");
    while (!client.connect(targetIP, targetPort)) {
      digitalWrite(LED, HIGH);
      delay(2000);
      digitalWrite(LED, LOW);
      delay(2000);
      Serial.print(".");
    }
    Serial.println("Reconnected to host");
    digitalWrite(LED, HIGH);
  }
}

void processReceivedData(String data) {
  String parts[3];
  int index = 0;
  int start = 0;
  for (int i = 0; i < data.length(); i++) {
    if (data.charAt(i) == ',') {
      parts[index] = data.substring(start, i);
      index++;
      start = i + 1;
    }
  }
  parts[index] = data.substring(start);

  int pixelValue_X = parts[0].toInt();
  int pixelValue_Y = parts[1].toInt();
  int receivedCategory = parts[2].toInt();

  if (pixelValue_X >= 0 && pixelValue_X <= MAX_PIXEL_X &&
      pixelValue_Y >= 0 && pixelValue_Y <= MAX_PIXEL_Y &&
      (receivedCategory == 1 || receivedCategory == 2 || receivedCategory == 3 || receivedCategory == 4)) {
    float circles_X = pixelValue_X / (float)MAX_PIXEL_X * 6;
    float circles_Y = pixelValue_Y / (float)MAX_PIXEL_Y * 6;
    long targetPosition1 = (long)(circles_X * CIRCLE_Puls);
    long targetPosition2 = (long)(circles_Y * CIRCLE_Puls);
    int category = receivedCategory;

    stepper2.runToNewPosition(targetPosition1);
    stepper1.runToNewPosition(targetPosition2);
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0);

    Serial.print("1# Motor Current Position (pulses): ");
    Serial.println(stepper1.currentPosition());
    Serial.print("2# Motor Current Position (pulses): ");
    Serial.println(stepper2.currentPosition());

    delay(1000);
    if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) {
      if (receivedCategory == 1) {
        targetPosition1 = 0;
        targetPosition2 = 0;
        stepper1.runToNewPosition(targetPosition1);
        stepper2.runToNewPosition(targetPosition2);
        Serial.println("Both Motors moved to 1 position.");
      } else if (receivedCategory == 2) {
        targetPosition1 = 0;
        targetPosition2 = (long)(6 * CIRCLE_Puls);
        stepper1.runToNewPosition(targetPosition1);
        stepper2.runToNewPosition(targetPosition2);
        Serial.println("Both Motors moved to 2 position.");
      } else if (receivedCategory == 3) {
        targetPosition1 = (long)(6 * CIRCLE_Puls);
        targetPosition2 = (long)(6 * CIRCLE_Puls);
        stepper1.runToNewPosition(targetPosition1);
        stepper2.runToNewPosition(targetPosition2);
        Serial.println("Both Motors moved to 3 position.");
      } else if (receivedCategory == 4) {
        targetPosition1 = (long)(3 * CIRCLE_Puls);
        targetPosition2 = (long)(3 * CIRCLE_Puls);
        stepper1.runToNewPosition(targetPosition1);
        stepper2.runToNewPosition(targetPosition2);
        Serial.println("Both Motors moved to 4 position.");
      }
      stepper1.runToNewPosition(0);
      stepper2.runToNewPosition(0);
      Serial.println("Both Motors moved back to zero position.");
    }
    client.println("Received!");
  }
}
