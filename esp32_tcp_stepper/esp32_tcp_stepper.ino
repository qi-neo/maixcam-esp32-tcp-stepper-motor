// ESP32 端（TCP 客户端）：接收 Maixcam 下发的 "x,y,category" 指令，驱动两台步进电机。
// 优化版特性（详见仓库 OPTIMIZATION.md / protocol.md）：
//   1) 非阻塞状态机运动控制 —— loop() 不再被 runToNewPosition 卡死，可随时接收新指令/急停
//   2) 可靠行缓冲 —— 按 '\n' 逐行解析，保留半包，逐行立即处理（修复原版 10s 节流丢帧/粘包 bug）
//   3) 像素→位置映射 —— 真实按检测坐标定位（修复原版"像素定位被类别预设覆盖"的死代码）
//   4) 双层重连 —— Wi-Fi 掉线重连 + TCP 断线重连，带退避
//   5) 结构化 ACK —— 回执 ACK:STATUS:CAT:POS1:POS2\n，便于 Maixcam 做请求-应答握手
//   6) 可配置 —— 控制模式(PIXEL/PRESET)、行程、运动参数、校验和均为顶部常量
//
// 接线与协议说明见 README.md / protocol.md。

#include <WiFi.h>
#include <WiFiClient.h>
#include <AccelStepper.h>
#include <string.h>
#include <stdlib.h>

// ===================== 用户配置（按需修改） =====================
const char* WIFI_SSID     = "205-03";
const char* WIFI_PASSWORD = "<YOUR_WIFI_PASSWORD>";   // 原文已脱敏，替换为你自己的网络
const char* HOST_IP       = "192.168.10.5";           // Maixcam TCP 服务器 IP
const uint16_t HOST_PORT  = 8080;

// 控制模式：
//   PIXEL  —— 按检测到的像素坐标线性映射到电机行程（推荐，符合"跟随目标"的本意）
//   PRESET —— 仅按类别映射到预设位（兼容旧教程的 4 个固定位）
#define CONTROL_MODE PIXEL

// 电机引脚（物理脚位保持与原教程一致）
#define ENA_PIN_1 27   // 电机 X 使能
#define STEP_PIN_1 25  // 电机 X 脉冲
#define DIR_PIN_1 26   // 电机 X 方向
#define ENA_PIN_2 17   // 电机 Y 使能
#define STEP_PIN_2 5   // 电机 Y 脉冲
#define DIR_PIN_2 18   // 电机 Y 方向

// 运动与映射参数
#define CIRCLE_PULSES   3200   // 电机转一圈的脉冲数
#define MAX_PIXEL_X     2560   // X 轴最大像素坐标
#define MAX_PIXEL_Y     2560   // Y 轴最大像素坐标
#define TRAVEL_CIRCLES  6      // 像素满量程对应的电机行程（圈）
#define MAX_SPEED       (CIRCLE_PULSES * 4)   // 最大速度（步/秒）
#define ACCEL           (CIRCLE_PULSES * 8)   // 加速度（步/秒^2）
#define DWELL_MS        1000   // 到位后停顿（毫秒），可用于"抓取"动作
#define LED_PIN         2

// 使能极性：多数闭环驱动器 ENA 低有效；本项目沿用原教程实测可用的电平（写 1 = 使能）。
// 若你的驱动器相反，把 ENABLE_LEVEL 改为 LOW 即可。
#define ENABLE_LEVEL HIGH

// 协议：是否启用校验和（下行第 4 字段 = (x+y+cat) mod 256）。
// 通信环境干扰大时建议开启；Maixcam 端需同步发送该校验字段。
#define USE_CHECKSUM false

// ===================== 全局对象 / 变量 =====================
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);  // 电机 X（对应像素 x）
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);  // 电机 Y（对应像素 y）
WiFiClient client;

// 行缓冲（按 '\n' 分帧，保留半包）
const size_t RX_BUF_SIZE = 128;
char rxBuf[RX_BUF_SIZE];
size_t rxLen = 0;

// 运动状态机
enum MotionState { IDLE, MOVING_TARGET, DWELL, MOVING_HOME };
MotionState state = IDLE;
unsigned long stateEnterTime = 0;

// PRESET 模式下的预设位：[category][motorX, motorY]（脉冲）。category 1..4。
const long presetPos[5][2] = {
  {0, 0},
  {0, 0},
  {0, 6L * CIRCLE_PULSES},
  {6L * CIRCLE_PULSES, 6L * CIRCLE_PULSES},
  {3L * CIRCLE_PULSES, 3L * CIRCLE_PULSES}
};

int pendingCategory = 0;
long targetX = 0, targetY = 0;

// 连接重试退避
unsigned long lastTcpTry = 0;
unsigned long lastWifiTry = 0;

// ===================== 辅助函数 =====================
void setEnable(bool on) {
  int lvl = on ? ENABLE_LEVEL : !ENABLE_LEVEL;
  digitalWrite(ENA_PIN_1, lvl);
  digitalWrite(ENA_PIN_2, lvl);
}

// 解析 "x,y,cat[,sum]"，返回 true 表示基本格式合法
bool parseCommand(const char* line, long& x, long& y, long& cat) {
  const char* p = line;
  if (*p == '\0') return false;
  x = atol(p);
  p = strchr(p, ','); if (!p) return false; p++;
  y = atol(p);
  p = strchr(p, ','); if (!p) return false; p++;
  cat = atol(p);
  return true;
}

void sendAck(const char* status, int cat, long posX, long posY) {
  if (!client.connected()) return;
  client.print("ACK:");
  client.print(status);
  client.print(":");
  client.print(cat);
  client.print(":");
  client.print(posX);
  client.print(":");
  client.print(posY);
  client.print("\n");
}

// 处理一行完整指令
void handleLine(const char* line) {
  long x, y, cat;
  if (!parseCommand(line, x, y, cat)) {
    sendAck("ERR", -1, stepperX.currentPosition(), stepperY.currentPosition());
    return;
  }

#if USE_CHECKSUM
  // 校验和第 4 字段（可选）
  const char* p = strrchr(line, ',');
  if (!p) { sendAck("ERR", cat, stepperX.currentPosition(), stepperY.currentPosition()); return; }
  long sum = atol(p + 1);
  if (((x + y + cat) % 256) != sum) {
    sendAck("ERR", cat, stepperX.currentPosition(), stepperY.currentPosition());
    return;
  }
#endif

  // 范围校验
  if (x < 0 || x > MAX_PIXEL_X || y < 0 || y > MAX_PIXEL_Y || cat < 1 || cat > 4) {
    sendAck("ERR", (int)cat, stepperX.currentPosition(), stepperY.currentPosition());
    return;
  }

  pendingCategory = (int)cat;

  if (CONTROL_MODE == PIXEL) {
    targetX = (long)((double)x / MAX_PIXEL_X * TRAVEL_CIRCLES * CIRCLE_PULSES);
    targetY = (long)((double)y / MAX_PIXEL_Y * TRAVEL_CIRCLES * CIRCLE_PULSES);
  } else {
    targetX = presetPos[cat][0];
    targetY = presetPos[cat][1];
  }

  // 仅在空闲时接管运动，避免打断正在进行的动作
  if (state == IDLE) {
    setEnable(true);
    stepperX.moveTo(targetX);
    stepperY.moveTo(targetY);
    state = MOVING_TARGET;
    stateEnterTime = millis();
    Serial.printf("CMD x=%ld y=%ld cat=%d -> targetX=%ld targetY=%ld\n",
                  x, y, cat, targetX, targetY);
  } else {
    sendAck("BUSY", (int)cat, stepperX.currentPosition(), stepperY.currentPosition());
  }
}

// 非阻塞状态机：每次 loop 推进一次，电机用 run() 连续步进
void runMotionFSM() {
  switch (state) {
    case IDLE:
      break;

    case MOVING_TARGET:
      stepperX.run();
      stepperY.run();
      if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {
        state = DWELL;
        stateEnterTime = millis();
      }
      break;

    case DWELL:
      if (millis() - stateEnterTime >= DWELL_MS) {
        stepperX.moveTo(0);
        stepperY.moveTo(0);
        state = MOVING_HOME;
      }
      break;

    case MOVING_HOME:
      stepperX.run();
      stepperY.run();
      if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {
        sendAck("DONE", pendingCategory, stepperX.currentPosition(), stepperY.currentPosition());
        state = IDLE;
        Serial.printf("DONE cat=%d posX=%ld posY=%ld\n",
                      pendingCategory, stepperX.currentPosition(), stepperY.currentPosition());
      }
      break;
  }
}

void ensureWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifiTry > 5000) {
      Serial.println("WiFi lost, reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      lastWifiTry = millis();
    }
  }
}

void ensureTCP() {
  if (!client.connected()) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // 闪烁表示未连接
    if (WiFi.status() == WL_CONNECTED && millis() - lastTcpTry > 3000) {
      Serial.printf("Connecting to %s:%d...\n", HOST_IP, HOST_PORT);
      if (client.connect(HOST_IP, HOST_PORT)) {
        Serial.println("TCP connected");
        digitalWrite(LED_PIN, HIGH);
      } else {
        Serial.println("TCP connect failed, retry later");
      }
      lastTcpTry = millis();
    }
  } else {
    digitalWrite(LED_PIN, HIGH);  // 常亮表示已连接
  }
}

// ===================== 初始化 =====================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(ENA_PIN_1, OUTPUT);
  pinMode(ENA_PIN_2, OUTPUT);
  setEnable(false);  // 上电先禁用，避免误触发脉冲

  Serial.begin(115200);
  delay(10);
  Serial.println("ESP32 stepper TCP client (optimized) starting...");

  // Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected, IP=" + WiFi.localIP().toString());

  // 电机参数
  stepperX.setMaxSpeed(MAX_SPEED);
  stepperX.setAcceleration(ACCEL);
  stepperY.setMaxSpeed(MAX_SPEED);
  stepperY.setAcceleration(ACCEL);
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);

  // 首次尝试建立 TCP（失败也无妨，ensureTCP 会持续重试）
  ensureTCP();
}

// ===================== 主循环 =====================
void loop() {
  ensureWiFi();
  ensureTCP();

  // 1) 接收并按行解析（非阻塞，逐行处理，保留半包）
  if (client.connected()) {
    while (client.available()) {
      char c = client.read();
      if (c == '\n') {
        rxBuf[rxLen] = '\0';
        if (rxLen > 0) handleLine(rxBuf);
        rxLen = 0;
      } else if (c != '\r') {  // 忽略 '\r'
        if (rxLen < RX_BUF_SIZE - 1) rxBuf[rxLen++] = c;
      }
    }
  }

  // 2) 推进运动状态机（非阻塞）
  runMotionFSM();

  // 3) 看门狗/心跳可在此扩展（如定时上报状态、急停按钮检测等）

  delay(1);
}
