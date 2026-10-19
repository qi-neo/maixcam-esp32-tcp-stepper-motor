// ESP32 端（TCP 客户端）：接收 Maixcam 下发的 "x,y,category,seq" 指令，驱动两台步进电机。
// 优化版特性（详见仓库 OPTIMIZATION.md / protocol.md）：
//   1) 非阻塞状态机运动控制 —— loop() 不再被 runToNewPosition 卡死，可随时接收新指令/急停
//   2) 可靠行缓冲 —— 按 '\n' 逐行解析，保留半包，逐行立即处理（修复原版 10s 节流丢帧/粘包 bug）
//   3) 像素→位置映射 —— 真实按检测坐标定位（修复原版"像素定位被类别预设覆盖"的死代码）
//   4) 双层重连 —— Wi-Fi 掉线重连 + TCP 断线重连，带退避
//   5) 结构化 + 带序号 ACK —— 回执 ACK:STATUS:CAT:POS1:POS2:SEQ\n；
//      SEQ 原样回带下行的序号，Maixcam 只认自己当前序号，迟到 ACK 会被丢弃，
//      从此不会出现"上一条的回执被当成本条的回执"的永久错位。
//   6) 严格解析 —— 下行字段数必须恰好匹配（4 个；启用校验和时 5 个），
//      每个字段必须全是十进制数字；任何尾随垃圾 / 多余逗号 / 空字段整行拒绝。
//   7) 软件急停 —— ESTOP 立即停止运动并关闭驱动器使能；RESUME 恢复接收指令。
//   8) 软限位 —— 目标脉冲被夹在 [0, TRAVEL_CIRCLES*CIRCLE_PULSES]；越界回 ERR 且不动。
//   9) 可配置 —— 控制模式(PIXEL/PRESET)、行程、运动参数、校验和均为顶部常量
//
// 接线与协议说明见 README.md / protocol.md。

#include <WiFi.h>
#include <WiFiClient.h>
#include <AccelStepper.h>
#include <limits.h>
#include <string.h>
#include <stdlib.h>

// ===================== 用户配置（按需修改） =====================
const char* WIFI_SSID     = "205-03";
const char* WIFI_PASSWORD = "<YOUR_WIFI_PASSWORD>";   // 原文已脱敏，替换为你自己的网络
const char* HOST_IP       = "192.168.10.5";           // Maixcam TCP 服务器 IP
const uint16_t HOST_PORT  = 8080;

// 可选认证：必须与 Maixcam 端 AUTH_TOKEN 完全一致。
// 非空时，TCP 连上后的首行发送 "AUTH <token>\n"；服务器校验失败会立即断开。
// 默认空 = 不认证（保持旧用法），此时服务器也不要求 AUTH 行。
const char* AUTH_TOKEN    = "";

// 控制模式：
//   MODE_PIXEL  —— 按检测到的像素坐标线性映射到电机行程（推荐，符合"跟随目标"的本意）
//   MODE_PRESET —— 仅按类别映射到预设位（兼容旧教程的 4 个固定位）
//
// 注意：这里必须给模式一个"有定义的值"。原先写的是 `#define CONTROL_MODE PIXEL`
// 并直接用裸标识符 PIXEL 做比较，而 PIXEL 全文从未定义，预处理后
// `if (CONTROL_MODE == PIXEL)` 变成 `if (PIXEL == PIXEL)`，编译直接失败。
#define MODE_PIXEL  1
#define MODE_PRESET 2
#define CONTROL_MODE MODE_PIXEL

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

// 协议：是否启用校验和。下行帧格式（严格解析，字段数必须完全匹配）：
//   USE_CHECKSUM=false：x,y,cat,seq\n          —— 恰好 4 个字段
//   USE_CHECKSUM=true ：x,y,cat,seq,sum\n      —— 恰好 5 个字段，sum=(x+y+cat+seq) mod 256
// 每个字段都必须是十进制整数；整行多一个逗号、少一个字段、字段里混有非数字
// 字符（如 "12,34,2junk"）都会被拒绝，不再像 atol 那样静默截断。
#define USE_CHECKSUM false

// 软限位：单个轴允许的最大脉冲位置（默认 6 圈 * 3200 = 19200）。
// 目标位置必须落在 [0, SOFT_MAX_POS]；越界回 ERR 且电机不动。
const long SOFT_MAX_POS = (long)TRAVEL_CIRCLES * CIRCLE_PULSES;

// ACK 里表示"这不是某条下行指令的回执"的保留序号（ESTOP / RESUME 使用）。
const long CONTROL_ACK_SEQ = -1;

// ===================== 全局对象 / 变量 =====================
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);  // 电机 X（对应像素 x）
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);  // 电机 Y（对应像素 y）
WiFiClient client;

// 行缓冲（按 '\n' 分帧，保留半包）
const size_t RX_BUF_SIZE = 128;
char rxBuf[RX_BUF_SIZE];
size_t rxLen = 0;
// 单行超长标记：超长行必须整行丢弃。若只截断前缀就交给 parseCommand，
// 截断后的 "12,34," 仍可能被解析成合法指令，真的驱动电机。
bool rxOverflow = false;

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
long pendingSeq = CONTROL_ACK_SEQ;  // 当前这条指令的序号，DONE 回执原样带回
long targetX = 0, targetY = 0;

// 软件急停状态：true 时拒绝一切运动指令，直到收到 RESUME。
bool estopActive = false;

// 连接重试退避
unsigned long lastTcpTry = 0;
unsigned long lastWifiTry = 0;

// ===================== 辅助函数 =====================
void setEnable(bool on) {
  int lvl = on ? ENABLE_LEVEL : !ENABLE_LEVEL;
  digitalWrite(ENA_PIN_1, lvl);
  digitalWrite(ENA_PIN_2, lvl);
}

// 严格解析一个十进制整数字段：只允许可选的 '-' 后跟至少一位数字。
// 返回时 p 指向字段后的第一个字符（',' 或 '\0'）。
bool parseLongField(const char*& p, long& out) {
  bool negative = false;
  if (*p == '-') {
    negative = true;
    p++;
  }
  if (*p < '0' || *p > '9') return false;  // 空字段 / 非数字
  unsigned long value = 0;
  while (*p >= '0' && *p <= '9') {
    unsigned long digit = (unsigned long)(*p - '0');
    if (value > (unsigned long)(LONG_MAX - digit) / 10UL) return false;  // 溢出
    value = value * 10UL + digit;
    p++;
  }
  out = negative ? -(long)value : (long)value;
  return true;
}

struct Command {
  long x;
  long y;
  long cat;
  long seq;
  long sum;  // 仅 USE_CHECKSUM=true 时有效
};

// 解析下行帧：x,y,cat,seq[,sum]。
// 要求字段数恰好等于 expectedFields，每个字段都是纯数字，行尾不能有垃圾。
bool parseCommand(const char* line, Command& cmd) {
  const int expectedFields = USE_CHECKSUM ? 5 : 4;
  long values[5];
  const char* p = line;
  int count = 0;
  while (true) {
    if (count >= expectedFields) return false;  // 字段过多
    if (!parseLongField(p, values[count])) return false;
    count++;
    if (*p == '\0') break;
    if (*p != ',') return false;   // 字段之间只允许单个逗号
    p++;
    if (*p == '\0') return false;  // 尾随逗号 / 空字段
  }
  if (count != expectedFields) return false;  // 字段过少

  cmd.x = values[0];
  cmd.y = values[1];
  cmd.cat = values[2];
  cmd.seq = values[3];
  cmd.sum = USE_CHECKSUM ? values[4] : 0;
  return true;
}

// 把目标脉冲夹到软限位 [0, SOFT_MAX_POS]；返回 false 表示原始目标越界。
// 上层对越界直接回 ERR 且不运动——不静默夹取，避免上位机"以为到位"。
bool clampToSoftLimit(long& pos) {
  long clamped = pos;
  if (clamped < 0) clamped = 0;
  if (clamped > SOFT_MAX_POS) clamped = SOFT_MAX_POS;
  if (clamped != pos) return false;
  pos = clamped;
  return true;
}

// 结构化回执：ACK:STATUS:CAT:POS1:POS2:SEQ
void sendAck(const char* status, int cat, long posX, long posY, long seq) {
  if (!client.connected()) return;
  client.print("ACK:");
  client.print(status);
  client.print(":");
  client.print(cat);
  client.print(":");
  client.print(posX);
  client.print(":");
  client.print(posY);
  client.print(":");
  client.print(seq);
  client.print("\n");
}

// 软件急停：立即停止运动并关闭驱动器使能。
// 用 setCurrentPosition(当前位置) 把速度和目标同时清零（AccelStepper 内部会
// 把 target 也设成该位置），不做减速、不等 DWELL，确保 ESTOP 立刻生效。
void doEstop() {
  stepperX.setCurrentPosition(stepperX.currentPosition());
  stepperY.setCurrentPosition(stepperY.currentPosition());
  setEnable(false);  // 关使能，电机不再有保持力矩，也不会再收到脉冲
  state = IDLE;
  estopActive = true;
  pendingCategory = 0;
  pendingSeq = CONTROL_ACK_SEQ;
  sendAck("ESTOP", 0, stepperX.currentPosition(), stepperY.currentPosition(), CONTROL_ACK_SEQ);
  Serial.println("ESTOP: motion halted, driver outputs disabled");
}

// 解除急停：恢复接收运动指令；驱动器使能保持关闭，等下一条有效指令再开启。
void doResume() {
  estopActive = false;
  state = IDLE;
  pendingCategory = 0;
  pendingSeq = CONTROL_ACK_SEQ;
  sendAck("RESUME", 0, stepperX.currentPosition(), stepperY.currentPosition(), CONTROL_ACK_SEQ);
  Serial.println("RESUME: ready to accept commands");
}

// 处理一行完整指令
void handleLine(const char* line) {
  // 控制命令不带序号，必须是整行精确匹配
  if (strcmp(line, "ESTOP") == 0) {
    doEstop();
    return;
  }
  if (strcmp(line, "RESUME") == 0) {
    doResume();
    return;
  }

  Command cmd;
  if (!parseCommand(line, cmd)) {
    // 解析失败的帧无法确定序号，只能回 seq=-1；Maixcam 会把它当错位回执丢弃。
    sendAck("ERR", -1, stepperX.currentPosition(), stepperY.currentPosition(), CONTROL_ACK_SEQ);
    Serial.printf("ERR: malformed frame: %s\n", line);
    return;
  }

  const long seq = cmd.seq;
  const int cat = (int)cmd.cat;

  // 急停期间拒绝一切运动指令（包括看起来合法的），必须 RESUME 才恢复
  if (estopActive) {
    sendAck("ERR", cat, stepperX.currentPosition(), stepperY.currentPosition(), seq);
    Serial.printf("ERR: command rejected, ESTOP active (seq=%ld)\n", seq);
    return;
  }

#if USE_CHECKSUM
  // 校验和第 5 字段，覆盖 x+y+cat+seq
  if (((cmd.x + cmd.y + cmd.cat + cmd.seq) % 256) != cmd.sum) {
    sendAck("ERR", cat, stepperX.currentPosition(), stepperY.currentPosition(), seq);
    Serial.printf("ERR: checksum mismatch (seq=%ld)\n", seq);
    return;
  }
#endif

  // 范围校验：像素坐标、类别、序号（seq 0..255 与 Maixcam 端回绕范围一致）
  if (cmd.x < 0 || cmd.x > MAX_PIXEL_X || cmd.y < 0 || cmd.y > MAX_PIXEL_Y ||
      cmd.cat < 1 || cmd.cat > 4 || cmd.seq < 0 || cmd.seq > 255) {
    sendAck("ERR", cat, stepperX.currentPosition(), stepperY.currentPosition(), seq);
    Serial.printf("ERR: out of range x=%ld y=%ld cat=%ld seq=%ld\n",
                  cmd.x, cmd.y, cmd.cat, cmd.seq);
    return;
  }

  long nextX;
  long nextY;
  if (CONTROL_MODE == MODE_PIXEL) {
    nextX = (long)((double)cmd.x / MAX_PIXEL_X * TRAVEL_CIRCLES * CIRCLE_PULSES);
    nextY = (long)((double)cmd.y / MAX_PIXEL_Y * TRAVEL_CIRCLES * CIRCLE_PULSES);
  } else {
    nextX = presetPos[cat][0];
    nextY = presetPos[cat][1];
  }

  // 软限位：越界回 ERR 且不运动（映射结果理论上不会越界，这里是最后一道保险）
  if (!clampToSoftLimit(nextX) || !clampToSoftLimit(nextY)) {
    sendAck("ERR", cat, stepperX.currentPosition(), stepperY.currentPosition(), seq);
    Serial.printf("ERR: target outside soft limit (seq=%ld)\n", seq);
    return;
  }

  // 仅在空闲时接管运动，避免打断正在进行的动作
  if (state == IDLE) {
    targetX = nextX;
    targetY = nextY;
    pendingCategory = cat;
    pendingSeq = seq;
    setEnable(true);
    stepperX.moveTo(targetX);
    stepperY.moveTo(targetY);
    state = MOVING_TARGET;
    stateEnterTime = millis();
    Serial.printf("CMD x=%ld y=%ld cat=%d seq=%ld -> targetX=%ld targetY=%ld\n",
                  cmd.x, cmd.y, cat, seq, targetX, targetY);
  } else {
    sendAck("BUSY", cat, stepperX.currentPosition(), stepperY.currentPosition(), seq);
    Serial.printf("BUSY: moving, ignored cat=%d seq=%ld\n", cat, seq);
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
        sendAck("DONE", pendingCategory, stepperX.currentPosition(), stepperY.currentPosition(),
                pendingSeq);
        state = IDLE;
        Serial.printf("DONE cat=%d seq=%ld posX=%ld posY=%ld\n",
                      pendingCategory, pendingSeq, stepperX.currentPosition(),
                      stepperY.currentPosition());
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
        // 可选认证：服务器 AUTH_TOKEN 非空时，首行必须是 AUTH <token>
        if (AUTH_TOKEN[0] != '\0') {
          client.print("AUTH ");
          client.print(AUTH_TOKEN);
          client.print("\n");
          Serial.println("AUTH line sent");
        }
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
  // 串口日志：显式构造 String 再拼接，避免 "字面量 + String" 在部分核心上
  // 找不到匹配的 operator+ 而编译失败
  Serial.println(String("\nWiFi connected, IP=") + WiFi.localIP().toString());

  // 电机参数
  stepperX.setMaxSpeed(MAX_SPEED);
  stepperX.setAcceleration(ACCEL);
  stepperY.setMaxSpeed(MAX_SPEED);
  stepperY.setAcceleration(ACCEL);
  // 驱动器（DM542 一类）要求 STEP 脉宽 ≥2.5 µs、DIR 建立时间 ≥5 µs；
  // AccelStepper 默认脉宽在部分平台只有 1 µs 量级，显式抬到 3 µs 更稳。
  stepperX.setMinPulseWidth(3);
  stepperY.setMinPulseWidth(3);
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
        if (rxOverflow) {
          // 整行丢弃并回 ERR：截断的前缀不可信，宁可不动（无法确定序号，回 -1）
          sendAck("ERR", -1, stepperX.currentPosition(), stepperY.currentPosition(),
                  CONTROL_ACK_SEQ);
          rxOverflow = false;
        } else {
          rxBuf[rxLen] = '\0';
          if (rxLen > 0) handleLine(rxBuf);
        }
        rxLen = 0;
      } else if (c != '\r') {  // 忽略 '\r'
        if (rxLen < RX_BUF_SIZE - 1) {
          rxBuf[rxLen++] = c;
        } else {
          rxOverflow = true;
        }
      }
    }
  }

  // 2) 推进运动状态机（非阻塞）
  runMotionFSM();

  // 3) 看门狗/心跳可在此扩展（如定时上报状态、急停按钮检测等）

  delay(1);
}
