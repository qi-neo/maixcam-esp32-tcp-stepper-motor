# 通信协议规范（ESP32 ↔ Maixcam）

> 对应优化后的 `esp32_tcp_stepper.ino` 与 `maixcam_tcp_server.py`。
> 旧教程版（固定 `data_set` + `time.sleep(18)`）已被本规范取代，详见 `OPTIMIZATION.md`。

## 1. 网络拓扑

| 角色 | 设备 | 模式 | 默认端口 |
|------|------|------|----------|
| 服务器 | Maixcam Pro | TCP server | 8080 |
| 客户端 | ESP32 | TCP client | — |

- ESP32 主动连接 Maixcam 的 IP（`HOST_IP`）。
- 断线后双方均自动重连：ESP32 端为 Wi-Fi + TCP 双层重连；Maixcam 端 `accept` 循环常驻。

## 2. 下行帧（Maixcam → ESP32）

文本行，以 `\n` 结尾（允许 `\r\n`）：

```
x,y,category[,checksum]\n
```

| 字段 | 类型 | 范围 | 说明 |
|------|------|------|------|
| x | int | [0, MAX_PIXEL_X] | 像素 X 坐标 |
| y | int | [0, MAX_PIXEL_Y] | 像素 Y 坐标 |
| category | int | 1..4 | 动作/类别 |
| checksum | int（可选） | — | 仅 `USE_CHECKSUM=true` 时出现，`= (x+y+category) mod 256` |

分帧规则：以 `\n` 为帧尾；半包（未收到 `\n`）在缓冲区保留，待后续字节补齐；超长行（>127B）截断丢弃。

## 3. 上行帧（ESP32 → Maixcam）

结构化回执，以 `\n` 结尾：

```
ACK:STATUS:CATEGORY:POS1:POS2\n
```

| 字段 | 说明 |
|------|------|
| STATUS | `DONE`（动作完成并归位）/ `ERR`（解析或范围错误）/ `BUSY`（正在执行上一条，已忽略本条） |
| CATEGORY | 本条指令的类别 |
| POS1 | 电机 X 当前位置（脉冲） |
| POS2 | 电机 Y 当前位置（脉冲） |

## 4. 时序（请求-应答握手）

```
Maixcam                          ESP32
  |--- "x,y,cat\n" ------------>|
  |                              | 解析 → 运动到目标 → 停顿(DWELL) → 归零
  |<-- "ACK:DONE:cat:px:py\n" --|
  |  (等待 ACK 超时 RECV_TIMEOUT)|  (状态机回到 IDLE)
  |--- 下一条指令 --------------->|
```

- Maixcam 下发一条后**等待 ACK**（受 `RECV_TIMEOUT` 保护，不会永久阻塞）。
- ESP32 仅在 `IDLE` 时接收新指令；若正在运动，回 `ACK:BUSY` 并忽略本条（不会打断当前动作）。
- 到位后统一归零，保证下一条指令从零点开始。

## 5. 控制模式

- **PIXEL**：`x,y` 线性映射到电机行程，像素满量程 → `TRAVEL_CIRCLES` 圈。实现"机械臂跟随目标"。
- **PRESET**：忽略像素，仅按 `category` 映射到预设位（兼容旧教程的 4 个固定位）。

切换方式：修改 `.ino` 顶部 `#define CONTROL_MODE PIXEL` 为 `PRESET`。

## 6. 抗干扰与可靠性

- 行缓冲分帧，天然容忍 TCP 粘包/半包。
- 可选校验和（`USE_CHECKSUM`），抵御单字节翻转。
- 断线自动重连（TCP 与 Wi-Fi 两层）。
- ACK 反馈电机实际位置，便于上位机校验执行结果。

## 7. 扩展方向

- 二进制协议（长度前缀 + CRC16）进一步省带宽、抗干扰。
- 多目标时由 Maixcam 端做优先级调度，连续下发多条。
- 改用 micro-ROS，让 ESP32 成为 ROS2 节点，直接订阅目标话题（见 `OPTIMIZATION.md` 研究思路）。
