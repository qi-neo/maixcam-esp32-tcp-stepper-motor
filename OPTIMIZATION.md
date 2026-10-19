# 优化说明与深入研究思路（OPTIMIZATION）

> 本文件记录对 `maixcam-esp32-tcp-stepper-motor` 的优化改动，并给出后续深入研究的路线图。
> 配套文档：[protocol.md](protocol.md)（通信协议规范）。

## 一、原版存在的缺陷（已修复）

| # | 位置 | 问题 | 后果 |
|---|------|------|------|
| 1 | `esp32_tcp_stepper.ino` `loop()` | `dataProcessInterval=10000ms` 节流 + 仅在节流窗口内才清空 `receivedData` | 10 秒内多条指令只处理首条，其余被**丢弃**；窗口内第二条 `\n` 触发时 `receivedData` 未被清空，导致**多帧粘包**解析出垃圾值 |
| 2 | `processReceivedData()` | 先用像素算出 `targetPosition1/2`，紧接着被 `category` 预设位**整体覆盖** | 像素→位置映射是**死代码**，所谓"按坐标定位"从未生效，实际只按类别 4 个固定位运动 |
| 3 | `processReceivedData()` | `runToNewPosition` + `while(distanceToGo()!=0)` **阻塞**整个 loop | 运动期间无法接收新指令、无法急停，单任务卡死 |
| 4 | 运动参数 | `setMaxSpeed(CIRCLE_PULSES*1000)` = 320 万步/秒（远超物理能力） | AccelStepper 被设到不合理速度，易丢步、共振 |
| 5 | `maixcam_tcp_server.py` | `time.sleep(18)` 硬等待 + `conn.recv(1024)` **无超时** | 时序靠猜；一旦 ESP32 不回数据，服务器**永久阻塞**；README 已自述该问题 |
| 6 | 两端 | 仅检查"是否收到"，不解析内容；无 ACK 语义；IP/端口硬编码 | 无错误反馈、调试困难、部署僵化 |
| 7 | `loop()` 重连逻辑 | 只做 TCP 重连，未处理 Wi-Fi 掉线 | Wi-Fi 断开后 TCP 重连死循环、永不恢复 |

## 二、本次优化改动

### ESP32（`esp32_tcp_stepper.ino`）
- **非阻塞状态机**（`IDLE → MOVING_TARGET → DWELL → MOVING_HOME → ACK`）：用 `moveTo()` + `run()` 推进，loop 始终可响应；新增 `BUSY` 回执避免打断在途动作。
- **可靠行缓冲**：定长 `rxBuf` 按 `\n` 分帧，半包保留、超长整行丢弃，逐行**立即**处理，删除 10s 节流。
- **像素→位置真正生效**（PIXEL 模式）：`x,y` 线性映射到 `TRAVEL_CIRCLES` 圈行程；保留 PRESET 模式兼容旧教程。
- **运动参数合理化**：`MAX_SPEED = 3200*4` 步/秒、`ACCEL = 3200*8`，可按负载调参。
- **双层重连**：`ensureWiFi()` + `ensureTCP()`，均带 3~5s 退避；LED 常亮=已连、闪烁=未连。
- **结构化 + 带序号 ACK**：`ACK:STATUS:CAT:POS1:POS2:SEQ\n`，`STATUS ∈ {DONE, ERR, BUSY, ESTOP, RESUME}`。
- **严格解析**：`parseCommand` 要求字段数恰好匹配（4 个；启用校验和时 5 个），每个字段必须是十进制数字，拒绝尾随垃圾 / 空字段 / 多余逗号。
- **软件急停**：`ESTOP` 立即停止运动并关闭驱动器使能，`RESUME` 恢复接收指令（均回 ACK）。
- **软限位**：目标脉冲夹在 `[0, TRAVEL_CIRCLES*CIRCLE_PULSES]`，越界回 `ERR` 且不动。
- **可选校验和**：`USE_CHECKSUM` 宏，下行可带 `(x+y+cat+seq)%256`。
- **使能极性可配置**：`ENABLE_LEVEL` 适配不同驱动器。

### Maixcam（`maixcam_tcp_server.py`）
- **请求-应答握手**：每条指令后 `recv` 等待带序号的 `ACK`，受 `RECV_TIMEOUT` 保护，移除 `time.sleep(18)`。
- **序号匹配**：只接受 `SEQ` 与当前 seq 相同的回执，迟到/错位回执丢弃并记日志（修掉 ACK 错位）。
- **可选认证**：`AUTH_TOKEN` 非空时要求客户端首行 `AUTH <token>`，校验失败立即断开；默认空保持旧行为。
- **单客户端**：已有连接时拒绝新连接；连续 ACK 超时会断开死连接释放槽位。
- **急停接口**：`send_estop()` / `send_resume()`；关机或会话异常结束时尽量发一次 ESTOP。
- **行缓冲**：服务端也按 `\n` 分帧，处理半包/粘包。
- **检测桩 `generate_command()`**：明确标注接入 Maixcam 真实 YOLO 检测结果的位置（取置信度最高目标）。
- **优雅退出**：`SIGINT/SIGTERM` 关闭监听套接字；`SO_REUSEADDR` 已保留。

### 新增文档
- `protocol.md`：协议规范（帧格式、时序、控制模式、可靠性）。
- 本文件：缺陷清单、改动、研究思路。

## 三、后续可扩展（工程层面，短期可做）

1. **硬件急停与限位开关**：软件 `ESTOP` 命令已实现；物理急停按钮 / 限位开关还需在 `loop()` 中轮询（中断或 GPIO），触发时 `stepper.stop()` 并进入安全态。
2. **OTA 与日志**：接入 ArduinoOTA，便于远程升级；把关键状态经 WebSocket/串口推到上位机可视化。
3. **多目标调度**：Maixcam 端按优先级连续下发多条指令（如先近后远），ESP32 用指令队列缓存。
4. **参数下发**：允许 Maixcam 通过指令动态设置速度/加速度/行程，无需重烧固件。
5. **故障上报**：丢步检测（对比 `currentPosition` 与理论值）、过温/堵转反馈到 `ACK` 或独立告警帧。

## 四、深入研究思路（学习/进阶路线）

> 面向嵌入式控制 + 机器视觉方向，适合作为大创/毕设/面试深度素材。

1. **闭环控制与丢步检测**
   - 现状：用闭环步进驱动器，但 ESP32 只读目标、不读编码器。
   - 方向：接入编码器（ABZ 中断或 PCNT 外设）做位置闭环；研究 SimpleFOC / 伺服方案；实现丢步检测与自动回正。

2. **运动规划：梯形 vs S 曲线加减速**
   - AccelStepper 是梯形（jerk 不连续）。研究 jerk-limited S 曲线，减少机械冲击与共振；
   - 参考 grbl / Marlin 的 planner（前瞻 look-ahead、junction 速度约束），把多段轨迹合并平滑。

3. **FreeRTOS 多任务与双核**
   - 把"通信 / 运动控制 / 传感读取"拆成独立任务，用队列/信号量解耦；
   - 利用 ESP32 双核：`xTaskCreatePinnedToCore` 把运动控制钉在某一核，保证实时性；
   - 研究 `esp_task_wdt` 看门狗防止单任务卡死。

4. **视觉—运动时间同步与预测控制**
   - 研究检测周期（帧率）与运动时间的匹配；用速度前馈 + 轨迹插值让机械臂"跟随"移动目标，而非到点停。
   - 引入卡尔曼/α-β 滤波对检测坐标去抖、预测。

5. **通信协议升级**
   - 二进制协议（长度前缀 + CRC16）替代文本，省带宽、抗干扰；
   - 应用层 ACK/重传/序号（本版已实现带 8 位序号的 ACK，可升级到 16 位并加重传）；
   - **micro-ROS**：让 ESP32 成为 ROS2 节点，直接订阅目标 `geometry_msgs/Pose`，与 Maixcam/上位机统一在 ROS2 生态；
   - **ESP-NOW**：本地低延迟控制用 ESP-NOW，TCP 仅负责任务级调度。

6. **像素↔电机空间标定（手眼标定）**
   - 现状：线性映射假设视野与行程线性对应。
   - 方向：相机内参/外参、eye-to-hand 标定；透视变换把像素映射到底平面坐标，再做逆运动学得到各轴目标；
   - 这一步是"精准抓取"的关键，也是把玩具 demo 升级为可用系统的分水岭。

7. **抓取与状态机策略**
   - 完整状态机：`idle → approach → grab → lift → place → home`；
   - 多目标优先级调度、轨迹避障、力控抓取（电流/力矩反馈）；
   - 失败重试与异常恢复（抓空、滑落检测）。

8. **电源与抗干扰工程**
   - 步进大电流对 ESP32 的干扰、共地噪声、上电顺序、使能极性；
   - 隔离（光耦/磁耦）、独立 LDO、电源时序；EMC 基础。

9. **可观测性与调试体系**
   - 实时位置/电流/温度经 WebSocket 推到 Web 仪表盘；
   - 用 RMT/PCNT 精确脉冲计数校验；轨迹回放与离线分析。

## 五、验证清单（上线前）

- [ ] Arduino IDE 安装 `AccelStepper`，编译通过（本环境无 ESP32 工具链，未做硬件编译，需你在本地确认）。
- [ ] 烧录后串口监视器（115200）可见 `CMD` / `DONE` 日志。
- [ ] 断开 Maixcam 再连，ESP32 自动重连；断开 Wi-Fi 后自动恢复。
- [ ] Maixcam 端可见 `ACK:DONE:...`，超时场景打印 `ACK timeout` 不卡死。
- [ ] 确认 `ENABLE_LEVEL` 与你的驱动器极性一致（电机应能被驱动）。
