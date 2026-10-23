# 硬件 / Hardware

本目录是 `maixcam-esp32-tcp-stepper-motor` 的硬件设计方案。目标是把原项目"面包板 + 杜邦线"的演示形态，升级成**可打样、可复现**的集成控制板，并作为开源硬件补充进仓库。

> 状态：**设计阶段（文档先行，KiCad/EasyEDA 工程待出）**。所有 BOM 与连接表均基于仓库现有固件 `esp32_tcp_stepper.ino` 的真实引脚与驱动器参数。

## 目录

| 路径 | 内容 |
|------|------|
| [`control-board/`](control-board/README.md) | 集成控制主板（ESP32 + 双步进驱动 + 电源 + IO）完整设计 |
| [`control-board/bom.md`](control-board/bom.md) | 主板物料清单（草案） |
| [`control-board/schematic.svg`](control-board/schematic.svg) | 主板功能框图 / 原理示意 |

## 为什么需要 PCB

原方案的痛点（详见仓库根 `OPTIMIZATION.md`）：

- 面包板 + 杜邦线接线脆弱、易松动，运输/演示时容易掉线；
- 大电流步进驱动对 ESP32 有共地噪声，影响 Wi-Fi 与稳定性；
- 驱动器（DM542 类）信号电平、使能极性需要可靠固定，跳线容易接错；
- 缺少反接保护、保险丝、去耦电容等基础防护。

一块集成主板可一次性解决上述问题，并把"教程仓库"升级为**开源硬件项目**。

## 设计工具与交付

- **原型**：[EasyEDA](https://easyeda.com)（浏览器、中文、LCSC 元件库、[JLCPCB](https://jlcpcb.com) 一键打样，5 片约 ¥20–50）。
- **归档**：最终使用 **KiCad** 工程提交到本目录（`*.kicad_pro` + `BOM.csv` + Gerber + 装配说明），保证 Git 友好、无账号锁定。
- 每个板子目录交付：`README.md`（设计说明）+ `bom.md` + `schematic.svg`（或可视图）+ 最终 `*.kicad_pro` / Gerber。

## 板子规划

1. **【必做】集成控制主板** —— 本仓库硬件核心，详见 `control-board/`。
2. **【可选】隔离限位板**：光耦隔离 endstop 输入，抗电机噪声。
3. **【可选】电源 / EMC 板**：反接保护 + 自恢复保险丝 + 共模扼流圈。
4. **【可选】编码器闭环板**：ABZ 编码器接 ESP32 PCNT，做丢步检测（呼应固件研究点）。
5. **【可选】相机转接 / 固定板**：Maixcam Pro 机械固定 + FFC。

> 优先做第 1 块主板；2–5 为按需扩展，不要一次性全做（过度工程）。
