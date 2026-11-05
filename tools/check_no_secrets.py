#!/usr/bin/env python3
"""凭据与配置外置检查（零第三方依赖）。

- 固件不得再内置真实 SSID / 密码 / token，必须 include secrets.h
- 服务端 token 必须来自环境变量
- secrets.h 不得进入版本库，且必须被 .gitignore 忽略
- P2-8：全仓（含 README 的迁移示例）不得再出现真实 SSID / 学号 / 个人邮箱

策略见主仓 docs/PRIVACY.md；本脚本自身是检测器，含这些字面量属白名单。

运行：python3 tools/check_no_secrets.py
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
errors = []

ino_path = ROOT / "esp32_tcp_stepper/esp32_tcp_stepper.ino"
py_path = ROOT / "maixcam_tcp_server.py"
ino = ino_path.read_text(encoding="utf-8")
py = py_path.read_text(encoding="utf-8")

for name, text in (("esp32_tcp_stepper.ino", ino), ("maixcam_tcp_server.py", py)):
    if re.search(r'(?:const\s+char\*|#define)\s*(?:WIFI_SSID|WIFI_PASSWORD|AUTH_TOKEN)\s*(?:=|\s)\s*"', text):
        errors.append(f"{name} 仍在源码里定义 Wi-Fi / token 常量，请改用 secrets.h 或环境变量")
    if "205-03" in text:
        errors.append(f"{name} 出现真实网络名 205-03，必须移除")
    if "1794194885" in text:
        errors.append(f"{name} 出现个人邮箱，必须移除")

if '#include "secrets.h"' not in ino:
    errors.append("固件未 include secrets.h")
if "WIFI_BOOT_TIMEOUT_MS" not in ino:
    errors.append("固件 setup() 缺少 Wi-Fi 有界等待，凭据填错会卡死")
if 'os.environ.get("MAIXCAM_AUTH_TOKEN"' not in py:
    errors.append("服务端 AUTH_TOKEN 未从环境变量读取")
if "MAX_LINE_BYTES" not in py:
    errors.append("服务端 read_line() 缺少行长度上限")

for candidate in (ROOT / "secrets.h", ROOT / "esp32_tcp_stepper/secrets.h"):
    if candidate.exists():
        errors.append(f"{candidate.relative_to(ROOT)} 是本地凭据文件，不应提交")

gitignore = (ROOT / ".gitignore").read_text(encoding="utf-8")
if "secrets.h" not in gitignore:
    errors.append(".gitignore 未忽略 secrets.h")

# P2-8：全仓扫描。早期 CSDN 迁移文档的示例代码里残留过真实 SSID，
# 只查固件/服务端会漏掉 README；这里把边界扩到所有文本文件。
SKIP_DIRS = {".git", "__pycache__", "build", "install", ".vscode", ".pio"}
FORBIDDEN = ("205-03", "182311123", "1794194885")
for path in sorted(ROOT.rglob("*")):
    if not path.is_file():
        continue
    rel = path.relative_to(ROOT).as_posix()
    if any(part in SKIP_DIRS for part in path.relative_to(ROOT).parts):
        continue
    if rel == "tools/check_no_secrets.py":
        continue  # 检测器自身
    try:
        body = path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        continue
    for token in FORBIDDEN:
        if token in body:
            errors.append(f"{rel} 出现隐私字面量 {token}，请按主仓 docs/PRIVACY.md 脱敏")
            break

if errors:
    for err in errors:
        print(f"[check_no_secrets] FAIL: {err}")
    sys.exit(1)
print("[check_no_secrets] OK：凭据全部外置，固件含 Wi-Fi 有界等待，服务端含行长度上限")
