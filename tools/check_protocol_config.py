#!/usr/bin/env python3
# 比对协议两端（Maixcam Python 端 / ESP32 .ino 端）必须一致的配置默认值（P1-8）。
#
# 背景：下行帧格式由 USE_CHECKSUM、MAX_PIXEL_X/Y、SEQ_MOD 等常量决定。两端
# 任一默认值不一致，就会出现"Python 发 5 字段、ESP32 按 4 字段解析"或"坐标范围
# 对不上"之类的静默错乱，且硬件没接时根本发现不了。P0-6 已把 ESP32 端凭据外置、
# 加了协议单测；本脚本补上"两端默认值一致性"这道门禁，改一端忘了改另一端时 CI 失败。
#
# 用法：python3 tools/check_protocol_config.py

import importlib.util
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PY = ROOT / "maixcam_tcp_server.py"
INO = ROOT / "esp32_tcp_stepper" / "esp32_tcp_stepper.ino"


def load_py_defaults():
    """导入 Python 端模块，读取协议相关常量的默认值。

    导入前清掉 MAIXCAM_* 环境变量，确保读到的是源码里的默认值，而不是某个
    偶然设置的覆盖值。模块顶层没有网络/相机副作用（maix 模块在 try 里懒加载），
    可安全导入。
    """
    for k in list(os.environ):
        if k.startswith("MAIXCAM_"):
            os.environ.pop(k)

    spec = importlib.util.spec_from_file_location("maixcam_srv", PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return {
        "USE_CHECKSUM": mod.USE_CHECKSUM,
        "MAX_PIXEL_X": mod.MAX_PIXEL_X,
        "MAX_PIXEL_Y": mod.MAX_PIXEL_Y,
        "SEQ_MOD": mod.SEQ_MOD,
    }


def load_ino_defaults():
    txt = INO.read_text(encoding="utf-8")

    def define(name):
        m = re.search(rf"#define\s+{name}\s+([^\s/]+)", txt)
        return m.group(1) if m else None

    def to_bool(s):
        return str(s).strip().lower() == "true"

    def to_int(s):
        return int(str(s).strip())

    return {
        "USE_CHECKSUM": to_bool(define("USE_CHECKSUM")),
        "MAX_PIXEL_X": to_int(define("MAX_PIXEL_X")),
        "MAX_PIXEL_Y": to_int(define("MAX_PIXEL_Y")),
    }


def main() -> int:
    if not PY.exists():
        print(f"ERROR: 找不到 {PY}")
        return 1
    if not INO.exists():
        print(f"ERROR: 找不到 {INO}")
        return 1

    py = load_py_defaults()
    ino = load_ino_defaults()

    # 只比对两端都定义的键（SEQ_MOD 仅 Python 端有，ESP32 用 0..255 范围与之等价）
    keys = [k for k in py if k in ino]
    mismatches = [k for k in keys if py[k] != ino[k]]

    if mismatches:
        print("协议配置两端不一致（必须相同，否则帧格式 / 序号对不上）：")
        for k in mismatches:
            print(f" - {k}: Python={py[k]}  ESP32={ino[k]}")
        return 1

    print("协议配置一致性校验通过：" + ", ".join(f"{k}={py[k]}" for k in keys))
    return 0


if __name__ == "__main__":
    sys.exit(main())
