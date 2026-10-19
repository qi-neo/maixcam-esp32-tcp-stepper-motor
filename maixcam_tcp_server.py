#!/usr/bin/env python3
# Maixcam 端（TCP 服务器）：向 ESP32 下发 "x,y,category" 并等待结构化 ACK。
# 优化版特性（详见仓库 OPTIMIZATION.md / protocol.md）：
#   1) 请求-应答握手 —— 每条指令后等待 ESP32 回执 ACK:...（带超时），取代原版 time.sleep(18) 的硬等待
#   2) recv 超时 —— conn.settimeout，避免无回执时永久阻塞（修复原版死锁）
#   3) 行缓冲 —— 服务端也按 '\n' 分帧，正确处理半包/粘包
#   4) 真实 YOLO 检测 —— generate_command() 接入 Maixcam 的 maix.nn 检测；
#      非 Maixcam 环境（如 PC 调试）自动回退到演示数据，文件可直接运行
#   5) 优雅退出 —— Ctrl-C 关闭监听套接字
#
# 用法（Maixcam 上）：python3 maixcam_tcp_server.py

import socket
import threading
import signal
import sys
import time

LOCAL_IP = "0.0.0.0"      # 监听所有网卡；Maixcam 上可填本机 IP，如 "192.168.10.5"
LOCAL_PORT = 8080
RECV_TIMEOUT = 5.0        # 等待 ESP32 回执的超时（秒）
ACK_TIMEOUT_ACTION = "continue"  # 超时后行为："continue" 继续发下一条 / "abort" 断开该客户端

# ----------------------------------------------------------------------------
# 检测配置（按你的设备 / 模型修改）
# ----------------------------------------------------------------------------
MODEL_PATH = "/root/models/yolov5s.mud"   # Maixcam 上的模型文件路径（.mud / .kmodel）
INPUT_W, INPUT_H = 224, 224               # 模型输入尺寸（需与模型一致）
CONF_THRES = 0.5
IOU_THRES = 0.45
MAX_PIXEL_X = 2560                        # 与 ESP32 端 MAX_PIXEL_X 对应（逻辑像素空间）
MAX_PIXEL_Y = 2560                        # 与 ESP32 端 MAX_PIXEL_Y 对应

# 把检测类别(class_id, 0-based) 映射到 ESP32 的 1..4 动作类别；未知类别默认 1。
# 例：0=person→1, 39=bottle→2。按你的模型与抓取策略修改。
CLASS_TO_CATEGORY = {
    0: 1,
    39: 2,
}

def map_class_to_category(class_id):
    return CLASS_TO_CATEGORY.get(int(class_id), 1)

# 坐标翻转（部分摄像头镜像）。若机械臂 X/Y 方向与实际相反，把对应项改为 True。
FLIP_X = False
FLIP_Y = False

# ----------------------------------------------------------------------------
# Maixcam 检测模块（懒初始化；非 Maixcam 环境自动回退演示）
# ----------------------------------------------------------------------------
try:
    from maix import camera, image
    import maix.nn as nn
    _HAS_MAIX = True
except Exception:
    _HAS_MAIX = False

_detector = None
_cam = None


def _init_maix():
    """懒初始化摄像头与 YOLO 检测器，仅首次调用时执行；失败则回退演示。"""
    global _detector, _cam
    if _detector is not None:
        return True
    if not _HAS_MAIX:
        return False
    try:
        _cam = camera.Camera(INPUT_W, INPUT_H)
        # 若使用 YOLOv8 模型，把下一行改为 nn.YOLOv8(model=MODEL_PATH, dual_buff=True)
        _detector = nn.YOLOv5(model=MODEL_PATH, dual_buff=True)
        print(f"[info] Maixcam YOLO loaded: {MODEL_PATH}")
        return True
    except Exception as e:
        print(f"[warn] Maixcam init failed ({e}); 回退到演示数据")
        _HAS_MAIX = False
        return False


def generate_command():
    """返回 (x, y, category)。
    优先用 Maixcam 真实 YOLO 检测：取置信度最高的目标，将其包围框中心坐标
    缩放到 MAX_PIXEL 逻辑空间后下发。
    非 Maixcam 环境或初始化失败时回退到演示数据。
    返回 category=0 时服务器会跳过本次发送并继续等待（无目标时持续检测）。
    """
    if _init_maix():
        img = _cam.read()
        objs = _detector.run(img, conf_threshold=CONF_THRES, iou_threshold=IOU_THRES)
        if not objs:
            return 0, 0, 0
        o = max(objs, key=lambda o: o.score)            # 取置信度最高目标
        cx = (o.x + o.w / 2.0) / INPUT_W * MAX_PIXEL_X  # 模型输入坐标 → 逻辑像素
        cy = (o.y + o.h / 2.0) / INPUT_H * MAX_PIXEL_Y
        if FLIP_X:
            cx = MAX_PIXEL_X - cx
        if FLIP_Y:
            cy = MAX_PIXEL_Y - cy
        cat = map_class_to_category(o.class_id)
        print(f"[detect] class={o.class_id} score={o.score:.2f} "
              f"-> x={int(cx)} y={int(cy)} cat={cat}")
        return int(cx), int(cy), int(cat)
    else:
        # 演示回退（非 Maixcam 环境，如 PC 调试）
        return 2560, 2560, 1


def handle_client(conn, addr):
    conn.settimeout(RECV_TIMEOUT)
    buf = b""
    try:
        while True:
            x, y, cat = generate_command()
            if cat == 0:
                time.sleep(0.2)
                continue

            msg = f"{x},{y},{cat}\n".encode("utf-8")
            conn.sendall(msg)
            print(f"[send] {x},{y},{cat} -> {addr}")

            # 等待客户端回执（按行解析，支持半包）
            try:
                while b"\n" not in buf:
                    chunk = conn.recv(1024)
                    if not chunk:
                        print(f"[warn] client {addr} closed connection")
                        return
                    buf += chunk
                line, _, buf = buf.partition(b"\n")
                ack = line.decode("utf-8", "replace").strip()
                print(f"[recv] {ack} <- {addr}")
            except socket.timeout:
                print(f"[warn] ACK timeout from {addr} ({RECV_TIMEOUT}s)")
                if ACK_TIMEOUT_ACTION == "abort":
                    return
    except Exception as e:
        print(f"[error] {addr}: {e}")
    finally:
        try:
            conn.close()
        except Exception:
            pass
        print(f"[info] {addr} disconnected")


def main():
    sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sk.bind((LOCAL_IP, LOCAL_PORT))
    sk.listen(5)
    print(f"Server listening on {LOCAL_IP}:{LOCAL_PORT}")

    stop = threading.Event()

    def shutdown(signum, frame):
        print("\nShutting down...")
        stop.set()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        while not stop.is_set():
            sk.settimeout(1.0)
            try:
                conn, addr = sk.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            print(f"[info] client {addr} connected")
            t = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
            t.start()
    finally:
        sk.close()
        print("Server stopped.")


if __name__ == "__main__":
    main()
