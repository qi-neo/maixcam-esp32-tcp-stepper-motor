#!/usr/bin/env python3
# Maixcam 端（TCP 服务器）：向 ESP32 下发 "x,y,category" 并等待结构化 ACK。
# 优化版特性（详见仓库 OPTIMIZATION.md / protocol.md）：
#   1) 请求-应答握手 —— 每条指令后等待 ESP32 回执 ACK:...（带超时），取代原版 time.sleep(18) 的硬等待
#   2) recv 超时 —— conn.settimeout，避免无回执时永久阻塞（修复原版死锁）
#   3) 行缓冲 —— 服务端也按 '\n' 分帧，正确处理半包/粘包
#   4) 检测桩函数 —— generate_command() 已留好接入 Maixcam 真实检测的接口
#   5) 优雅退出 —— Ctrl-C 关闭监听套接字
#
# 用法：python3 maixcam_tcp_server.py

import socket
import threading
import signal
import sys
import time

LOCAL_IP = "0.0.0.0"      # 监听所有网卡；Maixcam 上可填本机 IP，如 "192.168.10.5"
LOCAL_PORT = 8080
RECV_TIMEOUT = 5.0        # 等待 ESP32 回执的超时（秒）
ACK_TIMEOUT_ACTION = "continue"  # 超时后行为："continue" 继续发下一条 / "abort" 断开该客户端


def generate_command():
    """返回 (x, y, category)。
    演示用固定数据；实际请接入 Maixcam 的目标检测结果，例如：
        objs = maixcam_yolo.detect()      # 伪代码
        if objs:
            o = max(objs, key=lambda o: o.score)   # 取置信度最高目标
            return int(o.cx), int(o.cy), int(o.cls)
        return 0, 0, 0                     # 无目标
    返回 category=0 时服务器会跳过本次发送并继续等待。
    """
    return 2560, 2560, 1   # TODO: 替换为 Maixcam 真实检测


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
