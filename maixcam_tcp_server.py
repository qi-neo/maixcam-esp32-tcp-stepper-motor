#!/usr/bin/env python3
# Maixcam 端（TCP 服务器）：向 ESP32 下发 "x,y,category,seq" 并等待带序号的 ACK。
# 优化版特性（详见仓库 OPTIMIZATION.md / protocol.md）：
#   1) 请求-应答握手 —— 每条指令后等待 ESP32 回执 ACK:...:SEQ（带超时），
#      取代原版 time.sleep(18) 的硬等待
#   2) recv 超时 —— conn.settimeout，避免无回执时永久阻塞（修复原版死锁）
#   3) 行缓冲 —— 服务端也按 '\n' 分帧，正确处理半包/粘包
#   4) 真实 YOLO 检测 —— generate_command() 接入 Maixcam 的 maix.nn 检测；
#      非 Maixcam 环境（如 PC 调试）自动回退到演示数据，文件可直接运行
#   5) 优雅退出 —— Ctrl-C 关闭监听套接字，并尽量给 ESP32 发 ESTOP
#
# P1 安全/协议加固（详见 README.md「安全模型与运行限制」与 protocol.md）：
#   6) 可选认证 —— AUTH_TOKEN 非空时，客户端连上后的首行必须是
#      "AUTH <token>\n"；校验失败立即断开。默认空 = 不认证。
#   7) 单客户端 —— 同一时刻只允许一个 ESP32 连接，新连接直接拒绝并打印原因，
#      避免多线程抢同一台相机 / 同一条机械臂。
#   8) 带序号 ACK —— 只接受 SEQ 与当前 seq 匹配的回执；迟到/错位回执丢弃并记日志，
#      真正修掉"上一条的 ACK 被当成本条的回执"的永久错位。
#   9) ESTOP / RESUME —— send_estop() 立即急停，send_resume() 恢复接收指令。
#  10) 发送前校验 category ∈ [1,4]（以及 x/y 在像素范围内），坏指令不下发。
#
# ⚠️ 安全提示：AUTH_TOKEN 默认为空，即"不认证"——局域网内任何人都可以连接
# 8080 端口并驱动电机。仅限受信网络使用；生产/演示环境请设置 AUTH_TOKEN、
# 用防火墙限制来源，并保留物理急停。
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
MAX_CONSECUTIVE_TIMEOUTS = 3     # continue 模式下连续超时几次就判定链路已死并断开

# ----------------------------------------------------------------------------
# 安全 / 协议配置
# ----------------------------------------------------------------------------
AUTH_TOKEN = ""           # 非空时启用首行认证："AUTH <token>\n"；必须与 ESP32 端一致。
MAX_CLIENTS = 1           # 单客户端模式（本项目同一时刻只允许一个 ESP32 连接）
USE_CHECKSUM = False      # 必须与 ESP32 端同名常量一致；true 时帧尾追加 (x+y+cat+seq)%256
SEQ_MOD = 256             # ACK 序号范围 0..255，发完回绕
CONTROL_SEQ = -1          # ESTOP / RESUME 回执使用的保留序号（普通指令不会用到 -1）

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
    # _HAS_MAIX 必须一起声明 global：函数体后面有 `_HAS_MAIX = False`，
    # 少了这句，Python 会把 _HAS_MAIX 当成整个函数的局部变量，
    # 于是下面 `if not _HAS_MAIX` 每次都抛 UnboundLocalError（服务器一条指令都发不出去）。
    global _detector, _cam, _HAS_MAIX
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


# ============================================================================
# 协议工具（纯函数，便于不接硬件自测）
# ============================================================================

def is_valid_category(cat):
    """下发前校验：category 必须是 1..4 的整数。"""
    return isinstance(cat, int) and 1 <= cat <= 4


def build_command(x, y, cat, seq):
    """构造下行帧 x,y,cat,seq[,sum]\\n；任何字段非法抛 ValueError。

    发送前就把坏指令挡住：category 越界 / 坐标越界 / 非整数都不会发到 ESP32；
    这里不做 int() 截断，浮点数/字符串直接拒绝，避免 "1.9 变成 1" 这类静默错误。
    """
    for name, value in (("x", x), ("y", y), ("cat", cat), ("seq", seq)):
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{name} 必须是整数，收到 {value!r}")
    if not is_valid_category(cat):
        raise ValueError(f"category 必须在 1..4，收到 {cat}")
    if not (0 <= x <= MAX_PIXEL_X):
        raise ValueError(f"x 必须在 0..{MAX_PIXEL_X}，收到 {x}")
    if not (0 <= y <= MAX_PIXEL_Y):
        raise ValueError(f"y 必须在 0..{MAX_PIXEL_Y}，收到 {y}")
    seq = seq % SEQ_MOD
    fields = [x, y, cat, seq]
    if USE_CHECKSUM:
        fields.append(sum(fields) % 256)
    return (",".join(str(v) for v in fields) + "\n").encode("utf-8")


def parse_ack(line):
    """解析 ACK:STATUS:CAT:POS1:POS2:SEQ；格式不合法返回 None。"""
    parts = line.strip().split(":")
    if len(parts) != 6 or parts[0] != "ACK":
        return None
    try:
        cat, pos1, pos2, seq = (int(parts[i]) for i in range(2, 6))
    except ValueError:
        return None
    return {"status": parts[1], "cat": cat, "pos1": pos1, "pos2": pos2, "seq": seq}


def ack_matches(ack, seq):
    """只有序号匹配的回执才属于当前指令；迟到 ACK 的 seq 是旧值，必须丢弃。"""
    return ack is not None and ack["seq"] == seq


def read_line(conn, buf, timeout):
    """从连接读取一行（去掉 \\n），返回 (line, 剩余buf)。

    对端关闭返回 (None, buf)；超时抛 socket.timeout。
    """
    deadline = time.monotonic() + timeout
    while b"\n" not in buf:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise socket.timeout("read_line timeout")
        conn.settimeout(remaining)
        chunk = conn.recv(1024)
        if not chunk:
            return None, buf
        buf += chunk
    raw, _, buf = buf.partition(b"\n")
    return raw.decode("utf-8", "replace").strip(), buf


def wait_for_ack(conn, buf, seq, timeout=RECV_TIMEOUT, addr=None):
    """等待 SEQ 匹配的 ACK。

    不匹配的回执（迟到 / 上一条的 DONE / ERR）丢弃并打印日志；
    对端关闭抛 ConnectionError，超时抛 socket.timeout。
    返回 (ack_dict, 剩余buf)。
    """
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise socket.timeout(f"等待 seq={seq} 的 ACK 超时")
        line, buf = read_line(conn, buf, remaining)
        if line is None:
            raise ConnectionError("对端在等待 ACK 时关闭连接")
        ack = parse_ack(line)
        if ack is None:
            if line:
                print(f"[warn] 非回执行: {line!r}")
            continue
        if ack_matches(ack, seq):
            print(f"[recv] {line}" + (f" <- {addr}" if addr else ""))
            return ack, buf
        print(f"[warn] 丢弃迟到/错位回执: seq={ack['seq']} 期望={seq} line={line!r}")


def authenticate(conn, buf, addr=None):
    """AUTH_TOKEN 非空时校验客户端首行 "AUTH <token>"；返回 (ok, 剩余buf)。

    默认 AUTH_TOKEN 为空时直接放行，保持原行为不变。
    """
    if not AUTH_TOKEN:
        return True, buf
    try:
        line, buf = read_line(conn, buf, RECV_TIMEOUT)
    except socket.timeout:
        print(f"[warn] {addr} 认证超时：{RECV_TIMEOUT}s 内未收到 AUTH 行")
        return False, buf
    if line is None:
        print(f"[warn] {addr} 在认证完成前断开连接")
        return False, buf
    if line != f"AUTH {AUTH_TOKEN}":
        # 不回显 token 本身，避免日志泄露
        print(f"[warn] {addr} 认证失败：首行不是合法的 AUTH 行，立即断开")
        return False, buf
    print(f"[info] {addr} 认证通过")
    return True, buf


def _send_control(conn, command, expect_status, addr=None):
    """发送 ESTOP/RESUME 并等待 seq=-1 的控制回执；返回 (ok, 剩余buf)。"""
    buf = b""
    conn.sendall((command + "\n").encode("utf-8"))
    print(f"[send] {command}" + (f" -> {addr}" if addr else ""))
    ack, buf = wait_for_ack(conn, buf, CONTROL_SEQ, RECV_TIMEOUT, addr)
    ok = ack["status"] == expect_status
    if not ok:
        print(f"[warn] {command} 回执状态异常: {ack['status']}（期望 {expect_status}）")
    return ok, buf


def send_estop(conn, addr=None):
    """立即急停：ESP32 停止运动、关闭驱动器使能，并回 ACK:ESTOP:...:-1。

    可在检测到危险 / 收到外部中止请求时调用。返回是否收到预期回执。
    注意：单客户端模式下请从持有该 conn 的线程调用，避免与请求-应答循环抢收 ACK。
    """
    ok, _ = _send_control(conn, "ESTOP", "ESTOP", addr)
    return ok


def send_resume(conn, addr=None):
    """解除急停：ESP32 恢复接收运动指令并回 ACK:RESUME:...:-1。返回是否成功。"""
    ok, _ = _send_control(conn, "RESUME", "RESUME", addr)
    return ok


# ============================================================================
# 单客户端管理
# ============================================================================
_active_lock = threading.Lock()
_active_conn = None
_active_addr = None
_shutdown = threading.Event()


def _set_active(conn, addr):
    global _active_conn, _active_addr
    with _active_lock:
        _active_conn = conn
        _active_addr = addr


def _clear_active(addr):
    global _active_conn, _active_addr
    with _active_lock:
        if _active_addr == addr:
            _active_conn = None
            _active_addr = None


def _active_client_desc():
    with _active_lock:
        return _active_addr


# ============================================================================
# 客户端会话
# ============================================================================

def handle_client(conn, addr):
    conn.settimeout(RECV_TIMEOUT)
    buf = b""
    seq = 0
    consecutive_timeouts = 0
    authed = False
    try:
        authed, buf = authenticate(conn, buf, addr)
        if not authed:
            return

        while not _shutdown.is_set():
            x, y, cat = generate_command()
            if cat == 0:
                time.sleep(0.2)
                continue

            try:
                frame = build_command(x, y, cat, seq)
            except ValueError as e:
                # 发送前校验失败：跳过本次，不把坏指令发给 ESP32
                print(f"[warn] 跳过非法指令: {e}")
                continue

            # 发送前清掉缓冲区里残留的完整行：它们只可能是迟到 ACK。
            # 即使有残留，seq 匹配逻辑也会兜底丢弃，这里只是让日志更干净。
            while b"\n" in buf:
                stale, _, buf = buf.partition(b"\n")
                stale_text = stale.decode("utf-8", "replace").strip()
                if stale_text:
                    print(f"[warn] 发送前丢弃迟到回执: {stale_text}")

            conn.sendall(frame)
            print(f"[send] {frame.decode('utf-8').strip()} -> {addr} (seq={seq})")

            try:
                ack, buf = wait_for_ack(conn, buf, seq, RECV_TIMEOUT, addr)
                consecutive_timeouts = 0
                if ack["status"] == "BUSY":
                    print(f"[warn] ESP32 BUSY，本条已忽略 (seq={seq})")
            except socket.timeout:
                consecutive_timeouts += 1
                print(f"[warn] ACK timeout from {addr} ({RECV_TIMEOUT}s, "
                      f"seq={seq}, 连续 {consecutive_timeouts} 次)")
                if ACK_TIMEOUT_ACTION == "abort" or consecutive_timeouts >= MAX_CONSECUTIVE_TIMEOUTS:
                    print(f"[warn] 判定链路已失效，断开 {addr} 以释放单客户端槽位")
                    return

            seq = (seq + 1) % SEQ_MOD
    except ConnectionError as e:
        print(f"[warn] client {addr} closed connection: {e}")
    except Exception as e:
        print(f"[error] {addr}: {e}")
    finally:
        # 会话结束（关机 / 断链 / 协议异常）时尽量让 ESP32 停下来。
        # ESTOP 是幂等且安全的；认证失败或对端已断开时发送失败也无妨。
        if authed:
            try:
                send_estop(conn, addr)
            except Exception as e:
                print(f"[warn] 断开前 ESTOP 未确认: {e}")
        try:
            conn.close()
        except Exception:
            pass
        print(f"[info] {addr} disconnected")


def _client_worker(conn, addr):
    try:
        handle_client(conn, addr)
    finally:
        _clear_active(addr)


def _accept_client(conn, addr):
    """尝试接管新连接。

    单客户端模式：已有连接时打印拒绝原因、立刻关闭新连接并返回 None。
    成功时返回新启动的线程（便于 main 在退出时 join）。
    """
    active_addr = _active_client_desc()
    if active_addr is not None:
        # 避免多个线程同时抢同一台相机 / 同一条机械臂
        print(f"[warn] 拒绝新连接 {addr}：已有客户端 {active_addr} 连接中"
              f"（单客户端模式，MAX_CLIENTS={MAX_CLIENTS}）")
        try:
            conn.close()
        except OSError:
            pass
        return None

    print(f"[info] client {addr} connected")
    _set_active(conn, addr)
    thread = threading.Thread(target=_client_worker, args=(conn, addr), daemon=True)
    thread.start()
    return thread


def main():
    sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sk.bind((LOCAL_IP, LOCAL_PORT))
    sk.listen(5)
    print(f"Server listening on {LOCAL_IP}:{LOCAL_PORT}")
    if AUTH_TOKEN:
        print("[info] AUTH_TOKEN 已启用：客户端首行必须发送 AUTH <token>")
    else:
        print("[warn] AUTH_TOKEN 为空：未启用认证，局域网内任何人都可以驱动电机！")

    def shutdown(signum, frame):
        print("\nShutting down...")
        _shutdown.set()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    client_thread = None
    try:
        while not _shutdown.is_set():
            sk.settimeout(1.0)
            try:
                conn, addr = sk.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            active_addr = _active_client_desc()
            if active_addr is not None:
                # 单客户端：直接拒绝新连接，避免多个线程同时驱动同一台相机/机械臂
                print(f"[warn] 拒绝新连接 {addr}：已有客户端 {active_addr} 连接中"
                      f"（单客户端模式，MAX_CLIENTS={MAX_CLIENTS}）")
                try:
                    conn.close()
                except OSError:
                    pass
                continue

            client_thread = _accept_client(conn, addr)
    finally:
        _shutdown.set()
        sk.close()
        # 给当前会话一点时间发 ESTOP 并关闭
        if client_thread is not None and client_thread.is_alive():
            client_thread.join(timeout=RECV_TIMEOUT + 2)
        print("Server stopped.")


if __name__ == "__main__":
    main()
