# Maixcam 端（TCP 服务器）：向 ESP32 下发 "像素X,像素Y,类别"。
# 这里用固定数据演示通信流程，实际可接入 Maixcam 的识别结果。
# 说明见仓库 README。

import socket
import threading
import time

local_ip = "192.168.10.5"
local_port = 8080

data_set = [
    "2560,2560,1",
    "2560,2560,2",
    "2560,2560,3",
    "2560,2560,4",
]


def receiveThread(conn, addr):
    try:
        for message in data_set:
            conn.sendall(message.encode('utf-8') + b"\n")
            print(f"Sent {message} to {addr}")
            # 等一会儿再收客户端的回执
            time.sleep(18)
            client_data = conn.recv(1024)
            if client_data:
                print(f"Received {client_data.decode('utf-8')} from {addr}")
                print("Received!")
            else:
                print("No data received from client, continuing to send next message.")
    except Exception as e:
        print(f"Error in communication with {addr}: {e}")
    finally:
        print(f"Client {addr} disconnected")
        conn.close()


ip_port = (local_ip, local_port)
sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sk.bind(ip_port)
sk.listen(50)

print("Server is listening on", ip_port)
while True:
    conn, addr = sk.accept()
    print(f"Client {addr} connected")
    t = threading.Thread(target=receiveThread, args=(conn, addr))
    t.daemon = True
    t.start()
