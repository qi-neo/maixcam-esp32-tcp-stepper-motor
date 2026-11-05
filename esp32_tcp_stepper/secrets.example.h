// 配置模板：复制本文件为同目录下的 secrets.h，然后填入真实值。
// secrets.h 已被 .gitignore 忽略，任何真实 SSID / 密码 / token 都不要提交进仓库。
// 缺少 secrets.h 时固件会直接编译报错（而不是烧进占位密码后无限等待 Wi-Fi）。
#pragma once

#define WIFI_SSID      "YOUR_WIFI_SSID"
#define WIFI_PASSWORD  "YOUR_WIFI_PASSWORD"

// Maixcam 上 maixcam_tcp_server.py 监听的地址与端口
#define HOST_IP        "192.168.10.5"
#define HOST_PORT      8080

// 可选认证：非空时，TCP 连上后首行发送 "AUTH <token>\n"。
// 必须与服务端环境变量 MAIXCAM_AUTH_TOKEN 完全一致：
//   export MAIXCAM_AUTH_TOKEN="与这里相同的 token"
// 留空 = 不认证，仅限隔离的受信局域网；服务端启动时会打印告警。
#define AUTH_TOKEN     ""
