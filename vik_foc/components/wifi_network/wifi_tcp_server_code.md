对，你现在手里正好有**两个独立 Example**：

1. `smartconfig`：负责 **“ESP32 怎么拿到 Wi-Fi 密码并连上路由器”**
2. `tcp_server`：负责 **“ESP32 连上路由器以后，怎么监听 TCP 端口并和客户端通信”**

你现在要做的不是把两个 `app_main()` 硬拼起来，而是把它们串成：

```text
                 ESP32 上电
                     │
                     ▼
              初始化 NVS/WiFi
                     │
                     ▼
          ┌─────────────────────┐
          │ 有没有已保存的 WiFi │
          └──────────┬──────────┘
                     │
              ┌──────┴──────┐
              │             │
             有             没有
              │             │
              ▼             ▼
        直接连接WiFi      SmartConfig
              │             │
              │             ▼
              │       手机发送SSID/密码
              │             │
              │             ▼
              │       保存WiFi配置
              │             │
              └──────┬──────┘
                     ▼
             WiFi连接成功
                     │
                     ▼
              获取 IP 地址
                     │
                     ▼
              启动 TCP Server
                     │
                     ▼
            等待手机/PC连接
                     │
                     ▼
          ┌───────────────────┐
          │ TCP 收发数据       │
          │ send() / recv()    │
          └───────────────────┘
```

这就是你正式产品应该走的架构。

---

# 1. 先把两个 Example 的关系搞明白

你现在 TCP Example 的核心是：

```c
ESP_ERROR_CHECK(example_connect());
```

这个函数实际上帮你完成了：

```text
WiFi初始化
   ↓
连接WiFi
   ↓
等待IP
```

但是你的 SmartConfig Example 是：

```text
WiFi初始化
   ↓
SmartConfig
   ↓
手机给SSID/密码
   ↓
连接WiFi
```

所以：

> **TCP Server 这个 Example 里的 `example_connect()` 要去掉。**

否则你两个 Wi-Fi 流程会打架。

---

# 2. 最简单的整合方式

你现在先不要考虑“正式产品”。

先做一个：

> **SmartConfig 配网成功 → 自动启动 TCP Server**

这是最容易打通的第一步。

你的代码结构改成：

```text
main
 │
 ├── WiFi初始化
 │
 ├── SmartConfig
 │
 └── WiFi成功
       │
       ▼
   TCP Server
```

---

# 3. TCP Server 部分基本可以原封不动

你这个：

```c
static void do_retransmit(const int sock)
```

可以保留。

它现在的功能其实特别简单：

```text
客户端发送：

hello

ESP32收到：

hello

ESP32再发送：

hello

客户端收到：

hello
```

也就是：

> **TCP Echo Server**

---

# 4. `tcp_server_task()` 也基本不用改

你的：

```c
static void tcp_server_task(void *pvParameters)
```

整个函数可以继续使用。

真正需要改的是：

```c
app_main()
```

以及：

```c
SC_EVENT / IP_EVENT
```

这一部分。

---

# 5. 第一版整合代码

我建议你先做一个**最小可运行版本**。

把 SmartConfig Example 和 TCP Server Example 合起来。

核心代码如下：

```c
#include <string.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#define TCP_SERVER_PORT 5000

static const char *TAG = "APP";

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT  BIT0
#define SMARTCONFIG_DONE_BIT BIT1
```

---

# 6. TCP Server

把你原来的：

```c
do_retransmit()
```

直接拿过来：

```c
static void do_retransmit(int sock)
{
    char rx_buffer[128];

    while (1) {

        int len = recv(
            sock,
            rx_buffer,
            sizeof(rx_buffer) - 1,
            0
        );

        if (len < 0) {

            ESP_LOGE(TAG,
                     "recv error, errno=%d",
                     errno);

            break;

        } else if (len == 0) {

            ESP_LOGI(TAG,
                     "client disconnected");

            break;
        }

        rx_buffer[len] = '\0';

        ESP_LOGI(TAG,
                 "Received %d bytes: %s",
                 len,
                 rx_buffer);

        int to_write = len;

        while (to_write > 0) {

            int written = send(
                sock,
                rx_buffer + (len - to_write),
                to_write,
                0
            );

            if (written < 0) {

                ESP_LOGE(TAG,
                         "send error, errno=%d",
                         errno);

                return;
            }

            to_write -= written;
        }
    }
}
```

---

# 7. TCP Server Task

然后：

```c
static void tcp_server_task(void *pvParameters)
{
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(TCP_SERVER_PORT);

    int listen_sock = socket(
        AF_INET,
        SOCK_STREAM,
        IPPROTO_IP
    );

    if (listen_sock < 0) {

        ESP_LOGE(TAG,
                 "Unable to create socket");

        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG,
             "TCP socket created");

    int err = bind(
        listen_sock,
        (struct sockaddr *)&dest_addr,
        sizeof(dest_addr)
    );

    if (err != 0) {

        ESP_LOGE(TAG,
                 "bind failed errno=%d",
                 errno);

        close(listen_sock);

        vTaskDelete(NULL);

        return;
    }

    ESP_LOGI(TAG,
             "TCP server bind port=%d",
             TCP_SERVER_PORT);

    err = listen(listen_sock, 5);

    if (err != 0) {

        ESP_LOGE(TAG,
                 "listen failed");

        close(listen_sock);

        vTaskDelete(NULL);

        return;
    }

    while (1) {

        ESP_LOGI(TAG,
                 "TCP server waiting client...");

        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);

        int sock = accept(
            listen_sock,
            (struct sockaddr *)&source_addr,
            &addr_len
        );

        if (sock < 0) {

            ESP_LOGE(TAG,
                     "accept failed errno=%d",
                     errno);

            continue;
        }

        ESP_LOGI(TAG,
                 "TCP client connected");

        do_retransmit(sock);

        shutdown(sock, 0);
        close(sock);

        ESP_LOGI(TAG,
                 "TCP client disconnected");
    }
}
```

---

# 8. 关键来了：Wi-Fi连接成功以后启动 TCP Server

SmartConfig 的：

```c
event_handler()
```

里面原来有：

```c
else if (event_base == IP_EVENT &&
         event_id == IP_EVENT_STA_GOT_IP)
{
    xEventGroupSetBits(
        s_wifi_event_group,
        CONNECTED_BIT
    );
}
```

现在改成：

```c
else if (event_base == IP_EVENT &&
         event_id == IP_EVENT_STA_GOT_IP)
{
    ESP_LOGI(TAG, "WiFi connected!");

    esp_netif_ip_info_t ip_info;

    esp_netif_t *netif =
        esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

    esp_netif_get_ip_info(
        netif,
        &ip_info
    );

    ESP_LOGI(
        TAG,
        "IP:" IPSTR,
        IP2STR(&ip_info.ip)
    );

    xEventGroupSetBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT
    );

    static bool tcp_started = false;

    if (!tcp_started) {

        tcp_started = true;

        xTaskCreate(
            tcp_server_task,
            "tcp_server",
            4096,
            NULL,
            5,
            NULL
        );

        ESP_LOGI(TAG,
                 "TCP server started");
    }
}
```

这就是两个 Example **真正接起来的地方**。

---

# 9. 现在整个工作过程就非常清晰了

假设你手机配置：

```text
SSID     = HOME_WIFI
PASSWORD = 12345678
```

ESP32：

```text
上电
 ↓
WiFi STA启动
 ↓
SmartConfig启动
 ↓
等待手机
```

手机：

```text
ESPTOUCH
 ↓
发送WiFi信息
```

ESP32：

```text
SC_EVENT_GOT_SSID_PSWD
 ↓
拿到：
SSID
PASSWORD
 ↓
esp_wifi_set_config()
 ↓
esp_wifi_connect()
```

然后：

```text
WIFI连接成功
 ↓
DHCP
 ↓
192.168.0.152
 ↓
IP_EVENT_STA_GOT_IP
 ↓
tcp_server_task()
 ↓
bind(5000)
 ↓
listen()
```

此时：

```text
ESP32
192.168.0.152:5000
       ▲
       │
       │ TCP
       │
       ▼
   手机 / PC
```

---

# 10. PC怎么测试？

Windows 上直接可以用：

```text
PowerShell
```

例如：

```powershell
Test-NetConnection 192.168.0.152 -Port 5000
```

如果看到：

```text
TcpTestSucceeded : True
```

说明：

```text
PC
 │
 │ TCP
 ▼
ESP32:5000
```

已经打通。

---

# 11. 但这个 TCP Server 现在干了什么？

你的：

```c
do_retransmit(sock);
```

实际上是：

```text
客户端
   │
   │ "hello"
   ▼
ESP32
   │
   │ recv()
   ▼
rx_buffer
   │
   │ send()
   ▼
客户端
   │
   ▼
"hello"
```

所以它是：

> **Echo Server（回显服务器）**

你可以用 PC 测试：

```text
发送：

hello esp32
```

ESP32：

```text
Received 10 bytes: hello esp32
```

PC：

```text
hello esp32
```

这样就算第一阶段完全打通。

---

# 12. 你以后真正做产品，不应该让 TCP Server 在 `app_main()` 直接启动

这是一个非常重要的点。

**不要这样：**

```c
void app_main(void)
{
    initialise_wifi();

    xTaskCreate(tcp_server_task,...);
}
```

因为这时候 Wi-Fi 可能还没连接：

```text
TCP Server
    ↓
bind()
    ↓
listen()
```

虽然 TCP socket 可以建立，但你的应用逻辑很容易出现：

```text
Wi-Fi还没准备好
IP还没有
```

所以应该：

```text
IP_EVENT_STA_GOT_IP
        ↓
确认已经获得IP
        ↓
启动TCP Server
```

这是正确时机。

---

# 13. 还有一个更大的问题：SmartConfig不能每次开机都跑

你现在这个 Example 最大的问题其实是：

```c
esp_smartconfig_start(&cfg);
```

每次启动都会进入 SmartConfig。

产品不能这么干。

真正产品应该：

```text
                    ESP32上电
                        │
                        ▼
                 读取NVS WiFi配置
                        │
                  ┌─────┴─────┐
                  │           │
                有配置       没配置
                  │           │
                  ▼           ▼
              直接连接      SmartConfig
                  │           │
                  │           ▼
                  │       手机配网
                  │           │
                  │           ▼
                  │       保存NVS
                  │           │
                  └─────┬─────┘
                        ▼
                   WiFi GOT IP
                        │
                        ▼
                  启动 TCP Server
```

这样才是正常的产品架构。

---

# 14. 还有一个你特别需要注意的地方

你前面一直在说：

> **“以后用户怎么修改 Wi-Fi 密码？”**

这时候不能只设计：

```text
SmartConfig
```

而应该设计一个**重新进入配网模式的机制**。

例如实体按键：

```text
长按 5 秒
     ↓
清除WiFi配置
     ↓
ESP32重启
     ↓
发现没有WiFi配置
     ↓
进入SmartConfig
```

或者更好：

```text
TCP命令：

SET_CONFIG_WIFI
```

让设备进入重新配网状态。

---

# 15. 我建议你最终做成这个架构

对于你现在这个 ESP32 产品，我会建议：

```text
                    app_main
                       │
                       ▼
                System Init
                       │
          ┌────────────┼────────────┐
          │            │            │
          ▼            ▼            ▼
         NVS          WiFi         GPIO
          │
          ▼
    读取WiFi配置
          │
     ┌────┴────┐
     │         │
    有        没有
     │         │
     ▼         ▼
 WiFi连接   SmartConfig
     │         │
     │         ▼
     │      获取SSID
     │      获取PASSWORD
     │         │
     │         ▼
     │       NVS
     │         │
     └────┬────┘
          ▼
    GOT_IP事件
          │
          ▼
   ┌───────────────┐
   │ TCP Server    │
   │ port 5000     │
   └───────┬───────┘
           │
           ▼
     Client Connect
           │
           ▼
      recv()/send()
```

这就已经是一个比较正规的 ESP32 Wi-Fi 产品基础框架了。

---

## 你现在最应该做的实验

先别急着加 NVS、修改密码、协议解析这些东西。

**第一阶段只验证这一条链：**

```text
手机 ESPTOUCH
      ↓
ESP32 SmartConfig
      ↓
拿到SSID/PASSWORD
      ↓
连接路由器
      ↓
获得192.168.x.x
      ↓
自动启动TCP Server : 5000
      ↓
PC连接ESP32:5000
      ↓
发送 "hello"
      ↓
ESP32 recv()
      ↓
ESP32 send()
      ↓
PC收到 "hello"
```

只要这一条跑通，**SmartConfig + TCP Server 就真正打通了**。

然后第二阶段再加：

```text
NVS保存WiFi
↓
上电自动连接
↓
无配置才SmartConfig
↓
长按按键重新配网
↓
TCP自定义通信协议
```

这时候就已经从 **ESP-IDF Example** 开始变成你自己的**产品级 Wi-Fi 通信框架**了。
