# MQTT+Protobuf 实时时间传输系统 使用文档
## 一、文档概述
### 1. 系统简介
本系统基于 MQTT 3.1.1 协议和 Protobuf v3 序列化协议，实现**实时时间数据的发布/订阅传输**。系统分为发布者（Publisher）和订阅者（Subscriber）两个组件，发布者每秒生成一次实时时间数据并序列化后通过 MQTT 发布，订阅者订阅对应 Topic 后接收数据并反序列化解析，适用于本地/局域网内的结构化数据传输场景。

### 2. 核心特性
- 轻量级：基于 MQTT 协议，带宽占用低，适配物联网场景；
- 高效序列化：Protobuf 二进制序列化，比 JSON 体积小 30%+，解析速度快；
- 可靠性：MQTT QoS 1 保证消息至少一次送达；
- 易扩展：Protobuf 消息格式可快速扩展，支持后续整合视频控制、设备状态等数据。

### 3. 环境要求
| 环境        | 版本/要求                          |
|-------------|------------------------------------|
| 操作系统    | Ubuntu/Debian（Linux 发行版）|
| 依赖库      | Protobuf ≥ 3.0、Mosquitto ≥ 1.6    |
| 编译工具    | CMake ≥ 3.10、g++ ≥ 7.0            |
| MQTT 服务器 | Mosquitto（本地/远程均可）|

## 二、部署与安装
### 1. 依赖安装
执行以下命令安装编译和运行所需依赖：
```bash
# 更新软件源
sudo apt update

# 安装核心依赖
sudo apt install -y libprotobuf-dev protobuf-compiler libmosquitto-dev cmake g++ pkg-config

# 安装MQTT服务器（本地测试用）
sudo apt install -y mosquitto mosquitto-clients
```

### 2. 源码获取与目录结构
#### 2.1 目录结构（编译前）
```
mqtt_protobuf_time/  # 根目录
├── time_message.proto          # Protobuf消息定义
├── mqtt_protobuf_common.h      # 公共头文件（配置+工具函数）
├── mqtt_protobuf_common.cpp    # 公共工具函数实现
├── mqtt_publisher.cpp          # MQTT发布者源码
├── mqtt_subscriber.cpp         # MQTT订阅者源码
└── CMakeLists.txt              # CMake编译配置
```

#### 2.2 源码文件内容
所有源码文件的完整内容见附录 A。

### 3. 编译步骤
#### 3.1 生成 Protobuf 代码
在根目录执行以下命令，生成 C++ 语言的 Protobuf 序列化/反序列化代码：
```bash
cd mqtt_protobuf_time
protoc --cpp_out=. time_message.proto
```
执行后会生成 `time_message.pb.h` 和 `time_message.pb.cc` 文件。

#### 3.2 CMake 编译
```bash
# 创建编译目录
mkdir build && cd build

# CMake 配置
cmake ..

# 编译（-j4 表示4线程编译，速度更快）
make -j4
```
编译成功后，`build` 目录下会生成两个可执行文件：
- `mqtt_publisher`：发布者程序
- `mqtt_subscriber`：订阅者程序

## 三、运行与使用
### 1. 启动 MQTT 服务器（本地测试）
```bash
# 启动 Mosquitto 服务器
sudo systemctl start mosquitto

# 验证服务器状态（显示 active (running) 即为正常）
sudo systemctl status mosquitto

# 可选：设置开机自启
sudo systemctl enable mosquitto
```

### 2. 启动订阅者（Subscriber）
在 `build` 目录下执行：
```bash
./mqtt_subscriber
```
正常启动输出：
```
[2026-01-09 21:00:00] MQTT订阅者连接成功 ✅（Broker：localhost:1883，Topic：realtime/time）
[2026-01-09 21:00:00] 等待接收实时时间数据（按Ctrl+C退出）...
```

### 3. 启动发布者（Publisher）
新开终端，进入 `build` 目录执行：
```bash
./mqtt_publisher
```
正常启动输出：
```
[2026-01-09 21:00:01] MQTT发布者连接成功 ✅（Broker：localhost:1883，Topic：realtime/time）
[2026-01-09 21:00:01] 开始发布实时时间（按Ctrl+C退出）...
[2026-01-09 21:00:01] 消息发布成功（mid：1）✅
```

### 4. 验证数据传输
订阅者终端会实时接收并打印发布者发送的时间数据，示例输出：
```
========================================
[2026-01-09 21:00:01] 收到实时时间数据 ✅
时间戳（毫秒）：1736434801000
日期：2026-01-09
时间：21:00:01
时区：UTC+8
========================================
========================================
[2026-01-09 21:00:02] 收到实时时间数据 ✅
时间戳（毫秒）：1736434802000
日期：2026-01-09
时间：21:00:02
时区：UTC+8
========================================
```

### 5. 退出程序
按下 `Ctrl + C` 即可优雅退出发布者/订阅者程序，程序会自动清理 MQTT 连接资源。

## 四、配置说明
### 1. 核心配置项（修改 `mqtt_protobuf_common.h`）
| 配置项                | 说明                                  | 默认值                  |
|-----------------------|---------------------------------------|-------------------------|
| MQTT_BROKER           | MQTT 服务器地址（无需修改）| tcp://localhost:1883    |
| MQTT_CLIENT_ID_PUBLISHER | 发布者客户端ID（唯一即可）| time_publisher_001      |
| MQTT_CLIENT_ID_SUBSCRIBER | 订阅者客户端ID（唯一即可）| time_subscriber_001     |
| MQTT_TOPIC            | 数据传输的 Topic（自定义）| realtime/time           |
| MQTT_QOS              | MQTT 服务质量等级                     | 1（至少一次送达）|
| PUBLISH_INTERVAL_MS   | 发布者数据发送间隔（毫秒）| 1000（1秒/次）|

### 2. 连接远程 MQTT 服务器
若需连接远程 MQTT 服务器，修改发布者/订阅者代码中的连接地址：
```cpp
// 将 "localhost" 替换为远程服务器IP/域名
int ret = mosquitto_connect(mosq, 
                            "192.168.1.100",  // 远程MQTT服务器IP
                            1883,             // 远程服务器端口
                            60);              // 超时时间（秒）
```

## 五、常见问题与排查
### 1. 编译错误：`MOSQ_ERR_INTERRUPTED` 未定义
- 原因：低版本 Mosquitto 库未定义该宏；
- 解决方案：用 `EINTR` 替代 `MOSQ_ERR_INTERRUPTED`，并引入 `#include <errno.h>`。

### 2. 运行错误：Segmentation fault (core dumped)
- 原因：`strtok` 修改只读字符串导致内存错误；
- 解决方案：简化 MQTT 连接地址解析逻辑，直接使用 `localhost` 连接。

### 3. 连接错误：Lookup error.
- 原因：MQTT 服务器未启动或地址解析失败；
- 解决方案：
  1. 执行 `sudo systemctl start mosquitto` 启动服务器；
  2. 用 `127.0.0.1` 替换 `localhost` 重新编译。

### 4. 订阅者无数据接收
- 检查发布者是否正常启动并发布消息；
- 检查 Topic 是否一致（发布/订阅需使用相同 Topic）；
- 检查防火墙是否放行 1883 端口：`sudo ufw allow 1883`。

### 5. Protobuf 序列化失败
- 检查 Protobuf 代码是否生成：`ls -l time_message.pb.*`；
- 确保 Protobuf 版本 ≥ 3.0：`protoc --version`。

## 六、扩展说明
### 1. 扩展 Protobuf 消息格式
修改 `time_message.proto` 可扩展数据字段，例如新增视频控制指令：
```protobuf
syntax = "proto3";
package realtime;

// 实时时间消息
message TimeData {
  int64 timestamp_ms = 1;
  string date = 2;
  string time = 3;
  string timezone = 4;
}

// 视频控制消息
message VideoControl {
  enum Command {
    START = 0;
    STOP = 1;
    ADJUST_FPS = 2;
  }
  Command cmd = 1;
  int32 fps = 2;
  int32 width = 3;
  int32 height = 4;
}

// 合并消息（用于多数据传输）
message SystemData {
  TimeData time_data = 1;
  VideoControl video_ctrl = 2;
}
```
修改后重新执行 `protoc --cpp_out=. time_message.proto` 生成代码。

### 2. 与 HEVC 视频传输代码整合
- 将 MQTT 订阅者嵌入 HEVC 接收端，接收控制指令（如调整帧率、启停传输）；
- 将 MQTT 发布者嵌入 HEVC 发送端，同步发送视频帧时间戳和系统时间。

## 七、附录 A：完整源码
### 1. time_message.proto
```protobuf
syntax = "proto3";
package realtime;

message TimeData {
  int64 timestamp_ms = 1;
  string date = 2;
  string time = 3;
  string timezone = 4;
}
```

### 2. mqtt_protobuf_common.h
```cpp
#ifndef MQTT_PROTOBUF_COMMON_H
#define MQTT_PROTOBUF_COMMON_H

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <string>
#include <ctime>
#include <mosquitto.h>
#include "time_message.pb.h"

#define MQTT_BROKER "tcp://localhost:1883"
#define MQTT_CLIENT_ID_PUBLISHER "time_publisher_001"
#define MQTT_CLIENT_ID_SUBSCRIBER "time_subscriber_001"
#define MQTT_TOPIC "realtime/time"
#define MQTT_QOS 1
#define PUBLISH_INTERVAL_MS 1000

extern std::atomic<bool> g_is_running;
realtime::TimeData create_time_message();
std::string get_current_time_str();

#endif // MQTT_PROTOBUF_COMMON_H
```

### 3. mqtt_protobuf_common.cpp
```cpp
#include "mqtt_protobuf_common.h"

std::atomic<bool> g_is_running(true);

realtime::TimeData create_time_message() {
    realtime::TimeData time_msg;
    auto now = std::chrono::system_clock::now();
    auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    ).count();
    time_msg.set_timestamp_ms(timestamp_ms);

    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);

    char date_buf[32];
    std::strftime(date_buf, sizeof(date_buf), "%Y-%m-%d", &now_tm);
    time_msg.set_date(date_buf);

    char time_buf[32];
    std::strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &now_tm);
    time_msg.set_time(time_buf);

    time_msg.set_timezone("UTC+8");
    return time_msg;
}

std::string get_current_time_str() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);
    
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &now_tm);
    return std::string(buf);
}
```

### 4. mqtt_publisher.cpp
```cpp
#include "mqtt_protobuf_common.h"
#include <cstring>
#include <errno.h>

void on_publish_callback(struct mosquitto* mosq, void* userdata, int mid) {
    (void)mosq;
    (void)userdata;
    std::cout << "[" << get_current_time_str() << "] 消息发布成功（mid：" << mid << "）✅" << std::endl;
}

void publish_thread() {
    int lib_init = mosquitto_lib_init();
    if (lib_init != MOSQ_ERR_SUCCESS) {
        std::cerr << "[" << get_current_time_str() << "] MQTT库初始化失败 ❌" << std::endl;
        g_is_running = false;
        return;
    }

    struct mosquitto* mosq = mosquitto_new(MQTT_CLIENT_ID_PUBLISHER, true, nullptr);
    if (!mosq) {
        std::cerr << "[" << get_current_time_str() << "] 创建MQTT发布客户端失败 ❌" << std::endl;
        mosquitto_lib_cleanup();
        g_is_running = false;
        return;
    }

    mosquitto_publish_callback_set(mosq, on_publish_callback);

    int ret = mosquitto_connect(mosq, "localhost", 1883, 60);
    if (ret != MOSQ_ERR_SUCCESS) {
        std::cerr << "[" << get_current_time_str() << "] 连接MQTT服务器失败：" 
                  << mosquitto_strerror(ret) << " ❌" << std::endl;
        std::cerr << "提示：请先执行 sudo systemctl start mosquitto 启动MQTT服务器" << std::endl;
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        g_is_running = false;
        return;
    }

    std::cout << "[" << get_current_time_str() << "] MQTT发布者连接成功 ✅（Broker：localhost:1883，Topic：" << MQTT_TOPIC << "）" << std::endl;
    std::cout << "[" << get_current_time_str() << "] 开始发布实时时间（按Ctrl+C退出）..." << std::endl;

    while (g_is_running) {
        realtime::TimeData time_msg = create_time_message();
        std::string binary_data;
        if (!time_msg.SerializeToString(&binary_data)) {
            std::cerr << "[" << get_current_time_str() << "] Protobuf序列化失败 ❌" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(PUBLISH_INTERVAL_MS));
            continue;
        }

        ret = mosquitto_publish(mosq, nullptr, MQTT_TOPIC, binary_data.size(), binary_data.data(), MQTT_QOS, false);
        if (ret != MOSQ_ERR_SUCCESS) {
            std::cerr << "[" << get_current_time_str() << "] MQTT发布失败：" 
                      << mosquitto_strerror(ret) << " ❌" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(PUBLISH_INTERVAL_MS));
    }

    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    std::cout << "[" << get_current_time_str() << "] MQTT发布者退出 ✅" << std::endl;
}

int main() {
    signal(SIGINT, [](int sig) {
        (void)sig;
        g_is_running = false;
        std::cout << "\n[" << get_current_time_str() << "] 收到退出指令 ✅" << std::endl;
    });

    std::thread pub_thd(publish_thread);
    if (pub_thd.joinable()) {
        pub_thd.join();
    }

    std::cout << "[" << get_current_time_str() << "] 发布者程序退出 ✅" << std::endl;
    return 0;
}
```

### 5. mqtt_subscriber.cpp
```cpp
#include "mqtt_protobuf_common.h"
#include <cstring>
#include <mutex>
#include <errno.h>

std::mutex print_mutex;

void on_message_callback(struct mosquitto* mosq, void* userdata, const struct mosquitto_message* msg) {
    (void)mosq;
    (void)userdata;

    std::lock_guard<std::mutex> lock(print_mutex);
    if (msg == nullptr || msg->payloadlen == 0) {
        std::cerr << "[" << get_current_time_str() << "] 收到空消息或空指针 ❌" << std::endl;
        return;
    }

    realtime::TimeData time_msg;
    if (!time_msg.ParseFromArray(msg->payload, msg->payloadlen)) {
        std::cerr << "[" << get_current_time_str() << "] Protobuf反序列化失败 ❌" << std::endl;
        return;
    }

    std::cout << "========================================" << std::endl;
    std::cout << "[" << get_current_time_str() << "] 收到实时时间数据 ✅" << std::endl;
    std::cout << "时间戳（毫秒）：" << time_msg.timestamp_ms() << std::endl;
    std::cout << "日期：" << time_msg.date() << std::endl;
    std::cout << "时间：" << time_msg.time() << std::endl;
    std::cout << "时区：" << time_msg.timezone() << std::endl;
    std::cout << "========================================" << std::endl;
}

void subscribe_thread() {
    int lib_init = mosquitto_lib_init();
    if (lib_init != MOSQ_ERR_SUCCESS) {
        std::cerr << "[" << get_current_time_str() << "] MQTT库初始化失败 ❌" << std::endl;
        g_is_running = false;
        return;
    }

    struct mosquitto* mosq = mosquitto_new(MQTT_CLIENT_ID_SUBSCRIBER, true, nullptr);
    if (!mosq) {
        std::cerr << "[" << get_current_time_str() << "] 创建MQTT订阅客户端失败 ❌" << std::endl;
        mosquitto_lib_cleanup();
        g_is_running = false;
        return;
    }

    mosquitto_message_callback_set(mosq, on_message_callback);

    int ret = mosquitto_connect(mosq, "localhost", 1883, 60);
    if (ret != MOSQ_ERR_SUCCESS) {
        std::cerr << "[" << get_current_time_str() << "] 连接MQTT服务器失败：" 
                  << mosquitto_strerror(ret) << " ❌" << std::endl;
        std::cerr << "提示：请先执行 sudo systemctl start mosquitto 启动MQTT服务器" << std::endl;
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        g_is_running = false;
        return;
    }

    ret = mosquitto_subscribe(mosq, nullptr, MQTT_TOPIC, MQTT_QOS);
    if (ret != MOSQ_ERR_SUCCESS) {
        std::cerr << "[" << get_current_time_str() << "] 订阅Topic失败：" 
                  << mosquitto_strerror(ret) << " ❌" << std::endl;
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        g_is_running = false;
        return;
    }

    std::cout << "[" << get_current_time_str() << "] MQTT订阅者连接成功 ✅（Broker：localhost:1883，Topic：" << MQTT_TOPIC << "）" << std::endl;
    std::cout << "[" << get_current_time_str() << "] 等待接收实时时间数据（按Ctrl+C退出）..." << std::endl;

    while (g_is_running) {
        ret = mosquitto_loop(mosq, 100, 1);
        if (ret != MOSQ_ERR_SUCCESS && ret != EINTR) {
            std::cerr << "[" << get_current_time_str() << "] MQTT循环出错：" 
                      << mosquitto_strerror(ret) << " ❌" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    mosquitto_unsubscribe(mosq, nullptr, MQTT_TOPIC);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    std::cout << "[" << get_current_time_str() << "] MQTT订阅者退出 ✅" << std::endl;
}

int main() {
    signal(SIGINT, [](int sig) {
        (void)sig;
        g_is_running = false;
        std::cout << "\n[" << get_current_time_str() << "] 收到退出指令 ✅" << std::endl;
    });

    std::thread sub_thd(subscribe_thread);
    if (sub_thd.joinable()) {
        sub_thd.join();
    }

    std::cout << "[" << get_current_time_str() << "] 订阅者程序退出 ✅" << std::endl;
    return 0;
}
```

### 6. CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.10)
project(MQTT_Protobuf_Time)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
include_directories(${MOSQUITTO_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(PROTO_SRCS time_message.pb.cc)
set(PROTO_HDRS time_message.pb.h)
set(COMMON_SRCS mqtt_protobuf_common.cpp)

add_executable(mqtt_publisher 
    mqtt_publisher.cpp
    ${COMMON_SRCS}
    ${PROTO_SRCS}
)
target_link_libraries(mqtt_publisher
    ${PROTOBUF_LIBRARIES}
    ${MOSQUITTO_LIBRARIES}
    pthread
)

add_executable(mqtt_subscriber
    mqtt_subscriber.cpp
    ${COMMON_SRCS}
    ${PROTO_SRCS}
)
target_link_libraries(mqtt_subscriber
    ${PROTOBUF_LIBRARIES}
    ${MOSQUITTO_LIBRARIES}
    pthread
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mqtt_publisher PRIVATE -O3 -Wall -Wno-deprecated-declarations)
    target_compile_options(mqtt_subscriber PRIVATE -O3 -Wall -Wno-deprecated-declarations)
endif()
```

## 八、版本信息
| 组件         | 版本 |
|--------------|------|
| Protobuf     | 3.6.1|
| Mosquitto    | 1.6.9|
| CMake        | 3.28.0 |
| Ubuntu       | 20.04 LTS |