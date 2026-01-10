# MQTT C++ 机器人状态上报 Demo 使用文档
## 一、项目概述
本项目基于 Eclipse Paho MQTT C++ 库实现本地 MQTT 通信，发布端以 JSON 格式上报机器人状态（ID、电量、位置、速度等），订阅端接收并解析该 JSON 消息，适用于机器人状态监控类场景的基础开发。

### 核心功能
- 发布端：构建结构化的机器人状态 JSON 消息，发布到本地 MQTT 服务器 `robot/status` 主题。
- 订阅端：订阅 `robot/status` 主题，接收并解析 JSON 格式的机器人状态消息。
- 本地 MQTT 服务器：基于 Mosquitto 搭建，提供稳定的消息转发能力。

## 二、项目结构
```
mqtt_cpp_demo/
├── include/                  # 第三方头文件目录
│   └── json.hpp              # nlohmann/json 单头文件库（JSON 解析/构建）
├── CMakeLists.txt            # CMake 编译配置文件
├── publisher.cpp             # MQTT 发布端代码（发送机器人状态 JSON）
├── subscriber.cpp            # MQTT 订阅端代码（接收并解析 JSON）
├── run_test.sh               # 一键编译&运行测试脚本
└── README.md                 # 项目使用文档（本文档）
```

## 三、核心文件内容
### 1. CMakeLists.txt（编译配置）
```cmake
cmake_minimum_required(VERSION 3.10)
project(mqtt_cpp_demo)

# 配置 C++17 标准（兼容 Paho 库和 JSON 库）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项：开启警告、调试信息，屏蔽 Paho 废弃 API 警告
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -DPAHO_MQTT_CPP_NO_DEPRECATED")

# 可执行文件输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 引入 JSON 头文件目录
include_directories(${CMAKE_SOURCE_DIR}/include)

# 查找 Paho MQTT 库（C++ 库 + C 库）
find_library(PAHO_MQTT_CPP NAMES paho-mqttpp3 REQUIRED)
find_library(PAHO_MQTT_C NAMES paho-mqtt3as paho-mqtt3a REQUIRED)

# 查找依赖库（线程、OpenSSL）
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)

# 编译订阅端
add_executable(subscriber subscriber.cpp)
target_link_libraries(subscriber
    ${PAHO_MQTT_CPP}
    ${PAHO_MQTT_C}
    Threads::Threads
    OpenSSL::SSL
    OpenSSL::Crypto
)

# 编译发布端
add_executable(publisher publisher.cpp)
target_link_libraries(publisher
    ${PAHO_MQTT_CPP}
    ${PAHO_MQTT_C}
    Threads::Threads
    OpenSSL::SSL
    OpenSSL::Crypto
)

# 打印编译信息（便于调试）
message(STATUS "Paho MQTT C++ 库路径: ${PAHO_MQTT_CPP}")
message(STATUS "Paho MQTT C 库路径: ${PAHO_MQTT_C}")
message(STATUS "可执行文件输出目录: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "JSON 头文件路径: ${CMAKE_SOURCE_DIR}/include/json.hpp")
```

### 2. publisher.cpp（发布端核心代码）
```cpp
#include <iostream>
#include <cstdlib>
#include <string>
#include <ctime>
#include "mqtt/async_client.h"
#include "include/json.hpp"

// 本地 MQTT 服务器地址
const std::string SERVER_ADDRESS("tcp://127.0.0.1:1883");
const std::string CLIENT_ID("cpp_robot_publisher_demo");
const std::string TOPIC("robot/status");

// 简化 JSON 命名空间
using json = nlohmann::json;

// 构建机器人状态 JSON 字符串
std::string build_robot_status_json() {
    json robot_status;
    robot_status["robot_id"] = "robot_001";          // 机器人唯一ID
    robot_status["timestamp"] = time(nullptr);       // 状态时间戳（秒级）
    robot_status["running_status"] = "normal";       // 运行状态：normal/error/idle
    robot_status["battery"] = 85.5;                  // 剩余电量（%）
    robot_status["position"] = {                     // 三维位置坐标
        {"x", 10.2},
        {"y", 25.8},
        {"z", 0.5}
    };
    robot_status["speed"] = 1.2;                     // 移动速度（m/s）
    robot_status["error_code"] = 0;                  // 错误码：0=无错误

    // 格式化 JSON 输出（4空格缩进，便于阅读）
    return robot_status.dump(4);
}

int main(int argc, char* argv[]) {
    try {
        // 创建 MQTT 异步客户端
        mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);

        // 配置连接选项（清理会话、心跳间隔等）
        mqtt::connect_options connOpts;
        connOpts.set_clean_session(true);

        // 连接本地 MQTT 服务器
        std::cout << "连接到本地 MQTT 服务器: " << SERVER_ADDRESS << std::endl;
        mqtt::token_ptr conntok = client.connect(connOpts);
        conntok->wait();  // 等待连接完成
        std::cout << "连接成功！" << std::endl;

        // 构建并发布 JSON 消息
        std::string json_payload = build_robot_status_json();
        std::cout << "\n发布机器人状态 JSON 消息：" << std::endl;
        std::cout << json_payload << std::endl;

        // 构建 MQTT 消息（QoS=1，确保消息至少送达一次）
        mqtt::message_ptr pubmsg = mqtt::make_message(TOPIC, json_payload, 1, false);
        client.publish(pubmsg)->wait();  // 发布消息

        // 断开连接
        std::cout << "\n消息发布完成！断开连接..." << std::endl;
        client.disconnect()->wait();
        std::cout << "断开成功！" << std::endl;
    }
    catch (const mqtt::exception& exc) {
        std::cerr << "MQTT 异常: " << exc.what() << std::endl;
        return 1;
    }
    catch (const std::exception& exc) {
        std::cerr << "JSON 构建异常: " << exc.what() << std::endl;
        return 1;
    }

    return 0;
}
```

### 3. subscriber.cpp（订阅端核心代码）
```cpp
#include <iostream>
#include <cstdlib>
#include <string>
#include <thread>
#include <chrono>
#include "mqtt/async_client.h"
#include "include/json.hpp"

// 简化 JSON 命名空间
using json = nlohmann::json;

// 本地 MQTT 服务器配置
const std::string SERVER_ADDRESS("tcp://127.0.0.1:1883");
const std::string CLIENT_ID("cpp_robot_subscriber_demo");
const std::string TOPIC("robot/status");

// MQTT 回调类：处理消息接收、连接丢失等事件
class callback : public virtual mqtt::callback {
public:
    // 收到消息时的回调函数（解析 JSON 格式的机器人状态）
    void message_arrived(mqtt::const_message_ptr msg) override {
        std::cout << "\n收到机器人状态消息：" << std::endl;
        std::cout << "主题: " << msg->get_topic() << std::endl;
        std::cout << "原始 JSON: " << msg->to_string() << std::endl;

        // 解析 JSON 消息
        try {
            json robot_status = json::parse(msg->to_string());
            std::cout << "\n解析后的机器人状态：" << std::endl;
            std::cout << "机器人ID: " << robot_status["robot_id"] << std::endl;
            std::cout << "运行状态: " << robot_status["running_status"] << std::endl;
            std::cout << "电量: " << robot_status["battery"] << "%" << std::endl;
            std::cout << "位置(x/y/z): " 
                      << robot_status["position"]["x"] << "/" 
                      << robot_status["position"]["y"] << "/" 
                      << robot_status["position"]["z"] << std::endl;
            std::cout << "速度: " << robot_status["speed"] << " m/s" << std::endl;
        }
        catch (const json::parse_error& e) {
            std::cerr << "JSON 解析失败: " << e.what() << std::endl;
        }
    }

    // 连接丢失时的回调函数
    void connection_lost(const std::string& cause) override {
        std::cout << "\n连接丢失！原因: " << (cause.empty() ? "未知" : cause) << std::endl;
        std::cout << "尝试重新连接..." << std::endl;
    }
};

int main(int argc, char* argv[]) {
    try {
        // 创建 MQTT 异步客户端
        mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
        
        // 设置回调函数（处理消息/连接事件）
        callback cb;
        client.set_callback(cb);

        // 配置连接选项
        mqtt::connect_options connOpts;
        connOpts.set_clean_session(true);
        connOpts.set_keep_alive_interval(20);  // 心跳间隔20秒

        // 连接本地 MQTT 服务器
        std::cout << "连接到本地 MQTT 服务器: " << SERVER_ADDRESS << std::endl;
        mqtt::token_ptr conntok = client.connect(connOpts);
        conntok->wait();
        std::cout << "连接成功！" << std::endl;

        // 订阅机器人状态主题（QoS=1）
        std::cout << "订阅主题: " << TOPIC << std::endl;
        client.subscribe(TOPIC, 1)->wait();

        // 等待10秒接收消息（替代回车阻塞，适配脚本运行）
        std::cout << "等待接收机器人状态消息（10秒后自动退出）..." << std::endl;
        for (int i = 0; i < 10; ++i) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // 取消订阅
        std::cout << "取消订阅主题: " << TOPIC << std::endl;
        client.unsubscribe(TOPIC)->wait();

        // 断开连接
        std::cout << "断开连接..." << std::endl;
        client.disconnect()->wait();
        std::cout << "断开成功！" << std::endl;
    }
    catch (const mqtt::exception& exc) {
        std::cerr << "MQTT 异常: " << exc.what() << std::endl;
        return 1;
    }
    catch (const std::exception& exc) {
        std::cerr << "程序异常: " << exc.what() << std::endl;
        return 1;
    }

    return 0;
}
```

### 4. run_test.sh（一键测试脚本）
```bash
#!/bin/bash
set -e  # 遇到错误立即退出

# 定义颜色输出（便于区分日志级别）
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 步骤1：检查 Paho MQTT 库是否安装
echo -e "${YELLOW}=== 检查 Paho MQTT 库 ===${NC}"
if ! ldconfig -p | grep -q "paho-mqttpp3"; then
    echo -e "${RED}错误：未找到 Paho MQTT C++ 库，请先安装！${NC}"
    exit 1
fi

# 步骤2：编译代码
echo -e "\n${YELLOW}=== 开始编译代码 ===${NC}"
mkdir -p build
cd build
cmake ..
make -j$(nproc)  # 多核编译，加快速度
cd ..

# 步骤3：启动订阅端（后台运行，重定向日志）
echo -e "\n${YELLOW}=== 启动 MQTT 订阅端 ===${NC}"
SUBSCRIBER_BIN="./build/bin/subscriber"
PUBLISHER_BIN="./build/bin/publisher"

# 清理残留进程和日志
pkill -f "${SUBSCRIBER_BIN}" || true
rm -f subscriber.log nohup.out

# 后台启动订阅端，日志写入 subscriber.log
nohup ${SUBSCRIBER_BIN} > subscriber.log 2>&1 < /dev/null &
SUBSCRIBER_PID=$!
echo "订阅端已启动，PID: ${SUBSCRIBER_PID}，日志文件: subscriber.log"

# 等待3秒，确保订阅端完成连接+订阅
echo "等待3秒，确保订阅端就绪..."
sleep 3

# 步骤4：启动发布端（前台运行，输出到控制台）
echo -e "\n${YELLOW}=== 启动 MQTT 发布端 ===${NC}"
${PUBLISHER_BIN}

# 等待2秒，确保消息被订阅端接收并写入日志
sleep 2

# 步骤5：验证测试结果
echo -e "\n${YELLOW}=== 验证测试结果 ===${NC}"
if grep -q "收到机器人状态消息：" subscriber.log; then
    echo -e "${GREEN}✅ 测试成功！订阅端已收到发布端的消息${NC}"
    echo -e "\n订阅端日志关键内容："
    grep -A 10 "收到机器人状态消息：" subscriber.log
else
    echo -e "${RED}❌ 测试失败！未收到消息，请检查日志${NC}"
    echo -e "\n订阅端完整日志："
    cat subscriber.log
fi

# 步骤6：清理订阅端进程
echo -e "\n${YELLOW}=== 清理进程 ===${NC}"
if ps -p ${SUBSCRIBER_PID} > /dev/null; then
    kill ${SUBSCRIBER_PID} || true
    echo "订阅端进程（PID: ${SUBSCRIBER_PID}）已终止"
else
    echo "订阅端进程已提前退出（正常）"
fi

echo -e "\n${GREEN}=== 测试流程结束 ===${NC}"
```

## 四、环境准备
### 1. 安装依赖库
```bash
# 安装基础编译工具
sudo apt update && sudo apt install -y build-essential cmake git

# 安装 Mosquitto MQTT 服务器（本地测试用）
sudo apt install -y mosquitto mosquitto-clients

# 安装 OpenSSL（Paho 库依赖）
sudo apt install -y libssl-dev

# 安装 Paho MQTT C 库（基础依赖）
git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
git checkout v1.3.15
cmake -B build -DPAHO_BUILD_STATIC=ON -DPAHO_WITH_SSL=ON
sudo cmake --build build --target install
cd ..

# 安装 Paho MQTT C++ 库
git clone https://github.com/eclipse/paho.mqtt.cpp.git
cd paho.mqtt.cpp
git checkout v1.2.0
cmake -B build -DPAHO_BUILD_STATIC=ON
sudo cmake --build build --target install
cd ..

# 更新系统库缓存
sudo ldconfig
```

### 2. 下载 JSON 头文件
```bash
# 在项目根目录创建 include 文件夹
mkdir -p include

# 下载 nlohmann/json 单头文件
wget -O include/json.hpp https://raw.githubusercontent.com/nlohmann/json/v3.11.2/single_include/nlohmann/json.hpp
```

## 五、使用方法
### 1. 启动本地 MQTT 服务器
```bash
# 启动 Mosquitto 服务
sudo systemctl start mosquitto

# 设置开机自启（可选）
sudo systemctl enable mosquitto

# 验证服务状态（显示 active (running) 即为正常）
sudo systemctl status mosquitto
```

### 2. 一键编译&运行测试
```bash
# 进入项目根目录
cd /path/to/mqtt_cpp_demo

# 给测试脚本添加可执行权限
chmod +x run_test.sh

# 执行一键测试
./run_test.sh
```

### 3. 手动运行（可选）
```bash
# 编译代码（同脚本内逻辑）
mkdir -p build && cd build
cmake .. && make -j$(nproc)
cd ..

# 终端1：启动订阅端
./build/bin/subscriber

# 终端2：启动发布端
./build/bin/publisher
```

## 六、预期运行结果
### 1. 脚本运行成功输出
```
=== 检查 Paho MQTT 库 ===

=== 开始编译代码 ===
-- Configuring done
-- Generating done
-- Build files have been written to: /xxx/mqtt_cpp_demo/build
[ 25%] Building CXX object CMakeFiles/publisher.dir/publisher.cpp.o
[ 50%] Building CXX object CMakeFiles/subscriber.dir/subscriber.cpp.o
[ 75%] Linking CXX executable bin/publisher
[100%] Linking CXX executable bin/subscriber
[100%] Built target publisher
[100%] Built target subscriber

=== 启动 MQTT 订阅端 ===
订阅端已启动，PID: 12345，日志文件: subscriber.log
等待3秒，确保订阅端就绪...

=== 启动 MQTT 发布端 ===
连接到本地 MQTT 服务器: tcp://127.0.0.1:1883
连接成功！

发布机器人状态 JSON 消息：
{
    "battery": 85.5,
    "error_code": 0,
    "position": {
        "x": 10.2,
        "y": 25.8,
        "z": 0.5
    },
    "robot_id": "robot_001",
    "running_status": "normal",
    "speed": 1.2,
    "timestamp": 1765470609
}

消息发布完成！断开连接...
断开成功！

=== 验证测试结果 ===
✅ 测试成功！订阅端已收到发布端的消息

订阅端日志关键内容：
收到机器人状态消息：
主题: robot/status
原始 JSON: {
    "battery": 85.5,
    "error_code": 0,
    "position": {
        "x": 10.2,
        "y": 25.8,
        "z": 0.5
    },
    "robot_id": "robot_001",
    "running_status": "normal",
    "speed": 1.2,
    "timestamp": 1765470609
}

解析后的机器人状态：
机器人ID: robot_001
运行状态: normal
电量: 85.5%
位置(x/y/z): 10.2/25.8/0.5
速度: 1.2 m/s

=== 清理进程 ===
订阅端进程（PID: 12345）已终止

=== 测试流程结束 ===
```

### 2. 关键验证点
- 发布端：成功连接本地 MQTT 服务器，输出格式化的机器人状态 JSON。
- 订阅端：日志中包含“收到机器人状态消息”，且能正确解析 JSON 字段。
- 脚本：最终输出“✅ 测试成功！”。

## 七、常见问题排查
### 1. 报错：Permission denied（脚本无法执行）
```bash
# 给脚本添加可执行权限
chmod +x run_test.sh
```

### 2. 报错：MQTT error [-1]: TCP connect completion failure
- 原因：本地 Mosquitto 未启动，或 1883 端口被占用。
- 解决：
  ```bash
  # 启动 Mosquitto
  sudo systemctl start mosquitto

  # 检查 1883 端口占用
  netstat -tulpn | grep 1883
  ```

### 3. 测试失败：订阅端未收到消息
- 原因1：订阅端提前退出（`cin.get()` 阻塞被重定向打断）→ 已在代码中改为 10 秒循环等待。
- 原因2：MQTT 服务器未正确转发消息 → 用 `mosquitto_sub/mosquitto_pub` 验证：
  ```bash
  # 终端1：订阅主题
  mosquitto_sub -h 127.0.0.1 -t robot/status -v

  # 终端2：发布测试消息
  mosquitto_pub -h 127.0.0.1 -t robot/status -m '{"robot_id":"test"}'
  ```
  若终端1无输出，修改 Mosquitto 配置：
  ```bash
  sudo nano /etc/mosquitto/mosquitto.conf
  # 添加以下内容
  listener 1883 0.0.0.0
  allow_anonymous true
  # 重启服务
  sudo systemctl restart mosquitto
  ```

### 4. 编译报错：json.hpp 找不到
- 原因：`include` 目录缺失或 `json.hpp` 路径错误。
- 解决：确认 `include/json.hpp` 文件存在，且 CMakeLists.txt 中 `include_directories` 配置正确。

## 八、扩展方向
1. 动态更新机器人状态：读取传感器/系统数据，替换固定的电量、位置等字段。
2. 循环发布：发布端添加定时循环，持续上报机器人状态。
3. 消息持久化：配置 MQTT QoS=2，确保消息仅送达一次，避免重复。
4. 增加认证：私有 MQTT 服务器可添加用户名/密码认证，修改 `connOpts.set_user_name()`/`set_password()`。
5. SSL/TLS 加密：对接 HTTPS MQTT 服务器时，配置 SSL 选项（端口 8883）。