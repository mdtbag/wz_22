#include <iostream>
#include <cstdlib>
#include <string>
#include <ctime>  // 用于生成时间戳
#include "mqtt/async_client.h"
// 引入 JSON 头文件
#include "include/json.hpp"

// 本地 MQTT 服务器地址（已搭建的本地服务器）
const std::string SERVER_ADDRESS("tcp://127.0.0.1:1883");
const std::string CLIENT_ID("cpp_robot_publisher_demo");
// 机器人状态主题（更贴合业务场景）
const std::string TOPIC("robot/status");

// 简化命名空间
using json = nlohmann::json;

// 构建机器人状态的 JSON 字符串
std::string build_robot_status_json() {
    // 1. 定义机器人状态数据
    json robot_status;
    robot_status["robot_id"] = "robot_001";          // 机器人ID
    robot_status["timestamp"] = time(nullptr);       // 状态时间戳（秒级）
    robot_status["running_status"] = "normal";       // 运行状态：normal/error/idle
    robot_status["battery"] = 85.5;                  // 电量（%）
    robot_status["position"] = {                     // 位置坐标（x/y/z）
        {"x", 10.2},
        {"y", 25.8},
        {"z", 0.5}
    };
    robot_status["speed"] = 1.2;                     // 移动速度（m/s）
    robot_status["error_code"] = 0;                  // 错误码：0=无错误

    // 2. 转换为格式化的 JSON 字符串（便于阅读）
    // 若追求性能，可改用 robot_status.dump()（无格式化）
    return robot_status.dump(4);
}

int main(int argc, char* argv[]) {
    try {
        // 创建异步客户端
        mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);

        // 配置连接选项
        mqtt::connect_options connOpts;
        connOpts.set_clean_session(true);

        std::cout << "连接到本地 MQTT 服务器: " << SERVER_ADDRESS << std::endl;
        // 连接服务器
        mqtt::token_ptr conntok = client.connect(connOpts);
        conntok->wait();
        std::cout << "连接成功！" << std::endl;

        // 构建机器人状态 JSON 消息
        std::string json_payload = build_robot_status_json();
        std::cout << "\n发布机器人状态 JSON 消息：" << std::endl;
        std::cout << json_payload << std::endl;

        // 构建 MQTT 消息（QoS 1，确保消息至少送达一次）
        mqtt::message_ptr pubmsg = mqtt::make_message(TOPIC, json_payload, 1, false);
        // 发布消息
        client.publish(pubmsg)->wait();

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