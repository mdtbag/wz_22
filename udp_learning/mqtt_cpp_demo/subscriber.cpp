#include <iostream>
#include <cstdlib>
#include <string>
#include <thread>
#include <chrono>
// 先引入 MQTT 头文件（确保 mqtt 命名空间可用）
#include "mqtt/async_client.h"
// 引入 JSON 头文件
#include "include/json.hpp"

// 简化 JSON 命名空间
using json = nlohmann::json;

// MQTT 服务器地址、客户端ID、订阅主题（匹配发布端的 robot/status）
const std::string SERVER_ADDRESS("tcp://127.0.0.1:1883");
const std::string CLIENT_ID("cpp_robot_subscriber_demo");
const std::string TOPIC("robot/status");

// 回调类：处理收到的消息、连接状态等事件（核心：所有回调函数必须在类内部）
class callback : public virtual mqtt::callback {
public:
    // 收到消息时触发（正确定义在类内部，带 mqtt 命名空间）
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

    // 连接丢失时触发
    void connection_lost(const std::string& cause) override {
        std::cout << "\n连接丢失！原因: " << (cause.empty() ? "未知" : cause) << std::endl;
        std::cout << "尝试重新连接..." << std::endl;
    }
};
int main(int argc, char* argv[]) {
    try {
        // 创建异步客户端
        mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
        // 设置回调
        callback cb;
        client.set_callback(cb);

        // 配置连接选项
        mqtt::connect_options connOpts;
        connOpts.set_clean_session(true);
        connOpts.set_keep_alive_interval(20);

        std::cout << "连接到本地 MQTT 服务器: " << SERVER_ADDRESS << std::endl;
        // 连接服务器
        mqtt::token_ptr conntok = client.connect(connOpts);
        conntok->wait();
        std::cout << "连接成功！" << std::endl;

        // 订阅主题
        std::cout << "订阅主题: " << TOPIC << std::endl;
        client.subscribe(TOPIC, 1)->wait();
        std::cout << "等待接收机器人状态消息（10秒后自动退出）..." << std::endl;

        // 核心修改：循环等待10秒，期间持续接收消息（替代 cin.get()）
        // 用 sleep_for 分段等待，避免单次 sleep 导致无法响应消息
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