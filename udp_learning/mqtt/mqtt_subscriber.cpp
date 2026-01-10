#include "mqtt_protobuf_common.h"
#include <cstring>
#include <mutex>
#include <errno.h>  

std::mutex print_mutex; 

// MQTT消息接收回调函数
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

// MQTT订阅线程
void subscribe_thread() {
    // 初始化mosquitto库
    int lib_init = mosquitto_lib_init();
    if (lib_init != MOSQ_ERR_SUCCESS) {
        std::cerr << "[" << get_current_time_str() << "] MQTT库初始化失败 ❌" << std::endl;
        g_is_running = false;
        return;
    }

    // 创建MQTT客户端
    struct mosquitto* mosq = mosquitto_new(MQTT_CLIENT_ID_SUBSCRIBER, true, nullptr);
    if (!mosq) {
        std::cerr << "[" << get_current_time_str() << "] 创建MQTT订阅客户端失败 ❌" << std::endl;
        mosquitto_lib_cleanup();
        g_is_running = false;
        return;
    }

    // 设置消息接收回调
    mosquitto_message_callback_set(mosq, on_message_callback);

    // 终极修复：直接连接本地MQTT服务器，跳过复杂解析
    int ret = mosquitto_connect(mosq, 
                                "localhost",  // 本地服务器地址
                                1883,         // MQTT默认端口
                                60);          // 保持连接超时
    if (ret != MOSQ_ERR_SUCCESS) {
        std::cerr << "[" << get_current_time_str() << "] 连接MQTT服务器失败：" 
                  << mosquitto_strerror(ret) << " ❌" << std::endl;
        std::cerr << "提示：请先执行 sudo systemctl start mosquitto 启动MQTT服务器" << std::endl;
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        g_is_running = false;
        return;
    }

    // 订阅Topic
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

    // 非阻塞循环处理MQTT消息
    while (g_is_running) {
        ret = mosquitto_loop(mosq, 100, 1); 
        if (ret != MOSQ_ERR_SUCCESS && ret != EINTR) {
            std::cerr << "[" << get_current_time_str() << "] MQTT循环出错：" 
                      << mosquitto_strerror(ret) << " ❌" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // 清理资源
    mosquitto_unsubscribe(mosq, nullptr, MQTT_TOPIC);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    std::cout << "[" << get_current_time_str() << "] MQTT订阅者退出 ✅" << std::endl;
}

int main() {
    // 设置信号处理
    signal(SIGINT, [](int sig) {
        (void)sig;
        g_is_running = false;
        std::cout << "\n[" << get_current_time_str() << "] 收到退出指令 ✅" << std::endl;
    });

    // 启动订阅线程
    std::thread sub_thd(subscribe_thread);

    if (sub_thd.joinable()) {
        sub_thd.join();
    }

    std::cout << "[" << get_current_time_str() << "] 订阅者程序退出 ✅" << std::endl;
    return 0;
}