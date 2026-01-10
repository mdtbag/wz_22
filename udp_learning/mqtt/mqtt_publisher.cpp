#include "mqtt_protobuf_common.h"
#include <cstring>
#include <errno.h>  

// MQTT发布回调函数
void on_publish_callback(struct mosquitto* mosq, void* userdata, int mid) {
    (void)mosq;
    (void)userdata;
    std::cout << "[" << get_current_time_str() << "] 消息发布成功（mid：" << mid << "）✅" << std::endl;
}

// MQTT发布线程
void publish_thread() {
    // 初始化mosquitto库
    int lib_init = mosquitto_lib_init();
    if (lib_init != MOSQ_ERR_SUCCESS) {
        std::cerr << "[" << get_current_time_str() << "] MQTT库初始化失败 ❌" << std::endl;
        g_is_running = false;
        return;
    }

    // 创建MQTT客户端
    struct mosquitto* mosq = mosquitto_new(MQTT_CLIENT_ID_PUBLISHER, true, nullptr);
    if (!mosq) {
        std::cerr << "[" << get_current_time_str() << "] 创建MQTT发布客户端失败 ❌" << std::endl;
        mosquitto_lib_cleanup();
        g_is_running = false;
        return;
    }

    // 设置发布回调
    mosquitto_publish_callback_set(mosq, on_publish_callback);

    // 简化连接：直接连接localhost:1883
    int ret = mosquitto_connect(mosq, 
                                "localhost",  // 本地服务器
                                1883,         // 默认端口
                                60);          // 超时时间
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

    // 发布循环
    while (g_is_running) {
        // 生成实时时间Protobuf对象
        realtime::TimeData time_msg = create_time_message();

        // Protobuf序列化为二进制
        std::string binary_data;
        if (!time_msg.SerializeToString(&binary_data)) {
            std::cerr << "[" << get_current_time_str() << "] Protobuf序列化失败 ❌" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(PUBLISH_INTERVAL_MS));
            continue;
        }

        // MQTT发布二进制数据
        ret = mosquitto_publish(mosq, 
                                nullptr,          
                                MQTT_TOPIC,       
                                binary_data.size(),
                                binary_data.data(),
                                MQTT_QOS,         
                                false);           
        if (ret != MOSQ_ERR_SUCCESS) {
            std::cerr << "[" << get_current_time_str() << "] MQTT发布失败：" 
                      << mosquitto_strerror(ret) << " ❌" << std::endl;
        }

        // 控制发布频率
        std::this_thread::sleep_for(std::chrono::milliseconds(PUBLISH_INTERVAL_MS));
    }

    // 清理资源
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    std::cout << "[" << get_current_time_str() << "] MQTT发布者退出 ✅" << std::endl;
}

int main() {
    // 设置信号处理
    signal(SIGINT, [](int sig) {
        (void)sig;
        g_is_running = false;
        std::cout << "\n[" << get_current_time_str() << "] 收到退出指令 ✅" << std::endl;
    });

    // 启动发布线程
    std::thread pub_thd(publish_thread);

    if (pub_thd.joinable()) {
        pub_thd.join();
    }

    std::cout << "[" << get_current_time_str() << "] 发布者程序退出 ✅" << std::endl;
    return 0;
}