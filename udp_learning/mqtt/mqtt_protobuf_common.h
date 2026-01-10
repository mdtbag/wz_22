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

// MQTT配置（可根据实际环境修改）
#define MQTT_BROKER "tcp://localhost:1883"  // MQTT服务器地址
#define MQTT_CLIENT_ID_PUBLISHER "time_publisher_001"
#define MQTT_CLIENT_ID_SUBSCRIBER "time_subscriber_001"
#define MQTT_TOPIC "realtime/time"          // 时间数据Topic
#define MQTT_QOS 1                          // QoS 1（至少一次送达）
#define PUBLISH_INTERVAL_MS 1000            // 发布频率：1秒/次

// 全局控制变量
extern std::atomic<bool> g_is_running;

// 时间工具函数：生成当前时间的Protobuf对象
realtime::TimeData create_time_message();

// 错误处理工具函数
std::string get_current_time_str();

#endif // MQTT_PROTOBUF_COMMON_H