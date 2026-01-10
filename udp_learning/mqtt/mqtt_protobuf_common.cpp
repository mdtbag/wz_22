#include "mqtt_protobuf_common.h"

// 全局控制变量
std::atomic<bool> g_is_running(true);

// 生成当前时间的Protobuf TimeData对象
realtime::TimeData create_time_message() {
    realtime::TimeData time_msg;
    
    // 获取当前时间戳（毫秒）
    auto now = std::chrono::system_clock::now();
    auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    ).count();
    time_msg.set_timestamp_ms(timestamp_ms);

    // 格式化日期和时间（本地时间）
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);

    // 日期：YYYY-MM-DD
    char date_buf[32];
    std::strftime(date_buf, sizeof(date_buf), "%Y-%m-%d", &now_tm);
    time_msg.set_date(date_buf);

    // 时间：HH:MM:SS
    char time_buf[32];
    std::strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &now_tm);
    time_msg.set_time(time_buf);

    // 时区
    time_msg.set_timezone("UTC+8");

    return time_msg;
}

// 获取格式化的当前时间字符串（日志用）
std::string get_current_time_str() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);
    
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &now_tm);
    return std::string(buf);
}