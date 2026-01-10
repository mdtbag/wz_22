#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <map>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <unordered_map>

// FFmpeg相关头文件
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <libavutil/hwcontext.h>
}

// 修复av_err2str的临时数组问题
static std::string av_err_to_string(int errnum) {
    char errbuf[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(errnum, errbuf, sizeof(errbuf));
    return std::string(errbuf);
}

// 自定义UDP数据包头部结构
struct UdpPacketHeader {
    uint16_t frame_id;      // 帧编号（递增）2字节
    uint16_t slice_seq;     // 当前帧内分片序号 2字节
    uint32_t total_frame_size; // 当前帧总字节数 4字节
} __attribute__((packed));

// 全局配置（同步发送端）
#define UDP_PORT 3334               
#define BUFFER_SIZE 65507           
#define SLICE_DATA_SIZE (BUFFER_SIZE - sizeof(UdpPacketHeader))
#define DISPLAY_WIDTH 1280           
#define DISPLAY_HEIGHT 800
#define DECODE_THREAD_NUM 4          // 解码线程数
#define MAX_DECODE_QUEUE 30          // 解码队列最大长度
#define FRAME_TIMEOUT_MS 50          // 帧超时时间
#define CLEAN_INTERVAL_MS 100        // 超时清理间隔

// 全局控制变量
std::atomic<bool> is_running(true);

// 帧缓冲区（极简+优化）
struct FrameBuffer {
    std::vector<uint8_t> data;
    uint32_t recv_size = 0;
    uint32_t total_size = 0;
    std::chrono::steady_clock::time_point start;
};

// 解码任务结构体
struct DecodeTask {
    std::vector<uint8_t> hevc_data;
    uint16_t frame_id;
};

// 全局容器（带锁）
std::unordered_map<uint16_t, FrameBuffer> frame_buffers;
std::mutex buffer_mutex;
std::queue<DecodeTask> decode_queue;
std::mutex decode_queue_mutex;
std::condition_variable decode_queue_cv;

// 全局解码器（复用）
AVCodecContext* codec_ctx = nullptr;
SwsContext* sws_ctx = nullptr;
AVFrame* yuv_frame = nullptr;
AVFrame* bgr_frame = nullptr;
AVPacket* pkt = nullptr;
cv::Mat display_frame;               // 预分配显示帧
std::mutex display_mutex;

// 初始化硬件加速上下文（自动适配）
static AVBufferRef* create_hw_device_context(enum AVHWDeviceType type) {
    AVBufferRef* hw_device_ctx = nullptr;
    int ret = av_hwdevice_ctx_create(&hw_device_ctx, type, nullptr, nullptr, 0);
    if (ret < 0) {
        std::cerr << "[接收端] 创建硬件加速上下文失败(" << av_hwdevice_get_type_name(type) << ")：" << av_err_to_string(ret) << std::endl;
        return nullptr;
    }
    std::cout << "[接收端] 启用硬件加速：" << av_hwdevice_get_type_name(type) << std::endl;
    return hw_device_ctx;
}

// 初始化解码器（极致快+硬件加速+兼容旧版FFmpeg）
bool initHEVCDecoder() {
    // 查找HEVC解码器
    const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
    if (!codec) {
        std::cerr << "[接收端] 找不到HEVC解码器！" << std::endl;
        return false;
    }

    // 分配解码器上下文
    codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx) {
        std::cerr << "[接收端] 分配解码器上下文失败！" << std::endl;
        return false;
    }

    // 极致优化的解码参数
    codec_ctx->width = DISPLAY_WIDTH;
    codec_ctx->height = DISPLAY_HEIGHT;
    codec_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
    codec_ctx->thread_count = DECODE_THREAD_NUM;  // 多线程解码
    codec_ctx->thread_type = FF_THREAD_FRAME;     // 帧级多线程
    codec_ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;
    codec_ctx->error_concealment = 0;             // 关闭错误隐藏
    codec_ctx->skip_loop_filter = AVDISCARD_ALL;  // 跳过所有环路滤波
    codec_ctx->skip_idct = AVDISCARD_ALL;         // 跳过所有IDCT变换
    codec_ctx->skip_frame = AVDISCARD_NONREF;     // 跳过非参考帧
    codec_ctx->flags2 |= AV_CODEC_FLAG2_FAST;     // 修复：使用FLAG2_FAST（兼容新版FFmpeg）

    // 尝试硬件加速（自动检测）
    enum AVHWDeviceType hw_type = av_hwdevice_find_type_by_name("cuda");
    if (hw_type == AV_HWDEVICE_TYPE_NONE) {
        hw_type = av_hwdevice_find_type_by_name("vaapi");
    }
    if (hw_type != AV_HWDEVICE_TYPE_NONE) {
        AVBufferRef* hw_device_ctx = create_hw_device_context(hw_type);
        if (hw_device_ctx) {
            codec_ctx->hw_device_ctx = av_buffer_ref(hw_device_ctx);
            av_buffer_unref(&hw_device_ctx);
        }
    }

    // 极致快解码参数（兼容旧版FFmpeg）
    AVDictionary* opts = nullptr;
    av_dict_set(&opts, "ignore_errors", "1", 0);      // 忽略解码错误
    av_dict_set(&opts, "low_delay", "1", 0);          // 低延迟
    av_dict_set(&opts, "nal_hrd", "none", 0);         // 关闭HRD检查

    // 打开解码器
    int ret = avcodec_open2(codec_ctx, codec, &opts);
    if (ret < 0) {
        std::cerr << "[接收端] 打开解码器失败：" << av_err_to_string(ret) << std::endl;
        avcodec_free_context(&codec_ctx);
        av_dict_free(&opts);
        return false;
    }
    av_dict_free(&opts);

    // 预分配复用的帧和包
    yuv_frame = av_frame_alloc();
    bgr_frame = av_frame_alloc();
    pkt = av_packet_alloc();
    if (!yuv_frame || !bgr_frame || !pkt) {
        std::cerr << "[接收端] 分配帧/包内存失败！" << std::endl;
        return false;
    }

    // 初始化BGR帧缓冲区（复用）
    bgr_frame->format = AV_PIX_FMT_BGR24;
    bgr_frame->width = DISPLAY_WIDTH;
    bgr_frame->height = DISPLAY_HEIGHT;
    av_frame_get_buffer(bgr_frame, 32);

    // 最快的格式转换（POINT插值）
    sws_ctx = sws_getContext(DISPLAY_WIDTH, DISPLAY_HEIGHT, AV_PIX_FMT_YUV420P,
                              DISPLAY_WIDTH, DISPLAY_HEIGHT, AV_PIX_FMT_BGR24,
                              SWS_POINT, nullptr, nullptr, nullptr);
    if (!sws_ctx) {
        std::cerr << "[接收端] 初始化格式转换上下文失败！" << std::endl;
        avcodec_free_context(&codec_ctx);
        return false;
    }

    // 预分配显示帧（避免频繁创建）
    display_frame = cv::Mat(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC3);

    std::cout << "[接收端] 解码器初始化成功 ✅（" << DISPLAY_WIDTH << "x" << DISPLAY_HEIGHT << "/多线程解码/硬件加速）" << std::endl;
    return true;
}

// 解码线程（独立线程，避免阻塞接收）
void decodeThread() {
    while (is_running) {
        DecodeTask task;
        {
            std::unique_lock<std::mutex> lock(decode_queue_mutex);
            // 等待解码任务或退出
            decode_queue_cv.wait(lock, [&]() {
                return !is_running || !decode_queue.empty();
            });

            if (!is_running && decode_queue.empty()) break;
            if (decode_queue.empty()) continue;

            task = std::move(decode_queue.front());
            decode_queue.pop();
        }

        if (task.hevc_data.empty()) continue;

        // 复用AVPacket，避免重新分配
        av_packet_unref(pkt);
        pkt->data = const_cast<uint8_t*>(task.hevc_data.data());
        pkt->size = task.hevc_data.size();

        // 发送码流到解码器
        int ret = avcodec_send_packet(codec_ctx, pkt);
        if (ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
            std::cerr << "[接收端] 帧 " << task.frame_id << " 发送码流失败：" << av_err_to_string(ret) << std::endl;
            continue;
        }

        // 接收解码帧
        av_frame_unref(yuv_frame);
        ret = avcodec_receive_frame(codec_ctx, yuv_frame);
        if (ret == 0) {
            // 最快格式转换（直接写入预分配的BGR帧）
            sws_scale(sws_ctx, (const uint8_t* const*)yuv_frame->data, yuv_frame->linesize,
                      0, DISPLAY_HEIGHT, bgr_frame->data, bgr_frame->linesize);

            // 直接拷贝到显示帧（避免cv::Mat重新分配）
            {
                std::lock_guard<std::mutex> lock(display_mutex);
                memcpy(display_frame.data, bgr_frame->data[0], DISPLAY_WIDTH * DISPLAY_HEIGHT * 3);
            }

            std::cout << "[接收端] 帧 " << task.frame_id << " 解码完成 ✅" << std::endl;
        } else if (ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
            std::cerr << "[接收端] 帧 " << task.frame_id << " 解码失败：" << av_err_to_string(ret) << std::endl;
        }
    }

    std::cout << "[接收端] 解码线程退出 ✅" << std::endl;
}

// 清理超时帧（批量清理，减少锁竞争）
void cleanTimeoutFrames() {
    while (is_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(CLEAN_INTERVAL_MS));
        
        std::lock_guard<std::mutex> lock(buffer_mutex);
        auto now = std::chrono::steady_clock::now();
        
        for (auto it = frame_buffers.begin(); it != frame_buffers.end();) {
            auto& fb = it->second;
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - fb.start).count();
            
            if (duration > FRAME_TIMEOUT_MS) {
                std::cerr << "[接收端] 帧 " << it->first << " 接收超时（" << duration << "ms），丢弃 ❌" << std::endl;
                it = frame_buffers.erase(it);
            } else {
                ++it;
            }
        }
    }
}

// 处理UDP包（修复类型警告）
void handlePacket(uint8_t* packet, int len) {
    // 修复：将len转为无符号类型，避免有符号/无符号比较警告
    uint32_t packet_len = static_cast<uint32_t>(len);
    if (packet_len < sizeof(UdpPacketHeader)) {
        std::cerr << "[接收端] 收到无效数据包（长度不足），丢弃 ❌" << std::endl;
        return;
    }

    // 解析头部
    UdpPacketHeader* header = (UdpPacketHeader*)packet;
    uint16_t frame_id = ntohs(header->frame_id);
    uint16_t slice_seq = ntohs(header->slice_seq);
    uint32_t total_size = ntohl(header->total_frame_size);
    uint32_t data_len = packet_len - sizeof(UdpPacketHeader);

    std::lock_guard<std::mutex> lock(buffer_mutex);

    // 初始化缓冲区
    if (frame_buffers.find(frame_id) == frame_buffers.end()) {
        FrameBuffer fb;
        fb.data.resize(total_size);
        fb.total_size = total_size;
        fb.recv_size = 0;
        fb.start = std::chrono::steady_clock::now();
        frame_buffers[frame_id] = fb;
        std::cout << "[接收端] 初始化帧 " << frame_id << " 缓冲区（总大小：" << total_size << "字节）" << std::endl;
    }

    FrameBuffer& fb = frame_buffers[frame_id];
    
    // 拼接数据
    uint32_t offset = slice_seq * SLICE_DATA_SIZE;
    if (offset + data_len <= fb.total_size) {
        memcpy(fb.data.data() + offset, packet + sizeof(UdpPacketHeader), data_len);
        fb.recv_size += data_len;

        // 收全后加入解码队列
        if (fb.recv_size == fb.total_size) {
            std::cout << "[接收端] 帧 " << frame_id << " 接收完成，加入解码队列 ✅" << std::endl;
            
            // 加入解码队列
            {
                std::lock_guard<std::mutex> q_lock(decode_queue_mutex);
                if (decode_queue.size() < MAX_DECODE_QUEUE) {
                    decode_queue.push({std::move(fb.data), frame_id});
                    decode_queue_cv.notify_one();
                } else {
                    std::cerr << "[接收端] 解码队列满，丢弃帧 " << frame_id << " ❌" << std::endl;
                }
            }
            
            frame_buffers.erase(frame_id);
        }
    } else {
        std::cerr << "[接收端] 帧 " << frame_id << " 的分片 " << slice_seq << " 超出范围，丢弃 ❌" << std::endl;
    }
}

// 接收线程（只负责收包，轻量化）
void receiveThread() {
    // 创建UDP套接字
    int sock_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (sock_fd < 0) {
        std::cerr << "[接收端] 创建UDP套接字失败：" << strerror(errno) << std::endl;
        is_running = false;
        return;
    }

    // 增大接收缓冲区
    int buf_size = 4*1024*1024; // 4MB缓冲区
    int ret = setsockopt(sock_fd, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size));
    if (ret < 0) {
        std::cerr << "[接收端] 设置UDP接收缓冲区失败：" << strerror(errno) << std::endl;
    } else {
        std::cout << "[接收端] UDP接收缓冲区设置为4MB ✅" << std::endl;
    }

    // 绑定端口
    sockaddr_in recv_addr;
    memset(&recv_addr, 0, sizeof(recv_addr));
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_port = htons(UDP_PORT);
    recv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    ret = bind(sock_fd, (struct sockaddr*)&recv_addr, sizeof(recv_addr));
    if (ret < 0) {
        std::cerr << "[接收端] 绑定端口 " << UDP_PORT << " 失败：" << strerror(errno) << std::endl;
        close(sock_fd);
        is_running = false;
        return;
    }

    std::cout << "[接收端] UDP绑定成功 ✅（端口：" << UDP_PORT << "）" << std::endl;
    std::cout << "[接收端] 等待数据...（按ESC关闭）" << std::endl;

    uint8_t buffer[BUFFER_SIZE];
    sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);

    // 接收循环（轻量化）
    while (is_running) {
        ssize_t len = recvfrom(sock_fd, buffer, BUFFER_SIZE, 0, (struct sockaddr*)&sender_addr, &sender_len);
        if (len > 0) {
            // 快速处理包，不阻塞
            handlePacket(buffer, static_cast<int>(len));
        } else if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "[接收端] UDP接收失败：" << strerror(errno) << std::endl;
            break;
        }
        usleep(10); // 极低CPU占用
    }

    close(sock_fd);
    std::cout << "[接收端] UDP接收线程退出 ✅" << std::endl;
}

// 显示线程（独立线程，避免解码阻塞显示）
void displayThread() {
    cv::namedWindow("HEVC Receiver (极致流畅)", cv::WINDOW_NORMAL);
    cv::resizeWindow("HEVC Receiver (极致流畅)", DISPLAY_WIDTH, DISPLAY_HEIGHT);

    while (is_running) {
        {
            std::lock_guard<std::mutex> lock(display_mutex);
            if (!display_frame.empty()) {
                cv::imshow("HEVC Receiver (极致流畅)", display_frame);
            }
        }
        // 短延时，保证显示流畅
        if (cv::waitKey(1) == 27) {
            std::cout << "[接收端] 收到退出指令 ✅" << std::endl;
            is_running = false;
            break;
        }
    }

    cv::destroyAllWindows();
    std::cout << "[接收端] 显示线程退出 ✅" << std::endl;
}

int main() {
    // 初始化解码器
    if (!initHEVCDecoder()) {
        std::cerr << "[接收端] 解码器初始化失败，程序退出 ❌" << std::endl;
        return -1;
    }

    // 启动各线程
    std::thread recv_thd(receiveThread);
    std::thread decode_thd(decodeThread);
    std::thread clean_thd(cleanTimeoutFrames);
    std::thread display_thd(displayThread);

    // 等待退出
    display_thd.join();
    is_running = false;

    // 唤醒解码线程
    decode_queue_cv.notify_one();

    // 等待其他线程退出
    recv_thd.join();
    decode_thd.join();
    clean_thd.join();

    // 释放资源
    avcodec_free_context(&codec_ctx);
    sws_freeContext(sws_ctx);
    av_frame_free(&yuv_frame);
    av_frame_free(&bgr_frame);
    av_packet_free(&pkt);

    std::cout << "[接收端] 程序退出 ✅" << std::endl;
    return 0;
}