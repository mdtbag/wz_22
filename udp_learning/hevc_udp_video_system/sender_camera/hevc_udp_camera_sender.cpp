#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <cstdint>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <chrono>
#include <mutex>

// FFmpeg头文件（HEVC编码）
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <libavutil/time.h>
}

// 全局配置
#define UDP_PORT 3334
#define BUFFER_SIZE 65507          // UDP最大包长
#define HEADER_SIZE 8              // 自定义包头长度（2+2+4）
#define DATA_SIZE (BUFFER_SIZE - HEADER_SIZE) // 单包最大数据长度

// 摄像头配置（可根据设备调整）
#define CAMERA_WIDTH 1280
#define CAMERA_HEIGHT 800
#define CAMERA_FPS 15              // 摄像头采集帧率
#define ENCODE_FPS 15              // 编码发送帧率
#define FRAME_INTERVAL_US (1000000 / ENCODE_FPS)

// 自定义UDP包头（严格遵循协议：2+2+4字节）
struct UdpPacketHeader {
    uint16_t frame_id;     // 2字节：帧编号（递增）
    uint16_t slice_seq;    // 2字节：分片序号
    uint32_t total_size;   // 4字节：帧总字节数
} __attribute__((packed)); // 禁止内存对齐

// 全局变量
std::atomic<bool> is_running(true);
std::atomic<uint16_t> g_frame_id(0); // 2字节帧编号，自动递增
std::mutex codec_mutex;

// 错误信息转换
static std::string av_err_to_string(int errnum) {
    char errbuf[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(errnum, errbuf, sizeof(errbuf));
    return std::string(errbuf);
}

// 提取SPS/PPS参数集
std::vector<uint8_t> extractSPSPPS(AVCodecContext* codec_ctx) {
    std::vector<uint8_t> sps_pps;
    if (!codec_ctx || !codec_ctx->extradata) return sps_pps;

    // HEVC的extradata包含SPS+PPS，直接提取
    sps_pps.resize(codec_ctx->extradata_size);
    memcpy(sps_pps.data(), codec_ctx->extradata, codec_ctx->extradata_size);
    return sps_pps;
}

// 初始化HEVC编码器（强化SPS/PPS配置）
bool initHEVCEncoder(AVCodecContext** codec_ctx, SwsContext** sws_ctx) {
    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_HEVC);
    if (!codec) {
        std::cerr << "[发送端] 未找到HEVC编码器！请安装x265 ❌" << std::endl;
        return false;
    }

    *codec_ctx = avcodec_alloc_context3(codec);
    if (!*codec_ctx) return false;

    // 编码器核心配置（适配摄像头分辨率）
    (*codec_ctx)->width = CAMERA_WIDTH;
    (*codec_ctx)->height = CAMERA_HEIGHT;
    (*codec_ctx)->pix_fmt = AV_PIX_FMT_YUV420P;
    (*codec_ctx)->time_base = av_make_q(1, ENCODE_FPS);
    (*codec_ctx)->framerate = av_make_q(ENCODE_FPS, 1);
    (*codec_ctx)->bit_rate = 800000;       // 800Kbps，平衡实时性和画质
    (*codec_ctx)->gop_size = 15;           // 每15帧一个I帧（实时性优先）
    (*codec_ctx)->keyint_min = 15;
    (*codec_ctx)->max_b_frames = 0;        // 禁用B帧，保证帧序和实时性
    (*codec_ctx)->thread_count = 4;        // 多线程编码，提升速度
    (*codec_ctx)->flags |= AV_CODEC_FLAG_LOW_DELAY;
    (*codec_ctx)->flags |= AV_CODEC_FLAG_GLOBAL_HEADER; // 全局头，简化解码

    // x265编码参数（强化实时性）
    AVDictionary* opts = nullptr;
    av_dict_set(&opts, "preset", "ultrafast", 0);  // 最快编码预设
    av_dict_set(&opts, "tune", "zerolatency", 0);  // 零延迟调优
    av_dict_set(&opts, "repeat-headers", "1", 0);  // 强制每个I帧重复SPS/PPS
    av_dict_set(&opts, "sps-id", "0", 0);          // 固定SPS ID为0
    av_dict_set(&opts, "pps-id", "0", 0);          // 固定PPS ID为0
    av_dict_set(&opts, "crf", "30", 0);            // 质量控制（值越大画质越低，速度越快）

    int ret = avcodec_open2(*codec_ctx, codec, &opts);
    if (ret < 0) {
        std::cerr << "[发送端] 编码器打开失败：" << av_err_to_string(ret) << " ❌" << std::endl;
        avcodec_free_context(codec_ctx);
        av_dict_free(&opts);
        return false;
    }
    av_dict_free(&opts);

    // BGR转YUV格式转换（适配摄像头BGR输出）
    *sws_ctx = sws_getContext(CAMERA_WIDTH, CAMERA_HEIGHT, AV_PIX_FMT_BGR24,
                              CAMERA_WIDTH, CAMERA_HEIGHT, AV_PIX_FMT_YUV420P,
                              SWS_POINT, nullptr, nullptr, nullptr); // 最快插值方式
    if (!*sws_ctx) {
        avcodec_free_context(codec_ctx);
        return false;
    }

    std::cout << "[发送端] HEVC编码器初始化成功 ✅（" 
              << CAMERA_WIDTH << "x" << CAMERA_HEIGHT 
              << "@" << ENCODE_FPS << "fps）" << std::endl;
    return true;
}

// 编码单帧为HEVC码流（I帧前插入SPS/PPS）
bool encodeFrame(AVCodecContext* codec_ctx, SwsContext* sws_ctx, const cv::Mat& frame, std::vector<uint8_t>& hevc_data) {
    hevc_data.clear();
    if (frame.empty()) return false;

    std::lock_guard<std::mutex> lock(codec_mutex);

    // 分配YUV帧
    AVFrame* yuv_frame = av_frame_alloc();
    yuv_frame->format = AV_PIX_FMT_YUV420P;
    yuv_frame->width = CAMERA_WIDTH;
    yuv_frame->height = CAMERA_HEIGHT;
    if (av_frame_get_buffer(yuv_frame, 32) < 0) {
        av_frame_free(&yuv_frame);
        return false;
    }

    // OpenCV BGR转FFmpeg YUV（实时转换，最快方式）
    uint8_t* src_data[] = {const_cast<uint8_t*>(frame.data)};
    int src_linesize[] = {static_cast<int>(frame.step)};
    sws_scale(sws_ctx, src_data, src_linesize, 0, CAMERA_HEIGHT,
              yuv_frame->data, yuv_frame->linesize);

    // 设置帧时间戳（保证编码器时序正确）
    yuv_frame->pts = g_frame_id.load();

    // 强制关键帧（每GOP_SIZE帧插入一个I帧，确保SPS/PPS定期刷新）
    if (g_frame_id.load() % codec_ctx->gop_size == 0) {
        yuv_frame->pict_type = AV_PICTURE_TYPE_I;
    } else {
        yuv_frame->pict_type = AV_PICTURE_TYPE_P;
    }

    // 发送帧到编码器
    int ret = avcodec_send_frame(codec_ctx, yuv_frame);
    if (ret < 0) {
        av_frame_free(&yuv_frame);
        std::cerr << "[发送端] 发送帧到编码器失败：" << av_err_to_string(ret) << std::endl;
        return false;
    }

    // 接收编码后的HEVC码流
    AVPacket pkt = {0};
    ret = avcodec_receive_packet(codec_ctx, &pkt);
    if (ret == 0) {
        // 提取SPS/PPS并插入到I帧开头
        if (yuv_frame->pict_type == AV_PICTURE_TYPE_I) {
            std::vector<uint8_t> sps_pps = extractSPSPPS(codec_ctx);
            if (!sps_pps.empty()) {
                // SPS/PPS + I帧数据
                hevc_data.resize(sps_pps.size() + pkt.size);
                memcpy(hevc_data.data(), sps_pps.data(), sps_pps.size());
                memcpy(hevc_data.data() + sps_pps.size(), pkt.data, pkt.size);
            } else {
                hevc_data.resize(pkt.size);
                memcpy(hevc_data.data(), pkt.data, pkt.size);
            }
        } else {
            hevc_data.resize(pkt.size);
            memcpy(hevc_data.data(), pkt.data, pkt.size);
        }
        av_packet_unref(&pkt);
    } else if (ret == AVERROR(EAGAIN)) {
        av_frame_free(&yuv_frame);
        return true; // 预热阶段无输出，不报错
    } else {
        av_frame_free(&yuv_frame);
        std::cerr << "[发送端] 接收编码数据失败：" << av_err_to_string(ret) << std::endl;
        return false;
    }

    av_frame_free(&yuv_frame);
    return true;
}

// 发送HEVC码流（严格遵循自定义UDP协议）
void sendHEVCStream(int sock_fd, const sockaddr_in& target_addr, const std::vector<uint8_t>& hevc_data) {
    if (hevc_data.empty()) return;

    uint16_t frame_id = g_frame_id.load();
    uint32_t total_size = hevc_data.size();
    uint16_t total_slice = (total_size + DATA_SIZE - 1) / DATA_SIZE;

    // 分片发送每一个UDP包
    for (uint16_t slice_seq = 0; slice_seq < total_slice; slice_seq++) {
        uint8_t udp_buffer[BUFFER_SIZE];
        UdpPacketHeader* header = (UdpPacketHeader*)udp_buffer;

        // 填充包头（网络大端序）
        header->frame_id = htons(frame_id);       // 2字节帧编号（大端）
        header->slice_seq = htons(slice_seq);     // 2字节分片序号（大端）
        header->total_size = htonl(total_size);   // 4字节总长度（大端）

        // 填充HEVC数据
        uint32_t offset = slice_seq * DATA_SIZE;
        uint32_t data_len = std::min((uint32_t)DATA_SIZE, total_size - offset);
        memcpy(udp_buffer + HEADER_SIZE, hevc_data.data() + offset, data_len);

        // 发送UDP包（非阻塞发送，实时性优先）
        ssize_t send_ret = sendto(sock_fd, udp_buffer, HEADER_SIZE + data_len, MSG_DONTWAIT,
               (const sockaddr*)&target_addr, sizeof(target_addr));
        if (send_ret < 0) {
            std::cerr << "[发送端] UDP发送失败：" << strerror(errno) << std::endl;
        }

        // 轻微延时，避免网卡过载（可根据网络调整）
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }

    g_frame_id++; // 帧编号递增
    // 打印发送信息（可选，减少IO开销）
    // std::cout << "[发送端] 发送帧 " << frame_id << "（总大小：" << total_size << "字节，总分片：" << total_slice << "）" << std::endl;
}

// 摄像头采集线程（实时采集+编码+发送）
void cameraCaptureThread(int sock_fd, const sockaddr_in& target_addr) {
    // 打开摄像头（0为默认摄像头，多个摄像头可尝试1、2等）
    cv::VideoCapture cap(0, cv::CAP_V4L2); // 使用V4L2后端（Linux），Windows可省略
    if (!cap.isOpened()) {
        std::cerr << "[发送端] 摄像头打开失败 ❌，请检查摄像头是否可用" << std::endl;
        is_running = false;
        return;
    }

    // 配置摄像头参数（关键：强制设置分辨率和帧率）
    cap.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
    cap.set(cv::CAP_PROP_FPS, CAMERA_FPS);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1); // 缓冲区设为1，降低延迟
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); // 强制MJPG格式，提升采集速度

    // 验证摄像头参数（部分摄像头不支持自定义分辨率/帧率）
    int actual_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int actual_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    int actual_fps = cap.get(cv::CAP_PROP_FPS);
    std::cout << "[发送端] 摄像头初始化成功 ✅" << std::endl;
    std::cout << "[发送端] 实际分辨率：" << actual_width << "x" << actual_height << std::endl;
    std::cout << "[发送端] 实际帧率：" << actual_fps << "fps" << std::endl;

    // 初始化解码器
    AVCodecContext* codec_ctx = nullptr;
    SwsContext* sws_ctx = nullptr;
    if (!initHEVCEncoder(&codec_ctx, &sws_ctx)) {
        cap.release();
        is_running = false;
        return;
    }

    cv::Mat frame;
    int64_t last_send_time = av_gettime();

    // 实时采集循环
    while (is_running) {
        // 读取摄像头帧（阻塞式读取，保证实时性）
        if (!cap.read(frame)) {
            std::cerr << "[发送端] 读取摄像头帧失败 ❌" << std::endl;
            // 尝试重新打开摄像头
            cap.release();
            cap.open(0);
            if (!cap.isOpened()) {
                break;
            }
            // 重新配置参数
            cap.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
            cap.set(cv::CAP_PROP_FPS, CAMERA_FPS);
            continue;
        }

        // 严格控制发送帧率（避免编码过快导致延迟）
        int64_t current_time = av_gettime();
        int64_t delta = current_time - last_send_time;
        if (delta < FRAME_INTERVAL_US) {
            av_usleep(FRAME_INTERVAL_US - delta);
        }
        last_send_time = current_time;

        // 编码+发送（实时处理）
        std::vector<uint8_t> hevc_data;
        if (encodeFrame(codec_ctx, sws_ctx, frame, hevc_data)) {
            sendHEVCStream(sock_fd, target_addr, hevc_data);
        }

        // 检测ESC键退出（可选：显示预览窗口）
        cv::imshow("Camera Preview (HEVC Sender)", frame);
        if (cv::waitKey(1) == 27) {
            std::cout << "[发送端] 收到ESC退出指令 ✅" << std::endl;
            is_running = false;
            break;
        }
    }

    // 释放资源
    cap.release();
    cv::destroyAllWindows();
    avcodec_free_context(&codec_ctx);
    sws_freeContext(sws_ctx);
    std::cout << "[发送端] 摄像头采集线程退出 ✅" << std::endl;
}

int main(int argc, char* argv[]) {
    // 命令行参数检查
    if (argc != 2) {
        std::cerr << "使用方式：./HEVC_UDP_Camera_Sender <目标IP>" << std::endl;
        std::cerr << "示例：./HEVC_UDP_Camera_Sender 10.158.2.207" << std::endl;
        return -1;
    }

    std::string target_ip = argv[1];

    // 创建UDP套接字（非阻塞模式，实时性优先）
    int sock_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (sock_fd < 0) {
        std::cerr << "[发送端] UDP套接字创建失败 ❌：" << strerror(errno) << std::endl;
        return -1;
    }

    // 增大发送缓冲区（避免丢包）
    int buf_size = 2*1024*1024; // 2MB
    int ret = setsockopt(sock_fd, SOL_SOCKET, SO_SNDBUF, &buf_size, sizeof(buf_size));
    if (ret < 0) {
        std::cerr << "[发送端] 设置UDP发送缓冲区失败：" << strerror(errno) << std::endl;
    }

    // 配置目标地址
    sockaddr_in target_addr;
    memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(UDP_PORT);
    if (inet_pton(AF_INET, target_ip.c_str(), &target_addr.sin_addr) <= 0) {
        std::cerr << "[发送端] 目标IP格式错误 ❌：" << target_ip << std::endl;
        close(sock_fd);
        return -1;
    }

    std::cout << "========================" << std::endl;
    std::cout << " HEVC UDP 摄像头发送端（实时视频）" << std::endl;
    std::cout << "========================" << std::endl;
    std::cout << "目标IP：" << target_ip << ":" << UDP_PORT << std::endl;
    std::cout << "摄像头配置：" << CAMERA_WIDTH << "x" << CAMERA_HEIGHT << "@" << ENCODE_FPS << "fps" << std::endl;
    std::cout << "协议格式：帧编号(2B)+分片序号(2B)+总长度(4B)+HEVC数据" << std::endl;
    std::cout << "操作：按ESC键退出" << std::endl;
    std::cout << "========================" << std::endl;

    // 启动摄像头采集线程
    std::thread camera_thread(cameraCaptureThread, sock_fd, target_addr);

    // 主线程：等待退出
    while (is_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 等待线程退出
    if (camera_thread.joinable()) {
        camera_thread.join();
    }

    // 释放资源
    close(sock_fd);
    std::cout << "[发送端] 程序正常退出 ✅" << std::endl;

    return 0;
}