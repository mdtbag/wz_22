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
#define TARGET_WIDTH 1280
#define TARGET_HEIGHT 800
#define TARGET_FPS 10
#define FRAME_INTERVAL_US (1000000 / TARGET_FPS)

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

    // 编码器核心配置
    (*codec_ctx)->width = TARGET_WIDTH;
    (*codec_ctx)->height = TARGET_HEIGHT;
    (*codec_ctx)->pix_fmt = AV_PIX_FMT_YUV420P;
    (*codec_ctx)->time_base = av_make_q(1, TARGET_FPS);
    (*codec_ctx)->framerate = av_make_q(TARGET_FPS, 1);
    (*codec_ctx)->bit_rate = 200000;       // 200Kbps，平衡画质和带宽
    (*codec_ctx)->gop_size = 10; // 每10帧一个I帧（强制刷新SPS/PPS）
    (*codec_ctx)->keyint_min = 10;
    (*codec_ctx)->max_b_frames = 0;        // 禁用B帧，保证帧序
    (*codec_ctx)->thread_count = 2;
    (*codec_ctx)->flags |= AV_CODEC_FLAG_LOW_DELAY;
    (*codec_ctx)->flags |= AV_CODEC_FLAG_GLOBAL_HEADER; // 全局头，简化解码

    // x265编码参数（强化SPS/PPS重复）
    AVDictionary* opts = nullptr;
    av_dict_set(&opts, "preset", "fast", 0);
    av_dict_set(&opts, "tune", "zerolatency", 0);
    av_dict_set(&opts, "repeat-headers", "1", 0); // 强制每个I帧重复SPS/PPS
    av_dict_set(&opts, "sps-id", "0", 0);         // 固定SPS ID为0
    av_dict_set(&opts, "pps-id", "0", 0);         // 固定PPS ID为0
    av_dict_set(&opts, "crf", "35", 0);

    int ret = avcodec_open2(*codec_ctx, codec, &opts);
    if (ret < 0) {
        std::cerr << "[发送端] 编码器打开失败：" << av_err_to_string(ret) << " ❌" << std::endl;
        avcodec_free_context(codec_ctx);
        av_dict_free(&opts);
        return false;
    }
    av_dict_free(&opts);

    // BGR转YUV格式转换
    *sws_ctx = sws_getContext(TARGET_WIDTH, TARGET_HEIGHT, AV_PIX_FMT_BGR24,
                              TARGET_WIDTH, TARGET_HEIGHT, AV_PIX_FMT_YUV420P,
                              SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (!*sws_ctx) {
        avcodec_free_context(codec_ctx);
        return false;
    }

    std::cout << "[发送端] HEVC编码器初始化成功 ✅（640x480@5fps）" << std::endl;
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
    yuv_frame->width = TARGET_WIDTH;
    yuv_frame->height = TARGET_HEIGHT;
    if (av_frame_get_buffer(yuv_frame, 32) < 0) {
        av_frame_free(&yuv_frame);
        return false;
    }

    // OpenCV BGR转FFmpeg YUV
    uint8_t* src_data[] = {const_cast<uint8_t*>(frame.data)};
    int src_linesize[] = {static_cast<int>(frame.step)};
    sws_scale(sws_ctx, src_data, src_linesize, 0, TARGET_HEIGHT,
              yuv_frame->data, yuv_frame->linesize);

    // 设置帧时间戳
    yuv_frame->pts = g_frame_id.load();

    // 强制关键帧（每10帧插入一个I帧，确保SPS/PPS定期刷新）
    if (g_frame_id.load() % 10 == 0) {
        yuv_frame->pict_type = AV_PICTURE_TYPE_I;
        codec_ctx->gop_size = 10; // 每10帧一个I帧
    } else {
        yuv_frame->pict_type = AV_PICTURE_TYPE_P;
    }

    // 发送帧到编码器
    int ret = avcodec_send_frame(codec_ctx, yuv_frame);
    if (ret < 0) {
        av_frame_free(&yuv_frame);
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

    std::cout << "[发送端] 发送帧 " << frame_id << "（总大小：" << total_size << "字节，总分片：" << total_slice << "）" << std::endl;

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

        // 发送UDP包
        sendto(sock_fd, udp_buffer, HEADER_SIZE + data_len, MSG_DONTWAIT,
               (const sockaddr*)&target_addr, sizeof(target_addr));

        // 避免发包过快
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    g_frame_id++; // 帧编号递增
}

// 视频读取线程（无缝循环）
void videoReadThread(const std::string& video_path, int sock_fd, const sockaddr_in& target_addr) {
    cv::VideoCapture cap(video_path, cv::CAP_FFMPEG);
    if (!cap.isOpened()) {
        std::cerr << "[发送端] 视频文件打开失败 ❌：" << video_path << std::endl;
        is_running = false;
        return;
    }

    // 视频信息
    int video_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int video_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    int total_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    std::cout << "[发送端] 视频信息 ✅ 分辨率：" << video_width << "x" << video_height 
              << " 总帧数：" << total_frames << std::endl;

    // 初始化解码器
    AVCodecContext* codec_ctx = nullptr;
    SwsContext* sws_ctx = nullptr;
    if (!initHEVCEncoder(&codec_ctx, &sws_ctx)) {
        cap.release();
        is_running = false;
        return;
    }

    cv::Mat frame, scaled_frame;
    int64_t last_send_time = av_gettime();

    while (is_running) {
        // 读取视频帧（无缝循环）
        if (!cap.read(frame)) {
            cap.set(cv::CAP_PROP_POS_FRAMES, 0); // 重置到开头
            if (!cap.read(frame)) break;
        }

        // 缩放为目标分辨率
        cv::resize(frame, scaled_frame, cv::Size(TARGET_WIDTH, TARGET_HEIGHT), 0, 0, cv::INTER_LINEAR);

        // 严格控制发送帧率（5fps）
        int64_t current_time = av_gettime();
        int64_t delta = current_time - last_send_time;
        if (delta < FRAME_INTERVAL_US) {
            av_usleep(FRAME_INTERVAL_US - delta);
        }
        last_send_time = current_time;

        // 编码+发送
        std::vector<uint8_t> hevc_data;
        if (encodeFrame(codec_ctx, sws_ctx, scaled_frame, hevc_data)) {
            sendHEVCStream(sock_fd, target_addr, hevc_data);
        }
    }

    // 释放资源
    cap.release();
    avcodec_free_context(&codec_ctx);
    sws_freeContext(sws_ctx);
    std::cout << "[发送端] 视频读取线程退出 ✅" << std::endl;
}

int main(int argc, char* argv[]) {
    // 命令行参数检查
    if (argc != 3) {
        std::cerr << "使用方式：./HEVC_UDP_Sender <视频路径> <目标IP>" << std::endl;
        std::cerr << "示例：./HEVC_UDP_Sender ./test.mp4 10.158.2.207" << std::endl;
        return -1;
    }

    std::string video_path = argv[1];
    std::string target_ip = argv[2];

    // 创建UDP套接字
    int sock_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (sock_fd < 0) {
        std::cerr << "[发送端] UDP套接字创建失败 ❌：" << strerror(errno) << std::endl;
        return -1;
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
    std::cout << " HEVC UDP 发送端（自定义协议）" << std::endl;
    std::cout << "========================" << std::endl;
    std::cout << "目标IP：" << target_ip << ":" << UDP_PORT << std::endl;
    std::cout << "视频路径：" << video_path << std::endl;
    std::cout << "协议格式：帧编号(2B)+分片序号(2B)+总长度(4B)+HEVC数据" << std::endl;
    std::cout << "操作：按ESC键退出" << std::endl;
    std::cout << "========================" << std::endl;

    // 启动视频读取线程
    std::thread video_thread(videoReadThread, video_path, sock_fd, target_addr);

    // 主线程：等待ESC退出
    while (is_running) {
        if (cv::waitKey(1) == 27) {
            std::cout << "\n[发送端] 收到退出指令，正在停止..." << std::endl;
            is_running = false;
            break;
        }
    }

    // 等待线程退出
    if (video_thread.joinable()) {
        video_thread.join();
    }

    // 释放资源
    close(sock_fd);
    cv::destroyAllWindows();
    std::cout << "[发送端] 程序正常退出 ✅" << std::endl;

    return 0;
}