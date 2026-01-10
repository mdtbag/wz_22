#include "VideoPlayer.h"
#include <iostream>
#include <fstream>
VideoPlayer::VideoPlayer() : isRunning_(false), camWidth_(640), camHeight_(480) {}

VideoPlayer::~VideoPlayer() {
    stop();
}

bool VideoPlayer::init(VideoSourceType type, const std::string& source, int width, int height) {
    // 先停止之前的播放
    stop();

    // 保存摄像头参数
    camWidth_ = width;
    camHeight_ = height;

    // 根据类型初始化视频源
    switch (type) {
        case VideoSourceType::LOCAL_CAMERA: {
            // 本地摄像头：source转为int索引
            int camIndex = std::stoi(source);
            cap_.open(camIndex);
            if (!cap_.isOpened()) {
                std::cerr << "错误：无法打开摄像头 " << camIndex << std::endl;
                return false;
            }
            // 设置摄像头分辨率
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, camWidth_);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, camHeight_);
            cap_.set(cv::CAP_PROP_FPS, 30);
            break;
        }
        case VideoSourceType::LOCAL_FILE: {
    // 本地视频文件：source为文件路径，强制使用FFmpeg后端
    // 先检查文件是否存在
    std::ifstream fileCheck(source);
    if (!fileCheck.good()) {
        std::cerr << "错误：文件不存在 " << source << std::endl;
        return false;
    }
    fileCheck.close();

    // 强制使用FFmpeg后端打开文件（避开GStreamer问题）
    cap_.open(source, cv::CAP_FFMPEG);
    if (!cap_.isOpened()) {
        std::cerr << "错误：无法打开视频文件 " << source << std::endl;
        std::cerr << "建议：1. 检查文件路径 2. 安装ffmpeg（sudo apt install ffmpeg）" << std::endl;
        return false;
    }
    break;
}
        case VideoSourceType::REMOTE_STREAM: {
            // 远程视频流：source为流地址（RTSP/UDP/HTTP等）
            cap_.open(source);
            if (!cap_.isOpened()) {
                std::cerr << "错误：无法连接远程视频流 " << source << std::endl;
                return false;
            }
            break;
        }
        default:
            std::cerr << "错误：不支持的视频源类型" << std::endl;
            return false;
    }

    std::cout << "视频源初始化成功！" << std::endl;
    return true;
}

bool VideoPlayer::play(const std::string& windowName) {
    if (!cap_.isOpened()) {
        std::cerr << "错误：视频源未初始化！" << std::endl;
        return false;
    }

    isRunning_ = true;
    cv::namedWindow(windowName, cv::WINDOW_NORMAL); // 可调整窗口大小
    std::cout << "视频播放中！" << std::endl;
    std::cout << "操作说明：" << std::endl;
    std::cout << "  - ESC键：退出播放" << std::endl;
    std::cout << "  - s/S键：保存当前帧" << std::endl;
    std::cout << "  - 窗口可拖动/缩放" << std::endl;

    while (isRunning_) {
        // 读取一帧
        cap_ >> currentFrame_;
        if (currentFrame_.empty()) {
            if (cap_.get(cv::CAP_PROP_POS_AVI_RATIO) >= 1.0) {
                std::cout << "视频播放完毕！" << std::endl;
            } else {
                std::cerr << "错误：读取视频帧失败！" << std::endl;
            }
            break;
        }

        // 显示帧
        cv::imshow(windowName, currentFrame_);

        // 按键处理
        int key = cv::waitKey(1);
        if (key == 27) { // ESC键退出
            std::cout << "用户退出播放" << std::endl;
            break;
        } else if (key == 's' || key == 'S') { // s/S键保存帧
            saveCurrentFrame();
        }
    }

    // 停止播放并清理
    stop();
    return true;
}

void VideoPlayer::stop() {
    isRunning_ = false;
    if (cap_.isOpened()) {
        cap_.release(); // 释放视频源
    }
    cv::destroyAllWindows(); // 关闭所有窗口
    std::cout << "视频播放已停止" << std::endl;
}

bool VideoPlayer::saveCurrentFrame(const std::string& filePath) {
    if (currentFrame_.empty()) {
        std::cerr << "错误：无可用帧可保存！" << std::endl;
        return false;
    }
    bool success = cv::imwrite(filePath, currentFrame_);
    if (success) {
        std::cout << "当前帧已保存至：" << filePath << std::endl;
    } else {
        std::cerr << "错误：保存帧失败！" << std::endl;
    }
    return success;
}