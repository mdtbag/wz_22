#include "VideoPlayer.h"
#include <iostream>

int main(int argc, char* argv[]) {
    VideoPlayer player;
    VideoSourceType type;
    std::string source;

    // 打印使用说明
    std::cout << "=== 通用视频播放器 ===" << std::endl;
    std::cout << "请选择视频源类型：" << std::endl;
    std::cout << "1 - 本地摄像头（输入索引，如0）" << std::endl;
    std::cout << "2 - 本地视频文件（输入路径，如./test.mp4）" << std::endl;
    std::cout << "3 - 远程视频流（输入地址，如rtsp://192.168.1.100:554/stream）" << std::endl;
    
    // 选择视频源类型
    int choice;
    std::cin >> choice;
    switch (choice) {
        case 1:
            type = VideoSourceType::LOCAL_CAMERA;
            std::cout << "请输入摄像头索引（默认0）：";
            std::cin >> source;
            if (source.empty()) source = "0";
            break;
        case 2:
            type = VideoSourceType::LOCAL_FILE;
            std::cout << "请输入视频文件路径：";
            std::cin >> source;
            break;
        case 3:
            type = VideoSourceType::REMOTE_STREAM;
            std::cout << "请输入远程视频流地址：";
            std::cin >> source;
            break;
        default:
            std::cerr << "错误：无效选择！" << std::endl;
            return -1;
    }

    // 初始化并播放
    if (player.init(type, source)) {
        player.play("通用视频播放器");
    } else {
        std::cerr << "初始化失败，程序退出！" << std::endl;
        return -1;
    }

    return 0;
}