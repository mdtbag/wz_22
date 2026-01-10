#ifndef VIDEOPLAYER_H
#define VIDEOPLAYER_H

#include <opencv2/opencv.hpp>
#include <string>
#include <atomic>

// 视频源类型枚举
enum class VideoSourceType {
    LOCAL_CAMERA,   // 本地摄像头
    LOCAL_FILE,     // 本地视频文件
    REMOTE_STREAM   // 远程视频流（RTSP/UDP等）
};

// 视频播放器类（封装所有逻辑）
class VideoPlayer {
public:
    // 构造函数：初始化播放器
    VideoPlayer();
    
    // 析构函数：释放资源
    ~VideoPlayer();

    /**
     * @brief 初始化视频源
     * @param type 视频源类型（LOCAL_CAMERA/LOCAL_FILE/REMOTE_STREAM）
     * @param source 视频源参数：
     *        - LOCAL_CAMERA：摄像头索引（如"0"）
     *        - LOCAL_FILE：文件路径（如"./test.mp4"）
     *        - REMOTE_STREAM：流地址（如"rtsp://192.168.1.100:554/stream"）
     * @param width 视频宽度（仅本地摄像头生效）
     * @param height 视频高度（仅本地摄像头生效）
     * @return 是否初始化成功
     */
    bool init(VideoSourceType type, const std::string& source, int width = 640, int height = 480);

    /**
     * @brief 启动视频播放（实时显示）
     * @param windowName 显示窗口名称
     * @return 是否播放成功
     */
    bool play(const std::string& windowName = "Video Player");

    /**
     * @brief 停止播放
     */
    void stop();

    /**
     * @brief 保存当前帧
     * @param filePath 保存路径（如"./capture.jpg"）
     * @return 是否保存成功
     */
    bool saveCurrentFrame(const std::string& filePath = "./capture.jpg");

private:
    cv::VideoCapture cap_;          // OpenCV视频捕获对象
    cv::Mat currentFrame_;          // 当前帧缓存
    std::atomic<bool> isRunning_;   // 播放状态（原子变量，线程安全）
    int camWidth_;                  // 摄像头宽度
    int camHeight_;                 // 摄像头高度
};

#endif // VIDEOPLAYER_H