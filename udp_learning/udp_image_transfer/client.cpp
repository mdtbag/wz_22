// 引入标准输入输出头文件
#include <iostream>
// 引入字符串操作头文件
#include <cstring>
// 引入socket相关头文件，实现UDP网络通信
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
// 引入unistd.h，用于close等系统调用
#include <unistd.h>
// 引入OpenCV核心头文件，用于图片解码和显示
#include <opencv2/opencv.hpp>

// 定义UDP数据包最大大小（转为size_t类型）
#define BUF_SIZE (size_t)4096
// 定义UDP通信端口（与服务器保持一致）
#define SERVER_PORT 8888

/**
 * @brief 接收并显示图片的封装函数
 * @param sock_fd UDP套接字描述符
 * @return bool 接收流程是否正常（解码失败仍返回true，继续等待下一张）
 */
bool receive_and_show_image(int sock_fd) {
    // 定义发送方（服务器）地址结构体
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);

    // 1. 接收图片总大小（先获取需要接收的总字节数）
    uint64_t total_size_net = 0; // 网络字节序的总大小
    // recvfrom：UDP接收函数，阻塞等待数据
    ssize_t recv_len = recvfrom(sock_fd, &total_size_net, sizeof(total_size_net), 0,
                                (struct sockaddr*)&sender_addr, &sender_len);
    // 检查接收是否失败
    if (recv_len < 0) {
        perror("recv total size failed");
        return false;
    }
    // 转换为主机字节序（小端序），恢复真实大小
    size_t total_size = be64toh(total_size_net);
    std::cout << "\n=== 接收到新图片 ===" << std::endl;
    std::cout << "Received total image size: " << total_size << " bytes" << std::endl;

    // 2. 接收图片数据（分片接收）
    // 创建容器存储完整图片数据
    std::vector<uchar> img_data(total_size);
    size_t recv_bytes = 0; // 已接收的字节数
    // 循环接收直到所有数据接收完成
    while (recv_bytes < total_size) {
        // 计算剩余未接收的字节数
        size_t remaining = total_size - recv_bytes;
        // 计算当前分片大小：取BUF_SIZE和剩余字节数的较小值
        size_t chunk_size = std::min(BUF_SIZE, remaining);
        // 接收当前分片数据
        recv_len = recvfrom(sock_fd, img_data.data() + recv_bytes, chunk_size, 0,
                            (struct sockaddr*)&sender_addr, &sender_len);
        // 检查分片接收是否失败
        if (recv_len < 0) {
            perror("recv image chunk failed");
            return false;
        }
        // 更新已接收字节数
        recv_bytes += recv_len;
        // 实时打印接收进度（\r实现行内覆盖）
        std::cout << "Received: " << recv_bytes << "/" << total_size << " bytes\r" << std::flush;
    }
    // 换行，结束进度打印
    std::cout << std::endl << "Image received completely!" << std::endl;

    // 3. 解码图片数据（将字节流转为Mat格式）
    cv::Mat img = cv::imdecode(img_data, cv::IMREAD_UNCHANGED);
    // 检查解码是否失败
    if (img.empty()) {
        std::cerr << "Failed to decode image data!" << std::endl;
        return true; // 解码失败不退出，继续等待下一张
    }

    // 4. 显示图片（窗口标题自定义）
    cv::imshow("Received Image (Auto-close after 2s)", img);
    std::cout << "Press any key to close the image window..." << std::endl;
    // 等待2000ms（2秒）后自动关闭窗口，无需手动操作
    cv::waitKey(2000);
    // 销毁图片窗口，释放资源
    cv::destroyWindow("Received Image (Auto-close after 2s)");
    return true;
}

/**
 * @brief 主函数，程序入口
 * @return int 程序退出码
 */
int main() {
    // 1. 创建UDP套接字（AF_INET：IPv4，SOCK_DGRAM：UDP协议）
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    // 检查套接字创建是否失败
    if (sock_fd < 0) {
        perror("socket creation failed");
        return -1;
    }

    // 2. 配置服务器地址结构体（用于绑定端口）
    struct sockaddr_in server_addr;
    // 初始化地址结构体为0
    memset(&server_addr, 0, sizeof(server_addr));
    // 设置地址族为IPv4
    server_addr.sin_family = AF_INET;
    // 绑定所有网卡（INADDR_ANY），接收任意IP的数据包
    server_addr.sin_addr.s_addr = INADDR_ANY;
    // 设置监听端口（转换为网络字节序）
    server_addr.sin_port = htons(SERVER_PORT);

    // 3. 绑定端口（UDP必须绑定端口才能接收数据）
    if (bind(sock_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind failed");
        close(sock_fd); // 关闭套接字
        return -1;
    }

    // 打印启动提示
    std::cout << "=== 客户端已启动，持续监听端口 " << SERVER_PORT << " ===" << std::endl;
    std::cout << "按Ctrl+C退出客户端..." << std::endl;

    // 4. 无限循环接收图片（持续监听）
    while (true) {
        // 调用接收并显示函数
        receive_and_show_image(sock_fd);
        // 提示等待下一张图片
        std::cout << "\n等待下一张图片...\n" << std::endl;
    }

    // 5. 关闭套接字（实际被Ctrl+C中断，不会执行到这里）
    close(sock_fd);
    return 0;
}