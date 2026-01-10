// 引入标准输入输出头文件，用于控制台打印和输入
#include <iostream>
// 引入字符串操作头文件，用于内存操作（如memset）
#include <cstring>
// 引入socket相关头文件，实现UDP网络通信（Linux/macOS）
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
// 引入unistd.h，用于usleep/sleep/close等系统调用
#include <unistd.h>
// 引入OpenCV核心头文件，用于图片读写和编码
#include <opencv2/opencv.hpp>
// 引入string头文件，用于字符串处理
#include <string>

// 定义UDP数据包最大大小（转为size_t类型，避免类型不匹配）
#define BUF_SIZE (size_t)4096
// 定义UDP通信端口（客户端和服务器需保持一致）
#define SERVER_PORT 8888

/**
 * @brief 封装图片发送函数，负责读取图片、编码、分片发送
 * @param sock_fd UDP套接字描述符
 * @param client_addr 客户端地址结构体（目标发送地址）
 * @param img_path 图片文件路径（相对/绝对路径）
 * @return bool 发送成功返回true，失败返回false
 */
bool send_image(int sock_fd, const struct sockaddr_in& client_addr, const std::string& img_path) {
    // 打印当前尝试读取的图片路径，方便排查路径错误
    std::cout << "尝试读取图片：" << img_path << std::endl;
    
    // 1. 读取图片文件（IMREAD_UNCHANGED保留图片原格式）
    cv::Mat img = cv::imread(img_path, cv::IMREAD_UNCHANGED);
    // 检查图片是否读取成功（空矩阵表示读取失败）
    if (img.empty()) {
        std::cerr << "❌ 读取图片失败：" << img_path << "（检查文件是否存在/路径是否正确）" << std::endl;
        return false;
    }

    // 2. 将Mat格式图片编码为JPG字节流，减少传输数据量
    std::vector<uchar> img_data;
    cv::imencode(".jpg", img, img_data);
    // 获取编码后图片的总字节数
    size_t total_size = img_data.size();
    std::cout << "\n📤 图片大小：" << total_size << " 字节" << std::endl;

    // 3. 发送图片总大小（先告知客户端需要接收的总字节数）
    // 转换为网络字节序（大端序），保证跨平台数据一致性
    uint64_t total_size_net = htobe64(total_size);
    // sendto：UDP发送函数，指定目标客户端地址
    ssize_t sent = sendto(sock_fd, &total_size_net, sizeof(total_size_net), 0,
                          (struct sockaddr*)&client_addr, sizeof(client_addr));
    // 检查发送是否失败
    if (sent < 0) {
        perror("发送图片大小失败");
        return false;
    }
    std::cout << "已发送图片大小：" << sent << " 字节" << std::endl;
    // 短暂延迟（20ms），避免大小包和数据包包乱序
    usleep(20000);

    // 4. 分片发送图片数据（UDP单包大小有限，需拆分）
    size_t sent_bytes = 0; // 已发送的字节数
    // 循环发送直到所有数据发送完成
    while (sent_bytes < total_size) {
        // 计算当前分片大小：取BUF_SIZE和剩余字节数的较小值
        size_t chunk_size = std::min(BUF_SIZE, total_size - sent_bytes);
        // 发送当前分片数据
        ssize_t ret = sendto(sock_fd, img_data.data() + sent_bytes, chunk_size, 0,
                             (struct sockaddr*)&client_addr, sizeof(client_addr));
        // 检查分片发送是否失败
        if (ret < 0) {
            perror("发送图片分片失败");
            return false;
        }
        // 更新已发送字节数
        sent_bytes += ret;
        // 实时打印发送进度（\r实现行内覆盖）
        std::cout << "发送进度：" << sent_bytes << "/" << total_size << " 字节\r" << std::flush;
    }
    // 换行，结束进度打印
    std::cout << std::endl << "✅ 图片发送完成！" << std::endl;
    return true;
}

/**
 * @brief 字符串清理函数，去除首尾空格/换行/制表符
 * @param s 原始输入字符串
 * @return std::string 清理后的字符串
 */
std::string trim(const std::string& s) {
    // 找到第一个非空白字符的位置
    size_t start = s.find_first_not_of(" \t\n\r");
    // 找到最后一个非空白字符的位置
    size_t end = s.find_last_not_of(" \t\n\r");
    // 若全是空白字符返回空，否则返回子串
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
}

/**
 * @brief 主函数，程序入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码（0成功，非0失败）
 */
int main(int argc, char* argv[]) {
    // 检查命令行参数是否合法（支持2个或3个参数）
    if (argc != 2 && argc != 3) {
        // 打印用法提示
        std::cerr << "用法1：" << argv[0] << " <image_path>          （循环发送同一张图片）" << std::endl;
        std::cerr << "用法2：" << argv[0] << " manual <init_image>   （手动输入路径发送不同图片）" << std::endl;
        return -1; // 参数错误，返回非0退出码
    }

    // 1. 创建UDP套接字（AF_INET：IPv4，SOCK_DGRAM：UDP协议，0：默认协议）
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    // 检查套接字创建是否失败
    if (sock_fd < 0) {
        perror("创建socket失败");
        return -1;
    }

    // 2. 配置客户端地址结构体（目标接收方地址）
    struct sockaddr_in client_addr;
    // 初始化地址结构体为0
    memset(&client_addr, 0, sizeof(client_addr));
    // 设置地址族为IPv4
    client_addr.sin_family = AF_INET;
    // 设置客户端监听端口（转换为网络字节序）
    client_addr.sin_port = htons(SERVER_PORT);
    // 设置客户端IP地址（127.0.0.1为本机，实际使用可改为目标IP）
    if (inet_pton(AF_INET, "127.0.0.1", &client_addr.sin_addr) <= 0) {
        perror("客户端IP地址无效");
        close(sock_fd); // 关闭套接字释放资源
        return -1;
    }

    // 3. 模式1：循环发送同一张图片（参数个数为2时）
    if (argc == 2) {
        // 获取图片路径参数
        std::string img_path = argv[1];
        // 打印模式提示
        std::cout << "=== 循环发送图片模式 ===" << std::endl;
        std::cout << "发送图片：" << img_path << " (按Ctrl+C退出)" << std::endl;
        // 无限循环发送（按Ctrl+C终止）
        while (true) {
            // 调用发送函数
            send_image(sock_fd, client_addr, img_path);
            sleep(2); // 每2秒发送一次，可调整
        }
    }
    // 4. 模式2：手动输入路径发送不同图片（参数个数为3且第二个参数为manual）
    else if (argc == 3 && std::string(argv[1]) == "manual") {
        // 获取初始图片路径
        std::string img_path = argv[2];
        // 打印模式提示
        std::cout << "=== 手动发送图片模式 ===" << std::endl;
        std::cout << "初始图片路径：" << img_path << std::endl;
        
        // 先主动发送初始图片
        std::cout << "\n正在发送初始图片..." << std::endl;
        send_image(sock_fd, client_addr, img_path);
        
        // 打印操作提示
        std::cout << "\n=====================================" << std::endl;
        std::cout << "输入图片路径发送（支持相对/绝对路径）" << std::endl;
        std::cout << "直接回车：重复发送上一张图片" << std::endl;
        std::cout << "输入q/Q：退出服务器" << std::endl;
        std::cout << "=====================================\n" << std::endl;
        
        // 循环接收用户输入
        while (true) {
            std::string input_path;
            std::cout << "请输入路径 > ";
            // 读取用户输入的完整行（支持空输入）
            std::getline(std::cin, input_path);
            
            // 清理输入字符串（去除首尾空格/换行）
            std::string clean_input = trim(input_path);
            
            // 判断是否退出（输入q/Q）
            if (clean_input == "q" || clean_input == "Q") {
                std::cout << "\n📤 正在退出服务器..." << std::endl;
                break; // 退出循环
            }
            
            // 空输入：重复发送上一张图片
            if (clean_input.empty()) {
                std::cout << "\n🔄 重复发送上一张图片：" << img_path << std::endl;
                send_image(sock_fd, client_addr, img_path);
            }
            // 非空输入：更新路径并发送
            else {
                img_path = clean_input;
                std::cout << "\n📤 发送新图片：" << img_path << std::endl;
                send_image(sock_fd, client_addr, img_path);
            }
        }
    }

    // 5. 关闭套接字，释放资源
    close(sock_fd);
    std::cout << "✅ 服务器已退出" << std::endl;
    return 0; // 程序正常退出
}