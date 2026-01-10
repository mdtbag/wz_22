#!/bin/bash
set -e  # 遇到错误立即退出

# 定义颜色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 步骤1：检查依赖
echo -e "${YELLOW}=== 检查 Paho MQTT 库 ===${NC}"
if ! ldconfig -p | grep -q "paho-mqttpp3"; then
    echo -e "${RED}错误：未找到 Paho MQTT C++ 库，请先安装！${NC}"
    exit 1
fi

# 步骤2：编译代码
echo -e "\n${YELLOW}=== 开始编译代码 ===${NC}"
mkdir -p build
cd build
cmake ..
make -j$(nproc)
cd ..

# 步骤3：启动订阅端（关键：关闭 stdin，避免 cin.get() 阻塞导致提前退出）
echo -e "\n${YELLOW}=== 启动 MQTT 订阅端 ===${NC}"
SUBSCRIBER_BIN="./build/bin/subscriber"
PUBLISHER_BIN="./build/bin/publisher"

# 先杀死残留进程
pkill -f "${SUBSCRIBER_BIN}" || true
rm -f subscriber.log

# 核心修改1：用 nohup 启动订阅端，并重定向 stdin 到 /dev/null（避免 cin.get() 阻塞）
# 让订阅端持续运行，直到手动杀死
nohup ${SUBSCRIBER_BIN} > subscriber.log 2>&1 < /dev/null &
SUBSCRIBER_PID=$!
echo "订阅端已启动，PID: ${SUBSCRIBER_PID}，日志文件: subscriber.log"

# 核心修改2：延长延迟到3秒，确保订阅端完成连接+订阅
echo "等待3秒，确保订阅端就绪..."
sleep 3

# 步骤4：启动发布端
echo -e "\n${YELLOW}=== 启动 MQTT 发布端 ===${NC}"
${PUBLISHER_BIN}

# 核心修改3：发布后再延迟2秒，确保消息被订阅端接收并写入日志
sleep 2

# 步骤5：验证测试结果
echo -e "\n${YELLOW}=== 验证测试结果 ===${NC}"
if grep -q "收到机器人状态消息：" subscriber.log; then
    echo -e "${GREEN}✅ 测试成功！订阅端已收到发布端的消息${NC}"
    echo -e "\n订阅端日志关键内容："
    grep -A 10 "收到机器人状态消息：" subscriber.log
else
    echo -e "${RED}❌ 测试失败！未收到消息，请检查日志${NC}"
    echo -e "\n订阅端完整日志："
    cat subscriber.log
fi

# 步骤6：清理订阅端进程（增加容错）
echo -e "\n${YELLOW}=== 清理进程 ===${NC}"
if ps -p ${SUBSCRIBER_PID} > /dev/null; then
    kill ${SUBSCRIBER_PID} || true
    echo "订阅端进程（PID: ${SUBSCRIBER_PID}）已终止"
else
    echo "订阅端进程已提前退出（正常）"
fi

echo -e "\n${GREEN}=== 测试流程结束 ===${NC}"