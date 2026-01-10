# udp_recv.py
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 20000))

print("👂 UDP 接收端正在监听 20000 端口...")

data, addr = sock.recvfrom(65536)  # 最大接收 64KB
print(f"📥 收到来自 {addr} 的数据，大小: {len(data)} 字节") 

with open("received_udp.jpg", "wb") as f:
    f.write(data)

print("✅ 图片已保存为 received_udp.jpg")
sock.close()