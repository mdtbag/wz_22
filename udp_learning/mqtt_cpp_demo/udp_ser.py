# udp_send.py
import socket
# import os
# import sys

# desktop = os.path.join(os.path.expanduser("~"), "Desktop")
# image_path = os.path.join(desktop, "test.jpg")
image_path = "test.jpg"

# if not os.path.exists(image_path):
#     print(f"❌ 图片不存在: {image_path}")
#     sys.exit(1)

# file_size = os.path.getsize(image_path)
# if file_size > 60000:
#     print(f"⚠️ 图片太大 ({file_size} bytes)，UDP 可能无法完整传输！建议 <60KB")
#     sys.exit(1)

with open(image_path, "rb") as f:
    data = f.read()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 10000))  # 绑定本地 10000 端口（可选）
print("📤 正在通过 UDP 发送图片...")
sock.sendto(data, ('127.0.0.1', 20000))
print("✅ UDP 发送完成（无确认）")
sock.close()