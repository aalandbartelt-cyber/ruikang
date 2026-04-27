import socket

def start_receiver(ip="127.0.0.1", port=8080):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    
    print(f"UDP 接收端已启动，正在监听 {ip}:{port}...")
    print("等待接收数据 (按 Ctrl+C 停止)...")
    
    try:
        while True:
            data, addr = sock.recvfrom(1024) # 每次最多接收 1024 字节
            # 将接收到的字节流解码为字符串
            received_str = data.decode('utf-8')
            print(f"[接收] 来自 {addr} | 数据: {received_str}")
    except KeyboardInterrupt:
        print("\n接收端已关闭。")
    finally:
        sock.close()

if __name__ == "__main__":
    start_receiver()