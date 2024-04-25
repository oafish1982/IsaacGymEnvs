import socket
import pickle  # 用于反序列化数据

# UDP服务器的IP地址和端口号
SERVER_IP = '127.0.0.1'
SERVER_PORT = 12345

# 创建UDP套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((SERVER_IP, SERVER_PORT))

def receive_data():
    # 接收数据
    data_bytes, _ = sock.recvfrom(4096)

    # 将接收到的数据反序列化为Python对象
    data = pickle.loads(data_bytes)
    return data

if __name__ == '__main__':
    # 接收数据
    while(True):
        received_data = receive_data()
        print("Received data:", received_data)
