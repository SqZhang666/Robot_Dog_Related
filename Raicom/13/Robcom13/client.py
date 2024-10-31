import cv2
import socket
import pickle
import struct
import threading


def start_client():
   
    HOST = '192.168.123.20'
    PORT = 5003

    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        
        s.connect((HOST, PORT))
        while True:
            
            message = input("请输入要发送的消息：")
            
            if message == 'exit':
                break
            s.sendall(message.encode())
           
            data = s.recv(1024)
            print('收到服务器的响应：', data.decode())

if __name__ == "__main__":
    
    start_client()
