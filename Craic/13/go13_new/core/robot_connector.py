import socket     # 导入套接字模块
import json     # 导入 JSON 模块
import time

class RobotConnector():     # 定义一个名为 RobotConnector 的类
    def __init__(self, ip_address='192.168.123.161', port=8000):     # 构造函数，接受 IP 地址和端口作为参数，默认值分别为 '192.168.123.161' 和 8000
        address_server = (ip_address, port)      # 创建服务器地址元组
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # 创建一个 TCP 套接字实例
        self.s.connect(address_server)      # 连接到服务器地址

    def robot_high_control(self, mode=2, gaitType=1, speedLevel=1, 
                      footRaiseHeight=0, bodyHeight=0, 
                      euler=[0,0,0], velocity=[0,0], yawSpeed=0.0, reverve=0):     # 定义 robot_high_control 方法，接受一系列参数
        data = [mode, gaitType, speedLevel, footRaiseHeight, bodyHeight, euler, velocity, yawSpeed, reverve]    # 将参数封装成一个列表
        data = json.dumps(data)      # 将列表转换为 JSON 格式的字符串
        self.s.send(bytes(data.encode('utf-8')))    # 将 JSON 字符串编码为 UTF-8 字节，并通过套接字发送给服务器


if __name__ == '__main__':
    sdk = RobotConnector()
    motiontime = 0
    while True:
        time.sleep(0.002)
        motiontime = motiontime + 1
        mode = 0
        print(mode)
        sdk.robot_high_control(mode=mode)
        mode = 12
        print(motiontime)
        sdk.robot_high_control(mode=mode)
'''
        if (motiontime > 1000 and motiontime < 2000):
            mode = 1
            print(motiontime)
            sdk.robot_high_control(mode=mode)

        if (motiontime > 2000 and motiontime < 3000):
            mode = 7
            print(motiontime)
            sdk.robot_high_control(mode=mode)
        if (motiontime > 3000 and motiontime < 4000):
            mode = 5
            print(motiontime)
            sdk.robot_high_control(mode=mode)
        if (motiontime > 4000 and motiontime < 5000):
            mode = 6
            print(motiontime)
            sdk.robot_high_control(mode=mode)'''
        
        
















#这段代码定义了一个名为 RobotConnector 的类，该类主要用于与远程服务器（即运行 Unitree Legged SDK 的服务器）建立连接并发送控制命令。
# 通过调用 robot_high_control 方法，可以向机器人发送指令（如行走、转向等）。