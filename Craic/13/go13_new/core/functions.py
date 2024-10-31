import cv2
import numpy as np
import time
import cv2.aruco as aruco
import socket
import json


min_speed, max_speed = 0.05, 0.28

# 路口检测的阈值
INTERSECTION_THRESHOLD = 4400


# PID定义
class PID():  # 定义一个名为PID的类
    def __init__(self, dt, max, min, Kp, Kd, Ki):  # 类的初始化方法，接收6个参数：时长dt、最大值max、最小值min、比例增益Kp、微分增益Kd和积分增益Ki
        self.dt = dt  # 循环时长
        self.max = max  # 操作变量最大值
        self.min = min  # 操作变量最小值
        self.Kp = Kp  # 比例增益
        self.Kd = Kd  # 微分增益
        self.Ki = Ki  # 积分增益
        self.integral = 0  # 直到上一次的误差值      # 初始化积分项累计误差为0
        self.pre_error = 0  # 上一次的误差值         # 初始化上一次的误差值为0

    def calculate(self, setPoint, pv):  # 定义一个名为calculate的方法，用于计算PID控制器的输出。接收2个参数：设定点setPoint和过程值pv
        # 其中 pv:process value 即过程值，
        error = setPoint - pv  # 误差    # 计算误差值
        Pout = self.Kp * error  # 比例项   # 计算比例项输出
        self.integral += error * self.dt  # 更新积分项累计误差
        Iout = self.Ki * self.integral  # 积分项    # 计算积分项输出
        derivative = (error - self.pre_error) / self.dt  # 计算误差的变化率（导数）
        Dout = self.Kd * derivative  # 微分项   # 计算微分项输出

        output = Pout + Iout + Dout  # 新的目标值    # 新的目标值为比例、积分、微分项之和

        if (output > self.max):  # 限制输出值不超过最大值
            output = self.max
        elif (output < self.min):  # 限制输出值不低于最小值
            output = self.min

        self.pre_error = error  # 保存本次误差，以供下次计算
        return output  # 返回输出值


# 根据像素差判断路口
def intersection_Judge(previous_yellow_area, cv_image):
    # BGR 图像转换为 HSV 色彩空间
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # 定义黄色范围，创建了一个掩膜，只保留黄色部分
    lower_yellow = np.array([0, 5, 80])
    upper_yellow = np.array([50, 255, 255])
    color_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 减少噪声和小斑点
    # 定义一个用于膨胀操作的结构元素，
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))  # 椭圆形的内核
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))  # 矩形内核
    color_mask = cv2.dilate(color_mask, dilate_kernel)  # 对图像进行膨胀操作
    color_mask = cv2.erode(color_mask, erode_kernel)  # 对图像进行腐蚀操作
    color_mask = cv2.medianBlur(color_mask, 9)  # 对图像进行中值模糊操作

    # 计算黄色区域的面积
    current_yellow_area = np.sum(color_mask > 0)

    if previous_yellow_area is not None:
        if current_yellow_area > 83000 and previous_yellow_area < 83000:
            return 1
    return 0



def patrol_line(cv_image):
    height, width, channels = cv_image.shape
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([10, 40, 100])
    upper_yellow = np.array([40, 255, 255])
    color_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    color_mask = cv2.medianBlur(color_mask, 9)
    color_mask = cv2.dilate(color_mask, dilate_kernel)
    color_mask = cv2.erode(color_mask, erode_kernel)

    m = cv2.moments(color_mask, False)
    try:
        cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
    except ZeroDivisionError:
        cx, cy = height / 2, width / 2

    cv2.circle(color_mask, (int(cx), int(cy)), 0, (0, 255, 0), 50)
    cv2.imshow('color_mask', color_mask)

    deviation = abs(width / 2 - cx)
    velocity = [max_speed - (max_speed - min_speed) * (deviation / (width / 2)), 0.0]
    yawspeed = pid_yaw.calculate(0, cx - width / 2)

    return velocity, yawspeed



class detect_aruco():

    def __init__(self):
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

    def detect(self, frame):
        h, w = frame.shape[:2]
        new_width = 1000
        new_height = int(h * (new_width / w))
        frame = cv2.resize(frame, (new_width, new_height))
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return ids



class RobotConnector():  
    def __init__(self, ip_address='192.168.123.161', port=8000):  
        address_server = (ip_address, port)  
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
        self.s.connect(address_server)  

    def robot_high_control(self, mode=2, gaitType=1, speedLevel=0,
                           footRaiseHeight=0, bodyHeight=0,
                           euler=[0, 0, 0], velocity=[0, 0], yawSpeed=0.0,
                           reverve=0):  
        data = [mode, gaitType, speedLevel, footRaiseHeight, bodyHeight, euler, velocity, yawSpeed,reverve]  
        data = json.dumps(data)  
        self.s.send(bytes(data.encode('utf-8')))  
