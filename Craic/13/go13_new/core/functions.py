import cv2
import numpy as np
import time
import cv2.aruco as aruco
import socket
import json


min_speed, max_speed = 0.05, 0.28

# ·�ڼ�����ֵ
INTERSECTION_THRESHOLD = 4400


# PID����
class PID():  # ����һ����ΪPID����
    def __init__(self, dt, max, min, Kp, Kd, Ki):  # ��ĳ�ʼ������������6��������ʱ��dt�����ֵmax����Сֵmin����������Kp��΢������Kd�ͻ�������Ki
        self.dt = dt  # ѭ��ʱ��
        self.max = max  # �����������ֵ
        self.min = min  # ����������Сֵ
        self.Kp = Kp  # ��������
        self.Kd = Kd  # ΢������
        self.Ki = Ki  # ��������
        self.integral = 0  # ֱ����һ�ε����ֵ      # ��ʼ���������ۼ����Ϊ0
        self.pre_error = 0  # ��һ�ε����ֵ         # ��ʼ����һ�ε����ֵΪ0

    def calculate(self, setPoint, pv):  # ����һ����Ϊcalculate�ķ��������ڼ���PID�����������������2���������趨��setPoint�͹���ֵpv
        # ���� pv:process value ������ֵ��
        error = setPoint - pv  # ���    # �������ֵ
        Pout = self.Kp * error  # ������   # ������������
        self.integral += error * self.dt  # ���»������ۼ����
        Iout = self.Ki * self.integral  # ������    # ������������
        derivative = (error - self.pre_error) / self.dt  # �������ı仯�ʣ�������
        Dout = self.Kd * derivative  # ΢����   # ����΢�������

        output = Pout + Iout + Dout  # �µ�Ŀ��ֵ    # �µ�Ŀ��ֵΪ���������֡�΢����֮��

        if (output > self.max):  # �������ֵ���������ֵ
            output = self.max
        elif (output < self.min):  # �������ֵ��������Сֵ
            output = self.min

        self.pre_error = error  # ���汾�����Թ��´μ���
        return output  # �������ֵ


# �������ز��ж�·��
def intersection_Judge(previous_yellow_area, cv_image):
    # BGR ͼ��ת��Ϊ HSV ɫ�ʿռ�
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # �����ɫ��Χ��������һ����Ĥ��ֻ������ɫ����
    lower_yellow = np.array([0, 5, 80])
    upper_yellow = np.array([50, 255, 255])
    color_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # ����������С�ߵ�
    # ����һ���������Ͳ����ĽṹԪ�أ�
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))  # ��Բ�ε��ں�
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))  # �����ں�
    color_mask = cv2.dilate(color_mask, dilate_kernel)  # ��ͼ��������Ͳ���
    color_mask = cv2.erode(color_mask, erode_kernel)  # ��ͼ����и�ʴ����
    color_mask = cv2.medianBlur(color_mask, 9)  # ��ͼ�������ֵģ������

    # �����ɫ��������
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
