import cv2
import numpy as np
from core.robot_connector import RobotConnector
robot = RobotConnector(ip_address='192.168.123.161', port=8000)
from core.PID import PID
import time
from DetectArucotag import detect_aruco

# 设置两个常量存储要运输的物资编号1-4
FLAG_1 = 0
FLAG_2 = 1
FLAG_3 = 0
FLAG_4 = 1


pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
#pid_yaw = PID(0.1, 1.2, -1.2, 0.0081, 0.0108, 0.00)
min_speed, max_speed = 0.05, 0.28

# 初始化Aruco检测类
dt_aruco = detect_aruco()

def patrol_line(cv_image):
    height, width, channels = cv_image.shape

    #cv2.imshow('yuanshi', cv_image)
    # 将掩码应用到HSV图像
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # 黄色线条的阈值
    lower_yellow = np.array([0, 7, 0])
    upper_yellow = np.array([68, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # 处理黄色线条的掩码
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    yellow_mask = cv2.dilate(yellow_mask, dilate_kernel)
    yellow_mask = cv2.erode(yellow_mask, erode_kernel)
    yellow_mask = cv2.medianBlur(yellow_mask, 9)

    # 计算黄色线条的质心
    m_yellow = cv2.moments(yellow_mask, False)
    try:
        cx_yellow, cy_yellow = m_yellow['m10'] / m_yellow['m00'], m_yellow['m01'] / m_yellow['m00']
    except ZeroDivisionError:
        cx_yellow, cy_yellow = height / 2, width / 2
    # 计算黄色线条的面积
    yellow_area = np.sum(yellow_mask > 0)
    #print(f"Yellow area: {yellow_area}")

    # 在掩码图像上画出黄色线条的质心
    cv2.circle(yellow_mask, (int(cx_yellow), int(cy_yellow)), 0, (0, 255, 0), 50)
    cv2.imshow('yellow_mask', yellow_mask)

    # 调用Aruco检测
    aruco_ids = dt_aruco.detect(cv_image)

    velocity = [0.0, 0.0]
    yawspeed = 0.0

    deviation_yellow = abs(width / 2 - cx_yellow) 
    velocity[0] = max_speed - (max_speed - min_speed) * (deviation_yellow / (width / 2))
    yawspeed = pid_yaw.calculate(0, cx_yellow - width / 2  - 20)
    robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)

    return yellow_area, aruco_ids

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 设置宽度为640像素
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 设置高度为480像
    cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
    dt_aruco = detect_aruco()

    # 记录标记点被识别的次数
    count_ids_0 = 0
    count_ids_1 = 0
    count_ids_2 = 0
    count_ids_3 = 0
    count_ids_4 = 0
    count_ids_5 = 0

    # 记录分叉被识别的次数
    count_fencha = 0
    flag_1_jieshu = 0

    previous_yellow_area = None
    area_change = 10

    while True:
        ret, cv_image = cap.read()
        #cv2.imshow('color', cv_image)

        # 正常巡线
        current_yellow_area, aruco_ids = patrol_line(cv_image)

        # 面积变化差值
        if previous_yellow_area is not None:
            area_change = current_yellow_area - previous_yellow_area
        previous_yellow_area = current_yellow_area

        #print(area_change)
        # 识别特殊标记（结束标记：0；物品标记：1、2、3、4；障碍物标记：5）
        if aruco_ids == 0 and count_ids_0 == 0:
            count_ids_0 = count_ids_0 + 1
            print("0:结束操作：直行之后向右平移")
            while True:
                for t in range(4):
                    robot.robot_high_control(velocity=[0.0, -0.3])
                    time.sleep(0.5)
                break

        if aruco_ids == 1 and count_ids_1 == 0 and FLAG_1 == 1:
            count_ids_1 = count_ids_1 + 1
            print("1:第一个圈入圈操作：左转")
            while True:
                for t in range(5):
                    robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.3)
                    time.sleep(0.5)
                break
        if aruco_ids == 1 and count_ids_1 == 0 and FLAG_1 == 0:
            count_ids_1 = count_ids_1 + 1
            flag_1_jeishu = 1
            print("1:第一个圈不入圈操作：直行")
            while True:
                for t in range(4):
                    robot.robot_high_control(velocity=[0.3, 0.0])
                    time.sleep(0.5)
                break

        if aruco_ids == 2 and count_ids_2 == 0 and FLAG_2 == 1 :
            count_ids_2 = count_ids_2 + 1
            print("2:第二个圈入圈操作：左转")
            while True:
                for t in range(5):
                    robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.3)
                    time.sleep(0.5)
                break
        if aruco_ids == 2 and count_ids_2 == 0 and FLAG_2 == 0:
            count_ids_2 = count_ids_2 + 1
            print("2:第二个圈不入圈操作：直行")
            while True:
                for t in range(4):
                    robot.robot_high_control(velocity=[0.3, 0.0])
                    time.sleep(0.5)
                break

        if aruco_ids == 3 and count_ids_3 == 0 and FLAG_3 == 1:
            count_ids_3 = count_ids_3 + 1
            print("3:第三个圈入圈操作：直行")
            while True:
                for t in range(2):
                    robot.robot_high_control(velocity=[0.3, 0.0])
                    time.sleep(0.5)
                break
        if aruco_ids == 3 and count_ids_3 == 0 and FLAG_3 == 0:
            count_ids_3 = count_ids_3 + 1
            print("3:第三个圈不入圈操作：左转")
            while True:
                for t in range(6):
                    robot.robot_high_control(velocity=[0.3, 0.0], yawSpeed=0.7)
                    time.sleep(0.5)
                break

        if aruco_ids == 4 and count_ids_4 == 0 and FLAG_4 == 1:
            count_ids_4 = count_ids_4 + 1
            print("4:第四个圈入圈操作：直行")
            while True:
                for t in range(2):
                    robot.robot_high_control(velocity=[0.3, 0.0])
                    time.sleep(0.5)
                break
        if aruco_ids == 4 and count_ids_4 == 0 and FLAG_4 == 0:
            count_ids_4 = count_ids_4 + 1
            print("4:第四个圈不入圈操作：左转")
            while True:
                for t in range(6):
                    robot.robot_high_control(velocity=[0.3, 0.0], yawSpeed=0.7)
                    time.sleep(0.5)
                break

        if aruco_ids == 5 and count_ids_5 == 0:
            count_ids_5 = count_ids_5 + 1
            print("5:障碍区操作……")

        # “出圈”：出圈以颜色迅速增大为标志：1、2圈需要向左走；3、4圈需要直行

        # if area_change > 5000 and count_ids_1 == 1 and FLAG_1 == 1:
        #     count_ids_1 = count_ids_1 + 1
        #     flag_1_jeishu = 1
        #     print('圈1出去,左转')
        #     while True:
        #         for t in range(6):
        #             robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=1)
        #             time.sleep(0.5)
        #         break
        # if area_change > 5000 and count_ids_2 == 1 and FLAG_2 == 1:
        #     count_ids_2 = count_ids_2 + 1
        #     print('圈2出去，左转')
        #     while True:
        #         for t in range(6):
        #             robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=1)
        #             time.sleep(0.5)
        #         break
        # if area_change > 5000 and count_ids_3 == 1 and FLAG_3 == 1:
        #     count_ids_3 = count_ids_3 + 1
        #     print('圈3出去，直行')
        #     while True:
        #         for t in range(3):
        #             robot.robot_high_control(velocity=[0.2, 0.0])
        #             time.sleep(0.5)
        #         break
        # if area_change > 5000 and count_ids_4 == 1 and FLAG_4 == 1:
        #     count_ids_4 = count_ids_4 + 1
        #     print('圈4出去，直行')
        #     while True:
        #         for t in range(3):
        #             robot.robot_high_control(velocity=[0.2, 0.0])
        #             time.sleep(0.5)
        #         break

        # 分叉口行走
        if area_change > 2000 and flag_1_jieshu==1 and count_fencha == 0 and count_ids_2 == 0 and count_ids_3 == 0 and count_ids_4 == 0:
            count_fencha = count_fencha + 1
            print('分叉操作：左转')
            while True:
                for t in range(3):
                    robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.5)
                    time.sleep(0.5)
                break

        mykey = cv2.waitKey(1)

        if mykey & 0xFF == ord('q'):
            break