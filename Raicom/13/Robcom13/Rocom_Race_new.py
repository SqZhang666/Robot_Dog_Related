# -*- coding: utf-8 -*-
import cv2
import numpy as np
from core.robot_connector import RobotConnector
import argparse

robot = RobotConnector(ip_address='192.168.123.161', port=8000)
from core.PID import PID
from DetectArucotag import detect_aruco
import time
import socket

FLAG_1 = 0
FLAG_2 = 0
FLAG_3 = 0
FLAG_4 = 0
FLAG_obs = 1


def parser_flag():
    global FLAG_1, FLAG_2, FLAG_3, FLAG_4, FLAG_obs
    parser = argparse.ArgumentParser(description="control FLAG's number")
    parser.add_argument("x", type=int, choices=[1, 2, 3, 4], help="the first")
    parser.add_argument("y", type=int, choices=[1, 2, 3, 4], help="the second")
    # 依据比赛前障碍物位置，决定避障方案
    parser.add_argument("z", type=int, help="the possibility of obstacle")
    args = parser.parse_args()
    variables = [FLAG_1, FLAG_2, FLAG_3, FLAG_4]
    variables[args.x - 1] = 1
    variables[args.y - 1] = 1
    FLAG_1, FLAG_2, FLAG_3, FLAG_4 = variables
    FLAG_obs = args.z


parser_flag()
pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
min_speed, max_speed = 0.05, 0.28

dt_aruco = detect_aruco()


def find_connected_component_containing_point(image, points):
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(image, connectivity=8)
    for point in points:
        image = cv2.circle(image, point, 0, (0, 255, 0), 10)
    cv2.imshow('find_connected_components_containing_points', image)
    found_labels = set()
    for label in range(1, num_labels):
        for point in points:
            x, y = point
            if labels[y, x] == label:
                found_labels.add(label)
                break
    output_image = np.zeros_like(image)
    for label in found_labels:
        output_image[labels == label] = 255
    return output_image


def preprocess_image(cv_image):
    height, width, _ = cv_image.shape
    center = ((width // 2) + 9, (height // 2) - 10)
    radius = 145
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.circle(mask, center, radius, (255), thickness=-1)
    masked_image = cv2.bitwise_and(cv_image, cv2.merge([mask, mask, mask]))
    white_background = np.full_like(cv_image, 255)
    circular_image = np.where(cv2.merge([mask, mask, mask]) == 0, white_background, masked_image)
    return circular_image


def detect_yellow_line(circular_image):
    height, width, _ = circular_image.shape
    circular_image = circular_image[:height // 2, :]
    hsv = cv2.cvtColor(circular_image, cv2.COLOR_BGR2HSV)
    # lower_yellow = np.array([0, 7, 0])
    # upper_yellow = np.array([68, 255, 255])
    lower_yellow = np.array([0, 38, 0])
    upper_yellow = np.array([87, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    # erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    # yellow_mask = cv2.dilate(yellow_mask, dilate_kernel)
    # yellow_mask = cv2.erode(yellow_mask, erode_kernel)
    # yellow_mask = cv2.medianBlur(yellow_mask, 9)
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 1))
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    yellow_mask = cv2.dilate(yellow_mask, dilate_kernel)
    yellow_mask = cv2.erode(yellow_mask, erode_kernel)
    yellow_mask = cv2.medianBlur(yellow_mask, 5)
    points = [(230, 195), (260, 195)]
    yellow_mask = find_connected_component_containing_point(yellow_mask, points)
    m_yellow = cv2.moments(yellow_mask, False)
    try:
        cx_yellow, cy_yellow = m_yellow['m10'] / m_yellow['m00'], m_yellow['m01'] / m_yellow['m00']
    except ZeroDivisionError:
        cx_yellow, cy_yellow = height / 2, width / 2
    yellow_area = np.sum(yellow_mask > 0)
    cv2.circle(yellow_mask, (int(cx_yellow), int(cy_yellow)), 0, (0, 255, 0), 20)
    cv2.imshow('yellow_mask', yellow_mask)
    return yellow_area, cx_yellow, cy_yellow


def patrol_line(cv_image):
    height, width, _ = cv_image.shape
    circular_image = preprocess_image(cv_image)
    # cv2.imshow('chuli', circular_image)
    yellow_area, cx_yellow, cy_yellow = detect_yellow_line(circular_image)
    aruco_ids = dt_aruco.detect(cv_image)
    deviation_yellow = abs(width / 2 - cx_yellow)
    velocity = [0.0, 0.0]
    velocity[0] = max_speed - (max_speed - min_speed) * (deviation_yellow / (width / 2))
    yawspeed = pid_yaw.calculate(0, cx_yellow - width / 2)
    robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)
    return yellow_area, aruco_ids


# 依据障碍物的位置，执行相应的避障策略
def obstacle_avoidance(FLAG_obs):
    # 常规直行处-障碍物和机械狗正对，姿态比较正（左移-直行-右移）FLAG_obs == 1
    if FLAG_obs == 1:
        print('1Dodge Obstacles')
        for _ in range(4):
            robot.robot_high_control(velocity=[0.0, 0.3], yawSpeed=0.0)
            time.sleep(0.5)
        for _ in range(8):
            robot.robot_high_control(velocity=[0.3, 0.0], yawSpeed=0.0)
            time.sleep(0.5)
        for _ in range(4):
            robot.robot_high_control(velocity=[0.0, -0.3], yawSpeed=0.0)
            time.sleep(0.5)

    # 直角拐弯处和拐弯前处-8处-（平移-左/右前行）
    #          3处右拐 FLAG_obs == 2
    #          5处左拐 FLAG_obs == 3
    elif FLAG_obs == 2:
        print('2Dodge Obstacles')
        for _ in range(4):
            robot.robot_high_control(velocity=[0.0, -0.3], yawSpeed=0.0)
            time.sleep(0.5)
        for _ in range(6):
            robot.robot_high_control(velocity=[0.3, 0.0], yawSpeed=-1.0)
            time.sleep(0.5)
    elif FLAG_obs == 3:
        print('3Dodge Obstacles')
        for _ in range(4):
            robot.robot_high_control(velocity=[0.0, 0.3], yawSpeed=0.0)
            time.sleep(0.5)
        for _ in range(6):
            robot.robot_high_control(velocity=[0.3, 0.0], yawSpeed=1.0)
            time.sleep(0.5)

    # 直角拐弯后处-8处-（左/右平移和小幅度转（调正姿态）-左/右前行和平移）
    #          3处右拐 FLAG_obs == 4
    #          5处左拐 FLAG_obs == 5
    elif FLAG_obs == 4:
        print('2Dodge Obstacles')
        for _ in range(4):
            robot.robot_high_control(velocity=[0.0, -0.3], yawSpeed=-0.7)
            time.sleep(0.5)
        for _ in range(5):
            robot.robot_high_control(velocity=[0.5, 0.3], yawSpeed=0.0)
            time.sleep(0.5)
    elif FLAG_obs == 5:
        print('3Dodge Obstacles')
        for _ in range(4):
            robot.robot_high_control(velocity=[0.0, 0.3], yawSpeed=0.7)
            time.sleep(0.5)
        for _ in range(5):
            robot.robot_high_control(velocity=[0.3, -0.3], yawSpeed=0.0)
            time.sleep(0.5)

    # S弯处-2号和3号投放点之间极短的那段（右移-右前行） FLAG_obs == 6
    elif FLAG_obs == 6:
        print('4Dodge Obstacles')
        for _ in range(4):
            robot.robot_high_control(velocity=[0.0, -0.3], yawSpeed=0.0)
            time.sleep(0.5)
        for _ in range(5):
            robot.robot_high_control(velocity=[0.4, 0.0], yawSpeed=-1.0)
            time.sleep(0.5)

    # 3号与4号之间那段
    # 右拐那处（右移-右前行） FLAG_obs == 7
    # 左拐那处（左移-左前行） FLAG_obs == 8
    # 障碍物在中间，机身偏右（右移-前进和小幅度右转）  FLAG_obs == 9
    # 障碍物在中间，机身偏左（左移-前进和稍大幅度左转）  FLAG_obs == 10
    elif FLAG_obs == 7:
        print('5Dodge Obstacles')
        for _ in range(4):
            robot.robot_high_control(velocity=[0.0, -0.3], yawSpeed=0.0)
            time.sleep(0.5)
        for _ in range(5):
            robot.robot_high_control(velocity=[0.5, 0.0], yawSpeed=-1.0)
            time.sleep(0.5)
    elif FLAG_obs == 8:
        print('6Dodge Obstacles')
        for _ in range(4):
            robot.robot_high_control(velocity=[0.0, 0.3], yawSpeed=0.0)
            time.sleep(0.5)
        for _ in range(5):
            robot.robot_high_control(velocity=[0.5, 0.0], yawSpeed=1.0)
            time.sleep(0.5)
    elif FLAG_obs == 9:
        print('5Dodge Obstacles')
        for _ in range(4):
            robot.robot_high_control(velocity=[0.0, -0.3], yawSpeed=0.0)
            time.sleep(0.5)
        for _ in range(5):
            robot.robot_high_control(velocity=[0.5, 0.0], yawSpeed=-0.7)
            time.sleep(0.5)
    elif FLAG_obs == 10:
        print('6Dodge Obstacles')
        for _ in range(4):
            robot.robot_high_control(velocity=[0.0, 0.3], yawSpeed=0.0)
            time.sleep(0.5)
        for _ in range(5):
            robot.robot_high_control(velocity=[0.5, 0.0], yawSpeed=1.0)
            time.sleep(0.5)


def execute_movement(movements):
    for velocity, yawSpeed, t in movements:
        for _ in range(t):
            robot.robot_high_control(velocity=velocity, yawSpeed=yawSpeed)
            time.sleep(0.5)


# 定义每个 FLAG_obs 的动作序列
movement_dict = {
    # 调试
    1: [
        ([0.0, -0.3], 0.0, 4),
        ([0.3, 0.0], 0.0, 5),
        ([0.0, 0.0], -0.5, 4)
    ],
    # 调试
    2: [
        ([0.0, 0.3], 0.0, 4),
        ([0.3, 0.0], 2.0, 5)
    ],
    # 调试
    3: [
        ([0.0, 0.3], 3.0, 4),
        ([0.3, 0.0], 0.0, 5)
    ],
    # 已调试-投放一号物资
    4: [
        ([0.0, -0.35], 0.0, 3),
        ([0.3, 0.0], 0.0, 5),
        ([0.3, 0.0], 0.5, 4)
    ],
    # 调试-不投放一号物资
    -4: [
        ([0.0, 0.3], 0.0, 4),
        ([0.3, 0.0], 0.5, 5)
    ],
    # 调试
    5: [
        ([0.0, -0.3], 0.0, 4),
        ([0.3, 0.0], 0.5, 5)
    ],
    # 调试
    6: [
        ([0.0, 0.3], 0.0, 4),
        ([0.3, 0.0], 0.5, 5)
    ],
    # 调试
    7: [
        ([0.0, 0.3], 0.0, 4),
        ([0.4, 0.0], 0.0, 5),
        ([0.0, -0.3], 0.0, 3)
    ],
    # 调试
    8: [
        ([0.0, 0.35], 0.0, 4),
        ([0.3, 0.0], 0.5, 5)
    ],
    # 调试
    9: [
        ([0.0, 0.35], 0.0, 4),
        ([0.3, 0.0], 0.7, 5)
    ],
    # 调试
    10: [
        ([0.0, 0.3], 0.0, 4),
        ([0.3, 0.0], 0.0, 5),
        ([0.0, -0.3], 0.0, 4)
    ],
    # 调试
    11: [
        ([0.0, 0.35], 0.0, 4),
        ([0.3, 0.0], 0.0, 6),
        ([0.3, 0.0], 0.5, 4)
    ],
    # 调试
    12: [
        ([0.0, 0.35], 0.0, 4),
        ([0.3, 0.0], 0.7, 5)
    ],
    # 调试
    13: [
        ([0.0, 0.35], 0.0, 4),
        ([0.3, 0.0], 0.0, 5),
        ([0.0, -0.3], 0.0, 4)
    ],
    # 调试
    14: [
        ([0.0, 0.35], 0.0, 4),
        ([0.3, 0.0], 0.5, 5),
        ([0.3, 0.0], -0.5, 4)
    ],
    # 调试 还没转弯之前就看到了
    15: [
        ([0.3, 0.0], 0.0, 4),
        ([0.1, 0.0], 0.7, 5),
        ([0.3, 0.0], 0.0, 4),
        ([0.0, 0.0], -0.7, 4)
    ],
    # 调试
    16: [
        ([0.0, -0.3], 0.0, 5),
        ([0.2, 0.0], -0.7, 4)
    ],
    # 调试-投放三号物资
    17: [
        ([0.0, -0.3], 0.0, 4),
        ([0.3, 0.0], 0.0, 5),
        ([0.0, 0.3], 0.0, 4)
    ],
    # 调试-不投放三号物资
    -17: [
        ([0.0, -0.3], 0.0, 4),
        ([0.3, 0.0], 0.0, 5),
        ([0.0, 0.3], 0.0, 4)
    ],
    # 调试
    18: [
        ([0.0, -0.3], 0.0, 4),
        ([0.3, 0.0], -0.4, 5)
    ],
    # 调试
    19: [
        ([0.0, -0.3], 0.0, 4),
        ([0.3, 0.0], 0.0, 5),
        ([0.0, 0.0], 0.7, 4)
    ],
    # 调试
    20: [
        ([0.0, 0.3], 0.0, 3),
        ([0.3, 0.02], 0.0, 6),
        ([0.05, 0.0], 0.8, 4)
    ],
    # 调试
    21: [
        ([0.0, 0.35], 0.0, 4),
        ([0.3, 0.0], 0.5, 5)
    ],
    # 调试
    22: [
        ([0.0, 0.35], 0.0, 4),
        ([0.3, 0.0], 0.5, 5)
    ],
    # 调试
    23: [
        ([0.0, 0.35], 0.0, 4),
        ([0.3, 0.0], 0.0, 5),
        ([0.0, -0.3], 0.0, 4)
    ],
    # 调试
    24: [
        ([0.0, 0.35], 0.0, 4),
        ([0.3, 0.0], 0.0, 5),
        ([0.0, -0.3], -0.3, 4)
    ],
    # 调试
    25: [
        ([0.0, -0.35], 0.0, 4),
        ([0.3, 0.0], -0.5, 5)
    ],

}


def obstacle_avoidance_new(FLAG_obs):
    print(f'Dodging Obstacles, FLAG_obs: {FLAG_obs}')

    # 检查是否有对应的动作
    movements = movement_dict.get(FLAG_obs)

    if movements:
        execute_movement(movements)
    else:
        print(f"No movements defined for FLAG_obs: {FLAG_obs}")


def send_signal_to_server(signal):
    HOST = '192.168.123.20'
    PORT = 5003
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(signal.encode())
        data = s.recv(1024)
        print('receive:', data.decode())


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    cap1 = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap1.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    begin_time = time.time()

    count_ids_0 = 0
    count_ids_1 = 0
    count_ids_2 = 0
    count_ids_3 = 0
    count_ids_4 = 0
    count_ids_5 = 0

    flag_1_jinquan = 0
    flag_2_jinquan = 0
    flag_3_jinquan = 0
    flag_4_jinquan = 0

    count_jixie_1 = 0
    count_jixie_2 = 0
    count_jixie_3 = 0
    count_jixie_4 = 0

    flag_1_jieshu = 0
    flag_2_jieshu = 0
    flag_3_jieshu = 0
    flag_4_jieshu = 0

    count_fencha = 0
    fencha_over_time = -1
    count_fencha_chuqu = 0
    # flag_1_jieshu = 0

    Q1_start_time = -1
    Q2_start_time = -1
    Q3_start_time = -1
    Q4_start_time = -1
    Q1_over_time = -1
    min_interval = 5
    JIXIEBI_interval = 9

    previous_yellow_area = 10
    current_yellow_area = 10

    detect_obstacle = 0

    # yi chu qi ting qu
    for t in range(5):
        robot.robot_high_control(velocity=[0.0, 0.3])
        time.sleep(0.5)

    try:
        while True:
            ret, cv_image = cap.read()
            if not detect_obstacle:
                ret1, cv_image1 = cap1.read()
                if dt_aruco.detect(cv_image1) == 5:
                    obstacle_avoidance_new(FLAG_obs)
                    detect_obstacle = 1

            current_yellow_area, aruco_ids = patrol_line(cv_image)
            # print(current_yellow_area)

            current_time = time.time()
            previous_yellow_area = current_yellow_area

            if aruco_ids == 1 and count_ids_1 == 0 and FLAG_1 == 1:
                count_ids_1 = count_ids_1 + 1
                Q1_start_time = time.time()
                flag_1_jinquan = 1
                print("1:first zuoguai")
                while True:
                    for t in range(4):
                        robot.robot_high_control(velocity=[0.3, -0.1], yawSpeed=-0.2)
                        time.sleep(0.5)
                    t = 0
                    for t in range(3):
                        robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.8)
                        time.sleep(0.5)
                    break
                continue
            if aruco_ids == 1 and count_ids_1 == 0 and FLAG_1 == 0:
                count_ids_1 = count_ids_1 + 1
                flag_1_jieshu = 1
                Q1_over_time = time.time()
                print("1:first zhixing")
                while True:
                    for t in range(2):
                        robot.robot_high_control(velocity=[0.4, -0.2], yawSpeed=-0.25)
                        time.sleep(0.5)
                    break
                continue

            if aruco_ids == 2 and count_ids_2 == 0 and FLAG_2 == 1:
                count_ids_2 = count_ids_2 + 1
                Q2_start_time = time.time()
                flag_2_jinquan = 1
                print("2:second-ruquan")
                while True:
                    for t in range(4):
                        robot.robot_high_control(velocity=[0.3, 0.0], yawSpeed=-0.2)
                        time.sleep(0.5)
                    t = 0
                    for t in range(3):
                        robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.8)
                        time.sleep(0.5)
                    break
                continue
            if aruco_ids == 2 and count_ids_2 == 0 and FLAG_2 == 0:
                count_ids_2 = count_ids_2 + 1
                print("2:second-zhixing")
                flag_2_jieshu = 1
                while True:
                    for t in range(2):
                        robot.robot_high_control(velocity=[0.4, -0.2], yawSpeed=-0.25)
                        time.sleep(0.5)
                    break
                continue

            if aruco_ids == 3 and count_ids_3 == 0 and FLAG_3 == 1:
                count_ids_3 = count_ids_3 + 1
                Q3_start_time = time.time()
                flag_3_jinquan = 1
                print("3:ruquan-third-zhixing")
                while True:
                    for t in range(2):
                        robot.robot_high_control(velocity=[0.15, 0.0])
                        time.sleep(0.5)
                    break
                continue
            if aruco_ids == 3 and count_ids_3 == 1 and FLAG_3 == 1 and count_jixie_3 == 1:
                count_ids_3 = count_ids_3 + 1
                Q3_start_time = time.time()
                flag_3_jinquan = 1
                print("3:chuquan-third-zhixing")
                while True:
                    for t in range(2):
                        robot.robot_high_control(velocity=[0.15, -0.1])
                        time.sleep(0.5)
                    break
                continue
            if aruco_ids == 3 and count_ids_3 == 0 and FLAG_3 == 0:
                count_ids_3 = count_ids_3 + 1
                flag_3_jieshu = 1
                print("3:third-zuoguai")
                while True:
                    for t in range(6):
                        robot.robot_high_control(velocity=[0.15, 0.0], yawSpeed=0.7)
                        time.sleep(0.5)
                    break
                continue

            if aruco_ids == 4 and count_ids_4 == 0 and FLAG_4 == 1:
                count_ids_4 = count_ids_4 + 1
                Q4_start_time = time.time()
                flag_4_jinquan = 1
                print("4:first-forth-zhixing")
                while True:
                    for t in range(2):
                        robot.robot_high_control(velocity=[0.15, 0.0])
                        time.sleep(0.5)
                    break
                continue
            if aruco_ids == 4 and count_ids_4 == 1 and FLAG_4 == 1 and count_jixie_4 == 1:
                count_ids_4 = count_ids_4 + 1
                Q4_start_time = time.time()
                flag_4_jinquan = 1
                print("4:second-forth-zhixing")
                while True:
                    for t in range(2):
                        robot.robot_high_control(velocity=[0.15, -0.1])
                        time.sleep(0.5)
                    break
                continue
            if aruco_ids == 4 and count_ids_4 == 0 and FLAG_4 == 0:
                count_ids_4 = count_ids_4 + 1
                flag_4_jieshu = 1
                print("4:forth-zuoguai")
                while True:
                    for t in range(6):
                        robot.robot_high_control(velocity=[0.1, 0.0], yawSpeed=0.7)
                        time.sleep(0.5)
                    break
                continue

            if aruco_ids == 0 and count_ids_5 == 0 and (current_time - begin_time) > 30:
                count_ids_5 = count_ids_5 + 1
                print("0:over")
                while True:
                    for t in range(5):
                        robot.robot_high_control(velocity=[0.1, -0.3])
                        time.sleep(0.5)
                    break
                break

            if flag_1_jinquan == 1 and flag_1_jieshu == 0 and (
                    current_time - Q1_start_time) > JIXIEBI_interval and count_jixie_1 == 0:
                print("quan yi:ji xie bi cao zuo")
                for t in range(3):
                    robot.robot_high_control(velocity=[0.0, 0.1])
                    time.sleep(0.5)

                robot.robot_high_control(mode=2)
                time.sleep(1)
                robot.robot_high_control(mode=1)
                time.sleep(1)
                robot.robot_high_control(mode=6)
                time.sleep(1)
                robot.robot_high_control(mode=5)
                time.sleep(1)
                robot.robot_high_control(mode=7)
                time.sleep(1)
                response = send_signal_to_server('circle1:machineArm')
                while response == 'Circle1_END!':
                    break
                robot.robot_high_control(mode=7)
                time.sleep(1)
                robot.robot_high_control(mode=5)
                time.sleep(1)
                robot.robot_high_control(mode=6)
                time.sleep(1)
                count_jixie_1 = count_jixie_1 + 1
                continue

            if flag_2_jinquan == 1 and flag_2_jieshu == 0 and (
                    current_time - Q2_start_time) > JIXIEBI_interval and count_jixie_2 == 0:
                print("quan er: ji xie bi cao zuo")
                for t in range(3):
                    robot.robot_high_control(velocity=[0.0, 0.1])
                    time.sleep(0.5)
                robot.robot_high_control(mode=2)
                time.sleep(1)
                robot.robot_high_control(mode=1)
                time.sleep(1)
                robot.robot_high_control(mode=6)
                time.sleep(1)
                robot.robot_high_control(mode=5)
                time.sleep(1)
                robot.robot_high_control(mode=7)
                time.sleep(1)
                if FLAG_1 == 0:
                    response = send_signal_to_server('circle2:machineArm_first')
                    while response == 'Circle2_END!':
                        break
                else:
                    response = send_signal_to_server('circle2:machineArm_second')
                    while response == 'Circle2_END!':
                        break

                robot.robot_high_control(mode=5)
                time.sleep(1)
                robot.robot_high_control(mode=6)
                time.sleep(1)
                count_jixie_2 = count_jixie_2 + 1
                continue

            if flag_3_jinquan == 1 and flag_3_jieshu == 0 and (
                    current_time - Q3_start_time) > JIXIEBI_interval and count_jixie_3 == 0:
                print("quan san: ji xie bi cao zuo")
                for t in range(3):
                    robot.robot_high_control(velocity=[0.0, -0.1])
                    time.sleep(0.5)
                robot.robot_high_control(mode=2)
                time.sleep(1)
                robot.robot_high_control(mode=1)
                time.sleep(1)
                robot.robot_high_control(mode=6)
                time.sleep(1)
                robot.robot_high_control(mode=5)
                time.sleep(1)
                robot.robot_high_control(mode=7)
                time.sleep(1)
                if FLAG_1 == 0 and FLAG_2 == 0:
                    response = send_signal_to_server('circle3:machineArm_first')
                    while response == 'Circle3_END':
                        break
                else:
                    response = send_signal_to_server('circle3:machineArm_second')
                    while response == 'Circle3_END':
                        break

                robot.robot_high_control(mode=5)
                time.sleep(1)
                robot.robot_high_control(mode=6)
                time.sleep(1)
                count_jixie_3 = count_jixie_3 + 1
                continue

            if flag_4_jinquan == 1 and flag_4_jieshu == 0 and (
                    current_time - Q4_start_time) > JIXIEBI_interval and count_jixie_4 == 0:
                print("quan si: ji xie bi cao zuo")
                for t in range(3):
                    robot.robot_high_control(velocity=[0.0, -0.1])
                    time.sleep(0.5)
                robot.robot_high_control(mode=2)
                time.sleep(1)
                robot.robot_high_control(mode=1)
                time.sleep(1)
                robot.robot_high_control(mode=6)
                time.sleep(1)
                robot.robot_high_control(mode=5)
                time.sleep(1)
                robot.robot_high_control(mode=7)
                time.sleep(1)
                response = send_signal_to_server('circle4:machineArm')
                while response == 'Circle4_END':
                    break
                robot.robot_high_control(mode=5)
                time.sleep(1)
                robot.robot_high_control(mode=6)
                time.sleep(1)
                count_jixie_4 = count_jixie_4 + 1
                continue

            if aruco_ids == 1 and count_ids_1 == 1 and FLAG_1 == 1 and (current_time - Q1_start_time) > min_interval:
                count_ids_1 = count_ids_1 + 1
                flag_1_jieshu = 1
                Q1_over_time = time.time()
                print('quan yi: chu qu')
                while True:
                    for t in range(3):
                        robot.robot_high_control(velocity=[0.15, 0.0], yawSpeed=0.8)
                        time.sleep(0.5)
                    break
                continue
            if aruco_ids == 2 and count_ids_2 == 1 and FLAG_2 == 1 and (current_time - Q2_start_time) > min_interval:
                count_ids_2 = count_ids_2 + 1
                flag_2_jieshu = 1
                print('quan er : zuo zhuan')
                while True:
                    for t in range(3):
                        robot.robot_high_control(velocity=[0.15, 0.0], yawSpeed=0.8)
                        time.sleep(0.5)
                    break
                continue

            if current_yellow_area > 50000 and previous_yellow_area > 48000 and flag_1_jieshu == 1 and (
                    current_time - Q1_over_time) > 20 and count_fencha == 0 and count_ids_2 == 0 and count_ids_3 == 0 and count_ids_4 == 0 and 1 == 2:
                count_fencha = count_fencha + 1
                print('fen cha lu kou')
                while True:
                    for t in range(3):
                        robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.5)
                        time.sleep(0.5)
                    break
                continue

            if current_yellow_area > 50000 and previous_yellow_area > 48000 and flag_1_jieshu == 1 and (
                    current_time - fencha_over_time) > 3 and count_fencha_chuqu == 0 and count_fencha == 1 and count_ids_2 == 0 and count_ids_3 == 0 and count_ids_4 == 0 and 1 == 2:
                count_fencha_chuqu = count_fencha_chuqu + 1
                print('fen cha chu qu')

                while True:
                    for t in range(3):
                        robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.5)
                        time.sleep(0.5)
                    break
                continue

            mykey = cv2.waitKey(1)

            if mykey & 0xFF == ord('q'):
                break
    finally:
        print("释放相机资源")
        cap.release()
        cap1.release()
        cv2.destroyAllWindows()

# [ WARN:0@10.978] global /io/opencv/modules/videoio/src/cap_v4l.cpp (1000) tryIoctl VIDEOIO(V4L2:/dev/video1): select() timeout.
