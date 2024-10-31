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
# 定义一个全局变量用于存储前一次的偏差值
previous_deviation_yellow = None


def parser_flag():
    global FLAG_1, FLAG_2, FLAG_3, FLAG_4
    parser = argparse.ArgumentParser(description="control FLAG's number")
    parser.add_argument("x", type=int, choices=[1, 2, 3, 4], help="the first")
    parser.add_argument("y", type=int, choices=[1, 2, 3, 4], help="the second")
    args = parser.parse_args()
    variables = [FLAG_1, FLAG_2, FLAG_3, FLAG_4]
    variables[args.x - 1] = 1
    variables[args.y - 1] = 1
    FLAG_1, FLAG_2, FLAG_3, FLAG_4 = variables


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
    hsv = cv2.cvtColor(circular_image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([0, 7, 0])
    upper_yellow = np.array([68, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    yellow_mask = cv2.dilate(yellow_mask, dilate_kernel)
    yellow_mask = cv2.erode(yellow_mask, erode_kernel)
    yellow_mask = cv2.medianBlur(yellow_mask, 9)
    points = [(230, 220), (260, 220)]
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
    global previous_deviation_yellow  # 引用全局变量

    height, width, _ = cv_image.shape
    circular_image = preprocess_image(cv_image)
    cv2.imshow('chuli', circular_image)
    yellow_area, cx_yellow, cy_yellow = detect_yellow_line(circular_image)
    aruco_ids = dt_aruco.detect(cv_image)
    deviation_yellow = abs(width / 2 - cx_yellow)
    if previous_deviation_yellow is None:
        previous_deviation_yellow = deviation_yellow
    deviation_change = deviation_yellow - previous_deviation_yellow
    previous_deviation_yellow = deviation_yellow
    velocity = [0.0, 0.0]
    velocity[0] = max_speed - (max_speed - min_speed) * (deviation_yellow / (width / 2))
    yawspeed = pid_yaw.calculate(0, cx_yellow - width / 2)
    if abs(deviation_change) > certain_threshold:  # certain_threshold 是你设定的变化幅度阈值
        velocity[0] *= 0.5 if deviation_change > 0 else 1.5
        yawspeed *= 1.2 if deviation_change > 0 else 0.8
    robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)
    return yellow_area, aruco_ids


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
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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

    # yi chu qi ting qu
    for t in range(5):
        robot.robot_high_control(velocity=[0.0, 0.3])
        time.sleep(0.5)

    try:
        while True:
            ret, cv_image = cap.read()

            #
            current_yellow_area, aruco_ids = patrol_line(cv_image)
            # print(current_yellow_area)

            current_time = time.time()

            if aruco_ids == 0 and count_ids_0 == 0 and (current_time - begin_time) > 30:
                count_ids_0 = count_ids_0 + 1
                print("0:over")
                while True:
                    for t in range(4):
                        robot.robot_high_control(velocity=[0.0, -0.3])
                        time.sleep(0.5)
                    break
            if aruco_ids == 1 and count_ids_1 == 0 and FLAG_1 == 1:
                count_ids_1 = count_ids_1 + 1
                Q1_start_time = time.time()
                flag_1_jinquan = 1
                print("1:first zuoguai")
                while True:
                    for t in range(4):
                        robot.robot_high_control(velocity=[0.3, 0.0], yawSpeed=-0.2)
                        time.sleep(0.5)
                    t = 0
                    for t in range(3):
                        robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.8)
                        time.sleep(0.5)
                    break
            if aruco_ids == 1 and count_ids_1 == 0 and FLAG_1 == 0:
                count_ids_1 = count_ids_1 + 1
                flag_1_jieshu = 1
                Q1_over_time = time.time()
                print("1:first zhixing")
                while True:
                    for t in range(4):
                        robot.robot_high_control(velocity=[0.3, 0.0], yawSpeed=-0.3)
                        time.sleep(0.5)
                    break

            if aruco_ids == 2 and count_ids_2 == 0 and FLAG_2 == 1:
                count_ids_2 = count_ids_2 + 1
                Q2_start_time = time.time()
                flag_2_jinquan = 1
                print("2:second-ruquan")
                while True:
                    for t in range(4):
                        robot.robot_high_control(velocity=[0.3, 0.0], yawSpeed=-0.1)
                        time.sleep(0.5)
                    for t in range(3):
                        robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.9)
                        time.sleep(0.5)
                    break
            if aruco_ids == 2 and count_ids_2 == 0 and FLAG_2 == 0:
                count_ids_2 = count_ids_2 + 1
                print("2:second-zhixing")
                flag_2_jieshu = 1
                while True:
                    for t in range(4):
                        robot.robot_high_control(velocity=[0.3, 0.0])
                        time.sleep(0.5)
                    break

            if aruco_ids == 3 and count_ids_3 == 0 and FLAG_3 == 1:
                count_ids_3 = count_ids_3 + 1
                Q3_start_time = time.time()
                flag_3_jinquan = 1
                print("3:third-zhixing")
                while True:
                    for t in range(2):
                        robot.robot_high_control(velocity=[0.3, 0.0])
                        time.sleep(0.5)
                    break
            if aruco_ids == 3 and count_ids_3 == 0 and FLAG_3 == 0:
                count_ids_3 = count_ids_3 + 1
                flag_3_jieshu = 1
                print("3:third-zuoguai")
                while True:
                    for t in range(6):
                        robot.robot_high_control(velocity=[0.15, 0.0], yawSpeed=0.7)
                        time.sleep(0.5)
                    break

            if aruco_ids == 4 and count_ids_4 == 0 and FLAG_4 == 1:
                count_ids_4 = count_ids_4 + 1
                Q4_start_time = time.time()
                flag_4_jinquan = 1
                print("4:forth-zhixing")
                while True:
                    for t in range(2):
                        robot.robot_high_control(velocity=[0.3, 0.0])
                        time.sleep(0.5)
                    break
            if aruco_ids == 4 and count_ids_4 == 0 and FLAG_4 == 0:
                count_ids_4 = count_ids_4 + 1
                flag_4_jieshu = 1
                print("4:forth-zuoguai")
                while True:
                    for t in range(6):
                        robot.robot_high_control(velocity=[0.1, 0.0], yawSpeed=0.7)
                        time.sleep(0.5)
                    break

            if aruco_ids == 5 and count_ids_5 == 0 and (current_time - begin_time) > 30:
                count_ids_5 = count_ids_5 + 1
                print("5:zhangaiwu")
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
            if aruco_ids == 2 and count_ids_2 == 1 and FLAG_2 == 1 and (current_time - Q2_start_time) > min_interval:
                count_ids_2 = count_ids_2 + 1
                flag_2_jieshu = 1
                print('quan er : zuo zhuan')
                while True:
                    for t in range(3):
                        robot.robot_high_control(velocity=[0.15, 0.0], yawSpeed=0.8)
                        time.sleep(0.5)
                    break

            if current_yellow_area > 50000 and previous_yellow_area > 48000 and flag_1_jieshu == 1 and (
                    current_time - Q1_over_time) > 20 and count_fencha == 0 and count_ids_2 == 0 and count_ids_3 == 0 and count_ids_4 == 0 and 1 == 2:
                count_fencha = count_fencha + 1
                print('fen cha lu kou')
                while True:
                    for t in range(3):
                        robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.5)
                        time.sleep(0.5)
                    break

            if current_yellow_area > 50000 and previous_yellow_area > 48000 and flag_1_jieshu == 1 and (
                    current_time - fencha_over_time) > 3 and count_fencha_chuqu == 0 and count_fencha == 1 and count_ids_2 == 0 and count_ids_3 == 0 and count_ids_4 == 0 and 1 == 2:
                count_fencha_chuqu = count_fencha_chuqu + 1
                print('fen cha chu qu')
                while True:
                    for t in range(3):
                        robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.5)
                        time.sleep(0.5)
                    break

            previous_yellow_area = current_yellow_area

            mykey = cv2.waitKey(1)

            if mykey & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
