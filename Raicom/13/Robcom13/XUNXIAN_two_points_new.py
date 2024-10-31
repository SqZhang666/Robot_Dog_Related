import cv2
import numpy as np
from core.robot_connector import RobotConnector
from core.PID import PID
from DetectArucotag import detect_aruco
import time

# 设置两个常量存储要运输的物资编号1-4
FLAG_1 = 1
FLAG_2 = 0
FLAG_3 = 0
FLAG_4 = 1

pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
min_speed, max_speed = 0.05, 0.28

# 初始化Aruco检测类
dt_aruco = detect_aruco()
robot = RobotConnector(ip_address='192.168.123.161', port=8000)


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
    height, width, _ = cv_image.shape
    circular_image = preprocess_image(cv_image)
    cv2.imshow('chuli', circular_image)
    yellow_area, cx_yellow, cy_yellow = detect_yellow_line(circular_image)
    aruco_ids = dt_aruco.detect(cv_image)
    velocity = [0.0, 0.0]
    yawspeed = 0.0
    deviation_yellow = abs(width / 2 - cx_yellow)
    velocity[0] = max_speed - (max_speed - min_speed) * (deviation_yellow / (width / 2))
    yawspeed = pid_yaw.calculate(0, cx_yellow - width / 2)
    # 根据偏差调整速度和转向角度
    print(deviation_yellow)
    if deviation_yellow > 20:  # 根据需要设置合适的阈值
        velocity[0] *= 0.7  # 减速
        yawspeed *= 1.3  # 增加转向角度
    robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)
    return yellow_area, aruco_ids


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    count_ids = [0] * 6
    count_fencha = 0
    flag_1_jieshu = 0
    Q1_start_time = None
    Q2_start_time = None
    Q1_over_time = None
    min_interval = 5
    previous_yellow_area = 10

    while True:
        ret, cv_image = cap.read()
        current_yellow_area, aruco_ids = patrol_line(cv_image)
        current_time = time.time()

        if aruco_ids == 0 and count_ids[0] == 0:
            count_ids[0] += 1
            print("0:结束操作：直行之后向右平移")
            for _ in range(4):
                robot.robot_high_control(velocity=[0.0, -0.3])
                time.sleep(0.5)

        if aruco_ids == 1 and count_ids[1] == 0 and FLAG_1 == 1:
            count_ids[1] += 1
            Q1_start_time = time.time()
            print("1:第一个圈入圈操作：左转")
            for _ in range(4):
                robot.robot_high_control(velocity=[0.3, 0.0], yawSpeed=-0.2)
                time.sleep(0.5)
            for _ in range(3):
                robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.8)
                time.sleep(0.5)

        if aruco_ids == 1 and count_ids[1] == 0 and FLAG_1 == 0:
            count_ids[1] += 1
            flag_1_jieshu = 1
            Q1_over_time = time.time()
            print("1:第一个圈不入圈操作：直行")
            for _ in range(4):
                robot.robot_high_control(velocity=[0.3, 0.0])
                time.sleep(0.5)

        if aruco_ids == 2 and count_ids[2] == 0 and FLAG_2 == 1:
            count_ids[2] += 1
            Q2_start_time = time.time()
            print("2:第二个圈入圈操作：左转")
            for _ in range(4):
                robot.robot_high_control(velocity=[0.3, 0.0], yawSpeed=-0.2)
                time.sleep(0.5)
            for _ in range(3):
                robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.8)
                time.sleep(0.5)

        if aruco_ids == 2 and count_ids[2] == 0 and FLAG_2 == 0:
            count_ids[2] += 1
            print("2:第二个圈不入圈操作：直行")
            for _ in range(4):
                robot.robot_high_control(velocity=[0.3, 0.0])
                time.sleep(0.5)

        if aruco_ids == 3 and count_ids[3] == 0 and FLAG_3 == 1:
            count_ids[3] += 1
            print("3:第三个圈入圈操作：直行")
            for _ in range(2):
                robot.robot_high_control(velocity=[0.3, 0.0])
                time.sleep(0.5)

        if aruco_ids == 3 and count_ids[3] == 0 and FLAG_3 == 0:
            count_ids[3] += 1
            print("3:第三个圈不入圈操作：左转")
            for _ in range(6):
                robot.robot_high_control(velocity=[0.15, 0.0], yawSpeed=0.7)
                time.sleep(0.5)

        if aruco_ids == 4 and count_ids[4] == 0 and FLAG_4 == 1:
            count_ids[4] += 1
            print("4:第四个圈入圈操作：直行")
            for _ in range(2):
                robot.robot_high_control(velocity=[0.3, 0.0])
                time.sleep(0.5)

        if aruco_ids == 4 and count_ids[4] == 0 and FLAG_4 == 0:
            count_ids[4] += 1
            print("4:第四个圈不入圈操作：左转")
            for _ in range(6):
                robot.robot_high_control(velocity=[0.15, 0.0], yawSpeed=0.7)
                time.sleep(0.5)

        if aruco_ids == 5 and count_ids[5] == 0:
            count_ids[5] += 1
            print("5:障碍区操作……")
            for t in range(4):
                robot.robot_high_control(velocity=[0.0, -0.3])
                time.sleep(0.5)

        if aruco_ids == 1 and count_ids[1] == 1 and FLAG_1 == 1 and (current_time - Q1_start_time) > min_interval:
            count_ids[1] += 1
            flag_1_jieshu = 1
            Q1_over_time = time.time()
            print('圈1出去,左前行')
            for _ in range(3):
                robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.8)
                time.sleep(0.5)

        if aruco_ids == 2 and count_ids[2] == 1 and FLAG_2 == 1 and (current_time - Q2_start_time) > min_interval:
            count_ids[2] += 1
            print('圈2出去，直行')
            for _ in range(3):
                robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.8)
                time.sleep(0.5)
        
        print(previous_yellow_area)
        if current_yellow_area > 50000 and previous_yellow_area > 48000 and flag_1_jieshu == 1 and (
                current_time - Q1_over_time) > 20 and count_fencha == 0 and count_ids[2] == 0 and count_ids[3] == 0 and \
                count_ids[4] == 0:
            count_fencha += 1
            print('分叉操作：左转')
            for _ in range(3):
                robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.5)
                time.sleep(0.5)

        previous_yellow_area = current_yellow_area

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
