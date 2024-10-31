import cv2
import numpy as np
from core.robot_connector import RobotConnector

robot = RobotConnector(ip_address='192.168.123.161', port=8000)
from core.PID import PID
from DetectArucotag import detect_aruco
import time
import socket

# 设置两个常量存储要运输的物资编号1-4
FLAG_1 = 1
FLAG_2 = 1
FLAG_3 = 0
FLAG_4 = 0

pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
min_speed, max_speed = 0.05, 0.3

# 初始化Aruco检测类
dt_aruco = detect_aruco()


def find_connected_component_containing_point(image, points):
    # 查找连通域
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(image, connectivity=8)

    # 在图像上标记点
    for point in points:
        image = cv2.circle(image, point, 0, (0, 255, 0), 10)
    cv2.imshow('find_connected_components_containing_points', image)

    # 找到包含指定点的连通域标签
    found_labels = set()
    for label in range(1, num_labels):
        for point in points:
            x, y = point
            if labels[y, x] == label:
                found_labels.add(label)
                break

    # 创建一个只包含找到的连通域的新图像
    output_image = np.zeros_like(image)
    for label in found_labels:
        output_image[labels == label] = 255

    return output_image


def patrol_line(cv_image):
    height, width, channels = cv_image.shape
    # 设置圆形掩码的中心和半径
    center = ((width // 2) + 9, (height // 2) - 10)
    radius = 145
    # 创建圆形掩码
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.circle(mask, center, radius, (255), thickness=-1)
    # 创建一个三通道的掩码
    mask_3ch = cv2.merge([mask, mask, mask])
    # 应用掩码
    masked_image = cv2.bitwise_and(cv_image, mask_3ch)
    # 创建一个白色背景
    white_background = np.full_like(cv_image, 255)
    # 将圆形区域内的像素保留，其他区域设置为白色
    circular_image = np.where(mask_3ch == 0, white_background, masked_image)
    cv2.imshow('chuli', circular_image)

    # 将掩码应用到HSV图像
    hsv = cv2.cvtColor(circular_image, cv2.COLOR_BGR2HSV)

    # 黄色线条的阈值
    lower_yellow = np.array([0, 7, 0])
    upper_yellow = np.array([68, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 处理黄色线条的掩码
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    yellow_mask = cv2.dilate(yellow_mask, dilate_kernel)
    yellow_mask = cv2.erode(yellow_mask, erode_kernel)
    yellow_mask = cv2.medianBlur(yellow_mask, 9)

    points = [(230, 220), (260, 220)]  # 指定的多个坐标
    yellow_mask = find_connected_component_containing_point(yellow_mask, points)

    # 计算黄色线条的质心
    m_yellow = cv2.moments(yellow_mask, False)
    try:
        cx_yellow, cy_yellow = m_yellow['m10'] / m_yellow['m00'], m_yellow['m01'] / m_yellow['m00']
    except ZeroDivisionError:
        cx_yellow, cy_yellow = height / 2, width / 2
    # 计算黄色线条的面积
    yellow_area = np.sum(yellow_mask > 0)
    # print(f"Yellow area: {yellow_area}")

    # 在掩码图像上画出黄色线条的质心
    cv2.circle(yellow_mask, (int(cx_yellow), int(cy_yellow)), 0, (0, 255, 0), 20)
    cv2.imshow('yellow_mask', yellow_mask)

    # 调用Aruco检测
    aruco_ids = dt_aruco.detect(cv_image)

    velocity = [0.0, 0.0]
    yawspeed = 0.0

    deviation_yellow = abs(width / 2 - cx_yellow)
    velocity[0] = max_speed - (max_speed - min_speed) * (deviation_yellow / (width / 2))
    yawspeed = pid_yaw.calculate(0, cx_yellow - width / 2)
    robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)

    return yellow_area, aruco_ids


def send_signal_to_server(signal):
    HOST = '192.168.123.20'
    PORT = 5003
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(signal.encode())
        data = s.recv(1024)
        print('收到服务器的响应：', data.decode())


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 设置宽度为640像素
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 设置高度为480像
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 设置缓冲区大小为 1 帧
    
    begin_time = time.time()

    # 记录标记点被识别的次数
    count_ids_0 = 0
    count_ids_1 = 0
    count_ids_2 = 0
    count_ids_3 = 0
    count_ids_4 = 0
    count_ids_5 = 0

    # 记录已经进圈标记（0代表没进圈，1代表进圈）
    flag_1_jinquan = 0
    flag_2_jinquan = 0
    flag_3_jinquan = 0
    flag_4_jinquan = 0
    # 记录圈X机械臂操作次数
    count_jixie_1 = 0
    count_jixie_2 = 0
    count_jixie_3 = 0
    count_jixie_4 = 0

    # 记录圈已经结束标记（0代表没结束，1代表结束）
    flag_1_jieshu = 0
    flag_2_jieshu = 0
    flag_3_jieshu = 0
    flag_4_jieshu = 0

    # 记录分叉被识别的次数
    count_fencha = 0
    fencha_over_time = -1
    count_fencha_chuqu = 0
    # flag_1_jieshu = 0

    Q1_start_time = -1  # 圈一计时器开始时间
    Q2_start_time = -1  # 圈二计时器开始时间
    Q3_start_time = -1  # 圈三计时器开始时间
    Q4_start_time = -1  # 圈四计时器开始时间
    Q1_over_time = -1  # 圈一计时器结束时间
    min_interval = 5
    JIXIEBI_interval = 9

    previous_yellow_area = 10
    current_yellow_area = 10
    
    
    for t in range(4):
        robot.robot_high_control(velocity=[0.0, 0.3])
        time.sleep(0.5)
        
    while True:
        ret, cv_image = cap.read()

        # 正常巡线
        current_yellow_area, aruco_ids = patrol_line(cv_image)
        # print(current_yellow_area)

        current_time = time.time()

        # 识别特殊标记（结束标记：0；物品标记：1、2、3、4；障碍物标记：5）
        if aruco_ids == 0 and count_ids_0 == 0  and (current_time - begin_time) > 30:
            count_ids_0 = count_ids_0 + 1
            print("0:结束操作：直行之后向右平移")
            while True:
                for t in range(4):
                    robot.robot_high_control(velocity=[0.0, -0.3])
                    time.sleep(0.5)
                break
        if aruco_ids == 1 and count_ids_1 == 0 and FLAG_1 == 1:
            count_ids_1 = count_ids_1 + 1
            Q1_start_time = time.time()
            flag_1_jinquan = 1
            print("1:第一个圈入圈操作：左转")
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
            print("1:第一个圈不入圈操作：直行")
            while True:
                for t in range(4):
                    robot.robot_high_control(velocity=[0.3, 0.0])
                    time.sleep(0.5)
                break

        if aruco_ids == 2 and count_ids_2 == 0 and FLAG_2 == 1:
            count_ids_2 = count_ids_2 + 1
            Q2_start_time = time.time()
            flag_2_jinquan = 1
            print("2:第二个圈入圈操作：左转")
            while True:
                for t in range(2):
                    robot.robot_high_control(velocity=[0.3, 0.0], yawSpeed=-0.2)
                    time.sleep(0.5)
                for t in range(3):
                    robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.8)
                    time.sleep(0.5)
                break
        if aruco_ids == 2 and count_ids_2 == 0 and FLAG_2 == 0:
            count_ids_2 = count_ids_2 + 1
            print("2:第二个圈不入圈操作：直行")
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
            print("3:第三个圈入圈操作：直行")
            while True:
                for t in range(2):
                    robot.robot_high_control(velocity=[0.3, 0.0])
                    time.sleep(0.5)
                break
        if aruco_ids == 3 and count_ids_3 == 0 and FLAG_3 == 0:
            count_ids_3 = count_ids_3 + 1
            flag_3_jieshu = 1
            print("3:第三个圈不入圈操作：左转")
            while True:
                for t in range(6):
                    robot.robot_high_control(velocity=[0.15, 0.0], yawSpeed=0.7)
                    time.sleep(0.5)
                break

        if aruco_ids == 4 and count_ids_4 == 0 and FLAG_4 == 1:
            count_ids_4 = count_ids_4 + 1
            Q4_start_time = time.time()
            flag_4_jinquan = 1
            print("4:第四个圈入圈操作：直行")
            while True:
                for t in range(2):
                    robot.robot_high_control(velocity=[0.3, 0.0])
                    time.sleep(0.5)
                break
        if aruco_ids == 4 and count_ids_4 == 0 and FLAG_4 == 0:
            count_ids_4 = count_ids_4 + 1
            flag_4_jieshu = 1
            print("4:第四个圈不入圈操作：左转")
            while True:
                for t in range(6):
                    robot.robot_high_control(velocity=[0.15, 0.0], yawSpeed=0.7)
                    time.sleep(0.5)
                break
        
        if aruco_ids == 5 and count_ids_5 == 0 and (current_time - begin_time) > 30:
            count_ids_5 = count_ids_5 + 1
            print("5:障碍区操作……")
            while True:
                for t in range(4):
                    robot.robot_high_control(velocity=[0.0, -0.3])
                    time.sleep(0.5)
                break
            break

        if flag_1_jinquan == 1 and flag_1_jieshu == 0 and (
                current_time - Q1_start_time) > JIXIEBI_interval and count_jixie_1 == 0:
            print("圈一：机械臂操作")
            # 添加狗停止趴下
            #robot.robot_high_control(mode=5)
            #time.sleep(2)
            response = send_signal_to_server('circle1:machineArm')
            while response == 'Circle1_END!':
                break
            # 添加狗起立
            #robot.robot_high_control(mode=6)
            #time.sleep(2)
            count_jixie_1 = count_jixie_1 + 1

        if flag_2_jinquan == 1 and flag_2_jieshu == 0 and (
                current_time - Q2_start_time) > JIXIEBI_interval and count_jixie_2 == 0:
            print("圈二：机械臂操作")
            # 添加狗停止趴下
            #robot.robot_high_control(mode=5)
            #time.sleep(2)
            if FLAG_1 == 0:
                response = send_signal_to_server('circle2:machineArm_first')
                while response == 'Circle2_END!':
                    break
            else:
                response = send_signal_to_server('circle2:machineArm_second')
                while response == 'Circle2_END!':
                    break
            # 添加狗起立
            #robot.robot_high_control(mode=6)
            #time.sleep(2)
            count_jixie_2 = count_jixie_2 + 1

        if flag_3_jinquan == 1 and flag_3_jieshu == 0 and (
                current_time - Q3_start_time) > JIXIEBI_interval and count_jixie_3 == 0:
            print("圈三：机械臂操作")
            # 添加狗停止趴下
            #robot.robot_high_control(mode=5)
            #time.sleep(2)
            if FLAG_1 == 0 and FLAG_2 == 0:
                response = send_signal_to_server('circle3:machineArm_first')
                while response == 'Circle3_END':
                    break
            else:
                response = send_signal_to_server('circle3:machineArm_second')
                while response == 'Circle3_END':
                    break
            # 添加狗起立
            #robot.robot_high_control(mode=6)
            #time.sleep(2)
            count_jixie_3 = count_jixie_3 + 1

        if flag_4_jinquan == 1 and flag_4_jieshu == 0 and (
                current_time - Q4_start_time) > JIXIEBI_interval and count_jixie_4 == 0:
            print("圈四：机械臂操作")
            # 添加狗停止趴下
            #robot.robot_high_control(mode=5)
            #time.sleep(2)
            response = send_signal_to_server('circle4:machineArm')
            while response == 'Circle4_END':
                break
            #robot.robot_high_control(mode=6)
            #time.sleep(2)
            count_jixie_4 = count_jixie_4 + 1

        # “出圈”：出圈以颜色迅速增大为标志：1、2圈需要向左走；3、4圈需要直行
        if aruco_ids == 1 and count_ids_1 == 1 and FLAG_1 == 1 and (current_time - Q1_start_time) > min_interval:
            count_ids_1 = count_ids_1 + 1
            flag_1_jieshu = 1
            Q1_over_time = time.time()
            print('圈1出去,左前行')
            while True:
                for t in range(3):
                    robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.8)
                    time.sleep(0.5)
                break
        if aruco_ids == 2 and count_ids_2 == 1 and FLAG_2 == 1 and (current_time - Q2_start_time) > min_interval:
            count_ids_2 = count_ids_2 + 1
            flag_2_jieshu = 1
            print('圈2出去，直行')
            while True:
                for t in range(3):
                    robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.8)
                    time.sleep(0.5)
                break

        # 分叉口行走
        if current_yellow_area > 50000 and previous_yellow_area > 48000 and flag_1_jieshu == 1 and (
                current_time - Q1_over_time) > 20 and count_fencha == 0 and count_ids_2 == 0 and count_ids_3 == 0 and count_ids_4 == 0:
            count_fencha = count_fencha + 1
            print('分叉操作：左转')
            while True:
                for t in range(3):
                    robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.5)
                    time.sleep(0.5)
                break
                
        if current_yellow_area > 50000 and previous_yellow_area > 48000 and flag_1_jieshu == 1 and (
                current_time - fencha_over_time) > 3 and count_fencha_chuqu == 0 and count_fencha == 1 and count_ids_2 == 0 and count_ids_3 == 0 and count_ids_4 == 0:
            count_fencha_chuqu = count_fencha_chuqu + 1
            print('分叉出去：左转')
            while True:
                for t in range(3):
                    robot.robot_high_control(velocity=[0.2, 0.0], yawSpeed=0.5)
                    time.sleep(0.5)
                break

        # 更新“先前”黄色色素块的值
        previous_yellow_area = current_yellow_area

        mykey = cv2.waitKey(1)

        if mykey & 0xFF == ord('q'):
            break