import cv2
import numpy as np
from core.robot_connector import RobotConnector
import argparse

robot = RobotConnector(ip_address='192.168.123.161', port=8000)
from prodedure_code.PID import PID
import time

debug = 1


def parser_flag():
    global debug
    parser = argparse.ArgumentParser(description="debug or not")
    parser.add_argument("x", type=int, choices=[1, 0], help="debug")
    args = parser.parse_args()
    debug = args.x


parser_flag()


# 调试使用
def add_scale_bars(image, scale_interval=25):
    """
    添加刻度线和标记到图像上。
    :param image: 输入图像
    :param scale_interval: 刻度线的间隔（像素）
    """
    height, width, _ = image.shape
    font = cv2.FONT_HERSHEY_SIMPLEX

    # 添加水平刻度线和标记
    for x in range(0, width, scale_interval):
        cv2.line(image, (x, 0), (x, 10), (0, 255, 0), 2)
        cv2.putText(image, str(x), (x, 30), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    # 添加垂直刻度线和标记
    for y in range(0, height, scale_interval):
        cv2.line(image, (0, y), (10, y), (0, 255, 0), 2)
        cv2.putText(image, str(y), (15, y + 5), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    return image


pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
min_speed = 0.8
#min_speed = 0.1
global max_speed
max_speed = 1.3
#max_speed = 0.5

x_start2, y_start2 = 100, 100
x_end2, y_end2 = 350, 150


def patrol_line(cv_image):
    height, width, channels = cv_image.shape
    cv_image = cv_image[height // 2:, :]
    # 设置圆形掩码的中心和半径
    center = ((width // 2), 0)
    radius = 190
    # 创建圆形掩码
    mask = np.zeros((height // 2, width), dtype=np.uint8)
    cv2.circle(mask, center, radius, (255), thickness=-1)
    # 创建一个三通道的掩码
    mask_3ch = cv2.merge([mask, mask, mask])
    # 应用掩码
    masked_image = cv2.bitwise_and(cv_image, mask_3ch)
    # 创建一个白色背景
    white_background = np.full_like(cv_image, 255)
    # 将圆形区域内的像素保留，其他区域设置为白色
    circular_image = np.where(mask_3ch == 0, white_background, masked_image)

    # cropped_image_with_scales = add_scale_bars(circular_image)
    # cv2.imshow('caijianhou', cropped_image_with_scales)
    # cv2.imshow('chuli', circular_image)
    # 将掩码应用到HSV图像
    hsv = cv2.cvtColor(circular_image, cv2.COLOR_BGR2HSV)

    # 黄色线条的阈值
    lower_yellow = np.array([0, 80, 0])  # 省赛地图
    upper_yellow = np.array([69, 255, 255])# 省赛地图
    #lower_yellow = np.array([0, 24, 43])
    #upper_yellow = np.array([62, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # 黑色线条的阈值
    lower_black = np.array([35, 0, 0])
    upper_black = np.array([255, 255, 95])
    black_mask = cv2.inRange(hsv[25:,:], lower_black, upper_black)
    cv2.imshow('black_mask1', black_mask)
    # 处理黄色线条的掩
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 1))
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    # 处理heise线条的掩
    dilate_kernel1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
    erode_kernel1 = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))

    yellow_mask = cv2.dilate(yellow_mask, dilate_kernel)
    yellow_mask = cv2.erode(yellow_mask, erode_kernel)
    yellow_mask = cv2.medianBlur(yellow_mask, 5)

    # 定义四边形的顶点
    pts = np.array([[310, 75], [350, 150], [360, 150], [360, 75]], np.int32)
    pts = pts.reshape((-1, 1, 2))  # OpenCV要求这种形状
    # 创建一个空白的掩膜
    mask = np.zeros(yellow_mask.shape[:2], dtype=np.uint8)
    # 在掩膜上绘制白色的四边形区域
    cv2.fillPoly(mask, [pts], 255)
    # 使用掩膜提取四边形区域
    dst = cv2.bitwise_and(yellow_mask, yellow_mask, mask=mask)
    cv2.imshow('dst', dst)
    yellow_pixels = cv2.countNonZero(dst)
    # yellow_pixels = 0
    print('yellow_pixels:', yellow_pixels)
    black_mask = cv2.dilate(black_mask, dilate_kernel1)
    black_mask = cv2.erode(black_mask, erode_kernel1)
    black_mask = cv2.medianBlur(black_mask, 9)

    # 计算黄色线条的质心
    rect_yellow_mask = yellow_mask[y_start2:y_end2, x_start2:x_end2]
    m_yellow = cv2.moments(rect_yellow_mask, False)
    try:
        cx_yellow, cy_yellow = m_yellow['m10'] / m_yellow['m00'], m_yellow['m01'] / m_yellow['m00']
        cx_yellow += x_start2
        cy_yellow += y_start2
    except ZeroDivisionError:
        cx_yellow, cy_yellow = height / 2, width / 2
    # 计算黄色线条的面积
    yellow_area = np.sum(yellow_mask > 0)
    print(f"Yellow area: {yellow_area}")

    """# 指定点位 (239, 271)
    point = (239, 271)
    # 检查该点位上方的颜色
    offset = 10  # 定义上方区域的偏移量
    upper_region = yellow_mask[max(0, point[1] - offset):point[1], point[0]:point[0] + 1]
    cv2.imshow('1', upper_region)
    # 判断上方区域是否包含黄色
    is_yellow = np.any(upper_region > 0)
    print(is_yellow)
    if is_yellow:
        max_speed = 1.0
    else:
        max_speed = 0.0
        for _ in range(2):
            robot.robot_high_control(velocity=[0.0, 0.0], yawSpeed=-0.2)"""

    # 计算黑色线条的面积
    black_area = np.sum(black_mask > 0)
    print(f"Black area: {black_area}")

    # 在掩码图像上画出黄色线条的质心
    cv2.circle(yellow_mask, (int(cx_yellow), int(cy_yellow)), 0, (0, 255, 0), 50)
    cv2.rectangle(yellow_mask, (x_start2, y_start2), (x_end2, y_end2), (0, 255, 0), 2)
    cv2.imshow('yellow_mask', yellow_mask)

    cv2.imshow('black_mask2', black_mask)

    velocity = [0.0, 0.0]
    yawspeed = 0.0

    deviation_yellow = abs(width / 2 - cx_yellow)
    velocity[0] = max_speed - (max_speed - min_speed) * (deviation_yellow / (width / 2))
    # print('v!!!!!!!!!!!!!!!!!!!!!!',velocity[0])
    yawspeed = pid_yaw.calculate(0, cx_yellow - width / 2) - 0.3
    # print("yawspeed____________________________",yawspeed)
    if not debug:
        robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)
        time.sleep(0.002)
    rect_yellow_area = np.sum(rect_yellow_mask > 0)
    # print("rect_yellow_area", rect_yellow_area)
    return yellow_area, black_area, yellow_pixels, rect_yellow_area


def right_deliver():
    for _ in range(1):
        print("straight")
        turn_velocity = [0.80, 0.0]
        turn_yawspeed = 0
        if not debug:
            robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
        time.sleep(0.5)
    for _ in range(2):
        print("right")
        turn_velocity = [0.0, -1.0]
        turn_yawspeed = 0
        if not debug:
            robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
        time.sleep(0.5)
    for _ in range(2):
        time.sleep(0.3)
        if not debug:
            robot.robot_high_control(
                mode=1,
                velocity=[0.0, 0.0],
                euler=[0.0, -0.5, 0.0],
                footRaiseHeight=0.0,
                yawSpeed=0.0
            )
        print(1)

    for _ in range(3):
        time.sleep(0.3)
        if not debug:
            robot.robot_high_control(
                mode=1,
                euler=[0.0, -1.0, 0.0],
                footRaiseHeight=0.0,
                velocity=[0.0, 0.0],
                yawSpeed=0.0
            )
        print(2)
    for _ in range(3):
        time.sleep(0.3)
        if not debug:
            robot.robot_high_control(
                mode=1,
                euler=[0.0, -0.5, 0.0],
                footRaiseHeight=0.0,
                velocity=[0.0, 0.0],
                yawSpeed=0.0
            )
        print(3)
    for _ in range(3):
        time.sleep(0.3)
        if not debug:
            robot.robot_high_control(
                mode=1,
                euler=[0.0, 0.0, 0.0],
                footRaiseHeight=0.0,
                velocity=[0.0, 0.0],
                yawSpeed=0.0
            )
        print(2)
    for _ in range(2):
        print("left")
        turn_velocity = [0.2, 0.6]
        turn_yawspeed = 0.0
        if not debug:
            robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
        time.sleep(0.5)
    for _ in range(1):
        print("straight")
        turn_velocity = [1.0, 0.0]
        turn_yawspeed = 0.0
        if not debug:
            robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
        time.sleep(0.5)


def step():
    for _ in range(2):
        turn_velocity = [0.6, 0.0]
        turn_yawspeed = 0.0
        if not debug:
            robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
        time.sleep(0.5)
    for _ in range(4):
        print("step")
        turn_velocity = [0.6, 0.0]
        turn_yawspeed = 0.0
        if not debug:
            robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed,footRaiseHeight=1.2)
        time.sleep(0.5)
    print("over")


def obstacle():
    for _ in range(2):
        turn_velocity = [0.7, 0.0]
        turn_yawspeed = 0.0
        if not debug:
            robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
        time.sleep(0.5)
    for _ in range(2):
        turn_velocity = [0.0, 0.7]
        turn_yawspeed = 0.0
        if not debug:
            robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
        time.sleep(0.5)
    for _ in range(3):
        turn_velocity = [0.8, 0.0]
        turn_yawspeed = 0.2
        if not debug:
            robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
        time.sleep(0.5)
    for _ in range(2):
        turn_velocity = [0.2, -0.70]
        turn_yawspeed = 0
        if not debug:
            robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
        time.sleep(0.5)
    for _ in range(1):
        turn_velocity = [0.7, 0.0]
        turn_yawspeed = 0
        if not debug:
            robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
        time.sleep(0.5)


if __name__ == '__main__':
    cap = cv2.VideoCapture(1)
    last_execution_time = time.time()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    count_xiehuo = 0
    flag_xiehuo_over = 0
    count_taijie = 0
    flag_taijie_over = 0
    count_obstacle = 0
    flag_obstacle_over = 0
    count_over = 0

    xiehuo_start_time = 0  # 卸货区计时器开始时间
    taijie_start_time = 0  # 台阶区计时器开始时间
    s = 0
    T = 0
    turn_number = 0
    start_time = time.time()

    try:
        while True:

            ret, frame = cap.read()
            # cv2.imshow('color', frame)
            current_yellow_area, current_black_area, yellow_pixels, rect_yellow_area = patrol_line(frame)
            current_time = time.time()
            min_interval = 3

            if yellow_pixels > 1200 and ((current_time - xiehuo_start_time) > 0.5 or xiehuo_start_time == 0) and (
                    current_time - last_execution_time) > min_interval and flag_obstacle_over == 0 and (
                    (current_time - taijie_start_time) > 1 or taijie_start_time == 0) and (
                    current_time - start_time) > 4 and turn_number < 2:
                t = 0
                last_execution_time = time.time()
                turn_number += 1
                # if turn_number == 2:
                #    max_spedd = 0.8
                for _ in range(2):
                    print("turn")
                    turn_velocity = [0.4, -0.10]
                    turn_yawspeed = -2.2
                    if not debug:
                        robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
                    time.sleep(0.5)
            if yellow_pixels > 1200 and ((current_time - xiehuo_start_time) > 0.5 or xiehuo_start_time == 0) and (
                    current_time - last_execution_time) > min_interval and flag_obstacle_over == 0 and (
                    (current_time - taijie_start_time) > 1 or taijie_start_time == 0) and (
                    current_time - start_time) > 4 and turn_number == 2:
                t = 0
                last_execution_time = time.time()
                turn_number += 1
                for _ in range(2):
                    print("turn")
                    turn_velocity = [0.5, 0.0]
                    turn_yawspeed = 0.0
                    if not debug:
                        robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
                    time.sleep(0.5)
                for _ in range(2):
                    print("turn")
                    turn_velocity = [0.4, 0.0]
                    turn_yawspeed = -2.0
                    if not debug:
                        robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
                    time.sleep(0.5)

            # 卸货地点
            if current_black_area > 3000 and count_xiehuo == 0 and current_time - last_execution_time > 3 and turn_number == 1:
                count_xiehuo = count_xiehuo + 1
                print("到达卸货点：启动到达卸货点操作……")
                right_deliver()
                # 这里添加到达卸货点的操作
                # time.sleep(1)
                print("离开卸货点：继续巡线……")
                flag_xiehuo_over = 1
                xiehuo_start_time = time.time()  # 记录卸货区离开的时间

            # 台阶区     5000
            if current_black_area > 4000 and count_taijie == 0 and flag_xiehuo_over == 1 and turn_number == 2:
                current_time_taijie = time.time()
                if xiehuo_start_time and (current_time_taijie - xiehuo_start_time > 2):
                    count_taijie = count_taijie + 1
                    print("到达台阶区……")
                    # 这里添加到达台阶区的操作
                    step()

                    print("离开台阶区……")
                    flag_taijie_over = 1
                    taijie_start_time = time.time()  # 记录卸货区离开的时间
                    max_speed = 0.8

            # 障碍区
            if current_black_area > 10000 and flag_taijie_over == 1 and count_obstacle == 0 and turn_number == 3:
                current_time_obstacle = time.time()
                if taijie_start_time and (current_time_obstacle - taijie_start_time > 4):  # 判断是否已经超过10秒
                    count_obstacle = count_obstacle + 1
                    print("到达障碍区……")

                    # 这里添加到达台阶区的操作
                    obstacle()

                    print("离开障碍物区域……")
                    flag_obstacle_over = 1
                    obstacle_over_time = time.time()

            # 结束区
            if current_yellow_area < 10000 and flag_obstacle_over == 1 and count_over == 0:
                current_time_over = time.time()
                if current_time_over - obstacle_over_time > 1:
                    if not debug:
                        for _ in range(2):
                            robot.robot_high_control(velocity=[0.7, 0.0], yawSpeed=0.0)
                            time.sleep(0.5)
                        print("比赛结束...")
                    break

            mykey = cv2.waitKey(1)

            if mykey & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()



'''
（使用省赛的hsv）
转角出去的太多了
倾倒区停的有一些问题
由于第三个转角没转好引起的避障存在一些问题

（使用另一个hsv）
前两个转角好了一些，第三个还是有问题
倾倒物资的动作怎么回事，卡顿
'''
