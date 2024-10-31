import cv2
import numpy as np
from core.robot_connector import RobotConnector

robot = RobotConnector(ip_address='192.168.123.161', port=8000)
from prodedure_code.PID import PID
import time

pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
min_speed, max_speed = 0.2, 0.3

y_start = 200
y_end = 250
x_start = 350
x_end = 400

x_start2, y_start2 = 116, 248
x_end2, y_end2 = 355, 333


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
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    hsv1 = cv2.cvtColor(circular_image, cv2.COLOR_BGR2HSV)
    # 黄色线条的阈值
    lower_yellow = np.array([0, 80, 0])
    upper_yellow = np.array([69, 255, 255])
    yellow_mask1 = cv2.inRange(hsv1, lower_yellow, upper_yellow)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # 黑色线条的阈值
    lower_black = np.array([49, 0, 0])
    upper_black = np.array([255, 255, 109])
    black_mask = cv2.inRange(hsv1, lower_black, upper_black)

    # 处理黄色线条的掩码
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 1))
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    yellow_mask1 = cv2.dilate(yellow_mask1, dilate_kernel)
    yellow_mask1 = cv2.erode(yellow_mask1, erode_kernel)
    yellow_mask1 = cv2.medianBlur(yellow_mask1, 9)

    yellow_mask = cv2.dilate(yellow_mask, dilate_kernel)
    yellow_mask = cv2.erode(yellow_mask, erode_kernel)
    yellow_mask = cv2.medianBlur(yellow_mask, 9)

    yellow_pixels = cv2.countNonZero(yellow_mask1[y_start:y_end, x_start:x_end])

    black_mask = cv2.dilate(black_mask, dilate_kernel)
    black_mask = cv2.erode(black_mask, erode_kernel)
    black_mask = cv2.medianBlur(black_mask, 9)

    # 计算指定区域的黄色的质心
    rect_yellow_mask = yellow_mask[y_start2:y_end2, x_start2:x_end2]
    # cv2.imshow('chuli', rect_yellow_mask)
    m_yellow = cv2.moments(rect_yellow_mask, False)
    try:
        cx_yellow, cy_yellow = m_yellow['m10'] / m_yellow['m00'], m_yellow['m01'] / m_yellow['m00']
        cx_yellow += x_start2
        cy_yellow += y_start2
    except ZeroDivisionError:
        cx_yellow, cy_yellow = height / 2, width / 2
    # 计算黄色线条的面积
    yellow_area = np.sum(yellow_mask > 0)
    # print(f"Yellow area: {yellow_area}")

    # 指定点位 (239, 271)
    point = (239, 271)
    # 检查该点位上方的颜色
    offset = 10  # 定义上方区域的偏移量
    upper_region = yellow_mask[max(0, point[1] - offset):point[1], point[0]:point[0] + 1]
    cv2.imshow('1',upper_region)
    # 判断上方区域是否包含黄色
    is_yellow = np.any(upper_region > 0)
    print(is_yellow)
    if is_yellow:
        max_speed = 1.0
    else:
        max_speed = 0.0
        for _ in range(2):
            robot.robot_high_control(velocity=[0.0,0.0], yawSpeed=-0.2)
        

    # 计算黑色线条的面积
    black_area = np.sum(black_mask > 0)
    #print(f"Black area: {black_area}")

    # 在掩码图像上画出黄色线条的质心
    cv2.circle(yellow_mask, (int(cx_yellow), int(cy_yellow)), 0, (0, 0, 0), 10)
    cv2.imshow('yellow_mask', yellow_mask)

    cv2.imshow('black_mask', black_mask)

    velocity = [0.0, 0.0]
    yawspeed = 0.0

    deviation_yellow = abs(width / 2 - cx_yellow)
    velocity[0] = max_speed - (max_speed - min_speed) * (deviation_yellow / (width / 2))
    yawspeed = pid_yaw.calculate(0, cx_yellow - width / 2)
    robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)

    return yellow_area, black_area, yellow_pixels, is_yellow


def right_deliver():
    while True:

        t = 0
        while True:
            time.sleep(0.02)
            t += 1
            if 0 < t <= 1:
                print("right")
                turn_velocity = [0.30, 0.0]
                turn_yawspeed = 0
                robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
                time.sleep(0.5)
            else:
                break
        t = 0
        while True:
            time.sleep(0.002)
            t += 1
            if 0 < t <= 5:
                print("right")
                turn_velocity = [0.0, -0.30]
                turn_yawspeed = 0
                robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
                time.sleep(0.5)
            else:
                break
        motiontime = 0
        while True:
            time.sleep(0.002)
            motiontime = motiontime + 1
            if 0 < motiontime < 5:
                robot.robot_high_control(
                    mode=1,
                    velocity=[0.0, 0.0],
                    euler=[0.0, -0.5, 0.0],
                    footRaiseHeight=0.0,
                    yawSpeed=0.0
                )
                print(1)
                time.sleep(0.1)

            elif 5 <= motiontime < 8:
                robot.robot_high_control(
                    mode=1,
                    euler=[0.0, -1, 0.0],
                    footRaiseHeight=0.0,
                    velocity=[0.0, 0.0],
                    yawSpeed=0.0
                )
                print(2)
                time.sleep(0.1)
            elif 8 <= motiontime < 10:
                time.sleep(0.1)
                robot.robot_high_control(
                    mode=1,
                    euler=[0.0, -0.5, 0.0],
                    footRaiseHeight=0.0,
                    velocity=[0.0, 0.0],
                    yawSpeed=0.0
                )
                print(3)
                time.sleep(0.1)
            elif 10 <= motiontime < 12:
                time.sleep(0.1)
                robot.robot_high_control(
                    mode=1,
                    euler=[0.0, 0.0, 0.0],
                    footRaiseHeight=0.0,
                    velocity=[0.0, 0.0],
                    yawSpeed=0.0
                )
                print(2)
                time.sleep(0.5)
            else:
                break
        t = 0
        while True:
            time.sleep(0.02)
            t += 1
            if 0 < t <= 5:
                print("right")
                turn_velocity = [0.0, 0.30]
                turn_yawspeed = 0
                robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
                time.sleep(0.5)
            else:
                break
        t = 0
        while True:
            time.sleep(0.002)
            t += 1
            if 0 < t <= 2:
                print("left")
                turn_velocity = [0.3, 0.0]
                turn_yawspeed = 0
                robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
                time.sleep(0.5)
            else:
                break
        break


def step():
    t = 0
    while True:
        t += 1
        if 0 < t <= 12:
            print("step")
            turn_velocity = [max_speed + 0.1, 0.0]
            turn_yawspeed = 0.0
            # robot.robot_high_control(gaitType=3,velocity=turn_velocity, yawSpeed=turn_yawspeed,footRaiseHeight=20)
            robot.robot_high_control(gaitType=3, velocity=turn_velocity, yawSpeed=turn_yawspeed)
            time.sleep(0.5)
        elif 12 < t <= 14:

            turn_velocity = [0.3, 0.0]
            turn_yawspeed = 0
            robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
            break


if __name__ == '__main__':
    # 打开视频文件s
    cap = cv2.VideoCapture(1)
    last_execution_time = time.time() - 10
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 设置宽度为640像素
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 设置高度为480像
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    count_xiehuo = 0
    flag_xiehuo_over = 0
    count_taijie = 0
    flag_taijie_over = 0
    count_over = 0
    xiehuo_start_time = None  # 卸货区计时器开始时间
    taijie_start_time = None  # 台阶区计时器开始时间
    start_time = time.time()  # 记录开始的时间
    s = 0
    T = 0
    while True:
        ret, frame = cap.read()
        # cv2.imshow('color', cv_image)

        # 正常巡线
        current_yellow_area, current_black_area, yellow_pixels, is_yellow = patrol_line(frame)
        current_time = time.time()
        min_interval = 8
        if yellow_pixels > 900 and (current_time - last_execution_time) > min_interval:
            t = 0
            last_execution_time = time.time()
            while True:
                t += 1
                if 0 < t <= 2:
                    print("turn")
                    turn_velocity = [0.4, 0.0]
                    turn_yawspeed = -2
                    robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
                    time.sleep(0.5)
                else:
                    # robot.robot_high_control(velocity=[0.0, 0.0], yawSpeed=0.0)
                    # time.sleep(0.7)
                    break
        # 卸货地点
        #print(current_time - start_time)
        if current_yellow_area > 1000 and current_black_area > 8000 and count_xiehuo == 0 and current_time - start_time > 10:
            count_xiehuo = count_xiehuo + 1
            print("到达卸货点：启动到达卸货点操作……")
            time.sleep(1)
            right_deliver()
            # 这里添加到达卸货点的操作
            # time.sleep(1)
            print("离开卸货点：继续巡线……")
            flag_xiehuo_over = 1
            xiehuo_start_time = time.time()  # 记录卸货区离开的时间

        # 台阶区
        if current_black_area > 13000 and count_taijie == 0 and flag_xiehuo_over == 1:
            current_time_taijie = time.time()
            if xiehuo_start_time and (current_time_taijie - xiehuo_start_time > 5):  # 判断是否已经超过10秒
                count_taijie = count_taijie + 1
                print("到达台阶区……")

                # 这里添加到达台阶区的操作
                step()

                print("离开台阶区……")
                flag_taijie_over = 1
                taijie_start_time = time.time()  # 记录卸货区离开的时间

        # 结束区
        if current_yellow_area < 40000 and flag_taijie_over == 1 and count_over == 0:
            current_time_over = time.time()
            if taijie_start_time and (current_time_over - taijie_start_time > 10):  # 判断是否已经超过10秒
                print('程序终止……')
                # end_time = time.time() + 1  # 设定结束时间为当前时间加3秒
                t = 0
                while True:
                    # 持续三秒向机器狗传输相同的数据
                    t += 1
                    if 0 < t <= 3:
                        robot.robot_high_control(velocity=[0.4, 0.1], yawSpeed=0.0)
                        print("发送数据给机器狗...")  # 示例打印，可以替换为实际数据传输代码
                        time.sleep(0.5)  # 确保循环执行频率合适
                    else:
                        break
                break

        mykey = cv2.waitKey(1)

        if mykey & 0xFF == ord('q'):
            break