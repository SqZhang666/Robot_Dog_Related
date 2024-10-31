# 睿康比赛的思路程序


import cv2
import numpy as np

from prodedure_code.robot_connector import RobotConnector

robot = RobotConnector(ip_address='192.168.123.161', port=8000)

from prodedure_code.PID import PID

pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
min_speed, max_speed = 0.05, 0.28

from prodedure_code.DetectArucoTag import detect_aruco
dt_aruco = detect_aruco()   # 实例化一个ArucoTag检测器

from core.turning_judge import turning_judge


# 设置两个常量存储要运输的物资编号1-4
# 这个根据裁判抽取结果人为设置，在启停区夹取物资也需要用这个！！！！！！！！
TARGET1 = 1
TARGET2 = 2

# 巡线
# 对轨道信息处理，根绝轨道信息返回控制机械狗实现巡线的运动参数
def patrol_line(cv_image):
    height, width, channels = cv_image.shape
    # 将图像从RGB转换成HSV色彩空间
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([10, 40, 100])
    upper_yellow = np.array([40, 255, 255])
    color_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 定义矩形结构
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    # 中值滤波
    color_mask = cv2.medianBlur(color_mask, 9)
    # 膨胀
    color_mask = cv2.dilate(color_mask, dilate_kernel)
    # 腐蚀
    color_mask = cv2.erode(color_mask, erode_kernel)
    # res = cv2.bitwise_and(crop_img, crop_img, mask=color_mask)

    # 求图像的中心距（计算目标颜色区域的中心坐标
    m = cv2.moments(color_mask, False)
    try:
        cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
    except ZeroDivisionError:
        cx, cy = height / 2, width / 2

    # 在图像上绘制一个圆圈表示中心点
    radius = 0
    color = (0, 255, 0)  # 在这里选择标记的颜色，这里是绿色
    thickness = 50  # 圆圈的厚度
    cv2.circle(color_mask, (int(cx), int(cy)), radius, color, thickness)

    cv2.imshow('color_mask', color_mask)

    velocity = [0.0, 0.0]
    yawspeed = 0.0

    deviation = abs(width / 2 - cx)  # 摄像头图片中心与盲道中心的偏离
    velocity[0] = max_speed - (max_speed - min_speed) * (deviation / (width / 2))
    yawspeed = pid_yaw.calculate(0, cx - width / 2)
    return velocity, yawspeed




if __name__ == '__main__':

    NUMBER = None

    cap = cv2.VideoCapture(0)

    # 获取摄像头的帧率和帧尺寸
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    size = (width, height)

    # 设置视频编码器，输出文件名和帧率
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, fps, size)


    while True:
        ret, cv_image = cap.read()
        # 将帧写入文件
        out.write(cv_image)
        velocity, yawspeed = patrol_line(cv_image)

        ids = dt_aruco.detect(cv_image)

        id = turning_judge(cv_image)


        if ids is None:     #没识别到标签，未到达投放区正常巡线
            robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)
        else:               # 到达投放区
            # 到达1号投放区域
            if ids[0] == 1:
                # 需要投放
                if ids[0] == TARGET1 or TARGET2:
                    NUMBER = 1
                    velocity1 = None
                    yawspeed1 = None
                    robot.robot_high_control(velocity=velocity1, yawSpeed=yawspeed1)
                    print("在一号区投放物资")
                else:
                    # 这里需要实验看看需不要人为干预
                    print("直行")

            if ids[0] == 2:
                # 需要投放
                if ids[0] == TARGET1 or TARGET2:
                    NUMBER = 2
                    velocity2 = None
                    yawspeed2 = None
                    robot.robot_high_control(velocity=velocity2, yawSpeed=yawspeed2)
                    print("在二号区投放物资")
                else:
                    # 这里需要实验看看需不要人为干预
                    print("直行")

            if ids[0] == 3:
                # 需要投放
                if ids[0] == TARGET1 or TARGET2:
                    NUMBER = 3
                    velocity3 = None
                    yawspeed3 = None
                    # 这里需要实验看不人为干预会不会走进去
                    robot.robot_high_control(velocity=velocity3, yawSpeed=yawspeed3)
                    print("在三号区投放物资")
                else:
                    velocity3 = None
                    yawspeed3 = None
                    robot.robot_high_control(velocity=velocity3, yawSpeed=yawspeed3)
                    print("左拐")

            if ids[0] == 4:
                # 需要投放
                if ids[0] == TARGET1 or TARGET2:
                    NUMBER = 4
                    velocity4 = None
                    yawspeed4 = None
                    # 这里需要实验看不人为干预会不会走进去
                    robot.robot_high_control(velocity=velocity4, yawSpeed=yawspeed4)
                    print("在四号区投放物资")
                else:
                    velocity4 = None
                    yawspeed4 = None
                    robot.robot_high_control(velocity=velocity4, yawSpeed=yawspeed4)
                    print("左拐")

        if id:  # 走到岔路口
            velocity5 = None
            yawspeed5 = None
            robot.robot_high_control(velocity=velocity5, yawSpeed=yawspeed5)

        if NUMBER:#进入投放区
            # 设定循环运行的时间限制（秒）
            time_limit1 = 10
            # 设定走出圈的时间
            time_limit2 = 15
            # 记录开始时间
            start_time = time.time()


            while True:
                ret, cv_image = cap.read()
                velocity, yawspeed = patrol_line(cv_image)
                robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)


                current_time = time.time()
                elapsed_time = current_time - start_time
                if elapsed_time >= time_limit1:
                    # 让狗趴下
                    # 优化多进程
                    robot.robot_high_control()

                    # 放物资
                    place_program_thread = threading.Thread(target=place, args=NUMBER)
                    place_program_thread.start()
                    place_program_thread.join()


                    # 让狗站起来
                    robot.robot_high_control()

                    start_time = time.time()

                # 识别控制出口
                # 卸完物资并且巡线走到出口
                #if elapsed_time >= time_limit1 + time_limit2:
                    # 控制走出出口拐角处
                    # 可能要根据NUMBER写不同的程序
                    robot.robot_high_control()
                    # 退出走进物资投放区整个流程的循环，进入主循环
                    break

        mykey = cv2.waitKey(1)
        if mykey & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
