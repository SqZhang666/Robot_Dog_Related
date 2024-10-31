import cv2
import numpy as np

from prodedure_code.robot_connector import RobotConnector
robot = RobotConnector(ip_address='192.168.123.161', port=8000)

from go13.core.PID import PID
pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
min_speed, max_speed = 0.05, 0.28


def process_image(cv_image):
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

    # 求图像的中心距（计算目标颜色区域的中心坐标）
    m = cv2.moments(color_mask, False)
    try:
        cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
    except ZeroDivisionError:
        cx, cy = width / 2, height / 2

    # 在图像上绘制一个圆圈表示中心点
    radius = 0
    color = (0, 255, 0)  # 绿色
    thickness = 50  # 圆圈的厚度
    cv2.circle(color_mask, (int(cx), int(cy)), radius, color, thickness)

    return color_mask, cx, cy


def patrol_line(cx, width):
    deviation = abs(width / 2 - cx)  # 摄像头图片中心与盲道中心的偏离
    velocity = [max_speed - (max_speed - min_speed) * (deviation / (width / 2)), 0.0]
    yawspeed = pid_yaw.calculate(0, cx - width / 2)

    # robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)
    return velocity, yawspeed


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)

    # 设置窗口为可调整大小
    cv2.namedWindow('video_original', cv2.WINDOW_NORMAL)
    cv2.namedWindow('color_mask', cv2.WINDOW_NORMAL)

    while True:
        ret, cv_image = cap.read()
        if not ret:
            break

        color_mask, cx, cy = process_image(cv_image)
        height, width, channels = cv_image.shape

        cv2.imshow('video_original', cv_image)
        cv2.imshow('color_mask', color_mask)

        mykey = cv2.waitKey(1)
        # 按 q 退出循环，0xFF 是为了排除一些功能键对 q 的 ASCII 码的影响
        if mykey & 0xFF == ord('q'):
            break

        velocity, yawspeed = patrol_line(cx, width)
        robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)

    cap.release()
    cv2.destroyAllWindows()
